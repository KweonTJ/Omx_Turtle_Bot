#include "omx_turtlebot3_control_cpp/control.hpp"
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <algorithm>
#include <cmath>
#include <fcntl.h>
#include <spawn.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <unistd.h>
extern char **environ;

namespace omx_v6 {
namespace {
constexpr double tau = 6.2831853071795864769;
void require(bool ok, const std::string &why) { if (!ok) throw std::runtime_error(why); }
uint32_t u32(const std::vector<uint8_t> &b, size_t i) {
  return uint32_t(b.at(i)) | uint32_t(b.at(i+1)) << 8 | uint32_t(b.at(i+2)) << 16 | uint32_t(b.at(i+3)) << 24;
}
int32_t i32(const std::vector<uint8_t> &b, size_t i) { return static_cast<int32_t>(u32(b, i)); }
int16_t i16(const std::vector<uint8_t> &b, size_t i) { return static_cast<int16_t>(uint16_t(b.at(i)) | uint16_t(b.at(i+1)) << 8); }
void append(std::vector<uint8_t> &b, int32_t v) {
  for (unsigned shift = 0; shift < 32; shift += 8) b.push_back(uint32_t(v) >> shift);
}
std::vector<uint8_t> packed(int32_t x) { std::vector<uint8_t> b; append(b, x); return b; }
class SdkBus : public Bus {
 public:
  explicit SdkBus(const Config &config) : id_(config.data["serial"]["id"].as<int>()) {
    const auto device = config.data["serial"]["device"].as<std::string>();
    // Never open a second driver on a device already owned by ros2_control/Python.
    posix_spawn_file_actions_t actions;
    require(posix_spawn_file_actions_init(&actions) == 0, "Cannot initialize ownership check");
    posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, "/dev/null", O_WRONLY, 0);
    posix_spawn_file_actions_addopen(&actions, STDERR_FILENO, "/dev/null", O_WRONLY, 0);
    char exe[] = "/usr/bin/fuser";
    char *argv[] = {exe, const_cast<char *>(device.c_str()), nullptr};
    pid_t child;
    const int spawned = posix_spawn(&child, exe, &actions, nullptr, argv, environ);
    posix_spawn_file_actions_destroy(&actions);
    require(spawned == 0, "fuser is required for serial ownership check");
    int status = 0;
    require(waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 1, "Serial device owned or ownership check failed");
    lock_fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    require(lock_fd_ >= 0, "Cannot open OpenCR device");
    if (flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) { ::close(lock_fd_); lock_fd_ = -1; throw std::runtime_error("Serial device locked"); }
    port_.reset(dynamixel::PortHandler::getPortHandler(device.c_str()));
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    try {
      require(port_->openPort() && port_->setBaudRate(config.data["serial"]["baudrate"].as<int>()), "Cannot initialize OpenCR port");
      require(ioctl(lock_fd_, TIOCEXCL) == 0, "Cannot claim exclusive serial access");
    } catch (...) { close(); throw; }
  }
  ~SdkBus() override { close(); }
  uint16_t ping() override {
    uint16_t model = 0; uint8_t error = 0;
    const int comm = packet_->ping(port_.get(), id_, &model, &error);
    check(comm, error); return model;
  }
  std::vector<uint8_t> read() override {
    std::vector<uint8_t> raw(344); uint8_t error = 0;
    const int comm = packet_->readTxRx(port_.get(), id_, 0, raw.size(), raw.data(), &error);
    check(comm, error); return raw;
  }
  void write(uint16_t address, const std::vector<uint8_t> &value) override {
    auto bytes = value; uint8_t error = 0;
    const int comm = packet_->writeTxRx(port_.get(), id_, address, bytes.size(), bytes.data(), &error);
    check(comm, error);
  }
 private:
  void check(int comm, uint8_t error) {
    require(comm == COMM_SUCCESS && error == 0, std::string("OpenCR communication: ")+packet_->getTxRxResult(comm)+" device_error="+std::to_string(error));
  }
  void close() {
    if (port_) port_->closePort();
    if (lock_fd_ >= 0) { ioctl(lock_fd_, TIOCNXCL); ::close(lock_fd_); lock_fd_ = -1; }
  }
  uint8_t id_;
  int lock_fd_{-1};
  std::unique_ptr<dynamixel::PortHandler> port_;
  dynamixel::PacketHandler *packet_{};  // SDK singleton, not owned.
};
}
std::unique_ptr<Bus> sdk_bus(const Config &config) { return std::make_unique<SdkBus>(config); }
Transport::Transport(Config config, bool execute, std::unique_ptr<Bus> bus)
 : config_(std::move(config)), bus_(std::move(bus)), execute_(execute) {
  if (execute_) config_.require_commissioned();
  require(bus_ && bus_->ping() == config_.data["serial"]["model_number"].as<int>(), "Unexpected OpenCR identity");
  read();
  require(!last_.ros, "Existing OpenCR heartbeat active");
  watchdog_ = std::thread([this] {
    while (!shutdown_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      std::lock_guard<std::recursive_mutex> guard(mutex_);
      if (armed_ && now()-command_time_ > config_.max_cycle) {
        watchdog_fault_ = true;
        try { hold(); } catch (...) { armed_ = false; }
      }
    }
  });
}
Transport::~Transport() {
  shutdown_ = true;
  if (watchdog_.joinable()) watchdog_.join();
  try { hold(); } catch (...) {}
}
Sample Transport::read() {
  std::lock_guard<std::recursive_mutex> guard(mutex_);
  const double start = now(); const auto b = bus_->read(); const double stamp = now();
  require(b.size() == 344 && stamp-start <= .10, "OpenCR telemetry failed/deadline exceeded");
  Sample sample; sample.stamp = stamp; sample.millis = u32(b, 10); sample.ros = b[15] != 0;
  require(b[16] == 1 && b[199] == 1, "Manipulator disconnected or torque unavailable; never auto-enable");
  require(!armed_ || sample.ros, "OpenCR heartbeat lost");
  require(!have_sample_ || sample.millis > last_.millis, "Feedback counter stalled or rebooted");
  const auto s = config_.data["serial"];
  for (size_t i = 0; i < 4; ++i) {
    const int tick = i32(b, 224+4*i);
    require(tick >= 0 && tick <= 4096, "Joint encoder ticks invalid");
    sample.q[i] = (tick-s["joint_zero_ticks"][i].as<int>())*tau/4096*s["joint_signs"][i].as<int>();
    sample.currents[i] = i16(b, 264+2*i);
  }
  const int grip_tick = i32(b, 240);
  require(grip_tick >= 0 && grip_tick <= 4096, "Gripper encoder ticks invalid");
  sample.finger = (grip_tick-s["gripper_zero_tick"].as<int>())*tau/4096*s["gripper_m_per_rad"].as<double>();
  if (have_sample_) {
    const double dt = (sample.millis-last_.millis)/1000.;
    for (size_t i = 0; i < 4; ++i) sample.dq[i] = (sample.q[i]-last_.q[i])/dt;
    sample.finger_velocity = (sample.finger-last_.finger)/dt;
  }
  sample.current = i16(b, 272); sample.voltage = u32(b, 42)/100.;
  const auto c = config_.data["control"];
  require(sample.voltage >= c["voltage_range"][0].as<double>() && sample.voltage <= c["voltage_range"][1].as<double>(), "Voltage outside envelope");
  require(std::abs(sample.current) <= c["current_abort_raw"].as<double>(), "Gripper overcurrent");
  for (size_t i = 0; i < 2; ++i) {
    sample.wheels[i] = i32(b, 136+4*i);
    require(std::abs(int64_t(i32(b, 128+4*i))) <= 1, "Base motion outside stationary task");
    require(!have_sample_ || std::abs(int64_t(sample.wheels[i])-last_.wheels[i]) <= 3, "Base encoder movement");
  }
  for (size_t i = 0; i < 6; ++i) require(i32(b, 150+4*i) == 0, "Base velocity command present");
  have_sample_ = true; last_ = sample; return sample;
}
void Transport::write(uint16_t address, const std::vector<uint8_t> &bytes) {
  require(execute_, "Read-only C++ transport cannot write");
  // Validate address AND payload size: no range can spill into another register.
  size_t length = 0;
  switch (address) {
    case 19: case 220: case 221: case 304: case 305: case 306: case 328: case 329: case 330: case 343: case 344: length = 1; break;
    case 200: case 284: case 308: length = 16; break;
    case 216: case 300: case 324: length = 4; break;
    case 340: length = 2; break;
    default: throw std::runtime_error("Register outside arm-only write allowlist");
  }
  require(bytes.size() == length, "Invalid register write length");
  bus_->write(address, bytes); ++writes_;
}
void Transport::beat() { ++heartbeat_; write(19, {heartbeat_}); }
void Transport::goals(const Joints &q, double finger, bool include_gripper) {
  const auto s = config_.data["serial"];
  std::vector<uint8_t> ticks;
  for (size_t i = 0; i < 4; ++i) {
    require(std::isfinite(q[i]), "Nonfinite joint command");
    const double tick = std::round(q[i]/s["joint_signs"][i].as<int>()*4096/tau+s["joint_zero_ticks"][i].as<int>());
    require(tick >= 0 && tick <= 4096, "Command encoder range rejected");
    append(ticks, static_cast<int32_t>(tick));
  }
  const double grip = std::round(finger/s["gripper_m_per_rad"].as<double>()*4096/tau+s["gripper_zero_tick"].as<int>());
  require(std::isfinite(grip) && grip >= 0 && grip <= 4096, "Gripper command encoder range rejected");
  // One grouped position register write + commit for all four joints.
  write(200, ticks); write(220, {1});
  if (include_gripper) { write(216, packed(static_cast<int32_t>(grip))); write(221, {1}); }
}
void Transport::arm() {
  std::lock_guard<std::recursive_mutex> guard(mutex_);
  require(execute_ && !armed_ && have_sample_ && !last_.ros && !watchdog_fault_, "Cannot arm this C++ transport");
  config_.require_commissioned();
  require(now()-last_.stamp <= config_.max_cycle, "Cannot arm stale feedback");
  const auto s = config_.data["serial"];
  const int current = s["gripper_goal_current_raw"].as<int>();
  require(current >= 1 && current <= 80, "Unapproved gripper current");
  for (const char *key : {"arm_profile_acceleration", "arm_profile_velocity", "gripper_profile_acceleration", "gripper_profile_velocity"}) {
    const int value = s[key].as<int>(); require(value >= 1 && value <= 20, "Unapproved nonzero motor profile");
  }
  for (size_t i = 0; i < 4; ++i) require(last_.q[i] >= config_.low[i] && last_.q[i] <= config_.high[i], "Initial joint outside URDF limits");
  require(last_.finger >= -.010 && last_.finger <= .019, "Initial gripper outside limits");
  commanded_ = last_.q; finger_command_ = last_.finger;
  write(306, {1}); write(330, {1}); write(344, {1});
  beat(); goals(commanded_, finger_command_);
  for (auto pair : {std::pair<uint16_t, const char *>{284, "arm_profile_acceleration"}, {308, "arm_profile_velocity"}}) {
    std::vector<uint8_t> values;
    for (int i = 0; i < 4; ++i) append(values, s[pair.second].as<int>());
    write(pair.first, values); write(pair.first == 284 ? 304 : 328, {1});
  }
  write(300, packed(s["gripper_profile_acceleration"].as<int>())); write(305, {1});
  write(324, packed(s["gripper_profile_velocity"].as<int>())); write(329, {1});
  write(340, {uint8_t(current), uint8_t(current >> 8)}); write(343, {1});
  command_time_ = now(); armed_ = true;
}
void Transport::command(const Joints &q, double finger) {
  std::lock_guard<std::recursive_mutex> guard(mutex_);
  const double stamp = now();
  require(armed_ && !watchdog_fault_ && stamp-last_.stamp <= config_.max_cycle && stamp-command_time_ <= config_.max_cycle, "Command requires fresh armed feedback/deadline");
  require(std::isfinite(finger) && finger >= -.010 && finger <= .019, "Gripper command range rejected");
  const double dt = stamp-command_time_;
  require(dt > 0, "Invalid command clock");
  for (size_t i = 0; i < 4; ++i) {
    require(std::isfinite(q[i]) && q[i] >= config_.low[i] && q[i] <= config_.high[i], "Command outside URDF limits");
    require(std::abs(q[i]-commanded_[i]) <= config_.velocity*dt+.002, "Transport joint rate limit");
    require(std::abs(q[i]-last_.q[i]) <= config_.tracking, "Motor not tracking target");
  }
  require(std::abs(finger-finger_command_) <= config_.grip_velocity*dt+.0002, "Transport gripper rate limit");
  const Joints output = controller_trajectory(last_.q, q);
  beat(); goals(output, finger);
  commanded_ = output; finger_command_ = finger; command_time_ = now();
}
void Transport::hold() {
  std::lock_guard<std::recursive_mutex> guard(mutex_);
  if (!armed_) return;
  armed_ = false;  // Latched even if feedback or hold transmission fails.
  const Sample sample = read();
  beat(); goals(sample.q, finger_command_, false);  // Preserve the gripper goal register.
}
}  // namespace omx_v6
