#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <atomic>
#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace omx_v6 {
using Joints = std::array<double, 4>;
using Observation = std::array<float, 69>;
double now();
struct Config {
  YAML::Node data;
  Joints low{}, high{};
  double period{}, max_cycle{}, velocity{}, acceleration{}, grip_velocity{}, tracking{};
  double joint_step{}, residual_scale{}, training_period{};
  explicit Config(const std::string &yaml, const std::string &urdf);
  void require_commissioned() const;
};
void verify_model(const std::string &directory);
Joints controller_trajectory(const Joints &reference, const Joints &target);
class Actor {
 public:
  explicit Actor(const std::string &directory);
  ~Actor();
  Joints infer(const Observation &observation);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
struct Step {
  Joints q{}, action{}, velocity{};
  double finger{};
};
Step step(const Config &config, const Joints &target, const Joints &goal,
          const Joints &measured, Joints action, const Joints &velocity,
          int phase, int waypoint, int count, double dt,
          double finger, double finger_goal, const Joints &low, const Joints &high);

struct Sample {
  double stamp{};
  Joints q{}, dq{};
  double finger{}, finger_velocity{}, current{}, voltage{};
  std::array<int32_t, 2> wheels{};
  uint32_t millis{};
  bool ros{}, torque{};
  std::array<int32_t, 4> currents{};
};
// Coordinated minimum-jerk startup motion, independent of PPO residuals.
class StayTrajectory {
 public:
  StayTrajectory(const Config &config, const Sample &start, const Joints &goal, double stamp);
  Joints update(const Sample &sample, double stamp);
  bool ready() const { return ready_; }
  double duration() const { return duration_; }
 private:
  friend class Transport;
  void restart_clock(double stamp);
  Joints start_{}, goal_{};
  double started_{}, previous_{}, duration_{}, timeout_{}, max_cycle_{};
  double position_tolerance_{}, velocity_tolerance_{}, stable_s_{}, settled_since_{-1};
  bool ready_{};
};
class Bus {
 public:
  virtual ~Bus() = default;
  virtual uint16_t ping() = 0;
  virtual std::vector<uint8_t> read() = 0;
  virtual void write(uint16_t address, const std::vector<uint8_t> &value) = 0;
};
std::unique_ptr<Bus> sdk_bus(const Config &config);
class Transport {
 public:
  Transport(Config config, bool execute, std::unique_ptr<Bus> bus);
  ~Transport();
  Sample read();
  void arm();
  void begin_stay(const Joints &goal);
  bool stay_step();
  void finish_stay();
  bool torque_enabled();
  void command(const Joints &q, double finger);
  void hold();
  uint64_t writes() const { return writes_.load(); }
  bool armed() const { return armed_; }
  const Sample &last() const { return last_; }
 private:
  void write(uint16_t address, const std::vector<uint8_t> &bytes);
  void beat();
  void goals(const Joints &q, double finger, bool include_gripper = true);
  void arm_impl(bool enable_torque);
  Config config_;
  std::unique_ptr<Bus> bus_;
  bool execute_{}, armed_{}, have_sample_{};
  std::atomic<uint64_t> writes_{0};
  std::recursive_mutex mutex_;
  std::atomic<bool> shutdown_{false};
  std::thread watchdog_;
  bool watchdog_fault_{};
  uint8_t heartbeat_{};
  Sample last_;
  Joints commanded_{};
  double finger_command_{}, command_time_{};
  std::unique_ptr<StayTrajectory> stay_;
};
}  // namespace omx_v6
