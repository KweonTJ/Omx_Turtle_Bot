#include "omx_turtlebot3_control_cpp/control.hpp"
#include <gtest/gtest.h>
#include <chrono>
#include <limits>
#include <thread>

namespace v = omx_v6;
const std::string root = WORKSPACE_SRC;
v::Config config() { return v::Config(root+"/omx_turtlebot3_runtime/config/runtime.yaml", root+"/omx_turtlebot3_runtime/assets/robot_real.urdf"); }
void put(std::vector<uint8_t> &b, size_t i, int32_t n) { for (unsigned j = 0; j < 4; ++j) b.at(i+j) = uint32_t(n) >> (8*j); }
struct FakeBus : v::Bus {
  std::vector<uint8_t> bytes = std::vector<uint8_t>(344);
  std::vector<std::pair<uint16_t, std::vector<uint8_t>>> writes;
  uint32_t millis = 1000;
  bool tick = true;
  FakeBus() {
    bytes[16] = bytes[199] = 1; put(bytes, 42, 1217);
    for (size_t i = 0; i < 4; ++i) put(bytes, 224+4*i, 2048);
    put(bytes, 240, 2048); bytes[272] = 0xec; bytes[273] = 0xff;
  }
  uint16_t ping() override { return 20480; }
  std::vector<uint8_t> read() override { if (tick) millis += 50; put(bytes, 10, millis); return bytes; }
  void write(uint16_t a, const std::vector<uint8_t> &b) override { writes.emplace_back(a, b); if (a == 19) bytes[15] = 1; }
};
v::Config commissioned() {
  auto c = config();
  for (auto key : {"joint_mapping_verified", "camera_extrinsics_verified", "marker_mount_verified", "scene_alignment_verified", "grasp_thresholds_verified", "placement_thresholds_verified", "hardware_watchdog_verified"}) c.data["commissioning"][key] = true;
  c.data["commissioning"]["evidence_directory"] = "synthetic-test-only";
  c.data["estimation"]["current_idle_raw"] = 0;
  c.data["estimation"]["current_grip_delta_raw"] = 10;
  c.data["estimation"]["current_release_delta_raw"] = 5;
  return c;
}
TEST(Model, ExecutesOnnxAndRejectsNonfinite) {
  v::Actor actor(root+"/omx_turtlebot3_policy/models/omx_turtlebot3_v6");
  v::Observation obs{}; EXPECT_NO_THROW(actor.infer(obs));
  obs[1] = std::numeric_limits<float>::quiet_NaN(); EXPECT_THROW(actor.infer(obs), std::runtime_error);
}
TEST(Trajectory, SameReferenceAvoidsRatchetAndDoubleWristCorrection) {
  v::Joints q{0, .1, .1, -.4}, goal{.01, .11, .13, -.42};
  auto out = v::controller_trajectory(q, goal);
  for (size_t i = 0; i < 4; ++i) EXPECT_NEAR(out[i], goal[i], 1e-12);
  q[2] += .01; out = v::controller_trajectory(q, goal);
  EXPECT_NEAR(out[2], goal[2], 1e-12); EXPECT_DOUBLE_EQ(out[3], goal[3]);
}
TEST(Trajectory, RejectsJoint3DeltaAndRawRange) {
  EXPECT_THROW(v::controller_trajectory({0, 0, 0, 0}, {0, 0, .21, 0}), std::runtime_error);
  EXPECT_THROW(v::controller_trajectory({0, 0, 1.37, 0}, {0, 0, 1.20, 0}), std::runtime_error);
}
TEST(Control, UsesResidualAndAccelerationLimits) {
  const auto c = config(); const v::Joints q{}, goal{.1, .1, .1, .1}, action{1, -1, .5, -.5}, zero{};
  auto result = v::step(c, q, goal, q, action, zero, 2, 0, 4, .05, 0, .019, c.low, c.high);
  for (auto v : result.velocity) EXPECT_LE(std::abs(v), .3*.05+1e-12);
  for (auto v : result.q) EXPECT_LE(std::abs(v), .3*.05*.05+1e-12);
  EXPECT_DOUBLE_EQ(result.finger, .006*.05);
}
TEST(Control, RotationHoldsAllPitchJointsEvenWithPriorVelocity) {
  const auto c = config(); const v::Joints q{0, .1, .2, -.5}, goal{-1.57, .1, .2, -.5}, action{1, 1, 1, 1}, velocity{.1, .1, .1, .1};
  const auto out = v::step(c, q, goal, q, action, velocity, 5, 0, 1, .05, 0, -.01, c.low, c.high);
  for (size_t i = 1; i < 4; ++i) { EXPECT_DOUBLE_EQ(out.q[i], q[i]); EXPECT_DOUBLE_EQ(out.action[i], 0); EXPECT_DOUBLE_EQ(out.velocity[i], 0); }
}
TEST(Control, RejectsDeadlineInvalidPhaseAndUrdfViolation) {
  const auto c = config(); v::Joints q{}, goal{}, action{}, velocity{};
  EXPECT_THROW(v::step(c, q, goal, q, action, velocity, 2, 0, 1, .21, 0, 0, c.low, c.high), std::runtime_error);
  EXPECT_THROW(v::step(c, q, goal, q, action, velocity, 9, 0, 1, .05, 0, 0, c.low, c.high), std::runtime_error);
  goal[2] = -2.;
  EXPECT_THROW(v::step(c, q, goal, q, action, velocity, 2, 0, 1, .05, 0, 0, c.low, c.high), std::runtime_error);
}
TEST(Transport, ReadOnlyCannotArmOrWrite) {
  auto bus = std::make_unique<FakeBus>(); auto *fake = bus.get();
  v::Transport transport(config(), false, std::move(bus));
  EXPECT_THROW(transport.arm(), std::runtime_error);
  EXPECT_THROW(transport.command({}, 0), std::runtime_error);
  transport.hold(); EXPECT_EQ(transport.writes(), 0u); EXPECT_TRUE(fake->writes.empty());
}
TEST(Transport, PreservesSignedCurrentAndVoltage) {
  auto bus = std::make_unique<FakeBus>();
  v::Transport transport(config(), false, std::move(bus));
  auto s = transport.read(); EXPECT_DOUBLE_EQ(s.current, -20); EXPECT_DOUBLE_EQ(s.voltage, 12.17);
  for (double x : s.q) EXPECT_DOUBLE_EQ(x, 0);
}
TEST(Transport, RejectsStaleCounterBaseMotionAndTorqueLoss) {
  for (int variant = 0; variant < 4; ++variant) {
    auto bus = std::make_unique<FakeBus>(); auto *fake = bus.get();
    v::Transport transport(config(), false, std::move(bus));
    if (variant == 0) fake->tick = false;
    if (variant == 1) put(fake->bytes, 150, 1);
    if (variant == 2) put(fake->bytes, 128, 10);
    if (variant == 3) fake->bytes[199] = 0;
    EXPECT_THROW(transport.read(), std::runtime_error); EXPECT_EQ(transport.writes(), 0u);
  }
}
TEST(Transport, RequiresCommissioningBeforeAnyWrite) {
  auto bus = std::make_unique<FakeBus>();
  EXPECT_THROW(v::Transport(config(), true, std::move(bus)), std::runtime_error);
}
TEST(Transport, InvalidProfilesFailBeforeFirstWrite) {
  auto c = commissioned(); c.data["serial"]["arm_profile_velocity"] = 0;
  auto bus = std::make_unique<FakeBus>();
  v::Transport transport(c, true, std::move(bus));
  EXPECT_THROW(transport.arm(), std::runtime_error); EXPECT_EQ(transport.writes(), 0u);
}
TEST(Transport, GroupsJointsAndNeverWritesWheelsOrTorque) {
  auto bus = std::make_unique<FakeBus>(); auto *fake = bus.get();
  v::Transport transport(commissioned(), true, std::move(bus));
  transport.arm(); transport.read();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  transport.command({.001, .001, .001, -.001}, 0);
  transport.hold();
  bool grouped = false;
  for (const auto &[address, bytes] : fake->writes) {
    EXPECT_NE(address, 150); EXPECT_NE(address, 149); EXPECT_NE(address, 199); EXPECT_NE(address, 198);
    if (address == 200) { EXPECT_EQ(bytes.size(), 16u); grouped = true; }
  }
  EXPECT_TRUE(grouped); EXPECT_GT(transport.writes(), 0u);
}
TEST(Transport, HoldNeverChangesGripperGoal) {
  auto bus = std::make_unique<FakeBus>(); auto *fake = bus.get();
  v::Transport transport(commissioned(), true, std::move(bus)); transport.arm();
  const auto before = fake->writes.size(); transport.hold();
  for (size_t i = before; i < fake->writes.size(); ++i) { EXPECT_NE(fake->writes[i].first, 216); EXPECT_NE(fake->writes[i].first, 221); }
}
TEST(Transport, WatchdogStopsWithoutPythonAndLatches) {
  auto c = commissioned(); c.max_cycle = .04;
  auto bus = std::make_unique<FakeBus>();
  v::Transport transport(c, true, std::move(bus)); transport.arm();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_THROW(transport.command({}, 0), std::runtime_error);
  EXPECT_THROW(transport.arm(), std::runtime_error);
}
