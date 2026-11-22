#include "omx_turtlebot3_control_cpp/control.hpp"
#include <gtest/gtest.h>
#include <algorithm>
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
  int repeat_reads = 0, write_delay_ms = 0;
  FakeBus() {
    bytes[16] = bytes[199] = 1; put(bytes, 42, 1217);
    for (size_t i = 0; i < 4; ++i) put(bytes, 224+4*i, 2048);
    put(bytes, 240, 2048); bytes[272] = 0xec; bytes[273] = 0xff;
  }
  uint16_t ping() override { return 20480; }
  std::vector<uint8_t> read() override {
    if (repeat_reads > 0) --repeat_reads;
    else if (tick) millis += 50;
    put(bytes, 10, millis); return bytes;
  }
  void write(uint16_t a, const std::vector<uint8_t> &b) override {
    writes.emplace_back(a, b);
    std::this_thread::sleep_for(std::chrono::milliseconds(write_delay_ms));
    if (a+b.size() <= bytes.size()) std::copy(b.begin(), b.end(), bytes.begin()+a);
    if (a == 19) bytes[15] = 1;
    if (a == 220) std::copy_n(bytes.begin()+200, 16, bytes.begin()+224);
  }
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
  v::Observation obs{};
  const auto action = actor.infer(obs);
  // Jetson OpenCV 4.5.4 ReLU6 incorrectly returned zero for these negatives.
  EXPECT_NEAR(action[0], -.0559722148, 1e-6);
  EXPECT_NEAR(action[2], -.1057149619, 1e-6);
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
TEST(Transport, AwaitsNewCounterWithinTheReadDeadlineWithoutWrites) {
  auto bus = std::make_unique<FakeBus>(); auto *fake = bus.get();
  v::Transport transport(config(), false, std::move(bus));
  const auto old = transport.last().millis; fake->repeat_reads = 3;
  const auto sample = transport.read();
  EXPECT_GT(sample.millis, old); EXPECT_EQ(fake->repeat_reads, 0);
  EXPECT_EQ(transport.writes(), 0u);
}
TEST(Transport, StayClockStartsAfterProfileAndTorqueSetup) {
  auto bus = std::make_unique<FakeBus>(); auto *fake = bus.get(); fake->bytes[199] = 0;
  fake->write_delay_ms = 8;  // Total setup exceeds the 150 ms control deadline.
  v::Transport transport(commissioned(), true, std::move(bus));
  transport.begin_stay({0, 0, 0, 0});
  EXPECT_NO_THROW(transport.stay_step());
}
TEST(Transport, RejectsStaleCounterAndBaseMotion) {
  for (int variant = 0; variant < 3; ++variant) {
    auto bus = std::make_unique<FakeBus>(); auto *fake = bus.get();
    v::Transport transport(config(), false, std::move(bus));
    if (variant == 0) fake->tick = false;
    if (variant == 1) put(fake->bytes, 150, 1);
    if (variant == 2) put(fake->bytes, 128, 10);
    EXPECT_THROW(transport.read(), std::runtime_error); EXPECT_EQ(transport.writes(), 0u);
  }
}
TEST(Transport, IdleTorqueOffIsReadableButArmedTorqueLossIsRejected) {
  auto bus = std::make_unique<FakeBus>(); auto *fake = bus.get(); fake->bytes[199] = 0;
  v::Transport transport(commissioned(), true, std::move(bus));
  EXPECT_FALSE(transport.read().torque);
  EXPECT_THROW(transport.arm(), std::runtime_error); EXPECT_EQ(transport.writes(), 0u);
  transport.begin_stay({0, 0, 0, 0}); EXPECT_TRUE(transport.last().torque);
  EXPECT_THROW(transport.finish_stay(), std::runtime_error);
  fake->bytes[199] = 0;
  EXPECT_THROW(transport.read(), std::runtime_error);
}
TEST(Transport, StayEnablesOnlyArmAfterPreloadingGoalsAndProfiles) {
  auto bus = std::make_unique<FakeBus>(); auto *fake = bus.get(); fake->bytes[199] = 0;
  v::Transport transport(commissioned(), true, std::move(bus));
  EXPECT_THROW(transport.begin_stay({0, 0, -3, 0}), std::runtime_error);
  EXPECT_EQ(transport.writes(), 0u);
  transport.begin_stay({0, 0, 0, 0});
  bool goal = false, profile = false, torque = false;
  for (const auto &[address, bytes] : fake->writes) {
    EXPECT_NE(address, 149); EXPECT_NE(address, 150); EXPECT_NE(address, 198);
    if (address == 220) goal = true;
    if (address == 328) profile = true;
    if (address == 199) { EXPECT_TRUE(goal && profile); EXPECT_EQ(bytes, std::vector<uint8_t>{1}); torque = true; }
  }
  EXPECT_TRUE(torque);
}
TEST(Stay, RespectsCoordinatedVelocityAndAccelerationAndMeasuredArrival) {
  const auto c = config(); v::Sample sample; sample.stamp = 10;
  const v::Joints goal{.1, .2, -.1, -.25};
  v::StayTrajectory move(c, sample, goal, 10);
  v::Joints previous{}, velocity{};
  const double dt = .01;
  for (int tick = 1; tick <= int((move.duration()+1)/dt); ++tick) {
    sample.stamp = 10+tick*dt; sample.q = previous; sample.dq = velocity;
    const auto q = move.update(sample, sample.stamp);
    for (size_t i = 0; i < 4; ++i) {
      const double speed = (q[i]-previous[i])/dt;
      EXPECT_LE(std::abs(speed), c.velocity+1e-7);
      EXPECT_LE(std::abs(speed-velocity[i])/dt, c.acceleration+1e-5);
      EXPECT_NEAR(q[i]/goal[i], q[0]/goal[0], 1e-10);
      velocity[i] = speed;
    }
    previous = q;
    if (tick*dt < move.duration()+c.data["control"]["stable_s"].as<double>()) EXPECT_FALSE(move.ready());
  }
  EXPECT_TRUE(move.ready());
  for (size_t i = 0; i < 4; ++i) EXPECT_DOUBLE_EQ(previous[i], goal[i]);
}
TEST(Stay, ElapsedTimeAloneCannotCompleteAndTimeoutIsEnforced) {
  auto c = config(); c.data["control"]["stay_timeout_s"] = 4.;
  v::Sample sample; sample.stamp = 10;
  v::StayTrajectory move(c, sample, {.05, 0, 0, 0}, 10);
  for (int tick = 1; tick <= 39; ++tick) {
    sample.stamp = 10+tick*.1; move.update(sample, sample.stamp); EXPECT_FALSE(move.ready());
  }
  sample.stamp = 14.; move.update(sample, sample.stamp);
  sample.stamp = 14.1; EXPECT_THROW(move.update(sample, sample.stamp), std::runtime_error);
}
TEST(Stay, RejectsMovingStartAndMissedControlDeadline) {
  const auto c = config(); v::Sample sample; sample.stamp = 10; sample.dq[0] = .1;
  EXPECT_THROW(v::StayTrajectory(c, sample, {}, 10), std::runtime_error);
  sample.dq[0] = 0;
  v::StayTrajectory move(c, sample, {}, 10);
  sample.stamp = 10.2;
  EXPECT_THROW(move.update(sample, sample.stamp), std::runtime_error);
}
TEST(Stay, ArrivalRequiresStoppedEncodersForTheWholeSettleWindow) {
  const auto c = config(); v::Sample sample; sample.stamp = 10;
  v::StayTrajectory move(c, sample, {}, 10);
  for (int tick = 1; tick <= 40; ++tick) {
    sample.stamp = 10+tick*.1;
    sample.dq[0] = tick < 36 ? .04 : 0;
    move.update(sample, sample.stamp);
    if (tick < 39) EXPECT_FALSE(move.ready());
  }
  EXPECT_TRUE(move.ready());
  sample.dq[0] = .04; sample.stamp = 14.1;
  move.update(sample, sample.stamp); EXPECT_FALSE(move.ready());
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
