// Copyright 2026 KweonTJ

#include <gtest/gtest.h>

#include <cmath>

#include "omx_rl_control_cpp/runtime_math.hpp"

namespace omx_rl_control_cpp
{

TEST(RuntimeMath, WrapToPiUsesHalfOpenRange)
{
  constexpr double pi = 3.14159265358979323846;
  EXPECT_NEAR(wrapToPi(0.0), 0.0, 1.0e-12);
  EXPECT_NEAR(wrapToPi(pi), -pi, 1.0e-12);
  EXPECT_NEAR(wrapToPi(-pi), -pi, 1.0e-12);
  EXPECT_NEAR(wrapToPi(3.0 * pi), -pi, 1.0e-12);
}

TEST(RuntimeMath, ForwardKinematicsMatchesPythonGolden)
{
  const Vector4 low(-2.82743, -1.79071, -0.942478, -1.79071);
  const Vector4 high(2.82743, 1.57080, 1.38230, 2.04204);
  const Vector4 initial(0.0, 1.15968, -0.48813, -0.67155);
  OpenManipulatorKinematics kinematics(low, high, initial);
  const Vector4 joints(0.104311, 0.027612, -0.001534, -1.638291);
  const Vector3 actual = kinematics.forward(joints);
  const Vector3 expected(
    0.06547054710952178, 0.01522945448717368, 0.5044472477410328);
  EXPECT_TRUE(actual.isApprox(expected, 1.0e-9));
}

TEST(RuntimeMath, ObservationMatchesPythonGolden)
{
  const Vector4 low(-2.82743, -1.79071, -0.942478, -1.79071);
  const Vector4 high(2.82743, 1.57080, 1.38230, 2.04204);
  ObservationBuilder builder(ObservationConfig{
      low, high, Vector4::Constant(2.0),
      Vector3(0.10, -0.18, 0.08), Vector3(0.42, 0.18, 0.52),
      Vector2(-0.08, 0.0), Vector3(0.0, 0.0, 0.016),
      -0.010, 0.019, 0.25});

  const Vector4 qpos(
    0.10043393527776082, 0.022523214605905072,
    -0.010819704916832859, -1.6394194530321642);
  const Observation actual = builder.build(
    qpos, Vector4::Zero(), 0.0, 0.0,
    Vector3(0.0629760076503128, 0.014408120492818305, 0.5062717659680216),
    Vector3(0.1296955125440309, 0.0, 0.37833678084846856),
    0.0, false, 0, Vector4::Zero());
  Observation expected;
  expected <<
    0.03552128002047539, 0.07882066071033478,
    -0.1984969824552536, -0.9210537672042847,
    0.0, 0.0, 0.0, 0.0, -0.3103448152542114, 0.0,
    -1.0, 0.08004511147737503, 1.0,
    -0.8144030570983887, 0.0, 0.42880356311798096,
    0.41699689626693726, -0.08004511147737503, -0.5815226435661316,
    0.0, 1.0, 0.0, 1.0, -0.5181180834770203, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  EXPECT_TRUE(actual.isApprox(expected, 2.0e-7));
}

TEST(RuntimeMath, PitchCouplingMaintainsJointSum)
{
  ActionLimitConfig config{
    Vector4::Constant(-3.0), Vector4::Constant(3.0),
    Vector4::Constant(0.014), 1.0, 0.1, 0.02,
    Vector4::Constant(2.0), Vector4::Constant(100.0), true, 0.0, 0.028};
  ActionLimiter limiter(config);
  limiter.reset(Vector4::Zero());
  const Vector4 target = limiter.step(
    Vector4::Zero(), Vector4(0.0, 0.5, 0.25, 0.7), true, true);
  EXPECT_NEAR(target.tail<3>().sum(), 0.0, 1.0e-12);
}

TEST(RuntimeMath, PlacePhaseAndGraspFlagUseTrainingSchema)
{
  const Vector4 low(-2.82743, -1.79071, -0.942478, -1.79071);
  const Vector4 high(2.82743, 1.57080, 1.38230, 2.04204);
  ObservationBuilder builder(ObservationConfig{
      low, high, Vector4::Constant(2.0),
      Vector3(0.10, -0.18, 0.08), Vector3(0.42, 0.18, 0.52),
      Vector2(-0.08, 0.0), Vector3(0.0, 0.0, 0.016),
      -0.010, 0.019, 0.25});

  const Observation observation = builder.build(
    Vector4::Zero(), Vector4::Zero(), -0.010, 0.0,
    Vector3(0.20, 0.0, 0.20), Vector3(0.27, 0.0, 0.1815),
    0.0, true, 2, Vector4::Zero());
  EXPECT_DOUBLE_EQ(observation(24), 1.0);
  EXPECT_TRUE(
    observation.segment<4>(25).isApprox(
      Vector4(0.0, 0.0, 1.0, 0.0), 0.0));
}

}  // namespace omx_rl_control_cpp
