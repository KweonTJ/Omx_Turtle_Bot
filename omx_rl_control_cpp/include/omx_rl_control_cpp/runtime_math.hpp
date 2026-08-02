// Copyright 2026 KweonTJ

#ifndef OMX_RL_CONTROL_CPP__RUNTIME_MATH_HPP_
#define OMX_RL_CONTROL_CPP__RUNTIME_MATH_HPP_

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "omx_rl_control_cpp/policy_actor.hpp"

namespace omx_rl_control_cpp
{

using Vector2 = Eigen::Vector2d;
using Vector3 = Eigen::Vector3d;
using Vector4 = Eigen::Vector4d;

double wrapToPi(double value);

struct IkResult
{
  Vector4 joints;
  double position_error_m;
  int iterations;
};

class OpenManipulatorKinematics
{
public:
  OpenManipulatorKinematics(
    const Vector4 & joint_low, const Vector4 & joint_high,
    const Vector4 & initial_guess);

  Vector3 forward(const Vector4 & joints) const;
  double bearing(const Vector3 & target) const;
  IkResult solvePosition(
    const Vector3 & target, double pitch_target = 0.0,
    int max_iterations = 80, double tolerance_m = 1.0e-5,
    double max_position_error_m = 0.015) const;

  const Vector4 & jointLow() const noexcept {return joint_low_;}
  const Vector4 & jointHigh() const noexcept {return joint_high_;}

private:
  Eigen::Matrix<double, 4, 1> ikResidual(
    double joint_1, const Eigen::Vector3d & joints_2_to_4,
    const Vector3 & target, double pitch_target) const;

  Vector4 joint_low_;
  Vector4 joint_high_;
  Vector4 initial_guess_;
};

struct ObservationConfig
{
  Vector4 joint_low;
  Vector4 joint_high;
  Vector4 joint_velocity_scale;
  Vector3 workspace_min;
  Vector3 workspace_max;
  Vector2 arm_base_xy;
  Vector3 policy_frame_offset;
  double gripper_close{-0.010};
  double gripper_open{0.019};
  double gripper_velocity_scale{0.25};
};

class ObservationBuilder
{
public:
  explicit ObservationBuilder(ObservationConfig config);

  Observation build(
    const Vector4 & arm_qpos, const Vector4 & arm_qvel,
    double gripper_position, double gripper_velocity,
    const Vector3 & eef_position, const Vector3 & active_target,
    double target_yaw, bool grasped, int phase_index,
    const Vector4 & previous_action) const;

private:
  ObservationConfig config_;
};

class ReferenceController
{
public:
  ReferenceController(
    const OpenManipulatorKinematics & kinematics,
    const Vector4 & stay_joints, const Vector4 & action_scale,
    const std::vector<Vector4> & approach_waypoints,
    double waypoint_tolerance, double pregrasp_height_offset,
    double action_limit, double final_action_limit,
    double target_update_min_m, bool gripper_pitch_hold_enabled,
    double gripper_pitch_target);

  bool setTarget(const Vector3 & target, bool force = false);
  bool advance(const Vector4 & current_joints);
  Vector3 activeTarget(bool return_to_stay = false) const;
  Vector4 action(
    const Vector4 & arm_target, bool return_to_stay = false,
    bool gate_active = false) const;

  bool finalStage() const noexcept {return stage_ >= waypoints_.size();}
  std::size_t stage() const noexcept {return stage_;}
  std::size_t waypointCount() const noexcept {return waypoints_.size();}
  const Vector4 & stayJoints() const noexcept {return stay_joints_;}
  const Vector3 & target() const;
  const Vector3 & pregraspTarget() const;

private:
  const OpenManipulatorKinematics & kinematics_;
  Vector4 stay_joints_;
  Vector4 action_scale_;
  std::vector<Vector4> base_waypoints_;
  std::vector<Vector4> waypoints_;
  std::vector<Vector3> waypoint_targets_;
  std::optional<Vector3> target_;
  std::optional<Vector3> pregrasp_target_;
  std::optional<Vector4> pregrasp_joints_;
  std::size_t stage_{0};
  double waypoint_tolerance_;
  double pregrasp_height_offset_;
  double action_limit_;
  double final_action_limit_;
  double target_update_min_m_;
  bool gripper_pitch_hold_enabled_;
  double gripper_pitch_target_;
};

struct ActionLimitConfig
{
  Vector4 joint_low;
  Vector4 joint_high;
  Vector4 action_scale;
  double filter_coefficient;
  double residual_scale;
  double control_period_s;
  Vector4 max_velocity;
  Vector4 max_acceleration;
  bool gripper_pitch_hold_enabled{false};
  double gripper_pitch_target{0.0};
  double gripper_pitch_joint4_max_step{0.0};
};

class ActionLimiter
{
public:
  explicit ActionLimiter(ActionLimitConfig config);

  void reset(const Vector4 & current_joints);
  Vector4 step(
    const Vector4 & raw_action, const Vector4 & reference_action,
    bool integrate = true, bool couple_gripper_pitch = false);
  Vector4 hold();

  bool initialized() const noexcept {return initialized_;}
  const Vector4 & armTarget() const noexcept {return arm_target_;}

private:
  ActionLimitConfig config_;
  Vector4 arm_target_{Vector4::Zero()};
  Vector4 filtered_action_{Vector4::Zero()};
  Vector4 previous_velocity_{Vector4::Zero()};
  bool initialized_{false};
};

}  // namespace omx_rl_control_cpp

#endif  // OMX_RL_CONTROL_CPP__RUNTIME_MATH_HPP_
