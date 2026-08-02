// Copyright 2026 KweonTJ

#include "omx_rl_control_cpp/runtime_math.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace omx_rl_control_cpp
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

void requireFinite(const Eigen::Ref<const Eigen::VectorXd> & value, const char * name)
{
  if (!value.allFinite()) {
    throw std::invalid_argument(std::string(name) + " contains NaN or Inf");
  }
}

Eigen::Matrix4d translation(const Vector3 & vector)
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 1>(0, 3) = vector;
  return transform;
}

Eigen::Matrix4d rotationY(double angle)
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  transform(0, 0) = cosine;
  transform(0, 2) = sine;
  transform(2, 0) = -sine;
  transform(2, 2) = cosine;
  return transform;
}

Eigen::Matrix4d rotationZ(double angle)
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  transform(0, 0) = cosine;
  transform(0, 1) = -sine;
  transform(1, 0) = sine;
  transform(1, 1) = cosine;
  return transform;
}

template<typename Derived>
Derived clipped(
  const Eigen::MatrixBase<Derived> & value,
  const Eigen::MatrixBase<Derived> & low,
  const Eigen::MatrixBase<Derived> & high)
{
  return value.cwiseMax(low).cwiseMin(high);
}

}  // namespace

double wrapToPi(double value)
{
  double wrapped = std::fmod(value + kPi, 2.0 * kPi);
  if (wrapped < 0.0) {
    wrapped += 2.0 * kPi;
  }
  return wrapped - kPi;
}

OpenManipulatorKinematics::OpenManipulatorKinematics(
  const Vector4 & joint_low, const Vector4 & joint_high,
  const Vector4 & initial_guess)
: joint_low_(joint_low),
  joint_high_(joint_high),
  initial_guess_(clipped(initial_guess, joint_low, joint_high))
{
  requireFinite(joint_low_, "joint_low");
  requireFinite(joint_high_, "joint_high");
  requireFinite(initial_guess, "initial_guess");
  if ((joint_high_ - joint_low_).minCoeff() <= 0.0) {
    throw std::invalid_argument("joint_high must exceed joint_low");
  }
}

Vector3 OpenManipulatorKinematics::forward(const Vector4 & joints) const
{
  requireFinite(joints, "joints");
  const Vector3 joint_1_origin(-0.080, 0.0, 0.195);
  const Vector3 joint_2_origin(0.0, 0.0, 0.0595);
  const Vector3 joint_3_origin(0.024, 0.0, 0.128);
  const Vector3 joint_4_origin(0.124, 0.0, 0.0);
  const Vector3 eef_origin(0.126, 0.0, 0.0);

  Eigen::Matrix4d transform = translation(joint_1_origin) * rotationZ(joints(0));
  transform = transform * translation(joint_2_origin) * rotationY(joints(1));
  transform = transform * translation(joint_3_origin) * rotationY(joints(2));
  transform = transform * translation(joint_4_origin) * rotationY(joints(3));
  transform = transform * translation(eef_origin);
  return transform.block<3, 1>(0, 3);
}

double OpenManipulatorKinematics::bearing(const Vector3 & target) const
{
  requireFinite(target, "target");
  return std::atan2(target(1), target(0) + 0.080);
}

IkResult OpenManipulatorKinematics::solvePosition(
  const Vector3 & target, double pitch_target, int max_iterations,
  double tolerance_m, double max_position_error_m) const
{
  requireFinite(target, "target");
  if (!std::isfinite(pitch_target)) {
    throw std::invalid_argument("pitch_target must be finite");
  }

  Vector4 joints = initial_guess_;
  joints(0) = std::clamp(bearing(target), joint_low_(0), joint_high_(0));
  Eigen::Vector3d variables = joints.tail<3>();
  Eigen::Vector3d best_variables = variables;
  double best_position_error = std::numeric_limits<double>::infinity();
  double damping = 1.0e-4;
  int iteration = 0;

  for (iteration = 1; iteration <= max_iterations; ++iteration) {
    const auto residual = ikResidual(joints(0), variables, target, pitch_target);
    const double position_error = residual.head<3>().norm() / 10.0;
    if (position_error < best_position_error) {
      best_position_error = position_error;
      best_variables = variables;
    }
    if (position_error <= tolerance_m) {
      break;
    }

    Eigen::Matrix<double, 4, 3> jacobian;
    constexpr double epsilon = 1.0e-5;
    for (int index = 0; index < 3; ++index) {
      Eigen::Vector3d shifted = variables;
      shifted(index) += epsilon;
      shifted = shifted.cwiseMax(joint_low_.tail<3>()).cwiseMin(joint_high_.tail<3>());
      double denominator = shifted(index) - variables(index);
      if (std::abs(denominator) < 1.0e-12) {
        shifted(index) = std::max(joint_low_(index + 1), variables(index) - epsilon);
        denominator = shifted(index) - variables(index);
      }
      jacobian.col(index) =
        (ikResidual(joints(0), shifted, target, pitch_target) - residual) / denominator;
    }

    const Eigen::Matrix3d normal =
      jacobian.transpose() * jacobian + damping * Eigen::Matrix3d::Identity();
    const Eigen::Vector3d gradient = jacobian.transpose() * residual;
    const Eigen::Vector3d step = -normal.ldlt().solve(gradient);
    if (!step.allFinite()) {
      throw std::runtime_error("reference IK linear solve failed");
    }

    bool accepted = false;
    const double baseline = residual.squaredNorm();
    for (const double scale : {1.0, 0.5, 0.25, 0.1, 0.05}) {
      Eigen::Vector3d candidate = variables + scale * step;
      candidate = candidate.cwiseMax(joint_low_.tail<3>()).cwiseMin(joint_high_.tail<3>());
      const auto candidate_residual =
        ikResidual(joints(0), candidate, target, pitch_target);
      if (candidate_residual.squaredNorm() < baseline) {
        variables = candidate;
        accepted = true;
        damping = std::max(1.0e-8, damping * 0.5);
        break;
      }
    }
    if (!accepted) {
      damping = std::min(1.0e2, damping * 10.0);
    }
  }

  Vector4 solved;
  solved << joints(0), best_variables;
  if (best_position_error > max_position_error_m) {
    throw std::runtime_error(
            "reference IK target is outside the safe workspace; error=" +
            std::to_string(best_position_error));
  }
  return {solved, best_position_error, std::min(iteration, max_iterations)};
}

Eigen::Matrix<double, 4, 1> OpenManipulatorKinematics::ikResidual(
  double joint_1, const Eigen::Vector3d & joints_2_to_4,
  const Vector3 & target, double pitch_target) const
{
  Vector4 joints;
  joints << joint_1, joints_2_to_4;
  Eigen::Matrix<double, 4, 1> residual;
  residual.head<3>() = 10.0 * (forward(joints) - target);
  residual(3) = joints_2_to_4.sum() - pitch_target;
  return residual;
}

ObservationBuilder::ObservationBuilder(ObservationConfig config)
: config_(std::move(config))
{
  if ((config_.joint_high - config_.joint_low).minCoeff() <= 0.0) {
    throw std::invalid_argument("joint_high must exceed joint_low");
  }
  if ((config_.workspace_max - config_.workspace_min).minCoeff() <= 0.0) {
    throw std::invalid_argument("workspace_max must exceed workspace_min");
  }
  if (config_.gripper_open <= config_.gripper_close ||
    config_.gripper_velocity_scale <= 0.0)
  {
    throw std::invalid_argument("invalid gripper observation normalization");
  }
}

Observation ObservationBuilder::build(
  const Vector4 & arm_qpos, const Vector4 & arm_qvel,
  double gripper_position, double gripper_velocity,
  const Vector3 & eef_position, const Vector3 & active_target,
  double target_yaw, bool grasped, int phase_index,
  const Vector4 & previous_action) const
{
  if (phase_index < 0 || phase_index > 3) {
    throw std::invalid_argument("phase_index must be in [0, 3]");
  }
  if (!arm_qpos.allFinite() || !arm_qvel.allFinite() ||
    !eef_position.allFinite() || !active_target.allFinite() ||
    !previous_action.allFinite() || !std::isfinite(gripper_position) ||
    !std::isfinite(gripper_velocity) || !std::isfinite(target_yaw))
  {
    throw std::invalid_argument("observation input contains NaN or Inf");
  }

  const Vector4 qpos_normalized =
    2.0 * (arm_qpos - config_.joint_low).cwiseQuotient(
    config_.joint_high - config_.joint_low) - Vector4::Ones();
  const Vector4 qvel_normalized =
    arm_qvel.cwiseQuotient(config_.joint_velocity_scale);
  const double gripper_normalized =
    2.0 * (gripper_position - config_.gripper_close) /
    (config_.gripper_open - config_.gripper_close) - 1.0;
  const double gripper_velocity_normalized =
    gripper_velocity / config_.gripper_velocity_scale;
  const Vector3 policy_eef = eef_position + config_.policy_frame_offset;
  const Vector3 policy_target = active_target + config_.policy_frame_offset;
  const Vector3 workspace_span = config_.workspace_max - config_.workspace_min;
  const Vector3 eef_normalized =
    2.0 * (policy_eef - config_.workspace_min).cwiseQuotient(workspace_span) -
    Vector3::Ones();
  const Vector3 target_normalized =
    2.0 * (policy_target - config_.workspace_min).cwiseQuotient(workspace_span) -
    Vector3::Ones();
  const Vector3 relative_normalized =
    2.0 * (policy_target - policy_eef).cwiseQuotient(workspace_span);
  const Vector2 bearing_delta = active_target.head<2>() - config_.arm_base_xy;
  const double target_bearing = std::atan2(bearing_delta(1), bearing_delta(0));
  const double roll_error = wrapToPi(arm_qpos.tail<3>().sum()) / kPi;

  Observation observation;
  int offset = 0;
  observation.segment<4>(offset) = qpos_normalized;
  offset += 4;
  observation.segment<4>(offset) = qvel_normalized;
  offset += 4;
  observation(offset++) = gripper_normalized;
  observation(offset++) = gripper_velocity_normalized;
  observation.segment<3>(offset) = eef_normalized;
  offset += 3;
  observation.segment<3>(offset) = target_normalized;
  offset += 3;
  observation.segment<3>(offset) = relative_normalized;
  offset += 3;
  observation(offset++) = std::sin(target_bearing);
  observation(offset++) = std::cos(target_bearing);
  observation(offset++) = std::sin(target_yaw);
  observation(offset++) = std::cos(target_yaw);
  observation(offset++) = roll_error;
  observation(offset++) = grasped ? 1.0 : 0.0;
  observation.segment<4>(offset).setZero();
  observation(offset + phase_index) = 1.0;
  offset += 4;
  observation.segment<4>(offset) = previous_action;
  offset += 4;
  if (offset != kObservationSize || !observation.allFinite()) {
    throw std::runtime_error("internal observation construction failed");
  }
  return observation.cwiseMax(-1.0).cwiseMin(1.0);
}

ReferenceController::ReferenceController(
  const OpenManipulatorKinematics & kinematics,
  const Vector4 & stay_joints, const Vector4 & action_scale,
  const std::vector<Vector4> & approach_waypoints,
  double waypoint_tolerance, double pregrasp_height_offset,
  double action_limit, double final_action_limit,
  double target_update_min_m, bool gripper_pitch_hold_enabled,
  double gripper_pitch_target)
: kinematics_(kinematics),
  stay_joints_(stay_joints),
  action_scale_(action_scale),
  base_waypoints_(approach_waypoints),
  waypoint_tolerance_(waypoint_tolerance),
  pregrasp_height_offset_(pregrasp_height_offset),
  action_limit_(action_limit),
  final_action_limit_(final_action_limit),
  target_update_min_m_(target_update_min_m),
  gripper_pitch_hold_enabled_(gripper_pitch_hold_enabled),
  gripper_pitch_target_(gripper_pitch_target)
{
  if (action_scale_.minCoeff() <= 0.0 || waypoint_tolerance_ <= 0.0) {
    throw std::invalid_argument("invalid reference controller limits");
  }
}

bool ReferenceController::setTarget(const Vector3 & target, bool force)
{
  requireFinite(target, "target");
  if (!force && target_ && (target - *target_).norm() < target_update_min_m_) {
    return false;
  }

  const bool initialize = !target_.has_value() || force;
  const double target_bearing = kinematics_.bearing(target);
  if (initialize) {
    waypoints_ = base_waypoints_;
    waypoint_targets_.clear();
    for (auto & waypoint : waypoints_) {
      waypoint(0) = target_bearing;
      if (gripper_pitch_hold_enabled_) {
        waypoint(3) = gripper_pitch_target_ - waypoint(1) - waypoint(2);
        if (waypoint(3) < kinematics_.jointLow()(3) ||
          waypoint(3) > kinematics_.jointHigh()(3))
        {
          throw std::runtime_error("approach pitch target exceeds joint4 limits");
        }
      }
      waypoint_targets_.push_back(kinematics_.forward(waypoint));
    }
    stage_ = 0;
  }

  target_ = target;
  pregrasp_target_ = target + Vector3(0.0, 0.0, pregrasp_height_offset_);
  if (initialize) {
    pregrasp_joints_ =
      kinematics_.solvePosition(*pregrasp_target_, gripper_pitch_target_).joints;
  }
  return true;
}

bool ReferenceController::advance(const Vector4 & current_joints)
{
  if (finalStage()) {
    return false;
  }
  if ((current_joints - waypoints_.at(stage_)).norm() > waypoint_tolerance_) {
    return false;
  }
  ++stage_;
  if (finalStage() && pregrasp_target_) {
    pregrasp_joints_ =
      kinematics_.solvePosition(*pregrasp_target_, gripper_pitch_target_).joints;
  }
  return true;
}

Vector3 ReferenceController::activeTarget(bool return_to_stay) const
{
  if (return_to_stay) {
    return kinematics_.forward(stay_joints_);
  }
  if (!pregrasp_target_) {
    throw std::runtime_error("reference target is not configured");
  }
  if (!finalStage()) {
    return waypoint_targets_.at(stage_);
  }
  return *pregrasp_target_;
}

Vector4 ReferenceController::action(
  const Vector4 & arm_target, bool return_to_stay, bool gate_active) const
{
  if (gate_active) {
    return Vector4::Zero();
  }
  Vector4 goal;
  double limit = action_limit_;
  if (return_to_stay) {
    goal = stay_joints_;
  } else if (!finalStage()) {
    goal = waypoints_.at(stage_);
  } else {
    if (!pregrasp_joints_) {
      throw std::runtime_error("reference target is not configured");
    }
    goal = *pregrasp_joints_;
    limit = final_action_limit_;
  }
  return ((goal - arm_target).cwiseQuotient(action_scale_)).cwiseMax(-limit).cwiseMin(limit);
}

const Vector3 & ReferenceController::target() const
{
  if (!target_) {
    throw std::runtime_error("reference target is not configured");
  }
  return *target_;
}

const Vector3 & ReferenceController::pregraspTarget() const
{
  if (!pregrasp_target_) {
    throw std::runtime_error("reference target is not configured");
  }
  return *pregrasp_target_;
}

ActionLimiter::ActionLimiter(ActionLimitConfig config)
: config_(std::move(config))
{
  if ((config_.joint_high - config_.joint_low).minCoeff() <= 0.0 ||
    config_.action_scale.minCoeff() <= 0.0 ||
    config_.max_velocity.minCoeff() <= 0.0 ||
    config_.max_acceleration.minCoeff() <= 0.0 ||
    config_.filter_coefficient <= 0.0 || config_.filter_coefficient > 1.0 ||
    config_.residual_scale < 0.0 || config_.residual_scale > 1.0 ||
    config_.control_period_s <= 0.0)
  {
    throw std::invalid_argument("invalid action limiter configuration");
  }
  if (config_.gripper_pitch_hold_enabled &&
    config_.gripper_pitch_joint4_max_step <= 0.0)
  {
    throw std::invalid_argument("pitch coupling requires a positive joint4 max step");
  }
}

void ActionLimiter::reset(const Vector4 & current_joints)
{
  arm_target_ = current_joints.cwiseMax(config_.joint_low).cwiseMin(config_.joint_high);
  filtered_action_.setZero();
  previous_velocity_.setZero();
  initialized_ = true;
}

Vector4 ActionLimiter::step(
  const Vector4 & raw_action, const Vector4 & reference_action,
  bool integrate, bool couple_gripper_pitch)
{
  if (!initialized_) {
    throw std::runtime_error("ActionLimiter::reset must be called first");
  }
  Vector4 control = reference_action + config_.residual_scale * raw_action;
  control = control.cwiseMax(-1.0).cwiseMin(1.0);
  filtered_action_ += config_.filter_coefficient * (control - filtered_action_);
  if (!integrate) {
    return arm_target_;
  }

  Vector4 velocity =
    (filtered_action_.cwiseProduct(config_.action_scale) /
    config_.control_period_s).cwiseMax(-config_.max_velocity).cwiseMin(config_.max_velocity);
  const Vector4 maximum_change = config_.max_acceleration * config_.control_period_s;
  velocity = velocity.cwiseMax(previous_velocity_ - maximum_change).cwiseMin(
    previous_velocity_ + maximum_change);

  if (couple_gripper_pitch) {
    if (!config_.gripper_pitch_hold_enabled) {
      throw std::runtime_error("gripper pitch coupling is not configured");
    }
    Eigen::Vector2d next_2_3 =
      arm_target_.segment<2>(1) + velocity.segment<2>(1) * config_.control_period_s;
    next_2_3 = next_2_3.cwiseMax(config_.joint_low.segment<2>(1)).cwiseMin(
      config_.joint_high.segment<2>(1));
    const double desired_joint_4 = std::clamp(
      config_.gripper_pitch_target - next_2_3.sum(),
      config_.joint_low(3), config_.joint_high(3));
    const double joint_4_delta = std::clamp(
      desired_joint_4 - arm_target_(3),
      -config_.gripper_pitch_joint4_max_step,
      config_.gripper_pitch_joint4_max_step);
    const double joint_4_velocity = std::clamp(
      joint_4_delta / config_.control_period_s,
      -config_.max_velocity(3), config_.max_velocity(3));
    velocity(3) = std::clamp(
      joint_4_velocity,
      previous_velocity_(3) - maximum_change(3),
      previous_velocity_(3) + maximum_change(3));
  }

  arm_target_ += velocity * config_.control_period_s;
  arm_target_ = arm_target_.cwiseMax(config_.joint_low).cwiseMin(config_.joint_high);
  previous_velocity_ = velocity;
  return arm_target_;
}

Vector4 ActionLimiter::hold()
{
  if (!initialized_) {
    throw std::runtime_error("ActionLimiter::reset must be called first");
  }
  previous_velocity_.setZero();
  return arm_target_;
}

}  // namespace omx_rl_control_cpp
