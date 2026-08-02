// Copyright 2026 KweonTJ

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <action_msgs/msg/goal_status.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "omx_rl_control_cpp/policy_actor.hpp"
#include "omx_rl_control_cpp/runtime_math.hpp"

namespace omx_rl_control_cpp
{
namespace
{

using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;

double ageSeconds(const TimePoint & stamp, const TimePoint & now)
{
  if (stamp == TimePoint{}) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(now - stamp).count();
}

rclcpp::NodeOptions runtimeNodeOptions()
{
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  return options;
}

std::string boolText(bool value)
{
  return value ? "true" : "false";
}

}  // namespace

class RlControlCppNode : public rclcpp::Node
{
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GripperGoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;

  RlControlCppNode()
  : Node("rl_control_node", runtimeNodeOptions())
  {
    loadConfiguration();
    buildRuntime();
    createRosInterfaces();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(control_period_s_),
      std::bind(&RlControlCppNode::onControlTimer, this));
    RCLCPP_INFO(
      get_logger(),
      "C++ RL control initialized; actor_ready=%s backend=%s device=%s "
      "period=%.3fs mode=%s residual=%.3f",
      boolText(actor_.ready()).c_str(), actor_.backend().c_str(), actor_.device().c_str(),
      control_period_s_, control_mode_.c_str(), residual_scale_);
  }

private:
  enum class RuntimeState
  {
    NOT_READY,
    ALIGN_STAY,
    STAY_EMPTY,
    WAIT_PICK,
    OPEN_GRIPPER,
    PICK_REACH,
    CLOSE_GRIPPER,
    VERIFY_GRASP,
    PICK_TO_STAY,
    WAIT_DELIVERY,
    PLACE_REACH,
    OPEN_RELEASE,
    PLACE_TO_STAY,
    COMPLETE,
    HOLD,
    FAULT,
    E_STOP,
  };

  enum class GripperState {IDLE, SENDING, ACTIVE, SUCCEEDED, FAILED};

  template<typename T>
  T parameterOr(const std::string & name, const T & default_value)
  {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, default_value);
    }
    return get_parameter(name).get_value<T>();
  }

  Vector4 vector4Parameter(
    const std::string & name, const std::vector<double> & default_value)
  {
    const auto values = parameterOr<std::vector<double>>(name, default_value);
    if (values.size() != 4) {
      throw std::invalid_argument(name + " must contain four values");
    }
    Vector4 result;
    for (int index = 0; index < 4; ++index) {
      result(index) = values.at(index);
    }
    if (!result.allFinite()) {
      throw std::invalid_argument(name + " contains NaN or Inf");
    }
    return result;
  }

  Vector3 vector3Parameter(
    const std::string & name, const std::vector<double> & default_value)
  {
    const auto values = parameterOr<std::vector<double>>(name, default_value);
    if (values.size() != 3) {
      throw std::invalid_argument(name + " must contain three values");
    }
    Vector3 result(values.at(0), values.at(1), values.at(2));
    if (!result.allFinite()) {
      throw std::invalid_argument(name + " contains NaN or Inf");
    }
    return result;
  }

  Vector2 vector2Parameter(
    const std::string & name, const std::vector<double> & default_value)
  {
    const auto values = parameterOr<std::vector<double>>(name, default_value);
    if (values.size() != 2) {
      throw std::invalid_argument(name + " must contain two values");
    }
    Vector2 result(values.at(0), values.at(1));
    if (!result.allFinite()) {
      throw std::invalid_argument(name + " contains NaN or Inf");
    }
    return result;
  }

  void loadConfiguration()
  {
    joint_names_ = parameterOr<std::vector<std::string>>(
      "joint_names", {"joint1", "joint2", "joint3", "joint4"});
    if (joint_names_.size() != 4) {
      throw std::invalid_argument("joint_names must contain four names");
    }
    gripper_joint_name_ = parameterOr<std::string>(
      "gripper_joint_name", "gripper_left_joint");
    target_frame_id_ = parameterOr<std::string>("target_frame_id", "base_link");
    model_version_ = parameterOr<std::string>(
      "cpp_model_version", "arm_delivery_pitch_hold_v4");
    policy_backend_ = parameterOr<std::string>("cpp_policy_backend", "auto");
    policy_device_ = parameterOr<std::string>("cpp_policy_device", "auto");

    const double control_rate_hz = parameterOr<double>("control_rate_hz", 50.0);
    if (control_rate_hz <= 0.0) {
      throw std::invalid_argument("control_rate_hz must be positive");
    }
    control_period_s_ = 1.0 / control_rate_hz;
    trajectory_time_s_ = parameterOr<double>("trajectory_time_s", 0.08);
    inference_deadline_s_ = parameterOr<double>("inference_deadline_s", 0.018);
    inference_deadline_miss_limit_ = parameterOr<int64_t>(
      "inference_deadline_miss_limit", 3);
    inference_timing_window_ = parameterOr<int64_t>("inference_timing_window", 500);
    status_period_s_ = parameterOr<double>("status_period_s", 0.5);
    release_settle_time_s_ = parameterOr<double>("release_settle_time_s", 0.24);

    joint_low_ = vector4Parameter(
      "joint_low", {-2.82743, -1.79071, -0.942478, -1.79071});
    joint_high_ = vector4Parameter(
      "joint_high", {2.82743, 1.57080, 1.38230, 2.04204});
    joint_velocity_scale_ = vector4Parameter(
      "joint_velocity_scale", {2.0, 2.0, 2.0, 2.0});
    max_joint_velocity_ = vector4Parameter(
      "max_joint_velocity", {0.70, 0.70, 0.70, 0.70});
    max_joint_acceleration_ = vector4Parameter(
      "max_joint_acceleration", {8.0, 8.0, 8.0, 8.0});
    workspace_min_ = vector3Parameter("workspace_min", {0.10, -0.18, 0.08});
    workspace_max_ = vector3Parameter("workspace_max", {0.42, 0.18, 0.52});
    arm_base_xy_ = vector2Parameter("arm_base_xy", {-0.08, 0.0});
    policy_frame_offset_ = vector3Parameter("policy_frame_offset", {0.0, 0.0, 0.016});
    action_scale_ = vector4Parameter("action_scale", {0.014, 0.014, 0.014, 0.014});
    action_filter_coefficient_ = parameterOr<double>("action_filter_coefficient", 0.18);
    residual_scale_ = parameterOr<double>("residual_action_scale", 0.10);
    const double residual_override = parameterOr<double>(
      "residual_action_scale_override", -1.0);
    if (residual_override >= 0.0 && residual_override <= 1.0) {
      residual_scale_ = residual_override;
    } else if (std::abs(residual_override + 1.0) > 1.0e-9) {
      throw std::invalid_argument(
              "residual_action_scale_override must be -1 or in [0, 1]");
    }
    control_mode_ = residual_scale_ <= 1.0e-12 ?
      "reference_only" : "reference_plus_residual_with_joint4_pitch_coupling";

    stay_joint_positions_ = vector4Parameter(
      "stay_joint_positions", {0.104311, 0.027612, -0.001534, -1.638291});
    ik_initial_guess_ = vector4Parameter(
      "ik_initial_guess", {0.0, 1.15968, -0.48813, -0.67155});
    approach_waypoint_1_ = vector4Parameter(
      "approach_waypoint_1", {0.0, -0.5, 0.5, 0.0});
    approach_waypoint_2_ = vector4Parameter(
      "approach_waypoint_2", {0.0, 0.5, 0.2, -0.7});
    approach_waypoint_tolerance_ = parameterOr<double>("approach_waypoint_tolerance", 0.08);
    pregrasp_height_offset_ = parameterOr<double>("pregrasp_height_offset", 0.0225);
    reference_action_limit_ = parameterOr<double>("reference_action_limit", 1.0);
    final_action_limit_ = parameterOr<double>("final_approach_action_limit", 1.0);
    target_update_min_m_ = parameterOr<double>("target_update_min_m", 0.001);

    initialize_to_policy_stay_ = parameterOr<bool>("initialize_to_policy_stay", false);
    initialization_timeout_s_ = parameterOr<double>("initialization_timeout_s", 8.0);
    pick_start_min_marker_z_m_ = parameterOr<double>("pick_start_min_marker_z_m", 0.10);
    pick_start_max_marker_z_m_ = parameterOr<double>("pick_start_max_marker_z_m", 0.265);
    aruco_pose_timeout_s_ = parameterOr<double>("aruco_pose_timeout_s", 0.50);
    auto_pick_when_in_range_ = parameterOr<bool>("auto_pick_when_in_range", true);

    gripper_open_ = parameterOr<double>("gripper_open", 0.019);
    gripper_close_ = parameterOr<double>("gripper_close", -0.010);
    gripper_grasp_position_ = parameterOr<double>("gripper_grasp_position", -0.010);
    gripper_velocity_scale_ = parameterOr<double>("gripper_velocity_scale", 0.25);
    gripper_max_effort_ = parameterOr<double>("gripper_max_effort", 0.0);
    gripper_timeout_s_ = parameterOr<double>("gripper_timeout_s", 4.0);
    require_gripper_server_ = parameterOr<bool>("require_gripper_server", true);
    require_external_grasp_confirmation_ = parameterOr<bool>(
      "require_external_grasp_confirmation", false);
    grasp_confirmation_timeout_s_ = parameterOr<double>(
      "grasp_confirmation_timeout_s", 2.0);

    joint_state_timeout_s_ = parameterOr<double>("joint_state_timeout_s", 0.25);
    target_pose_timeout_s_ = parameterOr<double>("target_pose_timeout_s", 0.50);
    delivery_pose_required_ = parameterOr<bool>("delivery_pose_required", false);
    delivery_target_ = vector3Parameter(
      "fallback_delivery_position", {0.27, 0.0, 0.1815});
    delivery_yaw_ = parameterOr<double>("fallback_delivery_yaw", 0.0);
    require_base_arrived_ = parameterOr<bool>("require_base_arrived", false);
    require_odom_stopped_ = parameterOr<bool>("require_odom_stopped", false);
    base_arrival_timeout_s_ = parameterOr<double>("base_arrival_timeout_s", 2.0);
    odom_timeout_s_ = parameterOr<double>("odom_timeout_s", 0.50);
    stopped_linear_speed_mps_ = parameterOr<double>("stopped_linear_speed_mps", 0.01);
    stopped_angular_speed_radps_ = parameterOr<double>("stopped_angular_speed_radps", 0.05);

    close_distance_m_ = parameterOr<double>("close_distance_m", 0.042);
    close_xy_distance_m_ = parameterOr<double>("close_xy_distance_m", 0.035);
    close_z_tolerance_m_ = parameterOr<double>("close_z_tolerance_m", 0.030);
    close_bearing_tolerance_rad_ = parameterOr<double>(
      "close_bearing_tolerance_rad", 0.35);
    close_roll_tolerance_rad_ = parameterOr<double>("close_roll_tolerance_rad", 0.35);
    gate_stable_count_required_ = parameterOr<int64_t>("gate_stable_count", 4);
    stay_tolerance_rad_ = parameterOr<double>("stay_tolerance_rad", 0.050);
    stay_stable_count_required_ = parameterOr<int64_t>("stay_stable_count", 8);
    publish_every_n_cycles_ = parameterOr<int64_t>("publish_every_n_cycles", 1);

    gripper_pitch_hold_enabled_ = parameterOr<bool>("gripper_pitch_hold_enabled", true);
    gripper_pitch_target_ = parameterOr<double>("gripper_pitch_target", 0.0);
    gripper_pitch_joint4_max_step_ = parameterOr<double>(
      "gripper_pitch_joint4_max_step", 0.028);

    image_target_topic_ = parameterOr<std::string>("image_target_topic", "/target/object_pose");
    aruco_pose_topic_ = parameterOr<std::string>("aruco_pose_topic", "/target/aruco_pose");
    target_valid_topic_ = parameterOr<std::string>("target_valid_topic", "/target/valid");
    delivery_pose_topic_ = parameterOr<std::string>(
      "delivery_pose_topic", "/target/delivery_pose");
    joint_states_topic_ = parameterOr<std::string>("joint_states_topic", "/joint_states");
    odom_topic_ = parameterOr<std::string>("odom_topic", "/odom");
    base_arrived_topic_ = parameterOr<std::string>(
      "base_arrived_topic", "/turtlebot3_control/base_arrived");
    safety_stop_topic_ = parameterOr<std::string>("safety_stop_topic", "/safety_stop");
    grasp_confirmed_topic_ = parameterOr<std::string>(
      "grasp_confirmed_topic", "/rl_control/grasp_confirmed");
    command_topic_ = parameterOr<std::string>("command_topic", "/rl_control/command");
    compat_start_topic_ = parameterOr<std::string>("compat_start_topic", "/mp_control/start");
    arm_trajectory_topic_ = parameterOr<std::string>(
      "arm_trajectory_topic", "/arm_controller/joint_trajectory");
    gripper_action_name_ = parameterOr<std::string>(
      "gripper_action_name", "/gripper_controller/gripper_cmd");
    base_hold_topic_ = parameterOr<std::string>("base_hold_topic", "/target/base_hold");
    status_topic_ = parameterOr<std::string>("status_topic", "/rl_control/status");
    compat_status_topic_ = parameterOr<std::string>("compat_status_topic", "/mp_control/status");

    if (policy_backend_ != "auto" && policy_backend_ != "eigen" &&
      policy_backend_ != "torchscript")
    {
      throw std::invalid_argument("cpp_policy_backend must be auto, eigen, or torchscript");
    }
    if (policy_device_ != "auto" && policy_device_ != "cpu" && policy_device_ != "cuda") {
      throw std::invalid_argument("cpp_policy_device must be auto, cpu, or cuda");
    }
    if (inference_deadline_miss_limit_ <= 0 || inference_timing_window_ <= 0 ||
      gate_stable_count_required_ <= 0 || stay_stable_count_required_ <= 0)
    {
      throw std::invalid_argument(
              "stable counts, timing window, and deadline miss limit must be positive");
    }
  }

  void buildRuntime()
  {
    kinematics_ = std::make_unique<OpenManipulatorKinematics>(
      joint_low_, joint_high_, ik_initial_guess_);
    observation_builder_ = std::make_unique<ObservationBuilder>(
      ObservationConfig{
        joint_low_, joint_high_, joint_velocity_scale_, workspace_min_, workspace_max_,
        arm_base_xy_, policy_frame_offset_, gripper_close_, gripper_open_,
        gripper_velocity_scale_});
    reference_ = std::make_unique<ReferenceController>(
      *kinematics_, stay_joint_positions_, action_scale_,
      std::vector<Vector4>{approach_waypoint_1_, approach_waypoint_2_},
      approach_waypoint_tolerance_, pregrasp_height_offset_,
      reference_action_limit_, final_action_limit_, target_update_min_m_,
      gripper_pitch_hold_enabled_, gripper_pitch_target_);
    limiter_ = std::make_unique<ActionLimiter>(
      ActionLimitConfig{
        joint_low_, joint_high_, action_scale_, action_filter_coefficient_,
        residual_scale_, control_period_s_, max_joint_velocity_, max_joint_acceleration_,
        gripper_pitch_hold_enabled_, gripper_pitch_target_,
        gripper_pitch_joint4_max_step_});

    const std::string package_share =
      ament_index_cpp::get_package_share_directory("omx_rl_control_cpp");
    std::string actor_path = parameterOr<std::string>("actor_model_path", "");
    if (actor_path.empty()) {
      actor_path = package_share + "/models/arm_delivery_pitch_hold_v4/policy_actor.bin";
    }
    std::string torchscript_path = parameterOr<std::string>("torchscript_model_path", "");
    if (torchscript_path.empty()) {
      torchscript_path = package_share + "/models/arm_delivery_pitch_hold_v4/policy_actor.pt";
    }

    bool loaded = false;
    if (policy_backend_ != "eigen" && PolicyActor::torchRuntimeCompiled()) {
      loaded = actor_.loadTorchScript(torchscript_path, policy_device_, &startup_error_);
      if (!loaded && policy_backend_ == "auto") {
        RCLCPP_WARN(
          get_logger(), "TorchScript actor unavailable (%s); falling back to Eigen CPU",
          startup_error_.c_str());
      }
    } else if (policy_backend_ == "torchscript") {
      startup_error_ =
        "TorchScript backend requested but this build does not contain LibTorch";
    }
    if (!loaded && policy_backend_ != "torchscript") {
      loaded = actor_.load(actor_path, &startup_error_);
    }
    if (!loaded) {
      RCLCPP_ERROR(get_logger(), "C++ actor validation failed: %s", startup_error_.c_str());
    } else {
      startup_error_.clear();
    }
    inference_window_ms_.reserve(static_cast<std::size_t>(inference_timing_window_));
    gripper_position_ = gripper_open_;
  }

  void createRosInterfaces()
  {
    arm_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      arm_trajectory_topic_, 10);
    base_hold_publisher_ = create_publisher<std_msgs::msg::Bool>(base_hold_topic_, 10);
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    compat_status_publisher_ = create_publisher<std_msgs::msg::String>(
      compat_status_topic_, 10);
    gripper_client_ = rclcpp_action::create_client<GripperCommand>(this, gripper_action_name_);

    object_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      image_target_topic_, 10,
      std::bind(&RlControlCppNode::onObjectPose, this, std::placeholders::_1));
    aruco_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      aruco_pose_topic_, 10,
      std::bind(&RlControlCppNode::onArucoPose, this, std::placeholders::_1));
    target_valid_subscription_ = create_subscription<std_msgs::msg::Bool>(
      target_valid_topic_, 10,
      std::bind(&RlControlCppNode::onTargetValid, this, std::placeholders::_1));
    delivery_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      delivery_pose_topic_, 10,
      std::bind(&RlControlCppNode::onDeliveryPose, this, std::placeholders::_1));
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, 10,
      std::bind(&RlControlCppNode::onJointStates, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&RlControlCppNode::onOdom, this, std::placeholders::_1));
    base_arrived_subscription_ = create_subscription<std_msgs::msg::Bool>(
      base_arrived_topic_, 10,
      std::bind(&RlControlCppNode::onBaseArrived, this, std::placeholders::_1));
    safety_stop_subscription_ = create_subscription<std_msgs::msg::Bool>(
      safety_stop_topic_, 10,
      std::bind(&RlControlCppNode::onSafetyStop, this, std::placeholders::_1));
    grasp_confirmed_subscription_ = create_subscription<std_msgs::msg::Bool>(
      grasp_confirmed_topic_, 10,
      std::bind(&RlControlCppNode::onGraspConfirmed, this, std::placeholders::_1));
    command_subscription_ = create_subscription<std_msgs::msg::String>(
      command_topic_, 10,
      std::bind(&RlControlCppNode::onCommand, this, std::placeholders::_1));
    compat_start_subscription_ = create_subscription<std_msgs::msg::Bool>(
      compat_start_topic_, 10,
      std::bind(&RlControlCppNode::onCompatStart, this, std::placeholders::_1));
  }

  void onJointStates(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    std::unordered_map<std::string, std::size_t> indices;
    for (std::size_t index = 0; index < message->name.size(); ++index) {
      indices[message->name[index]] = index;
    }
    Vector4 positions;
    Vector4 velocities = Vector4::Zero();
    for (int joint = 0; joint < 4; ++joint) {
      const auto found = indices.find(joint_names_.at(joint));
      if (found == indices.end() || found->second >= message->position.size()) {
        return;
      }
      positions(joint) = message->position.at(found->second);
      if (found->second < message->velocity.size()) {
        velocities(joint) = message->velocity.at(found->second);
      }
    }
    if (!positions.allFinite() || !velocities.allFinite()) {
      return;
    }
    arm_qpos_ = positions;
    arm_qvel_ = velocities;
    const auto gripper = indices.find(gripper_joint_name_);
    if (gripper != indices.end() && gripper->second < message->position.size()) {
      gripper_position_ = message->position.at(gripper->second);
      gripper_velocity_ = gripper->second < message->velocity.size() ?
        message->velocity.at(gripper->second) : 0.0;
    }
    joint_state_time_ = SteadyClock::now();
  }

  bool poseFrameValid(const geometry_msgs::msg::PoseStamped & message)
  {
    if (message.header.frame_id == target_frame_id_) {
      return true;
    }
    if (message.header.frame_id != last_rejected_pose_frame_) {
      last_rejected_pose_frame_ = message.header.frame_id;
      RCLCPP_WARN(
        get_logger(), "Rejecting target pose frame '%s'; expected '%s'",
        message.header.frame_id.c_str(), target_frame_id_.c_str());
    }
    return false;
  }

  static double yawFromPose(const geometry_msgs::msg::PoseStamped & message)
  {
    const auto & q = message.pose.orientation;
    const double sin_yaw = 2.0 * (q.w * q.z + q.x * q.y);
    const double cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(sin_yaw, cos_yaw);
  }

  void onObjectPose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (!poseFrameValid(*message)) {
      return;
    }
    const Vector3 position(
      message->pose.position.x, message->pose.position.y, message->pose.position.z);
    const double yaw = yawFromPose(*message);
    if (!position.allFinite() || !std::isfinite(yaw)) {
      return;
    }
    object_target_ = position;
    object_yaw_ = yaw;
    object_pose_time_ = SteadyClock::now();
  }

  void onArucoPose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    const double marker_z = message->pose.position.z;
    if (std::isfinite(marker_z) && marker_z > 0.0) {
      aruco_marker_z_ = marker_z;
      aruco_pose_time_ = SteadyClock::now();
    }
  }

  void onTargetValid(const std_msgs::msg::Bool::SharedPtr message)
  {
    target_valid_ = message->data;
    if (target_valid_) {
      target_valid_time_ = SteadyClock::now();
    }
  }

  void onDeliveryPose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (!poseFrameValid(*message)) {
      return;
    }
    const Vector3 position(
      message->pose.position.x, message->pose.position.y, message->pose.position.z);
    const double yaw = yawFromPose(*message);
    if (!position.allFinite() || !std::isfinite(yaw)) {
      return;
    }
    delivery_target_ = position;
    delivery_yaw_ = yaw;
    delivery_pose_time_ = SteadyClock::now();
  }

  void onBaseArrived(const std_msgs::msg::Bool::SharedPtr message)
  {
    base_arrived_ = message->data;
    base_arrived_time_ = SteadyClock::now();
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const auto & twist = message->twist.twist;
    base_linear_speed_ = std::hypot(twist.linear.x, twist.linear.y);
    base_angular_speed_ = std::abs(twist.angular.z);
    odom_time_ = SteadyClock::now();
  }

  void onSafetyStop(const std_msgs::msg::Bool::SharedPtr message)
  {
    safety_stop_ = message->data;
    if (safety_stop_) {
      enterEstop("safety_stop=true");
    }
  }

  void onGraspConfirmed(const std_msgs::msg::Bool::SharedPtr message)
  {
    external_grasp_confirmed_ = message->data;
  }

  void onCommand(const std_msgs::msg::String::SharedPtr message)
  {
    std::string command = message->data;
    std::transform(
      command.begin(), command.end(), command.begin(),
      [](unsigned char character) {return static_cast<char>(std::toupper(character));});
    if (command == "E_STOP") {
      safety_stop_ = true;
      enterEstop("E_STOP command");
    } else if (command == "HOLD") {
      enterHold("HOLD command");
    } else if (command == "RESET") {
      resetRuntime();
    } else if (command == "PICK") {
      if (state_ == RuntimeState::NOT_READY || state_ == RuntimeState::STAY_EMPTY) {
        pending_command_ = command;
      } else {
        RCLCPP_WARN(
          get_logger(), "Ignoring PICK while state=%s", stateName(state_).c_str());
      }
    } else if (command == "PLACE") {
      if (state_ != RuntimeState::WAIT_DELIVERY || !grasped_) {
        RCLCPP_WARN(
          get_logger(),
          "Ignoring PLACE before a confirmed PICK has returned to WAIT_DELIVERY");
      } else {
        pending_command_ = command;
      }
    } else {
      RCLCPP_WARN(get_logger(), "Ignoring unknown command: %s", command.c_str());
    }
  }

  void onCompatStart(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (message->data &&
      (state_ == RuntimeState::NOT_READY || state_ == RuntimeState::STAY_EMPTY))
    {
      pending_command_ = "PICK";
    }
  }

  void onControlTimer()
  {
    ++cycle_count_;
    const auto now = SteadyClock::now();
    publishBaseHold();
    try {
      if (safety_stop_) {
        enterEstop("safety_stop=true");
      } else if (baseInterlockState() && !baseStopped(now)) {
        enterHold("base stop gate lost during arm operation");
      } else {
        switch (state_) {
          case RuntimeState::NOT_READY: handleNotReady(now); break;
          case RuntimeState::ALIGN_STAY: handleAlignStay(now); break;
          case RuntimeState::STAY_EMPTY: handleStayEmpty(now); break;
          case RuntimeState::WAIT_PICK: handleWaitPick(now); break;
          case RuntimeState::OPEN_GRIPPER: handleOpenGripper(now); break;
          case RuntimeState::PICK_REACH: handlePolicyMotion(now); break;
          case RuntimeState::CLOSE_GRIPPER: handleCloseGripper(now); break;
          case RuntimeState::VERIFY_GRASP: handleVerifyGrasp(now); break;
          case RuntimeState::PICK_TO_STAY: handlePolicyMotion(now); break;
          case RuntimeState::WAIT_DELIVERY: handleWaitDelivery(now); break;
          case RuntimeState::PLACE_REACH: handlePolicyMotion(now); break;
          case RuntimeState::OPEN_RELEASE: handleOpenRelease(now); break;
          case RuntimeState::PLACE_TO_STAY: handlePolicyMotion(now); break;
          case RuntimeState::COMPLETE:
          case RuntimeState::HOLD:
          case RuntimeState::FAULT:
          case RuntimeState::E_STOP:
            break;
        }
      }
    } catch (const std::exception & exception) {
      enterFault(std::string(typeid(exception).name()) + ": " + exception.what());
    }
    publishStatus(now);
  }

  void handleNotReady(const TimePoint & now)
  {
    std::vector<std::string> reasons;
    if (!actor_.ready()) {
      reasons.push_back(startup_error_.empty() ? "actor unavailable" : startup_error_);
    }
    if (!jointStateFresh(now)) {
      reasons.push_back("joint states unavailable");
    }
    if (require_gripper_server_ && !gripper_client_->action_server_is_ready()) {
      reasons.push_back("gripper action server unavailable");
    }
    if (!reasons.empty()) {
      std::ostringstream detail;
      for (std::size_t index = 0; index < reasons.size(); ++index) {
        if (index != 0) {
          detail << ", ";
        }
        detail << reasons[index];
      }
      state_detail_ = detail.str();
      return;
    }

    limiter_->reset(arm_qpos_);
    previous_action_.setZero();
    stay_stable_count_ = 0;
    if (initialize_to_policy_stay_) {
      setState(RuntimeState::ALIGN_STAY, "aligning arm to policy Stay");
      return;
    }
    const double error = (arm_qpos_ - reference_->stayJoints()).norm();
    if (error > stay_tolerance_rad_) {
      state_detail_ = "waiting for external policy Stay alignment; error=" +
        std::to_string(error) + "rad";
      return;
    }
    setState(RuntimeState::STAY_EMPTY, "runtime ready");
  }

  void handleAlignStay(const TimePoint & now)
  {
    if (!jointStateFresh(now)) {
      enterFault("joint state timeout during Stay alignment");
      return;
    }
    if (stateAge(now) > initialization_timeout_s_) {
      enterFault("policy Stay alignment timeout");
      return;
    }
    const Vector4 reference_action = reference_->action(
      limiter_->armTarget(), true);
    const Vector4 target = limiter_->step(Vector4::Zero(), reference_action);
    if (publish_every_n_cycles_ <= 0 || cycle_count_ % publish_every_n_cycles_ == 0) {
      publishArmTarget(target);
    }
    const double error = (arm_qpos_ - reference_->stayJoints()).norm();
    stay_stable_count_ = error <= stay_tolerance_rad_ ? stay_stable_count_ + 1 : 0;
    if (stay_stable_count_ >= stay_stable_count_required_) {
      limiter_->reset(arm_qpos_);
      stay_stable_count_ = 0;
      setState(RuntimeState::STAY_EMPTY, "runtime ready");
    }
  }

  void handleStayEmpty(const TimePoint & now)
  {
    if (!jointStateFresh(now)) {
      enterFault("joint state timeout while idle");
      return;
    }
    const bool manual_pick = pending_command_ == "PICK";
    const bool automatic_pick = auto_pick_when_in_range_ && baseStopped(now) &&
      objectTargetFresh(now) && arucoDepthFresh(now) &&
      aruco_marker_z_ >= pick_start_min_marker_z_m_ &&
      aruco_marker_z_ <= pick_start_max_marker_z_m_;
    if (manual_pick || automatic_pick) {
      pending_command_.clear();
      setState(
        RuntimeState::WAIT_PICK,
        automatic_pick && !manual_pick ?
        "automatic PICK: marker_z=" + std::to_string(aruco_marker_z_) + "m" :
        "manual PICK command");
    }
  }

  void handleWaitPick(const TimePoint & now)
  {
    if (!baseStopped(now)) {
      state_detail_ = "waiting for base arrival and zero velocity";
      return;
    }
    if (!objectTargetFresh(now)) {
      state_detail_ = "waiting for valid object pose";
      return;
    }
    if (!arucoDepthFresh(now)) {
      state_detail_ = "waiting for fresh ArUco marker depth";
      return;
    }
    if (aruco_marker_z_ < pick_start_min_marker_z_m_ ||
      aruco_marker_z_ > pick_start_max_marker_z_m_)
    {
      state_detail_ = "waiting for marker depth inside PICK range";
      return;
    }

    reference_->setTarget(*object_target_, true);
    limiter_->reset(arm_qpos_);
    previous_action_.setZero();
    stable_gate_count_ = 0;
    commandGripper(gripper_open_, gripper_max_effort_, false);
    setState(
      RuntimeState::OPEN_GRIPPER,
      "PICK range accepted: marker_z=" + std::to_string(aruco_marker_z_) + "m");
  }

  void handleOpenGripper(const TimePoint & now)
  {
    if (gripper_state_ == GripperState::SUCCEEDED) {
      resetGripper();
      setState(RuntimeState::PICK_REACH, "gripper open");
    } else if (gripper_state_ == GripperState::FAILED) {
      enterFault(gripper_message_);
    } else if (stateAge(now) > gripper_timeout_s_) {
      enterFault("gripper open timeout");
    }
  }

  void handleCloseGripper(const TimePoint & now)
  {
    if (gripper_state_ == GripperState::SUCCEEDED) {
      resetGripper();
      setState(RuntimeState::VERIFY_GRASP, "waiting for grasp confirmation");
    } else if (gripper_state_ == GripperState::FAILED) {
      enterFault(gripper_message_);
    } else if (stateAge(now) > gripper_timeout_s_) {
      enterFault("gripper close timeout");
    }
  }

  void handleVerifyGrasp(const TimePoint & now)
  {
    if (!require_external_grasp_confirmation_ || external_grasp_confirmed_) {
      grasped_ = true;
      stay_stable_count_ = 0;
      setState(RuntimeState::PICK_TO_STAY, "grasp confirmed; returning to Stay");
    } else if (stateAge(now) > grasp_confirmation_timeout_s_) {
      enterHold("grasp confirmation timeout");
    }
  }

  void handleWaitDelivery(const TimePoint & now)
  {
    if (!jointStateFresh(now)) {
      enterFault("joint state timeout during delivery");
      return;
    }
    if (!grasped_) {
      enterFault("WAIT_DELIVERY reached without a grasped object");
      return;
    }
    if (pending_command_ != "PLACE") {
      return;
    }
    if (!baseStopped(now)) {
      state_detail_ = "PLACE pending; waiting for base stop";
      return;
    }
    if (delivery_pose_required_ && !deliveryPoseFresh(now)) {
      state_detail_ = "PLACE pending; waiting for delivery pose";
      return;
    }
    pending_command_.clear();
    reference_->setTarget(delivery_target_, true);
    limiter_->reset(arm_qpos_);
    previous_action_.setZero();
    stable_gate_count_ = 0;
    release_complete_time_ = TimePoint{};
    setState(RuntimeState::PLACE_REACH, "starting place reach");
  }

  void handleOpenRelease(const TimePoint & now)
  {
    if (gripper_state_ == GripperState::SUCCEEDED) {
      if (release_complete_time_ == TimePoint{}) {
        release_complete_time_ = now;
      }
      if (ageSeconds(release_complete_time_, now) < release_settle_time_s_) {
        state_detail_ = "gripper open; settling object";
        return;
      }
      resetGripper();
      release_complete_time_ = TimePoint{};
      grasped_ = false;
      stay_stable_count_ = 0;
      setState(RuntimeState::PLACE_TO_STAY, "released; returning to Stay");
    } else if (gripper_state_ == GripperState::FAILED) {
      enterFault(gripper_message_);
    } else if (stateAge(now) > gripper_timeout_s_) {
      enterFault("gripper release timeout");
    }
  }

  void handlePolicyMotion(const TimePoint & now)
  {
    if (!jointStateFresh(now)) {
      enterHold("joint state timeout");
      return;
    }
    const bool pick_reach = state_ == RuntimeState::PICK_REACH;
    const bool place_reach = state_ == RuntimeState::PLACE_REACH;
    const bool return_to_stay =
      state_ == RuntimeState::PICK_TO_STAY ||
      state_ == RuntimeState::PLACE_TO_STAY;
    double target_yaw = 0.0;
    int phase = 0;
    if (state_ == RuntimeState::PICK_TO_STAY) {
      phase = 1;
    } else if (place_reach) {
      phase = 2;
    } else if (state_ == RuntimeState::PLACE_TO_STAY) {
      phase = 3;
    }
    if (!return_to_stay) {
      if (pick_reach) {
        if (!objectTargetFresh(now)) {
          enterHold("object pose timeout during reach");
          return;
        }
        reference_->setTarget(*object_target_);
      } else if (place_reach) {
        if (delivery_pose_required_ && !deliveryPoseFresh(now)) {
          enterHold("delivery pose timeout during reach");
          return;
        }
        reference_->setTarget(delivery_target_);
      } else {
        throw std::logic_error("policy reach called from an invalid state");
      }
      reference_->advance(arm_qpos_);
      if (reference_->finalStage()) {
        target_yaw = pick_reach ? object_yaw_ : delivery_yaw_;
      }
    }

    const Vector3 eef_position = kinematics_->forward(arm_qpos_);
    const Vector3 active_target = reference_->activeTarget(return_to_stay);
    const Observation observation = observation_builder_->build(
      arm_qpos_, arm_qvel_, gripper_position_, gripper_velocity_, eef_position,
      active_target, target_yaw, grasped_, phase, previous_action_);
    Vector4 raw_action = Vector4::Zero();
    if (residual_scale_ > 1.0e-12) {
      const auto started = SteadyClock::now();
      raw_action = actor_.predict(observation);
      const double elapsed = ageSeconds(started, SteadyClock::now());
      recordInferenceTiming(elapsed);
      if (elapsed > inference_deadline_s_) {
        ++inference_deadline_misses_;
        if (inference_deadline_misses_ >= inference_deadline_miss_limit_) {
          enterHold("C++ policy inference deadline exceeded");
        }
        return;
      }
      inference_deadline_misses_ = 0;
    }

    const Vector4 reference_action = reference_->action(
      limiter_->armTarget(), return_to_stay);
    const Vector4 target = limiter_->step(
      raw_action, reference_action, true,
      gripper_pitch_hold_enabled_ && (pick_reach || place_reach));
    previous_action_ = raw_action;
    if (publish_every_n_cycles_ <= 0 || cycle_count_ % publish_every_n_cycles_ == 0) {
      publishArmTarget(target);
    }

    if (pick_reach || place_reach) {
      stable_gate_count_ = reachGate(eef_position) ? stable_gate_count_ + 1 : 0;
      if (stable_gate_count_ >= gate_stable_count_required_) {
        stable_gate_count_ = 0;
        if (pick_reach) {
          commandGripper(gripper_grasp_position_, gripper_max_effort_, true);
          setState(RuntimeState::CLOSE_GRIPPER, "grasp gate reached");
        } else {
          commandGripper(gripper_open_, gripper_max_effort_, false);
          release_complete_time_ = TimePoint{};
          setState(RuntimeState::OPEN_RELEASE, "place gate reached");
        }
      }
    } else {
      const double stay_error = (arm_qpos_ - reference_->stayJoints()).norm();
      stay_stable_count_ = stay_error <= stay_tolerance_rad_ ? stay_stable_count_ + 1 : 0;
      if (stay_stable_count_ >= stay_stable_count_required_) {
        stay_stable_count_ = 0;
        if (state_ == RuntimeState::PICK_TO_STAY) {
          setState(RuntimeState::WAIT_DELIVERY, "grasp complete");
        } else {
          setState(RuntimeState::COMPLETE, "pick place complete");
        }
      }
    }
  }

  void recordInferenceTiming(double elapsed_s)
  {
    const double elapsed_ms = elapsed_s * 1000.0;
    inference_window_ms_.push_back(elapsed_ms);
    inference_total_ms_ += elapsed_ms;
    if (inference_window_ms_.size() < static_cast<std::size_t>(inference_timing_window_)) {
      return;
    }

    std::vector<double> sorted = inference_window_ms_;
    std::sort(sorted.begin(), sorted.end());
    const auto percentile = [&sorted](double probability) {
        const auto rank = static_cast<std::size_t>(
          std::ceil(probability * static_cast<double>(sorted.size())));
        return sorted.at(std::max<std::size_t>(1U, rank) - 1U);
      };
    RCLCPP_INFO(
      get_logger(),
      "C++ PPO inference timing: samples=%zu mean=%.4fms p95=%.4fms p99=%.4fms "
      "max=%.4fms deadline=%.1fms",
      sorted.size(), inference_total_ms_ / static_cast<double>(sorted.size()),
      percentile(0.95), percentile(0.99), sorted.back(), inference_deadline_s_ * 1000.0);
    inference_window_ms_.clear();
    inference_total_ms_ = 0.0;
  }

  bool reachGate(const Vector3 & eef_position) const
  {
    if (!reference_->finalStage()) {
      return false;
    }
    const Vector3 delta = reference_->pregraspTarget() - eef_position;
    const double bearing_error = std::abs(
      wrapToPi(
        arm_qpos_(0) - kinematics_->bearing(reference_->target())));
    const double roll_error = std::abs(wrapToPi(arm_qpos_.tail<3>().sum()));
    return delta.norm() <= close_distance_m_ &&
           delta.head<2>().norm() <= close_xy_distance_m_ &&
           std::abs(delta(2)) <= close_z_tolerance_m_ &&
           bearing_error <= close_bearing_tolerance_rad_ &&
           roll_error <= close_roll_tolerance_rad_;
  }

  bool jointStateFresh(const TimePoint & now) const
  {
    return ageSeconds(joint_state_time_, now) <= joint_state_timeout_s_;
  }

  bool objectTargetFresh(const TimePoint & now) const
  {
    return object_target_.has_value() && target_valid_ &&
           ageSeconds(object_pose_time_, now) <= target_pose_timeout_s_ &&
           ageSeconds(target_valid_time_, now) <= target_pose_timeout_s_;
  }

  bool deliveryPoseFresh(const TimePoint & now) const
  {
    return ageSeconds(delivery_pose_time_, now) <= target_pose_timeout_s_;
  }

  bool arucoDepthFresh(const TimePoint & now) const
  {
    return std::isfinite(aruco_marker_z_) &&
           ageSeconds(aruco_pose_time_, now) <= aruco_pose_timeout_s_;
  }

  bool baseStopped(const TimePoint & now) const
  {
    if (require_base_arrived_ &&
      (!base_arrived_ || ageSeconds(base_arrived_time_, now) > base_arrival_timeout_s_))
    {
      return false;
    }
    if (require_odom_stopped_) {
      if (ageSeconds(odom_time_, now) > odom_timeout_s_ ||
        base_linear_speed_ > stopped_linear_speed_mps_ ||
        base_angular_speed_ > stopped_angular_speed_radps_)
      {
        return false;
      }
    }
    return true;
  }

  bool baseInterlockState() const
  {
    return state_ == RuntimeState::OPEN_GRIPPER ||
           state_ == RuntimeState::PICK_REACH ||
           state_ == RuntimeState::CLOSE_GRIPPER ||
           state_ == RuntimeState::VERIFY_GRASP ||
           state_ == RuntimeState::PICK_TO_STAY ||
           state_ == RuntimeState::PLACE_REACH ||
           state_ == RuntimeState::OPEN_RELEASE ||
           state_ == RuntimeState::PLACE_TO_STAY;
  }

  bool baseHoldState() const
  {
    return state_ == RuntimeState::ALIGN_STAY || baseInterlockState() ||
           state_ == RuntimeState::WAIT_PICK || state_ == RuntimeState::HOLD ||
           state_ == RuntimeState::FAULT || state_ == RuntimeState::E_STOP;
  }

  double stateAge(const TimePoint & now) const
  {
    return ageSeconds(state_enter_time_, now);
  }

  void resetRuntime()
  {
    if (safety_stop_) {
      state_detail_ = "RESET rejected while safety_stop=true";
      return;
    }
    pending_command_.clear();
    external_grasp_confirmed_ = false;
    stable_gate_count_ = 0;
    stay_stable_count_ = 0;
    inference_deadline_misses_ = 0;
    release_complete_time_ = TimePoint{};
    previous_action_.setZero();
    resetGripper();
    if (jointStateFresh(SteadyClock::now())) {
      limiter_->reset(arm_qpos_);
    }
    setState(
      grasped_ ? RuntimeState::WAIT_DELIVERY : RuntimeState::NOT_READY,
      "manual reset");
  }

  void enterHold(const std::string & detail)
  {
    if (state_ == RuntimeState::E_STOP || state_ == RuntimeState::FAULT) {
      return;
    }
    if (jointStateFresh(SteadyClock::now())) {
      limiter_->reset(arm_qpos_);
      publishArmTarget(arm_qpos_);
    }
    setState(RuntimeState::HOLD, detail);
  }

  void enterFault(const std::string & detail)
  {
    if (state_ == RuntimeState::E_STOP) {
      return;
    }
    cancelGripper();
    if (jointStateFresh(SteadyClock::now())) {
      limiter_->reset(arm_qpos_);
      publishArmTarget(arm_qpos_);
    }
    setState(RuntimeState::FAULT, detail);
    RCLCPP_ERROR(get_logger(), "%s", detail.c_str());
  }

  void enterEstop(const std::string & detail)
  {
    if (state_ == RuntimeState::E_STOP) {
      return;
    }
    cancelGripper();
    if (jointStateFresh(SteadyClock::now())) {
      limiter_->reset(arm_qpos_);
      publishArmTarget(arm_qpos_);
    }
    setState(RuntimeState::E_STOP, detail);
  }

  void setState(RuntimeState next, const std::string & detail)
  {
    if (state_ == next && state_detail_ == detail) {
      return;
    }
    const std::string previous = stateName(state_);
    state_ = next;
    state_detail_ = detail;
    state_enter_time_ = SteadyClock::now();
    RCLCPP_INFO(
      get_logger(), "C++ RL state %s -> %s: %s", previous.c_str(),
      stateName(next).c_str(), detail.c_str());
  }

  static std::string stateName(RuntimeState state)
  {
    switch (state) {
      case RuntimeState::NOT_READY: return "NOT_READY";
      case RuntimeState::ALIGN_STAY: return "ALIGN_STAY";
      case RuntimeState::STAY_EMPTY: return "STAY_EMPTY";
      case RuntimeState::WAIT_PICK: return "WAIT_PICK";
      case RuntimeState::OPEN_GRIPPER: return "OPEN_GRIPPER";
      case RuntimeState::PICK_REACH: return "PICK_REACH";
      case RuntimeState::CLOSE_GRIPPER: return "CLOSE_GRIPPER";
      case RuntimeState::VERIFY_GRASP: return "VERIFY_GRASP";
      case RuntimeState::PICK_TO_STAY: return "PICK_TO_STAY";
      case RuntimeState::WAIT_DELIVERY: return "WAIT_DELIVERY";
      case RuntimeState::PLACE_REACH: return "PLACE_REACH";
      case RuntimeState::OPEN_RELEASE: return "OPEN_RELEASE";
      case RuntimeState::PLACE_TO_STAY: return "PLACE_TO_STAY";
      case RuntimeState::COMPLETE: return "COMPLETE";
      case RuntimeState::HOLD: return "HOLD";
      case RuntimeState::FAULT: return "FAULT";
      case RuntimeState::E_STOP: return "E_STOP";
    }
    return "UNKNOWN";
  }

  void publishArmTarget(const Vector4 & positions)
  {
    trajectory_msgs::msg::JointTrajectory message;
    message.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(positions.data(), positions.data() + positions.size());
    point.time_from_start = rclcpp::Duration::from_seconds(trajectory_time_s_);
    message.points.push_back(point);
    arm_publisher_->publish(message);
  }

  void publishBaseHold()
  {
    std_msgs::msg::Bool message;
    message.data = baseHoldState();
    base_hold_publisher_->publish(message);
  }

  void publishStatus(const TimePoint & now)
  {
    const double tracking_error = limiter_->initialized() ?
      (arm_qpos_ - limiter_->armTarget()).norm() :
      std::numeric_limits<double>::quiet_NaN();
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(4);
    stream << "state=" << stateName(state_) << " detail=" << state_detail_
           << " model=" << model_version_ << " runtime=cpp"
           << " actor_backend=" << actor_.backend() << " actor_device=" << actor_.device()
           << " grasped=" << boolText(grasped_) << " mode=" << control_mode_
           << " ref_stage=" << reference_->stage() << '/' << reference_->waypointCount()
           << " track_error=" << tracking_error
           << " pending=" << (pending_command_.empty() ? "none" : pending_command_);
    const std::string text = stream.str();
    if (text == last_status_text_ && ageSeconds(last_status_time_, now) < status_period_s_) {
      return;
    }
    last_status_text_ = text;
    last_status_time_ = now;
    std_msgs::msg::String status;
    status.data = text;
    status_publisher_->publish(status);

    std_msgs::msg::String compatibility;
    if (state_ == RuntimeState::WAIT_DELIVERY) {
      compatibility.data = "grasp complete";
    } else if (state_ == RuntimeState::COMPLETE) {
      compatibility.data = "pick place complete";
    } else if (state_ == RuntimeState::FAULT || state_ == RuntimeState::E_STOP) {
      compatibility.data = "ERROR " + state_detail_;
    } else {
      compatibility.data = text;
    }
    compat_status_publisher_->publish(compatibility);
  }

  bool commandGripper(double position, double effort, bool allow_stall)
  {
    if (gripper_state_ == GripperState::SENDING || gripper_state_ == GripperState::ACTIVE) {
      return false;
    }
    resetGripper();
    if (!gripper_client_->action_server_is_ready()) {
      gripper_state_ = GripperState::FAILED;
      gripper_message_ = "gripper action server is unavailable";
      return false;
    }
    GripperCommand::Goal goal;
    goal.command.position = position;
    goal.command.max_effort = effort;
    gripper_allow_stall_ = allow_stall;
    gripper_state_ = GripperState::SENDING;
    gripper_message_ = "gripper goal sending";
    const std::uint64_t command_id = ++gripper_command_id_;

    rclcpp_action::Client<GripperCommand>::SendGoalOptions options;
    options.goal_response_callback =
      [this, command_id](const GripperGoalHandle::SharedPtr & handle) {
        if (command_id != gripper_command_id_) {
          return;
        }
        if (!handle) {
          gripper_state_ = GripperState::FAILED;
          gripper_message_ = "gripper goal rejected";
          return;
        }
        gripper_goal_handle_ = handle;
        gripper_state_ = GripperState::ACTIVE;
        gripper_message_ = "gripper goal active";
      };
    options.result_callback =
      [this, command_id](const GripperGoalHandle::WrappedResult & wrapped) {
        if (command_id != gripper_command_id_) {
          return;
        }
        const bool reached = wrapped.result && wrapped.result->reached_goal;
        const bool stalled = wrapped.result && wrapped.result->stalled;
        const bool succeeded =
          (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
          (reached || (gripper_allow_stall_ && stalled))) ||
          (wrapped.code == rclcpp_action::ResultCode::ABORTED &&
          gripper_allow_stall_ && stalled);
        gripper_state_ = succeeded ? GripperState::SUCCEEDED : GripperState::FAILED;
        gripper_message_ = succeeded ? "gripper complete" : "gripper command failed";
      };
    gripper_client_->async_send_goal(goal, options);
    return true;
  }

  void cancelGripper()
  {
    ++gripper_command_id_;
    if (gripper_goal_handle_) {
      gripper_client_->async_cancel_goal(gripper_goal_handle_);
    }
    gripper_goal_handle_.reset();
    gripper_state_ = GripperState::FAILED;
    gripper_message_ = "gripper command canceled by safety stop";
  }

  void resetGripper()
  {
    if (gripper_state_ == GripperState::SENDING || gripper_state_ == GripperState::ACTIVE) {
      return;
    }
    gripper_state_ = GripperState::IDLE;
    gripper_message_.clear();
    gripper_goal_handle_.reset();
  }

  // Configuration and runtime math.
  std::vector<std::string> joint_names_;
  std::string gripper_joint_name_;
  std::string target_frame_id_;
  std::string model_version_;
  std::string policy_backend_;
  std::string policy_device_;
  double control_period_s_{0.02};
  double trajectory_time_s_{0.08};
  double inference_deadline_s_{0.018};
  int64_t inference_deadline_miss_limit_{3};
  int64_t inference_timing_window_{500};
  double status_period_s_{0.5};
  double release_settle_time_s_{0.24};
  Vector4 joint_low_;
  Vector4 joint_high_;
  Vector4 joint_velocity_scale_;
  Vector4 max_joint_velocity_;
  Vector4 max_joint_acceleration_;
  Vector3 workspace_min_;
  Vector3 workspace_max_;
  Vector2 arm_base_xy_;
  Vector3 policy_frame_offset_;
  Vector4 action_scale_;
  double action_filter_coefficient_{0.18};
  double residual_scale_{0.10};
  std::string control_mode_;
  Vector4 stay_joint_positions_;
  Vector4 ik_initial_guess_;
  Vector4 approach_waypoint_1_;
  Vector4 approach_waypoint_2_;
  double approach_waypoint_tolerance_{0.08};
  double pregrasp_height_offset_{0.0225};
  double reference_action_limit_{1.0};
  double final_action_limit_{1.0};
  double target_update_min_m_{0.001};
  bool initialize_to_policy_stay_{false};
  double initialization_timeout_s_{8.0};
  double pick_start_min_marker_z_m_{0.10};
  double pick_start_max_marker_z_m_{0.265};
  double aruco_pose_timeout_s_{0.50};
  bool auto_pick_when_in_range_{true};
  double gripper_open_{0.019};
  double gripper_close_{-0.010};
  double gripper_grasp_position_{-0.010};
  double gripper_velocity_scale_{0.25};
  double gripper_max_effort_{0.0};
  double gripper_timeout_s_{4.0};
  bool require_gripper_server_{true};
  bool require_external_grasp_confirmation_{false};
  double grasp_confirmation_timeout_s_{2.0};
  double joint_state_timeout_s_{0.25};
  double target_pose_timeout_s_{0.50};
  bool delivery_pose_required_{false};
  Vector3 delivery_target_{0.27, 0.0, 0.1815};
  double delivery_yaw_{0.0};
  bool require_base_arrived_{false};
  bool require_odom_stopped_{false};
  double base_arrival_timeout_s_{2.0};
  double odom_timeout_s_{0.50};
  double stopped_linear_speed_mps_{0.01};
  double stopped_angular_speed_radps_{0.05};
  double close_distance_m_{0.042};
  double close_xy_distance_m_{0.035};
  double close_z_tolerance_m_{0.030};
  double close_bearing_tolerance_rad_{0.35};
  double close_roll_tolerance_rad_{0.35};
  int64_t gate_stable_count_required_{4};
  double stay_tolerance_rad_{0.050};
  int64_t stay_stable_count_required_{8};
  int64_t publish_every_n_cycles_{1};
  bool gripper_pitch_hold_enabled_{true};
  double gripper_pitch_target_{0.0};
  double gripper_pitch_joint4_max_step_{0.028};

  std::string image_target_topic_;
  std::string aruco_pose_topic_;
  std::string target_valid_topic_;
  std::string delivery_pose_topic_;
  std::string joint_states_topic_;
  std::string odom_topic_;
  std::string base_arrived_topic_;
  std::string safety_stop_topic_;
  std::string grasp_confirmed_topic_;
  std::string command_topic_;
  std::string compat_start_topic_;
  std::string arm_trajectory_topic_;
  std::string gripper_action_name_;
  std::string base_hold_topic_;
  std::string status_topic_;
  std::string compat_status_topic_;

  PolicyActor actor_;
  std::unique_ptr<OpenManipulatorKinematics> kinematics_;
  std::unique_ptr<ObservationBuilder> observation_builder_;
  std::unique_ptr<ReferenceController> reference_;
  std::unique_ptr<ActionLimiter> limiter_;
  std::string startup_error_;

  RuntimeState state_{RuntimeState::NOT_READY};
  std::string state_detail_{"waiting for actor and robot state"};
  TimePoint state_enter_time_{SteadyClock::now()};
  std::string pending_command_;
  bool grasped_{false};
  bool external_grasp_confirmed_{false};
  bool safety_stop_{false};
  Vector4 previous_action_{Vector4::Zero()};
  Vector4 arm_qpos_{Vector4::Zero()};
  Vector4 arm_qvel_{Vector4::Zero()};
  double gripper_position_{0.019};
  double gripper_velocity_{0.0};
  TimePoint joint_state_time_{};
  std::optional<Vector3> object_target_;
  double object_yaw_{0.0};
  TimePoint object_pose_time_{};
  double aruco_marker_z_{std::numeric_limits<double>::infinity()};
  TimePoint aruco_pose_time_{};
  bool target_valid_{false};
  TimePoint target_valid_time_{};
  TimePoint delivery_pose_time_{};
  bool base_arrived_{false};
  TimePoint base_arrived_time_{};
  double base_linear_speed_{std::numeric_limits<double>::infinity()};
  double base_angular_speed_{std::numeric_limits<double>::infinity()};
  TimePoint odom_time_{};
  std::string last_rejected_pose_frame_;
  int64_t stable_gate_count_{0};
  int64_t stay_stable_count_{0};
  int64_t inference_deadline_misses_{0};
  std::vector<double> inference_window_ms_;
  double inference_total_ms_{0.0};
  int64_t cycle_count_{0};
  TimePoint release_complete_time_{};
  std::string last_status_text_;
  TimePoint last_status_time_{};

  GripperState gripper_state_{GripperState::IDLE};
  std::string gripper_message_;
  bool gripper_allow_stall_{false};
  std::uint64_t gripper_command_id_{0};
  GripperGoalHandle::SharedPtr gripper_goal_handle_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr base_hold_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr compat_status_publisher_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_valid_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    delivery_pose_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr base_arrived_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_stop_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasp_confirmed_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr compat_start_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace omx_rl_control_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<omx_rl_control_cpp::RlControlCppNode>());
  } catch (const std::exception & exception) {
    std::cerr << "rl_control_cpp_node startup failed: " << exception.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
