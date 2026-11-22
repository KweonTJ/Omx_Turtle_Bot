# 2026-11-22 host completion and next robot run

The robot was powered down at the operator's request. No pick-and-place success
was confirmed. The wrist-hold patch below is built on the host and has not yet
been deployed to or physically retested on the powered-down Jetson.

## Camera wrist correction

The actual robot's mp_control source was imported into this workspace, retaining
the original robot checkout. `hold_joint4_during_extension: true` captures the
measured joint4 angle on entry to pregrasp and reuses it for final/stepped/raw
targets and every synchronized/lead waypoint. Analytic and Bezier EEF extension
use the same reference. Primary joint completion and measured wrist tracking
remain checked. Handoff retains its own wrist commands.

The C++ ONNX relay explicitly zeroes the wrist residual for raw extension
references; direct handoff references retain it. This prevents a nonzero model
output from undoing the wrist hold. The model/checkpoint, URDF and encoder zero
mapping were not changed. Runtime commissioning flags remain unverified.

The prior setting `eef_forward_joint4_max_delta_rad: 0` did not cover the
pregrasp tool-pitch compensation or Bezier's independent wrist step. In the
physical trial joint4 changed from about -1.628 to -1.779 rad during initial
extension, and marker detection was then lost. This motivated the patch.

## Repeatable mission entry point

Build and source this checkout, so the patched mp_control is selected:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to omx_turtlebot3_runtime \
  --allow-overriding mp_control
source install/setup.bash
```

Keep an existing hardware bringup at Stay. For a new hardware session:

```bash
export ROS_DOMAIN_ID=218
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py \
  move_to_stay_pose:=true start_rviz:=false start_camera:=false \
  start_lidar:=false start_eef_camera_driver:=false
```

From another terminal with the same workspace sourced and domain selected:

```bash
# Monitor only; never starts the gripper/mission or publishes arm commands.
ros2 launch omx_turtlebot3_runtime ros_policy_pick_place.launch.py \
  ros_domain_id:=218

# Operator-requested ID 0 trial; one sequence, then mission components exit.
ros2 launch omx_turtlebot3_runtime ros_policy_pick_place.launch.py \
  ros_domain_id:=218 execute:=true
```

The mission launch opens no serial port. The existing C++ ros2_control driver is
the sole OpenCR owner. It starts the patched C++ mp_control, C++ ONNX relay, RGB
driver and physical observation adapter together. It verifies original ONNX
conformance, the controller's wrist-hold parameter, active arm/gripper
controllers, stopped measured Stay, and both fresh marker IDs before one start
request. Default monitor mode sends no start/cancel/arm commands. Missing startup
conditions and mission timeouts end the trial with logs. The external hardware
bringup retains its position hold after mission components exit.

This is the empirically tuned mp_control reference path with one bounded v6
residual step per incoming segment. It is separate from the 9-phase native IK
runtime. Contact/deck policy bits remain false in this reference trial; a
controller `cargo_loaded` event is not treated as measured grasp/placement proof.

## Validation and retained data

Final host results are recorded in `logs/joint4_extension_hold_20261122/`.
The final checks passed: 22 native C++ tests, 5 wrist-hold C++ tests, and
152 Python/ROS tests. One existing optional Torch test was skipped; the original
973-vector ONNX comparison was executed. The installed mission launch's
`--show-args` check also passed.
The C++ wrist tests cover fixed final/step/raw goals, lead and synchronized
waypoints, measured wrist drift, primary-joint completion and handoff separation.
ROS integration tests verify actual ONNX residual application, one joint3 mirror,
repeat-segment coalescing, wrist suppression and passive monitor faults.

The earlier Jetson verification was copied and checksum-verified on the host in
`logs/omx_cpp_robot_verify_20261122_004oc_6p/robot` and
`logs/omx_cpp_stay_verify_20261122_171300/robot`.
The later trial directories below were not fully retrieved before shutdown:

```text
/home/a/omx_v6_cpp_ws_20261122/mission_20261122_174000
/home/a/omx_v6_cpp_ws_20261122/pick_preflight_20261122_172100
/home/a/omx_v6_cpp_ws_20261122/stay_hardware_20261122_173000
```

An export was attempted at `/home/a/omx_robot_final_data_20261122.tar` with a
manifest `/home/a/omx_v6_cpp_ws_20261122/robot_final_transfer_20261122.json`, but
completion was not acknowledged before SSH became unavailable. On reconnect,
retrieve the complete run directories and verify lists, sizes and SHA-256; do
not assume the export is complete and do not delete the originals. These trials
used only the EEF RGB camera, not Astra or D-SLAM.
