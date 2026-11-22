# Verified lift-turn-lower pick and place, 2026-11-23

Successful run: `lift_near_id0_20261123_0320` on the Jetson workspace
`/home/a/omx_v6_cpp_ws_20261122`, ROS domain 25. The operator confirmed that the
complete physical sequence succeeded. The controller completed it automatically
in approximately 39 seconds, without the manual advancement used in the preceding trial.

**Saved sequence:** pick ID 0 → lift → turn joint1 right 90° → lower → open → Stay.
The separate inward elbow-folding movement is disabled. Lowering restores the
joint2/3/4 positions measured before lifting, while keeping the rotated joint1.
The arm must settle before release, and measured gripper opening precedes Stay return.

| Setting | Successful value |
| --- | --- |
| `retract_mode` | `lift` (default) |
| `handoff_lift_joint2_delta_rad` | `-0.55` |
| `handoff_retract_joint3_delta_rad` | `0.0` |
| `handoff_restore_grasp_pose` | `true` |
| `handoff_lift_joint_tolerance_rad` | `0.10`, with stopped, fresh joint feedback |
| `handoff_lower_before_release` | `true` |
| Lift / lower / turn duration | `1.8 / 2.0 / 7.0` seconds |
| `hold_joint4_during_extension` | `false` |
| `gripper_down_joint4_offset_rad` | `0.20` |
| Pregrasp move / settle | `0.35 / 0.05` seconds |

Joint4 follows the pitch compensation during lift. It is not fixed. The lift
accepts the stopped measured pose within its own tolerance instead of waiting
indefinitely at the pregrasp tolerance. An operator can also advance a stopped
lift with `/mp_control/continue_handoff`; fresh joint feedback is still required.

Only the selected marker is needed at startup. The live successful run saw ID 0
without requiring ID 1. Selection, observation slots, and launch forwarding for
both IDs are covered by the local tests. Put the object within the actual grasp
reach; the preceding, farther placement did not provide the successful pickup.

The existing ROS hardware driver remains the sole OpenCR owner. Motion and ONNX
inference use C++; Python supplies perception and the mission gate. This is the
documented `mp_control` reference trajectory plus bounded ONNX residual mode.
The original actor SHA-256 is
`fe3656701f104022872ad804ac120496c4b31de8cf9e7c3a2f94250e4ceef00c`.
No provisional calibration fit or commissioning flag was applied for this success.

With the existing hardware bringup at Stay, run on the Jetson:

```bash
export ROS_DOMAIN_ID=25 PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source /home/a/turtlebot3_ws/install/setup.bash
source /home/a/omx_v6_cpp_ws_20261122/install/setup.bash
ros2 launch omx_turtlebot3_runtime ros_policy_pick_place.launch.py execute:=true marker_id:=0 retract_mode:=lift
```

Host checkpoint: `artifacts/success_lift_pick_place_20261123_0320/` under
`/home/ktj/omx_turtle_ws`. It contains the exact successful controller profile,
result, final measured joints, source archive, and manifest. All run logs and
camera frames are retained in the checksum-verified host experiment archive.
The robot hardware driver is left holding Stay with the gripper open.
