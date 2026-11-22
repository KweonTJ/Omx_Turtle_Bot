# Robot controller baseline and camera wrist hold

Imported on 2026-11-22 from the controller actually installed on
`a@192.168.0.11`, source `/home/a/turtlebot3_ws/src/mp_control`.
The original robot workspace is retained. The source archive SHA-256 is
`52c7b20cab61bd0c72f30230036a0f40b050c5874203a24df410065a4203999a`.

`hold_joint4_during_extension: true` captures joint4 when pregrasp starts and
keeps that reference throughout final targets, stepped/raw targets, lead and
interpolated waypoints, and analytic/Bezier EEF extension. Wrist tracking still
uses measured feedback. Handoff commands retain their separate wrist behavior.
The C++ ONNX relay masks the wrist residual for raw extension references, so
the policy cannot undo this hold. Direct handoff references retain their wrist
residual channel.

The former `eef_forward_joint4_max_delta_rad: 0` setting did not protect
pregrasp, which independently recomputed joint4 from tool pitch and used
`pregrasp_joint4_max_step_rad`. Bezier extension also had its own wrist step
parameter. The new hold takes precedence over those coupled adjustments.

Set the new parameter to `false` only when the old coupled wrist behavior is
intended. The runtime configuration's physical commissioning flags are not
changed by this patch.
