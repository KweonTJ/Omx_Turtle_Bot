#!/usr/bin/env bash
set -euo pipefail
tool_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace_dir=$(cd -- "$tool_dir/../../.." && pwd)
cd -- "$workspace_dir"
runtime_python=${OMX_CPP_PYTHON:-$workspace_dir/.venv_omx_cpp/bin/python}
set +u
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
set -u
colcon test --base-paths src --packages-select omx_turtlebot3_control_cpp --event-handlers console_direct+
colcon test-result --test-result-base build/omx_turtlebot3_control_cpp
ros2 run omx_turtlebot3_control_cpp onnx_probe_cpp src/omx_turtlebot3_policy/models/omx_turtlebot3_v6
export PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="$workspace_dir/src/omx_turtlebot3_runtime:$workspace_dir/src/aruco_eef_tracker:${PYTHONPATH:-}"
# The optional legacy Torch parity test is separate; ONNX parity above is mandatory.
unset OMX_V6_MODEL_DIR
exec "$runtime_python" -m pytest -q src/omx_turtlebot3_runtime/test "$@"
