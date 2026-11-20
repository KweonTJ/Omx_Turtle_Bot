#!/usr/bin/env bash
set -euo pipefail
tool_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace_dir=$(cd -- "$tool_dir/../../.." && pwd)
runtime_python=${OMX_CPP_PYTHON:-$workspace_dir/.venv_omx_cpp/bin/python}
if [[ ! -x "$runtime_python" ]]; then
  printf '%s\n' 'Set OMX_CPP_PYTHON to a ROS Humble compatible Python 3.10 environment with MuJoCo, SciPy and OpenCV ArUco.' >&2
  exit 1
fi
# ROS setup scripts may refer to unset environment variables.
set +u
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
set -u
export PYTHONNOUSERSITE=1
exec "$runtime_python" -c 'from omx_turtlebot3_runtime.node import main_cpp; main_cpp()' "$@"
