#!/usr/bin/env bash
set -eo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."

robot_workspace="${OMX_ROBOT_WORKSPACE:-/home/ktj/omx_turtle_ws}"
model_version="${OMX_MODEL_VERSION:-omx_turtlebot3_v1}"
refresh_seconds="${OMX_TWIN_REFRESH_SECONDS:-0.5}"
stale_seconds="${OMX_TWIN_STALE_SECONDS:-3}"
start_rviz=true
demo_mode=false

usage() {
  printf '%s\n' \
    'Usage: scripts/run_leader_digital_twin.sh [options]' \
    '' \
    'Read-only leader digital twin: RViz robot pose plus a live task dashboard.' \
    '' \
    'Options:' \
    '  --workspace PATH       Robot ROS 2 workspace (default: /home/ktj/omx_turtle_ws)' \
    '  --model-version NAME   Displayed policy release (default: omx_turtlebot3_v1)' \
    '  --refresh SECONDS      Dashboard refresh interval (default: 0.5)' \
    '  --stale SECONDS        Mark a topic stale after this time (default: 3)' \
    '  --no-rviz              Run the terminal dashboard only' \
    '  --demo                 Publish non-actuating sample task states for UI review' \
    '  -h, --help             Show this help' \
    '' \
    'This script never publishes joint trajectories, gripper goals, or /cmd_vel.'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      [[ $# -ge 2 ]] || { printf '%s\n' 'Missing value for --workspace' >&2; exit 2; }
      robot_workspace="$2"
      shift 2
      ;;
    --model-version)
      [[ $# -ge 2 ]] || { printf '%s\n' 'Missing value for --model-version' >&2; exit 2; }
      model_version="$2"
      shift 2
      ;;
    --refresh)
      [[ $# -ge 2 ]] || { printf '%s\n' 'Missing value for --refresh' >&2; exit 2; }
      refresh_seconds="$2"
      shift 2
      ;;
    --stale)
      [[ $# -ge 2 ]] || { printf '%s\n' 'Missing value for --stale' >&2; exit 2; }
      stale_seconds="$2"
      shift 2
      ;;
    --no-rviz)
      start_rviz=false
      shift
      ;;
    --demo)
      demo_mode=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown option: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! "$refresh_seconds" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
   [[ ! "$stale_seconds" =~ ^[0-9]+$ ]]; then
  printf '%s\n' '--refresh must be numeric and --stale must be an integer.' >&2
  exit 2
fi

ros_setup="/opt/ros/humble/setup.bash"
workspace_setup="${robot_workspace}/install/setup.bash"
description_xacro="${robot_workspace}/src/turtlebot3_manipulation/turtlebot3_manipulation_description/xacro/leader_turtlebot3.urdf.xacro"
description_exporter="${robot_workspace}/src/turtlebot3_manipulation/turtlebot3_manipulation_description/scripts/export_leader_urdf.py"
rviz_source="${robot_workspace}/src/turtlebot3_manipulation/turtlebot3_manipulation_description/rviz/leader_model.rviz"

for required_file in "$ros_setup" "$workspace_setup" "$description_xacro" "$description_exporter"; do
  if [[ ! -f "$required_file" ]]; then
    printf 'Required file not found: %s\n' "$required_file" >&2
    exit 1
  fi
done

source "$ros_setup"
source "$workspace_setup"

for required_command in ros2 xacro python3; do
  if ! command -v "$required_command" >/dev/null 2>&1; then
    printf 'Required command not found: %s\n' "$required_command" >&2
    exit 1
  fi
done

runtime_dir="$(mktemp -d /tmp/omx_leader_digital_twin.XXXXXX)"
state_dir="${runtime_dir}/state"
mkdir -p "$state_dir"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${runtime_dir}/ros_logs}"
mkdir -p "$ROS_LOG_DIR"

child_pids=()
cleanup() {
  local pid
  trap - EXIT INT TERM
  for pid in "${child_pids[@]}"; do
    pkill -TERM -P "$pid" 2>/dev/null || true
    kill -TERM "$pid" 2>/dev/null || true
  done
  for pid in "${child_pids[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
  rm -rf -- "$runtime_dir"
  printf '\nLeader digital twin stopped.\n'
}
trap cleanup EXIT INT TERM

write_state() {
  local key="$1"
  local value="$2"
  local value_tmp="${state_dir}/${key}.value.tmp"
  local stamp_tmp="${state_dir}/${key}.stamp.tmp"
  printf '%s\n' "$value" > "$value_tmp"
  date +%s > "$stamp_tmp"
  mv -- "$value_tmp" "${state_dir}/${key}.value"
  mv -- "$stamp_tmp" "${state_dir}/${key}.stamp"
}

watch_topic() {
  local key="$1"
  local topic="$2"
  local line
  while true; do
    while IFS= read -r line; do
      line="${line//$'\r'/}"
      [[ -z "$line" || "$line" == '---' ]] && continue
      [[ "$line" == WARNING:* ]] && continue
      [[ "$line" == 'Could not determine the type'* ]] && continue
      [[ "$line" == 'Cannot echo topic'* ]] && continue
      write_state "$key" "$line"
    done < <(ros2 topic echo "$topic" --field data 2>/dev/null || true)
    sleep 1
  done
}

state_value() {
  local key="$1"
  local fallback="$2"
  local value_file="${state_dir}/${key}.value"
  local stamp_file="${state_dir}/${key}.stamp"
  local value="$fallback"
  local stamp=0
  local now
  local age

  if [[ -s "$value_file" ]]; then
    IFS= read -r value < "$value_file" || true
  fi
  if [[ -s "$stamp_file" ]]; then
    IFS= read -r stamp < "$stamp_file" || true
  fi
  if [[ "$stamp" =~ ^[0-9]+$ ]] && (( stamp > 0 )); then
    now="$(date +%s)"
    age=$((now - stamp))
    if (( age > stale_seconds )); then
      value="[STALE ${age}s] ${value}"
    fi
  fi
  printf '%s' "$value"
}

drawer_label() {
  local raw="$1"
  raw="${raw//\"/}"
  raw="${raw//\'/}"
  case "$raw" in
    1|TOP_DRAWER|top|TOP) printf '%s' '1번 (상단 서랍)' ;;
    2|BOTTOM_DRAWER|bottom|BOTTOM) printf '%s' '2번 (하단 서랍)' ;;
    *) printf '%s' "$raw" ;;
  esac
}

topics=(
  'selected_drawer|/drawer_task/selected_id'
  'mission_phase|/drawer_task/phase'
  'drawer_open_ratio|/drawer_task/open_ratio'
  'pick_state|/drawer_task/pick_state'
  'grasp_state|/drawer_task/grasp_state'
  'object_state|/drawer_task/object_state'
  'task_status|/drawer_task/status'
  'rl_status|/rl_control/status'
  'vision_status|/target/aruco_status'
  'target_valid|/target/valid'
  'leader_task|/leader/task_state'
  'cargo_state|/leader/cargo_state'
  'follower_ready|/follower/ready'
)

for entry in "${topics[@]}"; do
  IFS='|' read -r key topic <<< "$entry"
  watch_topic "$key" "$topic" &
  child_pids+=("$!")
done

resolved_urdf="${runtime_dir}/leader.urdf"
python3 "$description_exporter" --output "$resolved_urdf"

# Always use this display profile, even while hardware publishes its description.
# Consume real /joint_states but keep preview TF/description off the control graph.
ros2 run robot_state_publisher robot_state_publisher "$resolved_urdf" \
  --ros-args -r __node:=leader_digital_twin_state_publisher \
  -r robot_description:=/leader_digital_twin/robot_description \
  -r /tf:=/leader_digital_twin/tf \
  -r /tf_static:=/leader_digital_twin/tf_static \
  > "${runtime_dir}/robot_state_publisher.log" 2>&1 &
child_pids+=("$!")

if "$start_rviz"; then
  if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    printf '%s\n' 'No graphical display detected; continuing with dashboard only.' >&2
    start_rviz=false
  elif [[ ! -f "$rviz_source" ]]; then
    printf 'RViz config not found: %s; continuing with dashboard only.\n' "$rviz_source" >&2
    start_rviz=false
  elif ! command -v rviz2 >/dev/null 2>&1; then
    printf '%s\n' 'rviz2 is unavailable; continuing with dashboard only.' >&2
    start_rviz=false
  fi
fi

if "$start_rviz"; then
  rviz_config="${runtime_dir}/leader_digital_twin.rviz"
  sed 's#/leader_preview/robot_description#/leader_digital_twin/robot_description#g' "$rviz_source" > "$rviz_config"
  rviz2 -d "$rviz_config" \
    --ros-args -r /tf:=/leader_digital_twin/tf \
    -r /tf_static:=/leader_digital_twin/tf_static \
    > "${runtime_dir}/rviz.log" 2>&1 &
  child_pids+=("$!")
fi

if "$demo_mode"; then
  ros2 topic pub -r 2 /drawer_task/selected_id std_msgs/msg/Int32 '{data: 1}' \
    > "${runtime_dir}/demo_selected_drawer.log" 2>&1 &
  child_pids+=("$!")
  ros2 topic pub -r 2 /drawer_task/phase std_msgs/msg/String '{data: DRAWER_PICK}' \
    > "${runtime_dir}/demo_phase.log" 2>&1 &
  child_pids+=("$!")
  ros2 topic pub -r 2 /drawer_task/open_ratio std_msgs/msg/Float32 '{data: 0.82}' \
    > "${runtime_dir}/demo_open_ratio.log" 2>&1 &
  child_pids+=("$!")
  ros2 topic pub -r 2 /drawer_task/pick_state std_msgs/msg/String \
    '{data: EEF camera target -> PPO joint reach}' \
    > "${runtime_dir}/demo_pick_state.log" 2>&1 &
  child_pids+=("$!")
  ros2 topic pub -r 2 /drawer_task/grasp_state std_msgs/msg/String '{data: GRASPED}' \
    > "${runtime_dir}/demo_grasp_state.log" 2>&1 &
  child_pids+=("$!")
  ros2 topic pub -r 2 /drawer_task/object_state std_msgs/msg/String '{data: HELD_IN_GRIPPER}' \
    > "${runtime_dir}/demo_object_state.log" 2>&1 &
  child_pids+=("$!")
  ros2 topic pub -r 2 /drawer_task/status std_msgs/msg/String \
    '{data: extracting object from top drawer}' \
    > "${runtime_dir}/demo_task_status.log" 2>&1 &
  child_pids+=("$!")
  ros2 topic pub -r 2 /follower/ready std_msgs/msg/Bool '{data: true}' \
    > "${runtime_dir}/demo_follower_ready.log" 2>&1 &
  child_pids+=("$!")
fi

render_dashboard() {
  local selected_drawer
  selected_drawer="$(state_value selected_drawer 'WAITING /drawer_task/selected_id')"

  printf '\033[2J\033[H'
  printf '%s\n' '======================================================================'
  printf '%s\n' ' OMX LEADER DIGITAL TWIN — READ ONLY'
  printf '%s\n' '======================================================================'
  printf '%-22s %s\n' 'Model release' "$model_version"
  printf '%-22s %s\n' 'ROS domain' "${ROS_DOMAIN_ID:-0}"
  printf '%-22s %s\n' 'Leader base' 'LOCKED — wheel action disabled'
  printf '%-22s %s\n' 'RViz' "$(if "$start_rviz"; then printf RUNNING; else printf DISABLED; fi)"
  printf '%s\n' '----------------------------------------------------------------------'
  printf '%-22s %s\n' '선택 서랍' "$(drawer_label "$selected_drawer")"
  printf '%-22s %s\n' '현재 작업 단계' "$(state_value mission_phase 'WAITING /drawer_task/phase')"
  printf '%-22s %s\n' '서랍 개방률' "$(state_value drawer_open_ratio 'WAITING /drawer_task/open_ratio')"
  printf '%-22s %s\n' 'Pick 접근 방식' "$(state_value pick_state 'WAITING /drawer_task/pick_state')"
  printf '%-22s %s\n' '그리퍼/파지 상태' "$(state_value grasp_state 'WAITING /drawer_task/grasp_state')"
  printf '%-22s %s\n' '물체 상태' "$(state_value object_state 'WAITING /drawer_task/object_state')"
  printf '%-22s %s\n' '팔로워 준비' "$(state_value follower_ready 'WAITING /follower/ready')"
  printf '%s\n' '----------------------------------------------------------------------'
  printf '%-22s %s\n' '작업 관리자' "$(state_value task_status 'WAITING /drawer_task/status')"
  printf '%-22s %s\n' 'RL 제어 상세' "$(state_value rl_status 'WAITING /rl_control/status')"
  printf '%-22s %s\n' '비전 상태' "$(state_value vision_status 'WAITING /target/aruco_status')"
  printf '%-22s %s\n' '인식 목표 유효' "$(state_value target_valid 'WAITING /target/valid')"
  printf '%-22s %s\n' '리더 상위 상태' "$(state_value leader_task 'WAITING /leader/task_state')"
  printf '%-22s %s\n' '화물 상태' "$(state_value cargo_state 'WAITING /leader/cargo_state')"
  printf '%s\n' '----------------------------------------------------------------------'
  printf '%s\n' 'Ctrl+C: digital twin 종료 | 이 프로세스는 로봇 제어 명령을 발행하지 않음'
}

while true; do
  render_dashboard
  sleep "$refresh_seconds"
done
