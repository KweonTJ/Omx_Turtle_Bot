#!/usr/bin/env bash
set -eo pipefail
robot_workspace="${OMX_ROBOT_WORKSPACE:-/home/ktj/omx_turtle_ws}"
host_domain="${ROS_DOMAIN_ID:-16}"
source_domain=""
demo=false
fullscreen=false
transport=raw
extra_args=()
usage() {
  printf '%s\n' \
    '리더 픽앤플레이스 호스트 모니터 (읽기 전용)' \
    'Usage: run_leader_pick_place_monitor.sh [--demo] [--domain N] [--bridge-from N]' \
    '       [--compressed] [--fullscreen] [--workspace PATH] [--config PATH]' \
    '       [--bridge-config PATH]' \
    '기본: 이미 브릿지된 /leader_monitor/input/* 수신. 로봇 제어 명령 없음.' \
    '--demo: localhost/domain 223에서 합성 데이터. 실제 입력 및 브릿지와 분리.' \
    '--bridge-from N: 지정한 로봇 domain -> 호스트의 허용된 텔레메트리만 단방향 복사.'
}
while [[ $# -gt 0 ]]; do
  case "$1" in
    --demo) demo=true; shift ;;
    --fullscreen) fullscreen=true; shift ;;
    --compressed) transport=compressed; shift ;;
    --domain|--bridge-from|--workspace|--config|--bridge-config)
      [[ $# -ge 2 ]] || { printf '값 누락: %s\n' "$1" >&2; exit 2; }
      case "$1" in
        --domain) host_domain="$2" ;;
        --bridge-from) source_domain="$2" ;;
        --workspace) robot_workspace="$2" ;;
        --config) extra_args+=("config:=$2") ;;
        --bridge-config) extra_args+=("bridge_config:=$2") ;;
      esac
      shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) printf '알 수 없는 옵션: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done
if "$demo"; then
  [[ -z "$source_domain" ]] || { printf '%s\n' '--demo와 --bridge-from은 함께 사용 불가' >&2; exit 2; }
  host_domain=223
  export ROS_LOCALHOST_ONLY=1
fi
for monitor_domain in "$host_domain" ${source_domain:+"$source_domain"}; do
  [[ "$monitor_domain" =~ ^[0-9]{1,3}$ ]] && (( 10#$monitor_domain <= 232 )) || {
    printf '%s\n' 'domain은 0~232의 정수여야 합니다.' >&2; exit 2;
  }
done
[[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]] || { printf '%s\n' '호스트의 그래픽 세션에서 실행하세요.' >&2; exit 1; }
[[ -f "$robot_workspace/install/setup.bash" ]] || { printf '%s\n' '먼저 omx_turtle_ws를 빌드하세요.' >&2; exit 1; }
source /opt/ros/humble/setup.bash
source "$robot_workspace/install/setup.bash"
ros2 pkg prefix omx_leader_monitor >/dev/null || { printf '%s\n' 'omx_leader_monitor 패키지 빌드가 필요합니다.' >&2; exit 1; }
export ROS_DOMAIN_ID="$host_domain"
# RViz/Ogre uses X11; XWayland is supported. Do not change display/auth settings.
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$(mktemp -d /tmp/omx_leader_monitor_logs.XXXXXX)}"
start_bridge=false
if [[ -n "$source_domain" ]]; then
  start_bridge=true
  extra_args+=("source_domain:=$source_domain")
fi
exec ros2 launch omx_leader_monitor leader_monitor.launch.py \
  "host_domain:=$host_domain" "start_bridge:=$start_bridge" \
  "demo:=$demo" "fullscreen:=$fullscreen" "image_transport:=$transport" "${extra_args[@]}"
