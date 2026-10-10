#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 默认取本工作区内的地图，不再硬编码 /home/car/...（那台机器的路径在本机不存在）。
# 仍可用 INDOOR_MAP 覆盖。
map_file="${INDOOR_MAP:-${workspace_dir}/src/wheeltec_nav2/map/map.yaml}"

if [[ ! -f "${map_file}" ]]; then
  echo "Map YAML does not exist: ${map_file}" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u

# USE_COVERAGE=true 才会拉起覆盖触发链（覆盖动作桥接 + RViz 区域选择）。
# 默认 false：日常室内导航不启动这两个节点。
exec ros2 launch robot_bringup indoor_amcl.launch.py \
  map:="${map_file}" \
  use_coverage:="${USE_COVERAGE:-false}" \
  use_rviz:="${USE_RVIZ:-true}" \
  can_serial_port:="${CAN_SERIAL_PORT:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0}" \
  max_forward_speed:="${MAX_FORWARD_SPEED:-0.25}" \
  max_reverse_speed:="${MAX_REVERSE_SPEED:-0.10}" \
  max_yaw_rate:="${MAX_YAW_RATE:-0.50}" \
  max_linear_accel:="${MAX_LINEAR_ACCEL:-0.40}" \
  max_yaw_accel:="${MAX_YAW_ACCEL:-0.50}"
