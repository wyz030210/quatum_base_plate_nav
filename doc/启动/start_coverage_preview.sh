#不用底盘，单独调试全覆盖

#!/usr/bin/env bash
set -e

preview_ws_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$preview_ws_dir/install/setup.bash" ]]; then
  echo "Build nav2_mission, opennav_coverage and wheeltec_nav2 first." >&2
  exit 1
fi
source "$preview_ws_dir/install/setup.bash"
export ROS_DOMAIN_ID="${COVERAGE_PREVIEW_DOMAIN_ID:-94}"
export ROS_LOCALHOST_ONLY=1

exec ros2 launch wheeltec_nav2 coverage_preview.launch.py \
  "params_file:=$preview_ws_dir/src/wheeltec_nav2/param/coverage_params.yaml" \
  "rviz_config:=$preview_ws_dir/src/wheeltec_nav2/rviz/coverage_preview.rviz" "$@"
