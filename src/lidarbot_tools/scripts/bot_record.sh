#!/usr/bin/env bash
#
# Запись основных топиков стадии «Минимум» в ros2 bag.
#
# Использование:
#   bot record sensors
#   bot record sensors /path/to/bag/dir

set -o pipefail

ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${HOME}/lidarbot_ws/install/setup.bash"

if [ -f "${ROS_SETUP}" ]; then
    # shellcheck disable=SC1090
    source "${ROS_SETUP}"
fi

if [ -f "${WORKSPACE_SETUP}" ]; then
    # shellcheck disable=SC1090
    source "${WORKSPACE_SETUP}"
fi

BAG_DIR="${1:-${HOME}/lidarbot_ws/bags}"
mkdir -p "${BAG_DIR}"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
BAG_NAME="${BAG_DIR}/lidarbot_sensors_${TIMESTAMP}"

TOPICS=(
    /scan
    /imu/data
    /camera/image_raw/compressed
    # /pan_tilt_state
    /odom_raw
    /cmd_vel
    /system_status
    /diagnostics
)

echo "Запись ros2 bag:"
echo "  ${BAG_NAME}"
echo ""
echo "Топики:"
for topic in "${TOPICS[@]}"; do
    echo "  ${topic}"
done
echo ""
echo "Для остановки нажмите Ctrl+C."

ros2 bag record -o "${BAG_NAME}" "${TOPICS[@]}"
