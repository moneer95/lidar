#!/bin/bash
# Start TurtleBot3 bringup + YDLidar + selected controller in one command.
#
# Usage:
#   ./start_robot.sh
#   ./start_robot.sh [teleop|nav] [burger] [tb3_nav_params_file]
# Examples:
#   ./start_robot.sh
#   ./start_robot.sh teleop
#   ./start_robot.sh nav burger /path/to/your_tb3_nav.yaml
#
# Optional environment overrides:
#   YDLIDAR_WS=~/ros2_ws
#   LIDAR_WS=~/Desktop/lidar
#   LIDAR_PARAMS_FILE=~/Desktop/lidar/config/ydlidar_g4_params.yaml
#   TB3_NAV_PARAMS_FILE=~/Desktop/lidar/config/tb3_nav_template.yaml
#   SHOW_PLOT=1

set -euo pipefail

MODE="${1:-nav}"
MODEL="${2:-burger}"
YDLIDAR_WS="${YDLIDAR_WS:-$HOME/ros2_ws}"
LIDAR_WS="${LIDAR_WS:-$HOME/Desktop/lidar}"
LIDAR_PARAMS_FILE="${LIDAR_PARAMS_FILE:-$LIDAR_WS/config/ydlidar_g4_params.yaml}"
TB3_NAV_PARAMS_FILE="${TB3_NAV_PARAMS_FILE:-$LIDAR_WS/config/tb3_nav_template.yaml}"
CLI_TB3_NAV_PARAMS_FILE="${3:-}"

source_safe() {
  # ROS setup scripts may reference unset vars; temporarily disable nounset.
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

stop_group() {
  local pid="$1"
  [[ -z "$pid" ]] && return 0
  # Negative PID targets the process group.
  kill -TERM -- "-$pid" 2>/dev/null || true
}

ensure_tb3_nav_executable() {
  if ros2 pkg executables tb3_tools 2>/dev/null | grep -q "tb3_nav"; then
    return 0
  fi

  echo "tb3_nav executable not found. Rebuilding tb3_tools ..."
  (
    cd "$LIDAR_WS"
    source_safe /opt/ros/humble/setup.bash
    colcon build --packages-select tb3_tools --symlink-install
  )
}

get_lidar_param_or_default() {
  local key="$1"
  local default_value="$2"
  python3 - "$LIDAR_PARAMS_FILE" "$key" "$default_value" <<'PY'
import re
import sys

path, key, default = sys.argv[1], sys.argv[2], sys.argv[3]
pattern = re.compile(r'^\s*' + re.escape(key) + r'\s*:\s*("?)([-+0-9.eE]+)\1\s*$')

try:
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            m = pattern.match(line)
            if m:
                print(m.group(2))
                raise SystemExit(0)
except Exception:
    pass

print(default)
PY
}

case "$MODE" in
  teleop|nav) ;;
  *)
    echo "Invalid mode: $MODE"
    echo "Usage: ./start_robot.sh [teleop|nav] [burger] [tb3_nav_params_file]"
    exit 1
    ;;
esac

if [[ "$MODEL" != "burger" ]]; then
  echo "Invalid model: $MODEL"
  echo "Only burger is supported in this workspace."
  exit 1
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Missing /opt/ros/humble/setup.bash. Install/source ROS2 Humble first."
  exit 1
fi

if [[ ! -f "$YDLIDAR_WS/install/setup.bash" ]]; then
  echo "Missing $YDLIDAR_WS/install/setup.bash"
  if [[ ! -d "$YDLIDAR_WS" ]]; then
    echo "YDLidar workspace directory not found: $YDLIDAR_WS"
    echo "Set YDLIDAR_WS to your driver workspace path (the one containing src/ydlidar_ros2_driver)."
    exit 1
  fi
  echo "Auto-building ydlidar workspace at $YDLIDAR_WS ..."
  (
    cd "$YDLIDAR_WS"
    source_safe /opt/ros/humble/setup.bash
    colcon build --symlink-install
  )
  if [[ ! -f "$YDLIDAR_WS/install/setup.bash" ]]; then
    echo "YDLidar workspace build finished but install/setup.bash is still missing."
    echo "Make sure ydlidar_ros2_driver exists under $YDLIDAR_WS/src and build succeeds."
    exit 1
  fi
fi

if [[ ! -f "$LIDAR_WS/install/setup.bash" ]]; then
  echo "Missing $LIDAR_WS/install/setup.bash"
  echo "Auto-building lidar workspace at $LIDAR_WS ..."
  (
    cd "$LIDAR_WS"
    source_safe /opt/ros/humble/setup.bash
    colcon build --symlink-install
  )
fi

if [[ -n "$CLI_TB3_NAV_PARAMS_FILE" ]]; then
  TB3_NAV_PARAMS_FILE="$CLI_TB3_NAV_PARAMS_FILE"
fi

if [[ ! -f "$LIDAR_PARAMS_FILE" ]]; then
  echo "Missing lidar params file: $LIDAR_PARAMS_FILE"
  exit 1
fi

if [[ "$MODE" == "nav" ]]; then
  if [[ ! -f "$TB3_NAV_PARAMS_FILE" ]]; then
    echo "Missing TB3 nav params file: $TB3_NAV_PARAMS_FILE"
    exit 1
  fi
fi

source_safe /opt/ros/humble/setup.bash
source_safe "$YDLIDAR_WS/install/setup.bash"
source_safe "$LIDAR_WS/install/setup.bash"

export TURTLEBOT3_MODEL="$MODEL"
# Some turtlebot3 bringup launch files require LDS_MODEL env.
export LDS_MODEL="${LDS_MODEL:-LDS-01}"

# Ensure latest nav executable exists after refactors.
ensure_tb3_nav_executable
source_safe "$LIDAR_WS/install/setup.bash"

echo "Starting TurtleBot3 bringup (model=$MODEL)..."
setsid ros2 launch turtlebot3_bringup robot.launch.py &
P_BRINGUP=$!

echo "Starting YDLidar driver (params: $LIDAR_PARAMS_FILE)..."
setsid ros2 launch ydlidar_ros2_driver ydlidar_launch.py params_file:="$LIDAR_PARAMS_FILE" &
P_LIDAR=$!

# Start live plot by default with borders matching LiDAR YAML limits.
if [[ "${SHOW_PLOT:-1}" != "0" ]]; then
  ANGLE_MIN_DEG="$(get_lidar_param_or_default angle_min 0.0)"
  ANGLE_MAX_DEG="$(get_lidar_param_or_default angle_max 360.0)"
  RANGE_MAX_M="$(get_lidar_param_or_default range_max 0.0)"
  PLOT_FOV_DEG="$(python3 - "$ANGLE_MIN_DEG" "$ANGLE_MAX_DEG" <<'PY'
import sys
a0 = float(sys.argv[1])
a1 = float(sys.argv[2])
print(abs(a1 - a0))
PY
)"
  PLOT_CENTER_DEG="$(python3 - "$ANGLE_MIN_DEG" "$ANGLE_MAX_DEG" <<'PY'
import sys
a0 = float(sys.argv[1])
a1 = float(sys.argv[2])
print((a0 + a1) / 2.0)
PY
)"
  echo "Starting scan plot (FOV=$PLOT_FOV_DEG°, center=$PLOT_CENTER_DEG°, max=$RANGE_MAX_M m)..."
  setsid ros2 run lidar_tools scan_plot_node --ros-args \
    -p fov_degrees:="$PLOT_FOV_DEG" \
    -p fov_center_deg:="$PLOT_CENTER_DEG" \
    -p max_scan_range_m:="$RANGE_MAX_M" &
  P_PLOT=$!
else
  P_PLOT=""
fi

cleanup() {
  echo
  echo "Stopping background processes..."
  stop_group "$P_BRINGUP"
  stop_group "$P_LIDAR"
  stop_group "$P_PLOT"
  sleep 0.5
  kill -KILL -- "-$P_BRINGUP" 2>/dev/null || true
  kill -KILL -- "-$P_LIDAR" 2>/dev/null || true
  [[ -n "$P_PLOT" ]] && kill -KILL -- "-$P_PLOT" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Let bringup and lidar initialize before control node starts.
sleep 3

case "$MODE" in
  teleop)
    echo "Running arrow-key teleop..."
    ros2 run tb3_tools tb3_arrow_teleop
    ;;
  nav)
    echo "Running generic nav from params file: $TB3_NAV_PARAMS_FILE"
    ros2 run tb3_tools tb3_nav --ros-args --params-file "$TB3_NAV_PARAMS_FILE"
    ;;
esac
