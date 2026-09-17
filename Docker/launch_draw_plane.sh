#!/bin/bash
# Copyright 2026 Robotic Systems Integration, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Launch the draw-plane operator stack inside the ros2_rapidcode container,
# from the host: move_group, the draw-plane bridge, MoveIt Servo, and RViz.
# Mirrors Docker/RUNBOOK.md sections 8, 10 and 13, plus the draw-plane bridge.
#
# The hardware/controller bringup (rapidcode_bringup elfin5.launch.py) is NOT
# started here on purpose: it owns the RMP and, on real hardware, the amps, and
# its use_hardware choice must stay a deliberate act. Start it yourself first
# (runbook section 2 step 1); this script refuses to run without it.
#
# Every component is skipped if it is already running, so re-running this after
# a partial start is safe.
#
# Usage:
#   ./launch_draw_plane.sh                    # move_group + bridge + servo + rviz
#   ./launch_draw_plane.sh --no-rviz          # headless host, or an RT-sensitive run
#   ./launch_draw_plane.sh --no-servo         # batch drawing only, no Live route
#   ./launch_draw_plane.sh --display :1       # override $DISPLAY for RViz
#   ./launch_draw_plane.sh --rviz-on-hardware # allow RViz while bringup holds real hardware
#   ./launch_draw_plane.sh --stop             # stop these four, leave the bringup up
set -u -o pipefail

CONTAINER=ros2_rapidcode
SETUP='source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash'

WANT_SERVO=1
WANT_RVIZ=1
RVIZ_ON_HARDWARE=0
DISPLAY_VALUE="${DISPLAY:-:0}"
MODE=start

# ---- plumbing ---------------------------------------------------------------

die() {
  echo "error: $*" >&2
  exit 1
}

note() {
  echo "  $*"
}

# Run a command in the container with ROS and the workspace overlay sourced.
in_container() {
  docker exec "$CONTAINER" bash -c "$SETUP && $1"
}

# Start a long-lived component, detached, with its output in a container log.
start_detached() {
  local command="$1" logfile="$2"
  docker exec -d "$CONTAINER" bash -c "$SETUP && $command > $logfile 2>&1"
}

process_running() {
  docker exec "$CONTAINER" pgrep -f "$1" >/dev/null 2>&1
}

node_present() {
  in_container "ros2 node list 2>/dev/null" | grep -qx "$1"
}

log_contains() {
  docker exec "$CONTAINER" grep -qF "$2" "$1" 2>/dev/null
}

# Poll a predicate until it holds. wait_until <seconds> <description> <cmd...>
wait_until() {
  local limit="$1" description="$2"
  shift 2
  local waited=0
  until "$@"; do
    if [ "$waited" -ge "$limit" ]; then
      printf '\r'
      return 1
    fi
    sleep 1
    waited=$((waited + 1))
    printf '\r  waiting for %s (%ss)' "$description" "$waited"
  done
  printf '\r  %s is up%20s\n' "$description" ''
  return 0
}

# Show why a component never came up, then give up.
fail_with_log() {
  local description="$1" logfile="$2"
  printf '\n'
  echo "error: $description did not come up. Last lines of $logfile:" >&2
  docker exec "$CONTAINER" tail -n 20 "$logfile" 2>/dev/null >&2 ||
    echo "  ($logfile is empty or missing)" >&2
  exit 1
}

# ---- preflight --------------------------------------------------------------

require_container() {
  local state
  state="$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)"
  [ "$state" = "true" ] ||
    die "container $CONTAINER is not running. Start the whole stack with:
    ./run.sh up phantom      (or: ./run.sh up hardware)"
}

require_bringup() {
  process_running ros2_control_node ||
    die "the bringup is not running (no ros2_control_node in $CONTAINER).
Start it first, choosing phantom or real hardware deliberately:
    ./run.sh up phantom      (or: ./run.sh up hardware)
which starts the bringup and then runs this script."
}

bringup_uses_hardware() {
  docker exec "$CONTAINER" pgrep -af 'elfin5.launch.py' 2>/dev/null |
    grep -q 'use_hardware:=true'
}

# RViz needs a live X server on the host and containers authorized to use it.
check_display() {
  local socket="/tmp/.X11-unix/X${DISPLAY_VALUE#*:}"
  if [ ! -S "$socket" ]; then
    echo "  no X server at $DISPLAY_VALUE ($socket missing). Skipping RViz."
    echo "  To get one (runbook section 3), at the physical monitor:"
    echo "      sudo systemctl start lightdm"
    echo "      xhost +local:        # in a terminal inside that session"
    return 1
  fi
  if [ "$RVIZ_ON_HARDWARE" -eq 0 ] && bringup_uses_hardware; then
    die "the bringup is running with use_hardware:=true. The desktop perturbs
RT determinism (RVIZ_SETUP.md), so RViz is refused here. Pass
--rviz-on-hardware to override, or --no-rviz to skip it."
  fi
  return 0
}

# ---- components -------------------------------------------------------------

start_move_group() {
  echo "move_group (planning framework)"
  if process_running 'move_group'; then
    note "already running"
    return
  fi
  start_detached 'ros2 launch rapidcode_moveit_config move_group.launch.py' \
    /tmp/movegroup.log
  wait_until 90 move_group log_contains /tmp/movegroup.log \
    'You can start planning now!' ||
    fail_with_log move_group /tmp/movegroup.log
}

start_bridge() {
  echo "draw-plane bridge"
  if process_running 'lib/rapidcode_draw_plane/bridge'; then
    note "already running"
    return
  fi
  start_detached 'ros2 run rapidcode_draw_plane bridge' /tmp/bridge.log
  wait_until 60 bridge log_contains /tmp/bridge.log 'bridge up: ws://' ||
    fail_with_log bridge /tmp/bridge.log
  in_container "grep -m1 'bridge up: ws://' /tmp/bridge.log" | sed 's/^/  /'
}

start_servo() {
  echo "MoveIt Servo (Live route)"
  if process_running servo_node; then
    note "already running"
    return
  fi
  start_detached 'ros2 launch rapidcode_moveit_config servo.launch.py' \
    /tmp/servo.log
  wait_until 60 servo_node node_present /servo_node ||
    fail_with_log servo_node /tmp/servo.log
  note "the bridge unpauses it and selects POSE mode on Live activation"
}

start_rviz() {
  echo "RViz (robot model + planning displays)"
  if process_running rviz2; then
    note "already running"
    return
  fi
  start_detached "export DISPLAY=$DISPLAY_VALUE && \
ros2 launch rapidcode_moveit_config moveit_rviz.launch.py" /tmp/rviz.log
  # RViz takes a while to build its scene; the process surviving startup is the
  # signal, then look at the monitor.
  wait_until 60 rviz2 process_running rviz2 ||
    fail_with_log rviz2 /tmp/rviz.log
  note "window is on $DISPLAY_VALUE at the physical monitor"
}

# ---- stop -------------------------------------------------------------------

stop_stack() {
  require_container
  echo "Stopping the draw-plane stack (the bringup is left alone)."
  # Narrow patterns, reverse start order. ros2_control_node is never matched.
  local pattern
  for pattern in rviz2 servo_node 'lib/rapidcode_draw_plane/bridge' move_group; do
    if process_running "$pattern"; then
      docker exec "$CONTAINER" pkill -f "$pattern" >/dev/null 2>&1
      note "stopped $pattern"
    else
      note "$pattern was not running"
    fi
  done
  echo
  echo "The bringup still holds the RMP. To stop that too:"
  echo "    docker exec $CONTAINER pkill -f ros2_control_node"
}

# ---- summary ----------------------------------------------------------------

print_summary() {
  local address
  address="$(hostname -I 2>/dev/null | awk '{print $1}')"
  echo
  echo "Stack up. Logs (inside the container):"
  echo "    docker exec $CONTAINER tail -f /tmp/bridge.log"
  echo "    /tmp/movegroup.log  /tmp/bridge.log  /tmp/servo.log  /tmp/rviz.log"
  echo
  echo "Connect the drawing client to the bridge:"
  echo "    ./run.sh client                                   (this host)"
  echo "    ./run.sh client ws://${address:-<this-host>}:8765      (another Linux machine)"
  echo "    run_client.bat ws://${address:-<this-host>}:8765       (Windows, from the repo root)"
  echo
  echo "Stop everything with:  ./run.sh down"
}

# ---- main -------------------------------------------------------------------

while [ $# -gt 0 ]; do
  case "$1" in
    --no-servo) WANT_SERVO=0 ;;
    --no-rviz) WANT_RVIZ=0 ;;
    --rviz-on-hardware) RVIZ_ON_HARDWARE=1 ;;
    --display)
      [ $# -ge 2 ] || die "--display needs a value, e.g. --display :0"
      DISPLAY_VALUE="$2"
      shift
      ;;
    --stop) MODE=stop ;;
    -h | --help)
      # Print the header comment block, minus the shebang, and stop at the code.
      awk 'NR == 1 { next } /^#/ { sub(/^# ?/, ""); print; next } { exit }' "$0"
      exit 0
      ;;
    *) die "unknown option: $1 (try --help)" ;;
  esac
  shift
done

if [ "$MODE" = stop ]; then
  stop_stack
  exit 0
fi

require_container
require_bringup

start_move_group
start_bridge
[ "$WANT_SERVO" -eq 1 ] && start_servo
if [ "$WANT_RVIZ" -eq 1 ] && check_display; then
  start_rviz
fi

print_summary
