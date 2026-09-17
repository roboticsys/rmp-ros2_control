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
# One entry point for the whole stack: build the image, bring up the robot and
# the draw-plane stack in the container, run the drawing client, stop it all.
#
# Usage:
#   ./run.sh build                      build the Docker image
#   ./run.sh up phantom [options]       start everything with phantom axes (no robot)
#   ./run.sh up hardware [options]      start everything on the real robot (asks first)
#   ./run.sh client [ws://host:port]    run the drawing surface client (default: this host)
#   ./run.sh status                     show what is running
#   ./run.sh logs [bringup|movegroup|bridge|servo|rviz]   follow a component log
#   ./run.sh shell                      open a shell in the container
#   ./run.sh down                       stop everything and remove the container
#
# Options for "up":
#   --no-rviz              skip RViz (headless host)
#   --no-servo             skip MoveIt Servo (batch drawing only, no Live mode)
#   --rviz-on-hardware     allow RViz while the real robot runs (perturbs RT timing)
#   --display :N           X display for RViz (default: $DISPLAY or :0)
#   --yes                  skip the hardware safety confirmation
#
# The RSI files you supply (the rmp .deb, rsi.lic, EtherCAT.xml) live in
# inputs/. See inputs/README.md.
set -u -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INPUTS="$ROOT/inputs"
COMPOSE=(docker compose -f "$ROOT/Docker/compose.yml")
CONTAINER=ros2_rapidcode
SETUP='source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash'
BRINGUP_LOG=/tmp/bringup.log
CLIENT_VENV="$ROOT/.venv-client"
CLIENT_PKG_DIR="$ROOT/src/rapidcode_draw_plane"

# ---- plumbing ---------------------------------------------------------------

die() {
  echo "error: $*" >&2
  exit 1
}

note() {
  echo "  $*"
}

usage() {
  awk 'NR == 1 { next } /^#/ { sub(/^# ?/, ""); print; next } { exit }' "$0"
}

in_container() {
  docker exec "$CONTAINER" bash -c "$SETUP && $1"
}

start_detached() {
  local command="$1" logfile="$2"
  docker exec -d "$CONTAINER" bash -c "$SETUP && $command > $logfile 2>&1"
}

process_running() {
  docker exec "$CONTAINER" pgrep -f "$1" >/dev/null 2>&1
}

container_running() {
  [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = "true" ]
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
}

fail_with_log() {
  local description="$1" logfile="$2"
  printf '\n'
  echo "error: $description did not come up. Last lines of $logfile:" >&2
  docker exec "$CONTAINER" tail -n 20 "$logfile" 2>/dev/null >&2 ||
    echo "  ($logfile is empty or missing)" >&2
  exit 1
}

# ---- inputs -----------------------------------------------------------------

deb_files() {
  find "$INPUTS" -maxdepth 1 -name 'rmp_*_amd64.deb' 2>/dev/null
}

require_deb() {
  local count
  count="$(deb_files | wc -l)"
  [ "$count" -ge 1 ] ||
    die "no rmp package found. Put one rmp_*_amd64.deb in $INPUTS/ (see inputs/README.md)."
  [ "$count" -eq 1 ] ||
    die "more than one rmp_*_amd64.deb in $INPUTS/. Keep exactly one."
}

require_input() {
  local name="$1" why="$2"
  [ -f "$INPUTS/$name" ] ||
    die "$INPUTS/$name is missing. $why (see inputs/README.md)."
}

# ---- build ------------------------------------------------------------------

cmd_build() {
  require_deb
  echo "Building the image with $(basename "$(deb_files)")"
  "${COMPOSE[@]}" build
}

# ---- up ---------------------------------------------------------------------

bringup_mode() {
  # "phantom", "hardware", or "" when the bringup is not running.
  local args
  args="$(docker exec "$CONTAINER" pgrep -af 'elfin5.launch.py' 2>/dev/null)" || return
  case "$args" in
    *use_hardware:=true*) echo hardware ;;
    *elfin5.launch.py*) echo phantom ;;
  esac
}

clear_shared_memory() {
  # Stale /dev/shm/RSI.* from a hard kill makes the next rmp start segfault.
  # Only safe with no rmp running.
  if process_running ros2_control_node || process_running '^/rsi/rmp'; then
    return
  fi
  docker exec "$CONTAINER" bash -c 'rm -f /dev/shm/RSI.*'
}

confirm_hardware() {
  cat <<'EOF'

  You are about to power motion on the REAL robot. Confirm all of these:

    [ ] Robot powered and its EtherCAT link connected on the dedicated NIC.
    [ ] inputs/EtherCAT.xml matches the physical network topology.
    [ ] Joint-to-axis mapping and origins in elfin5_hardware.yaml verified for this robot.
    [ ] STO / E-stop within reach and tested.
    [ ] Workspace cleared and restricted.
    [ ] A second person present.

  Details: Docker/RUNBOOK.md section 7.1.

EOF
  local answer
  read -r -p "  Type yes to continue: " answer
  [ "$answer" = "yes" ] || die "aborted."
}

start_bringup() {
  local mode="$1" use_hardware=false
  [ "$mode" = hardware ] && use_hardware=true

  echo "bringup: ros2_control + controllers ($mode)"
  local running
  running="$(bringup_mode)"
  if [ -n "$running" ]; then
    [ "$running" = "$mode" ] ||
      die "the bringup is already running in $running mode. Run ./run.sh down first."
    note "already running"
    return
  fi

  clear_shared_memory
  start_detached "ros2 launch rapidcode_bringup elfin5.launch.py use_hardware:=$use_hardware" \
    "$BRINGUP_LOG"
  wait_until 120 "the trajectory controller" bringup_ready ||
    fail_with_log "the bringup" "$BRINGUP_LOG"
}

bringup_ready() {
  in_container "ros2 control list_controllers 2>/dev/null" |
    grep -q 'rapidcode_passthrough_trajectory_controller.*active'
}

cmd_up() {
  local mode="${1:-}"
  shift || true
  case "$mode" in
    phantom | hardware) ;;
    *) die "usage: ./run.sh up phantom|hardware [options]" ;;
  esac

  local launch_args=() confirm=1
  while [ $# -gt 0 ]; do
    case "$1" in
      --no-rviz | --no-servo | --rviz-on-hardware) launch_args+=("$1") ;;
      --display)
        [ $# -ge 2 ] || die "--display needs a value, e.g. --display :0"
        launch_args+=("$1" "$2")
        shift
        ;;
      --yes) confirm=0 ;;
      *) die "unknown option: $1 (try --help)" ;;
    esac
    shift
  done

  require_input rsi.lic "The RapidCode license is needed to start rmp"
  if [ "$mode" = hardware ]; then
    require_input EtherCAT.xml "The ENI is needed to start the EtherCAT network"
    [ "$confirm" -eq 0 ] || confirm_hardware
  fi

  if ! container_running; then
    echo "container"
    "${COMPOSE[@]}" up -d || die "docker compose up failed."
  fi

  start_bringup "$mode"
  "$ROOT/Docker/launch_draw_plane.sh" ${launch_args[@]+"${launch_args[@]}"}
}

# ---- client -----------------------------------------------------------------

have_python_module() {
  "$1" -c "import $2" >/dev/null 2>&1
}

apt_install() {
  # System packages need root. The script itself must NOT run under sudo:
  # the virtual environment and the client window belong to your user. So
  # either sudo works here, or you run the one apt-get line yourself.
  local manual="sudo apt-get install -y $*"
  command -v apt-get >/dev/null 2>&1 ||
    die "the client needs these system packages: $*
Install them with your system package manager, then re-run ./run.sh client."
  if [ "$(id -u)" -eq 0 ]; then
    die "do not run ./run.sh client as root. As your normal user, first run:
    $manual
then re-run ./run.sh client."
  fi
  command -v sudo >/dev/null 2>&1 ||
    die "the client needs these system packages: $*
sudo is not installed, so as root run:
    apt-get install -y $*
then re-run ./run.sh client as your normal user."
  echo "Installing $* (sudo may ask for your password)"
  sudo apt-get install -y "$@" ||
    die "could not install $*.
If sudo refused you, ask an administrator to run:
    $manual
then re-run ./run.sh client."
}

prepare_client() {
  command -v python3 >/dev/null 2>&1 || die "python3 is not installed."

  # tkinter and venv cannot come from pip; they are system packages.
  local system_pkgs=()
  have_python_module python3 tkinter || system_pkgs+=(python3-tk)
  have_python_module python3 venv && have_python_module python3 ensurepip ||
    system_pkgs+=(python3-venv)
  [ ${#system_pkgs[@]} -eq 0 ] || apt_install "${system_pkgs[@]}"

  # Everything else goes in a private virtual environment that can still see
  # the system tkinter.
  if [ ! -x "$CLIENT_VENV/bin/python" ]; then
    echo "Creating the client's virtual environment in .venv-client/"
    python3 -m venv --system-site-packages "$CLIENT_VENV" ||
      die "could not create the virtual environment."
  fi
  local py="$CLIENT_VENV/bin/python"
  if ! have_python_module "$py" websockets; then
    echo "Installing the websockets package"
    "$py" -m pip install --quiet websockets || die "pip install websockets failed."
  fi
  if ! have_python_module "$py" sv_ttk; then
    echo "Installing the optional sv-ttk theme"
    "$py" -m pip install --quiet sv-ttk ||
      note "sv-ttk did not install; the client will use the native ttk theme."
  fi
}

cmd_client() {
  [ -f "$CLIENT_PKG_DIR/rapidcode_draw_plane/surface_client.py" ] ||
    die "client source not found under $CLIENT_PKG_DIR."
  prepare_client
  echo "Starting the drawing surface client ${1:-(ws://127.0.0.1:8765)}"
  PYTHONPATH="$CLIENT_PKG_DIR${PYTHONPATH:+:$PYTHONPATH}" \
    exec "$CLIENT_VENV/bin/python" -m rapidcode_draw_plane.surface_client "$@"
}

# ---- status / logs / shell --------------------------------------------------

cmd_status() {
  if ! container_running; then
    echo "container $CONTAINER: not running"
    return
  fi
  echo "container $CONTAINER: running"
  local mode
  mode="$(bringup_mode)"
  echo "bringup: ${mode:-not running}"
  local pattern label
  for pattern in move_group 'lib/rapidcode_draw_plane/bridge' servo_node rviz2; do
    case "$pattern" in
      *bridge*) label=bridge ;;
      *) label="$pattern" ;;
    esac
    if process_running "$pattern"; then
      echo "$label: running"
    else
      echo "$label: not running"
    fi
  done
  if process_running 'lib/rapidcode_draw_plane/bridge'; then
    local address
    address="$(hostname -I 2>/dev/null | awk '{print $1}')"
    echo "client address: ws://${address:-<this-host>}:8765"
  fi
}

cmd_logs() {
  container_running || die "container $CONTAINER is not running."
  local name="${1:-bridge}"
  case "$name" in
    bringup | movegroup | bridge | servo | rviz) ;;
    *) die "unknown log: $name (bringup, movegroup, bridge, servo, rviz)" ;;
  esac
  exec docker exec "$CONTAINER" tail -n 50 -f "/tmp/$name.log"
}

cmd_shell() {
  container_running || die "container $CONTAINER is not running. Start it with ./run.sh up."
  exec docker exec -it "$CONTAINER" bash
}

# ---- down -------------------------------------------------------------------

bringup_gone() {
  ! process_running ros2_control_node
}

cmd_down() {
  if container_running; then
    "$ROOT/Docker/launch_draw_plane.sh" --stop
    if process_running ros2_control_node; then
      # SIGINT the launch so on_deactivate/on_cleanup run: motion aborts, amps
      # disable, the network shuts down, and rmp releases shared memory.
      echo "Stopping the bringup"
      docker exec "$CONTAINER" pkill -INT -f 'elfin5.launch.py' >/dev/null 2>&1
      wait_until 30 "bringup shutdown" bringup_gone ||
        echo "  warning: ros2_control_node is still running; compose down will stop it."
    fi
    clear_shared_memory
  fi
  "${COMPOSE[@]}" down
}

# ---- main -------------------------------------------------------------------

require_docker() {
  command -v docker >/dev/null 2>&1 || die "docker is not installed."
}

case "${1:-}" in
  build) require_docker; cmd_build ;;
  up) require_docker; shift; cmd_up "$@" ;;
  client) shift; cmd_client "$@" ;;
  status) require_docker; cmd_status ;;
  logs) require_docker; shift; cmd_logs "$@" ;;
  shell) require_docker; cmd_shell ;;
  down) require_docker; cmd_down ;;
  -h | --help | "") usage ;;
  *) die "unknown command: $1 (try --help)" ;;
esac
