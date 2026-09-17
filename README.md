# rmp-ros2_control

ROS 2 (Jazzy) integration of the RSI RapidCode / RMP motion controller, with a
draw-plane operator product on top. A `ros2_control` hardware plugin maps joints
onto RapidCode axes, a passthrough trajectory controller streams MoveIt output
to it, and a drawing surface client lets an operator draw on a plane that the
robot then traces.

Everything ROS-side runs in one Docker container. The drawing client is a plain
Python program that runs on any machine, Windows or Linux, with no ROS install.

See it in action in a video on the [RSI blog](https://www.roboticsys.com/blog/ros2-control-ethercat-robot-arm).

Most of this repo was written with assistance from Claude Fable 5.1 as an
experiment for LLM usage with RMP.

## Before you build

You need three files from RSI that are not in this repository. Put them in
`inputs/`:

| File | Needed for |
|---|---|
| `rmp_*_amd64.deb` | Building the image. Provides RapidCode. Exactly one. |
| `rsi.lic` | Starting the container. Your RapidCode license. |
| `EtherCAT.xml` | Running on real hardware. The ENI for your EtherCAT topology. |

See `inputs/README.md` for details. The license and the ENI are read through a
bind mount, so replacing them needs no rebuild.

You also need to set two host-specific values in
`src/rapidcode_bringup/config/elfin5_hardware.yaml`, in the `hardware:` block.
`cpu_affinity` ships as `-1`, which the plugin refuses in both modes.
`primary_nic` ships empty, which the plugin refuses only on hardware.

| Key | Phantom | Hardware |
|---|---|---|
| `cpu_affinity` | Any core except 0. | An isolated core; see `Docker/RUNBOOK.md` section 2.2. |
| `primary_nic` | Leave empty. | The EtherCAT interface name from `ip link`. |

The file is copied into the image at build time. Set the values before you
build, and run `./run.sh build` again after any later change.

The host needs Docker with the compose plugin. Real-time behaviour on hardware
needs a real-time kernel; see `Docker/RUNBOOK.md`.

## Build

```bash
./run.sh build
```

## Run

All commands run from the repository root on the Linux host.

```bash
./run.sh up phantom            # no robot: RapidCode phantom axes
./run.sh up hardware           # the real robot, after a safety confirmation
```

Either command starts the container, the `ros2_control` bringup with the
controllers, MoveIt `move_group`, the draw-plane bridge, MoveIt Servo for Live
mode, and RViz when an X display is available. It prints the WebSocket address
the drawing client connects to.

Useful options:

```bash
./run.sh up phantom --no-rviz  # headless host
./run.sh up phantom --no-servo # batch drawing only, no Live mode
./run.sh status                # what is running
./run.sh logs bridge           # follow a log: bringup, movegroup, bridge, servo, rviz
./run.sh shell                 # a shell inside the container with ROS sourced
./run.sh down                  # stop everything cleanly and remove the container
```

RViz is refused while the real robot runs because the desktop perturbs
real-time timing. Pass `--rviz-on-hardware` to override.

### Hardware safety

`./run.sh up hardware` prints a checklist and waits for you to type `yes`.
Read `Docker/RUNBOOK.md` section 7 before the first run on a robot. Never
SIGKILL the container during motion; use `./run.sh down`, which shuts the
bringup down in order and releases the RMP.

## Run the drawing client

The client needs Python 3.10 or newer with tkinter. Both launchers install the
remaining Python packages on first run.

**On the Linux host, or any Linux machine with the repository:**

```bash
./run.sh client                        # connects to the bridge on this host
./run.sh client ws://<linux-host>:8765 # connects to another machine
```

This may ask for `sudo` once to install `python3-tk` and `python3-venv`. The
Python packages go into `.venv-client/` in the repository, not into your system
Python.

**On Windows**, with the repository checked out or copied to the machine,
run from the repository root:

```
run_client.bat ws://<linux-host>:8765
```

The address is the one `./run.sh up` printed. The window, plane calibration,
recipes, and Live mode are described in `src/rapidcode_draw_plane/README.md`.

## Components

| Path | Purpose |
|---|---|
| `run.sh`, `run_client.bat` | The commands above. |
| `inputs/` | Where you put the RSI `.deb`, license, and ENI. |
| `Docker/` | Dockerfile, compose file, entrypoint, runbooks, and the draw-plane stack launcher that `run.sh` calls. |
| `src/rapidcode_system/` | `ros2_control` SystemInterface plugin that maps read()/write() onto RapidCode axes. |
| `src/rapidcode_bringup/` | URDF, controller configs, launch files, and demo scripts for phantom and Elfin5 hardware. |
| `src/elfin_description/` | Elfin robot URDF and meshes. |
| `src/rapidcode_moveit_config/` | MoveIt 2 config: move_group, Servo, and RViz launch for the RapidCode robot. |
| `src/rapidcode_passthrough_trajectory_controller/` | Controller hosting FollowJointTrajectory and streaming points through to the hardware plugin. |
| `src/rapidcode_trajectory_transfer/` | Header-only passthrough protocol shared by the controller and the hardware plugin. |
| `src/rapidcode_draw_plane/` | Draw-plane bridge (ROS side) and the drawing surface client. Its README describes the operator window. |

## Further reading

- `Docker/RUNBOOK.md`: every step `run.sh` performs, one at a time, with
  verification, hardware preconditions, recipes, jogging, faults and recovery,
  and diagnostics.
- `Docker/RVIZ_SETUP.md`: preparing a headless host's desktop and GPU so the
  container can draw RViz on it.
- `Docker/PILZ_RUNBOOK.md`: running Pilz LIN and CIRC Cartesian recipes.
- `src/rapidcode_draw_plane/README.md`: the drawing surface client's window,
  calibration, and Live mode.
