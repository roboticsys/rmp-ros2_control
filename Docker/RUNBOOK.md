# ROS 2 ↔ RapidCode — Operating Runbook

End-to-end operating procedure for everything in this repository: build the container, bring up
the elfin5 robot (phantom **or** real hardware), start the RapidCode `ros2_control` system
interface and the passthrough trajectory controller, start MoveIt `move_group`, turn on the
host desktop, run RViz, move the robot from RViz, draw Cartesian recipes (monolithic, Pilz,
or streamed/pipelined), and jog with MoveIt Servo.

**The short path:** `./run.sh` at the repository root does §3, §6 or §7, §8, §10 and §13.1 for
you (`./run.sh build`, `./run.sh up phantom|hardware`, `./run.sh down`; see the root
`README.md`). This runbook is the long path: the same steps one at a time, with what each
does, how to verify it, and what to do when it fails. Use it for first-time bring-up on a new
robot, for anything `run.sh` does not cover (recipes, jogging, tests, diagnostics), and for
recovery.

**Run every `docker compose` command from `Docker/`.** Each launch is a **long-running
foreground process** — give each its own terminal. Every `docker compose exec` opens a new
shell in the already-running container (ROS 2 and the workspace overlay are auto-sourced in
both interactive and `-lc` shells). Bring things up in the order below; later steps depend on
earlier ones.

> ⚠️ **This is a surgical-grade motion controller.** Do all first-time testing in phantom
> mode (`use_hardware:=false`). Never issue a motion command whose preconditions you have not
> just confirmed, and never leave a jog publisher running unattended. The real-hardware gate
> is §7 and has hard preconditions.

### Companion documents

| Doc | Covers |
|---|---|
| `../README.md` | The `run.sh` commands: build, up, client, down |
| `../inputs/README.md` | The RSI files you supply: the rmp `.deb`, `rsi.lic`, `EtherCAT.xml` |
| `RVIZ_SETUP.md` | Full X / GPU / `lightdm` detail behind §9–§10, and the real-time caution |
| `PILZ_RUNBOOK.md` | Focused walkthrough of the Pilz LIN/CIRC recipe path (§12.1 here) |
| `../src/rapidcode_draw_plane/README.md` | The drawing surface client: the operator window and its behaviour |

---

## 1. Architecture and the single-owner rule

```
 MoveIt move_group            recipe senders                  MoveIt Servo
 (RViz Plan+Execute,          (send_cartesian_path_streamed.py,   (servo_node)
  Pilz sequences)              direct action client)
        │                             │                              │
        │  FollowJointTrajectory      │  FollowJointTrajectory        │ trajectory_msgs/
        │  action                     │  action                       │ JointTrajectory
        └──────────────┬──────────────┘                              │ (rolling window,
                       ▼                                             ▼  100 Hz)
        ~/follow_joint_trajectory                        ~/joint_trajectory  (topic)
                       │                                             │
                       └──────────────► rapidcode_passthrough_trajectory_controller
                                        (resample to 1 ms P/V/A/J, chunk, ack-gate)
                                                     │
                                     trajectory_transfer command/state interfaces
                                     (8 + 64×(1+4·N) cmd, 6 + 2·N state)
                                                     ▼
                                        rapidcode_system  (SystemInterface)
                                        frame accounting, host FIFO, fault latch
                                                     ▼
                                        RapidCode::MultiAxis → RMP firmware
```

`~` = `/rapidcode_passthrough_trajectory_controller`.

**Single motion owner.** The controller allows exactly **one** motion source at a time: a
finite `FollowJointTrajectory` goal (MoveIt / a recipe sender) **or** one online jog stream —
never both. The rule is *reject, not preempt*:

- A goal arriving while a jog is streaming is **rejected**.
- A jog window arriving while a goal is active is **dropped** (throttled warning).

So: do not press **Execute** in RViz while jogging, and do not jog while a recipe is drawing.

Separately, the `controller_manager` process is the **sole owner of the RMP
`MotionController`**. Use RapidSetupX / WorkBench read-only for inspection while it runs —
co-commanding the same axes contends with the stream and trips an e-stop.

**Two frame-budget numbers that matter operationally:**

| Number | Value | Where | Consequence if violated |
|---|---|---|---|
| Firmware out-of-frames watchdog | 64 frames = 32 points = 32 ms @ 1 kHz | `rapidcode_system_hardware.hpp` (`kEmptyCount`) | e-stop |
| Jog feed-gate refill floor | `online.low_water: 0.04` s | `config/elfin5_controllers.yaml:60` | must stay **above** 0.032 s or the watchdog wins the drain race |

---

## 2. Prerequisites

### 2.1 Files you supply, in `inputs/`

Three files come from RSI and are not in the repository. Put them in the repository's
`inputs/` directory (`inputs/README.md`); git ignores everything there but the README.

| File | Why | How it gets in |
|---|---|---|
| `rmp_*_amd64.deb` | Provides RapidCode as a customer install would (`/rsi` + headers). Exactly one; the build globs, so a version suffix is fine. | Installed into the image by `docker compose build`. |
| `rsi.lic` | Read from `/rsi/rsi.lic` by the rmp firmware (started by `MotionController::Create`). | Bind-mounted: `inputs/` is mounted at `/inputs`, and the entrypoint symlinks it into `/rsi` at container start. Replace the file and recreate the container; no rebuild. |
| `EtherCAT.xml` | The ENI, read as `/rsi/EtherCAT.xml` by `NetworkStart()`. Topology-specific; hardware only. | Same bind mount as the license. |

> ⚠️ **Check the deb's version.** The container's RapidCode comes from this `.deb`. The
> hardware plugin streams with `MovePVT` and relies on streaming-move behaviour (multi-axis
> closing-frame ordering, sub-count `Position` resolution) that older RMP builds lack. Use
> the RMP version this repository was developed against, or newer.

### 2.2 Host

- Docker + `docker compose`.
- A Linux host with a real-time kernel for hardware use (developed on Debian 13 with the
  `-rt` kernel). Phantom mode runs on any Linux host with Docker.
- For RViz, an X desktop the container can draw on. The reference host keeps `lightdm`
  installed but **not enabled at boot** (`RVIZ_SETUP.md`).
- `/dev/shm` is bind-mounted into the container for rmp ↔ RapidCode shared memory.
- For both modes: a CPU core for the rmp real-time thread, named in
  `config/elfin5_hardware.yaml` (`cpu_affinity`). It ships as the placeholder `-1`, which
  the plugin refuses at `on_init`. Never use core 0. For phantom any other core works. For
  hardware isolate the core on the kernel command line, e.g. `isolcpus=managed_irq,domain,15`
  on a 16-core host, and set the same 0-based number.
- For hardware: a dedicated NIC for EtherCAT, named in the same file (`primary_nic`). It
  carries no IP address. Find the name with `ip link`. It ships empty, which the plugin
  refuses at `on_init` when `use_hardware:=true`; phantom leaves it empty.
- Both values are copied into the image by `./run.sh build`. Set them **before** building,
  and rebuild after any change; editing the file on the host does not affect a built image.

### 2.3 Shared-memory hygiene

A container or launch that is **hard-killed** mid-stream (SIGKILL) skips teardown and can
leave stale `/dev/shm/RSI.*` files behind. The next cold boot of rmp then segfaults (it
typically presents as a `UserUnitsSet` timeout). With **no rmp running**:

```bash
rm -f /dev/shm/RSI.*
```

---

## 3. Build the image

Optionally tag the current image first so you can roll back:

```bash
docker tag rsi/ros2-rapidcode:latest rsi/ros2-rapidcode:backup-$(date +%F)
./run.sh build            # = docker compose build, after checking inputs/
```

**What it does:** rebuilds `rsi/ros2-rapidcode:latest` — `ros:jazzy` + `ros2_control` +
`ros2_controllers` + `moveit` + `moveit_servo` + `rviz2` + `teleop_twist_keyboard` from apt,
installs the rmp `.deb` from `inputs/`, copies `src/` to `/ros2_ws/src` and runs
`colcon build --symlink-install`. The apt and deb layers are cached, so a source-only change
recompiles just the workspace layer. The license and the ENI are not in the image (§2.1).

> **The workspace is baked into the image, not bind-mounted.** Host edits under `src/` take
> effect only after `docker compose build` **and** a container recreate. A plain
> `docker compose restart` keeps the old image.
>
> ⚠️ Conversely, anything you built *inside* a running container (`docker compose cp` +
> `colcon build`) is **discarded** by `docker compose down` or `--force-recreate`. If you are
> relying on an in-container build, use `docker compose restart` only — and bake it into the
> image before you trust it.
>
> ⚠️ A full compile perturbs RT determinism. Build while **not** connected to the robot.

---

## 4. Hermetic tests (no RMP, no motion)

```bash
docker compose run --rm --no-deps ros2_rapidcode bash -lc '
  cd /ros2_ws &&
  colcon test \
    --packages-select \
      rapidcode_passthrough_trajectory_controller \
      rapidcode_system \
    --event-handlers console_cohesion+ &&
  colcon test-result --test-result-base build --verbose
'
```

**Pass:** zero failures/errors — 35 controller cases + 14 hardware cases.

Two further executables are **compiled but not run** by `colcon test` (they are registered
with `SKIP_TEST` because they need exclusive ownership of a live phantom RMP):
`test_rapidcode_system_motion` (1 case) and
`test_rapidcode_system_passthrough_controller_motion` (2 cases). Run them manually per §16.

> **Coverage caveat.** The hermetic controller fixture is a `friend` that sets private state
> directly, so it bypasses the action server: goal *rejection* paths, cancel handling, action
> feedback, `on_configure` parameter validation, and `on_activate` interface resolution are
> **not** covered. Phantom (§5 onward) is the first place those run.

---

## 5. Static validation (no motion)

Cheap pre-flight that catches a broken launch file or URDF before you touch a robot:

```bash
docker compose run --rm --no-deps ros2_rapidcode bash -lc '
  cd /ros2_ws &&
  xacro src/rapidcode_bringup/urdf/elfin5_rapidcode.urdf.xacro use_hardware:=false >/tmp/elfin5.urdf &&
  ros2 launch rapidcode_bringup elfin5.launch.py --show-args &&
  ros2 launch rapidcode_moveit_config move_group.launch.py --show-args &&
  ros2 launch rapidcode_moveit_config servo.launch.py --show-args &&
  ros2 run rapidcode_bringup send_cartesian_path_streamed.py --self-test
'
```

**Pass:** every command exits 0, and `--self-test` prints `self-test PASSED` (it unit-tests
the streamed sender's pure retiming/goal-splitting helpers offline, no ROS).

---

## 6. Start the container + the elfin5 robot (phantom)

> `./run.sh up phantom` does this section, then §8, §13.1 and (with a display) §10, and
> prints the client address. The manual steps follow.

```bash
docker compose up -d
docker compose exec ros2_rapidcode bash -lc \
  'ros2 launch rapidcode_bringup elfin5.launch.py use_hardware:=false'
```

**Terminal 1 holds this.** What it does:

- `docker compose up -d` — starts the `ros2_rapidcode` container (host network for DDS
  discovery + the EtherCAT NIC, `privileged`, `SYS_NICE`, `rtprio: 99`, `/dev/shm` bound).
- `elfin5.launch.py` — starts `ros2_control_node` (the `controller_manager`, hosting the
  `rapidcode_system` plugin), `robot_state_publisher` (publishes `/robot_description` +
  `/tf`), and spawners for `joint_state_broadcaster` and
  `rapidcode_passthrough_trajectory_controller`.
- `use_hardware:=false` — RapidCode **phantom** axes; no EtherCAT. This is the launch file's
  only argument (default `false`).

Config in play: `rapidcode_bringup/config/elfin5_controllers.yaml`
(`update_rate: 500` Hz, `chunk_size: 64`, `sample_period: 0.001`,
`interpolation: quadratic`, tolerances `0.05`, and the `online.*` jog block) and
`urdf/elfin5_rapidcode.urdf.xacro`.

### Optional traces (add to the launch command's environment)

| Env var | Effect |
|---|---|
| `RAPIDCODE_TRAJ_INPUT_CSV=/tmp/in.csv` | Controller's **input** waypoints per accepted goal + per-segment interpolation method and c0..c5 coefficients |
| `RAPIDCODE_PVT_CSV=/tmp/fed.csv` | Every point the hardware feeds to `MovePVT`, tagged with `chunk_id`; one position, velocity, acceleration and jerk column group per joint (the last two are not sent to RapidCode) |
| `RAPIDCODE_FRAME_DEBUG=1` | Throttled per-axis firmware frame state (`FRAME_INDEX`/`FRAME_LOAD_INDEX`, STATUS, feedrate, cmd) |

```bash
docker compose exec ros2_rapidcode bash -lc \
  'RAPIDCODE_TRAJ_INPUT_CSV=/tmp/in.csv RAPIDCODE_PVT_CSV=/tmp/fed.csv \
   ros2 launch rapidcode_bringup elfin5.launch.py use_hardware:=false'
```

### Verify (Terminal 2)

```bash
docker compose exec ros2_rapidcode bash -lc '
  ros2 control list_hardware_components &&
  ros2 control list_controllers &&
  ros2 control list_hardware_interfaces | head -20 &&
  ros2 topic echo --once /joint_states
'
```

**Expect:** the `RapidCodeSystem` component `active`; `joint_state_broadcaster` **and**
`rapidcode_passthrough_trajectory_controller` both `active`; the `trajectory_transfer`
command/state interfaces present; `/joint_states` publishing six joints.

> Interface counts are large by design — 6 joints ⇒ **1608** command interfaces
> (`8 + 64 × (1 + 4×6)`) and **18** state interfaces (`6 + 2×6`). A truncated
> `list_hardware_interfaces` is normal.

> ⚠️ Activate this controller **XOR** the stock `joint_trajectory_controller` — they both
> claim the same command interfaces. (In practice the stock JTC cannot drive this hardware
> anyway: its per-joint position/velocity/acceleration command interfaces are commented out.)

---

## 7. Start the elfin5 robot on REAL HARDWARE

> ⚠️⚠️ **The plugin does not enable the physical amps. You do.** This is deliberate.
> Powering a drive is an operator action taken with the E-stop in reach, not a side effect
> of a ros2_control lifecycle transition. With `use_hardware:=true` the component activates,
> arms the PVT stream, and logs a WARN: `"Physical amps are NOT enabled by this plugin"`.
> Nothing moves until you enable the amps out of band (§7.2a). Only phantom mode enables
> its simulated amps itself. The per-joint `group` and `amp_enable_delay` values in
> `elfin5_hardware.yaml` document the enable order for that manual step; the plugin does
> not act on them.
>
> The per-joint `error_limit` from `elfin5_hardware.yaml` is armed with the E_STOP action.
> On a joint with `invert: true` the trigger is passed with the sign of the user units so
> that the firmware stores a positive count threshold; RapidSetupX therefore shows it as a
> **negative** value in user units. That display is expected and the limit works.

### 7.1 Preconditions — confirm **all** of these before powering motion

- [ ] EtherCAT link up: robot powered and connected on the **dedicated** NIC, the one named
      by `primary_nic` in `config/elfin5_hardware.yaml`. That NIC carries no IP address — that
      is correct. A `NetworkStart` error 9 (`MASTER_STARTUP`) with `carrier=0` on it means the
      **robot is unplugged**, not a bad ENI.
- [ ] `inputs/EtherCAT.xml` matches the physical topology (§2.1).
- [ ] `sample_rate` in `config/elfin5_hardware.yaml` (`1000.0`) matches the ENI.
- [ ] `cpu_affinity` in `config/elfin5_hardware.yaml` names an isolated core (§2.2), not 0
      and not the `-1` placeholder.
- [ ] Joint↔axis mapping and origins verified for **this** robot. `elfin5_hardware.yaml`
      encodes a deliberate **joint1↔joint2 axis swap** (`elfin_joint1 → axis 1`,
      `elfin_joint2 → axis 0`) plus per-joint `origin`, `invert`, `group`, `error_limit`, and
      `counts_per_radian: 2106935.4267950314`.
- [ ] STO / E-stop within reach and **tested**.
- [ ] Workspace cleared and restricted.
- [ ] Desktop (`lightdm`) **off** — it perturbs RT determinism (§9).
- [ ] A second person present.

### 7.2 Launch

> `./run.sh up hardware` prints the §7.1 checklist, waits for `yes`, then does this section,
> §8 and §13.1. It refuses RViz unless you pass `--rviz-on-hardware`.

```bash
docker compose exec ros2_rapidcode bash -lc \
  'ros2 launch rapidcode_bringup elfin5.launch.py use_hardware:=true'
```

**Differences from phantom:** `NetworkStart()` is called and the network must reach
`OPERATIONAL` (otherwise `on_configure` throws and the component drops back to
`UNCONFIGURED`, including `LastNetworkStartErrorGet` in the message); origin calibration runs
per joint for joints that declare an `origin`; `read()` reports `ActualPosition` /
`ActualVelocity` rather than the phantom's `CommandPosition` / `CommandVelocity`.

Conversely, launching **phantom** while the network is live throws deliberately
("shut it down or set `use_hardware:=true`") — the plugin refuses to straddle the two.

### 7.2a Enable the amps (operator step)

The component is now `ACTIVE`, but the drives are unpowered. Enable them yourself, from
RapidSetupX or WorkBench connected to the running rmp, in the order `elfin5_hardware.yaml`
describes: the even-numbered axes (`group: 0`) first, then the odd (`group: 1`), about
`amp_enable_delay` seconds apart. Before you do:

- [ ] Confirm the bringup log shows the `on_activate` WARN about amps and no faults.
- [ ] Confirm every axis reads a sane position in RapidSetupX.
- [ ] E-stop in hand.

Enable, then confirm each axis reports `AmpEnable = true` and holds position. Do not issue
any other command from RapidSetupX while ROS is streaming (§8).

### 7.3 First motion on hardware

Do **not** start with MoveIt or a recipe. Use the dedicated bring-up probe, which builds its
quintic from the **live** position (so there is no startup jump) and holds the other five
joints:

```bash
docker compose exec ros2_rapidcode bash -lc \
  'python3 /ros2_ws/src/rapidcode_bringup/scripts/jog_single_joint.py \
     --joint 4 --delta 0.02 --duration 3.0 --dry-run'
# then, without --dry-run, watch the arm AND RapidSetupX PositionError per axis:
docker compose exec ros2_rapidcode bash -lc \
  'python3 /ros2_ws/src/rapidcode_bringup/scripts/jog_single_joint.py \
     --joint 4 --delta 0.02 --duration 3.0'
```

> **`jog_single_joint.py` is not installed as an executable** — it is absent from
> `rapidcode_bringup/CMakeLists.txt`'s `install(PROGRAMS …)` list, so `ros2 run` will not find
> it. Invoke it by path as above. Same for `jog_gate_demo.py` and `servo_smoke_phase3.py`.
> Everything in §12 and §19 **is** installed and works with `ros2 run`.

`--joint` accepts an index `0-5` or a name (`elfin_joint5`); other flags are `--delta`,
`--duration`, `--points` (default 41), `--dry-run`. Confirm the joint moves the way RViz shows
it should — if it moves the other way, `invert` is wrong for that axis.

Record for each first-time motion: command-to-commit latency, max committed horizon,
following error, stop latency. **Any** unexpected hard abort, frame starvation,
discontinuity, or nonzero `error_code` blocks further testing until understood.

---

## 8. Start the MoveIt `move_group`

```bash
docker compose exec ros2_rapidcode bash -lc 'ros2 launch rapidcode_moveit_config move_group.launch.py'
```

**Terminal 2 holds this.** It launches `move_group` with **both** the OMPL (default) and Pilz
pipelines plus the `MoveGroupSequence` capability, so it serves `/sequence_move_group`.

Required for: RViz's MotionPlanning panel (§11), every recipe sender (§12), and Servo's
planning-scene collision checking (§13).

Execution bridge (`config/moveit_controllers.yaml`): MoveIt uses
`MoveItSimpleControllerManager` and dispatches to the **existing**
`rapidcode_passthrough_trajectory_controller` via
`/rapidcode_passthrough_trajectory_controller/follow_joint_trajectory`.
`moveit_manage_controllers: false` — the bringup owns controller lifecycle, MoveIt only talks
to it.

### Verify

```bash
docker compose exec ros2_rapidcode bash -lc '
  ros2 action list | grep -E "sequence_move_group|move_action|follow_joint_trajectory" &&
  ros2 service list | grep -E "compute_cartesian_path|compute_ik|compute_fk"
'
```

---

## 9. Turn on the host desktop (`lightdm`)

RViz runs **inside** the container but draws on the **host's** X display. On this headless RT
host the desktop is off by default. At the **physical monitor** (or its session):

```bash
sudo systemctl start lightdm     # on-demand; NOT enabled at boot
# then, in a terminal INSIDE that Xfce session:
xhost +local:                    # authorize local containers to use the X server
echo $DISPLAY                    # note the value — usually :0
```

Turn it off afterwards with `sudo systemctl stop lightdm`.

> If you are SSH'd in rather than sitting at the monitor, `xhost` needs the session's
> display/auth — easiest is to run it from a terminal on the monitor.
>
> ⚠️ **RT caution.** The desktop and RViz consume CPU on non-isolated cores and perturb
> real-time determinism. Treat the GUI as a dev/viz tool, not something running during
> timing-critical hardware motion. See `RVIZ_SETUP.md` for the full X/GPU detail.

---

## 10. Launch RViz (MoveIt view)

```bash
DISPLAY=:0 docker compose --profile gui run --rm rviz bash -lc \
  'ros2 launch rapidcode_moveit_config moveit_rviz.launch.py'
```

**Terminal 3 holds this.** It runs the on-demand `rviz` compose sidecar (same image, host
network, `privileged` for `/dev/dri`, X socket bind), overriding its default `rviz2` command
to load MoveIt's RViz config (`config/moveit.rviz` + the semantic model + kinematics + joint
limits + both planning pipelines, so the planner dropdown lists OMPL **and** Pilz
PTP/LIN/CIRC).

`--profile gui` keeps it out of a plain `docker compose up`; `--rm` cleans up on exit;
`DISPLAY=:0` targets the monitor rather than an SSH X-forward.

### Verify GPU acceleration (optional)

```bash
docker compose --profile gui run --rm rviz bash -lc 'glxinfo -B | grep "OpenGL renderer"'
# expect: AMD Radeon ... (radeonsi ...)      NOT: llvmpipe   (= software rendering)
```

The robot follows executed motion via `/joint_states → robot_state_publisher → /tf → RViz`.

---

## 11. Move the robot from RViz

In the **MotionPlanning** panel:

1. **Planning** tab → **Planning Group** = `elfin_arm`.
2. Set a goal, either:
   - drag the orange interactive marker on the end effector, or
   - **Goal State** → pick a named state from the SRDF, or
   - **Joints** tab → move the per-joint sliders.
3. **Plan** — the planned trajectory previews in the scene. Confirm it is sane.
4. **Execute** — **this moves the robot.**

**What Execute does:** `move_group` sends the planned, time-parameterized trajectory to
`/rapidcode_passthrough_trajectory_controller/follow_joint_trajectory`. The controller
validates it, resamples it to the 1 ms grid, and streams it in 64-point chunks to the
hardware, which feeds the RMP `MultiAxis`.

> ⚠️ **Do not press Execute while a jog session is streaming** (§13) — the goal will simply be
> rejected ("online stream is active"), but the intent is unsafe. Stop the jog first and let it
> reach rest.
>
> A goal is also **rejected** if its first point is farther than `first_point_tolerance`
> (0.05 rad) from the current measured position (or from the seam of an already-queued goal).
> If Execute fails immediately with `INVALID_GOAL`, that is usually why — re-plan from the
> current state.
>
> A Pilz **sequence** cannot be authored from the RViz panel. You *view* sequences here while
> a recipe script drives them (§12). Single-goal OMPL and Pilz-PTP planning work fine.

### Verify the motion actually executed

```bash
docker compose exec ros2_rapidcode bash -lc \
  'timeout 25 ros2 run tf2_ros tf2_echo world elfin_end_link | grep Translation'
```

---

## 12. Run Cartesian recipe files

Three senders share the **same recipe YAML conventions** (`coordinates: relative|absolute`,
`ready:`, `anchor: {xyz, rpy}`, `defaults: {vel, acc, z, samples}`) but differ in segment
vocabulary and in how planning is pipelined against execution. All live in
`rapidcode_bringup/scripts/` and run with `ros2 run rapidcode_bringup <script> …`.
All require §6 (or §7) **and** §8.

| Sender | Segment vocabulary | Planning | Use when |
|---|---|---|---|
| `send_pilz_cartesian_recipe.py` | Pilz `LIN` / `CIRC` / `PTP`, blended into one trajectory | one `MoveGroupSequence` up front | straight lines and arcs |
| `send_cartesian_path.py` | arbitrary curves: B-spline, spline-`through`-points, parametric `func` | monolithic — solves IK for **every** dense waypoint before moving | curves Pilz can't express, and you don't mind waiting |
| `send_cartesian_path_streamed.py` | same as above | **pipelined** — plans goal *k+1* while the robot executes goal *k* | the same curves, but you want motion to start in ~0.02 s instead of ~30 s |

Available recipes in `rapidcode_bringup/config/`:

| Recipe | Kind | Source |
|---|---|---|
| `recipe_box.yaml` | Pilz LIN rectangle | original |
| `recipe_rsi_logo_svg.yaml` | curves | RSI logo |
| `recipe_japanese_woman_simple.yaml` | curves | Wikimedia Commons, CC0 |
| `recipe_man_throwing_discus_simple.yaml` | curves, heavy | Wikimedia Commons, CC BY 4.0 |

Each traced recipe names its source and licence in its header.

A bundled name is resolved from the package's `config/` dir; an absolute path also works
(`/ros2_ws/src/rapidcode_bringup/config/<name>.yaml`).

Recipe schema (shared): `coordinates: relative|absolute`, `ready: [j1..j6]` (an optional Pilz
PTP first, for a good arm configuration / IK seed), `anchor: {xyz, rpy}` (an **absolute** world
pose that all `relative` points are offsets from — **edit `anchor` alone to relocate a whole
drawing**), `defaults: {blend, vel, acc, z, samples}`, then `segments:`.

> ⚠️ `recipe_man_throwing_discus_simple.yaml` is the heavy one. Prefer the streamed sender
> (§12.3) for it; the monolithic sender's 60 s planning timeout is close.

### 12.1 Pilz LIN/CIRC recipe

```bash
docker compose exec ros2_rapidcode bash -lc \
  'ros2 run rapidcode_bringup send_pilz_cartesian_recipe.py recipe_box.yaml'
```

Builds one Pilz `MoveGroupSequence` (a PTP to `ready`, then the LIN/CIRC/PTP segments blended
into a single trajectory) and plans+executes it via `/sequence_move_group`. Flags override the
file's defaults: `--blend 0.0` (sharp corners — stop at each), `--vel 0.2`, `--acc 0.2`.

See `PILZ_RUNBOOK.md` for the focused walkthrough. Two other senders run **hardcoded**
sequences rather than a file: `send_pilz_cartesian_sequence.py` (its built-in `CART_RECIPE`)
and `send_pilz_sequence.py` (joint-space PTP blend).

### 12.2 Arbitrary curves — monolithic

Always dry-run first (offline, no ROS, no motion), then plan-only, then live:

```bash
docker compose exec ros2_rapidcode bash -lc \
  'ros2 run rapidcode_bringup send_cartesian_path.py recipe_japanese_woman_simple.yaml --dry-run'
docker compose exec ros2_rapidcode bash -lc \
  'ros2 run rapidcode_bringup send_cartesian_path.py recipe_japanese_woman_simple.yaml --plan-only'
docker compose exec ros2_rapidcode bash -lc \
  'ros2 run rapidcode_bringup send_cartesian_path.py recipe_japanese_woman_simple.yaml'
```

Flags: `--dry-run`, `--plan-only`, `--force`, `--no-collision`, `--vel V`, `--acc A`,
`--eef-step M` (MoveIt IK/interpolation step, m), `--jump J`, `--min-fraction F`,
`--scale S`, `--rotate DEG`.

The curve lives in the **sender**: it samples the curve into dense Cartesian poses and hands
them to `/compute_cartesian_path`, which interpolates **linearly** between them and runs IK at
each — so sample densely. The approach to `ready`/`anchor` is a Pilz **PTP** (joint space, no
straight-line constraint) precisely so it cannot trip the anchor-approach singularity.

Use `print_tcp_pose.py` to produce a paste-ready `anchor:` block:

```bash
docker compose exec ros2_rapidcode bash -lc 'ros2 run rapidcode_bringup print_tcp_pose.py'
docker compose exec ros2_rapidcode bash -lc \
  'ros2 run rapidcode_bringup print_tcp_pose.py 0 0 -1.5708 0 -1.5708 3.1415'
```

### 12.3 Arbitrary curves — streamed / pipelined (recommended for heavy recipes)

```bash
# offline goal-split table, no ROS:
docker compose exec ros2_rapidcode bash -lc \
  'ros2 run rapidcode_bringup send_cartesian_path_streamed.py recipe_man_throwing_discus_simple.yaml --dry-run'
# plan every goal, report, no motion:
docker compose exec ros2_rapidcode bash -lc \
  'ros2 run rapidcode_bringup send_cartesian_path_streamed.py recipe_man_throwing_discus_simple.yaml --plan-only'
# live:
docker compose exec ros2_rapidcode bash -lc \
  'ros2 run rapidcode_bringup send_cartesian_path_streamed.py recipe_man_throwing_discus_simple.yaml'
# tuning / fallback:
docker compose exec ros2_rapidcode bash -lc \
  'ros2 run rapidcode_bringup send_cartesian_path_streamed.py recipe_man_throwing_discus_simple.yaml \
     --goal-seconds 3.0 --max-inflight 2'
docker compose exec ros2_rapidcode bash -lc \
  'ros2 run rapidcode_bringup send_cartesian_path_streamed.py recipe_man_throwing_discus_simple.yaml --rest-seams'
```

Own flags: `--goal-seconds S` (target motion per goal), `--max-inflight N`, `--rest-seams`,
`--self-test`. All `send_cartesian_path.py` flags above are forwarded.

**What it does:** splits the dense waypoint list into goals of ~`--goal-seconds` of motion
(cut at recipe segment boundaries where possible), plans each with
`/compute_cartesian_path`, and sends them **directly** to the controller's
`follow_joint_trajectory` action — bypassing MoveIt's serializing `/execute_trajectory`. So
planning goal *k+1* overlaps execution of goals ≤ *k*.

**Seam continuity.** MoveIt's per-goal timing is **discarded**; a sender-side accel-limited
forward/backward retimer (with grbl-style junction speed caps) re-times each goal across the
seam into the next, so the shared seam waypoint carries one consistent nonzero velocity and
the pen does not dwell. This costs a one-goal emission lag (goal *k* is emitted only after
goal *k+1* is planned). To achieve it the sender **flips the controller's
`finalize_last_chunk` parameter live** via
`/rapidcode_passthrough_trajectory_controller/set_parameters`: `false` for interior goals (the
firmware move stays open across seams) and `true` for the chain's final goal. The controller
re-reads that parameter per goal, which is what makes this work.

`--rest-seams` disables all of it (per-goal rest-to-rest MoveIt timing, every goal finalizes,
a dwell at every seam).

> ⚠️ **Failure story.** If a goal fails to plan (fraction < 1, IK failure, timeout) the
> previous goal is re-timed to **end at rest** and sent as final — the pen lands at a known
> point, never a smear. But if this sender **dies** mid-chain with the firmware move open, the
> out-of-frames watchdog e-stops in ~32 ms. That is the documented correct failure mode for an
> unattended producer — expect an e-stop, not a graceful stop, if you SIGKILL the sender.

---

## 13. Jog the robot with MoveIt Servo

Servo publishes a rolling `trajectory_msgs/JointTrajectory` at 100 Hz **straight** to the
controller's online intake `~/joint_trajectory`. The controller resamples, splices each new
window onto the live stream, and synthesizes the decel-to-rest stop tail itself. There is no
adapter node and no `/stream/*` protocol on this branch.

Requires §6 (or §7) and §8 (Servo needs `/planning_scene` from `move_group`).

### 13.1 Launch Servo

```bash
docker compose exec ros2_rapidcode bash -lc 'ros2 launch rapidcode_moveit_config servo.launch.py'
```

**Terminal 4 holds this.** Config: `rapidcode_moveit_config/config/servo.yaml` —
`move_group_name: elfin_arm`, `publish_period: 0.01`, `command_in_type: speed_units`,
`command_out_topic: /rapidcode_passthrough_trajectory_controller/joint_trajectory`,
`command_out_type: trajectory_msgs/JointTrajectory`, collision checking on,
`use_smoothing: true` with the Ruckig filter plugin.

> Note `servo.launch.py:33` hard-codes `use_hardware: "false"` in the robot-description
> mapping. That is description-only (Servo ignores the `<ros2_control>` block), so it is
> harmless on hardware — but it is surprising, so don't read it as a mode switch.
>
> The `servo.yaml` comment above `use_smoothing` says smoothing was toggled **off**; the
> committed value is `true`. If you see ~50 % joint-velocity ripple on a constant twist, the
> Ruckig online filter is the prime suspect — set it to `false` and relaunch (then re-do
> §13.2, which resets on every relaunch).

### 13.2 Select the command type — **mandatory**

```bash
# select the input type: 0 = JOINT_JOG, 1 = TWIST, 2 = POSE  (this does NOT move the robot)
docker compose exec ros2_rapidcode bash -lc \
  'timeout 5s ros2 service call /servo_node/switch_command_type \
     moveit_msgs/srv/ServoCommandType "{command_type: 0}"'
```

Require `success: true`. Exit code 124 (timeout) means Servo is unhealthy — go back to §13.1
and read its log; **do not command motion**.

> ⚠️ **The #1 gotcha.** Servo silently ignores every jog until a command type is selected,
> and **restarting `servo_node` resets it to unset**. Symptom: no motion at all, and
> `servo_node` logging `Command type has not been set, cannot accept input` at your publish
> rate. Re-issue `switch_command_type` after **every** servo relaunch.

On this Jazzy build `switch_command_type` is the only enable step needed — the working
`servo_smoke_phase3.py` calls nothing else. Some `moveit_servo` versions additionally require
un-pausing the node; if Servo accepts the command type but never publishes, check for and call
that service:

```bash
docker compose exec ros2_rapidcode bash -lc 'ros2 service list | grep -i servo'
# if /servo_node/start_servo exists:
docker compose exec ros2_rapidcode bash -lc \
  'timeout 5s ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}'
```

Confirm Servo is actually emitting before blaming the controller:

```bash
docker compose exec ros2_rapidcode bash -lc \
  'ros2 topic hz /rapidcode_passthrough_trajectory_controller/joint_trajectory'
```

### 13.3 Joint jog first (lowest risk)

```bash
docker compose exec ros2_rapidcode bash -lc \
  "ros2 topic pub --rate 50 --times 150 /servo_node/delta_joint_cmds \
     control_msgs/msg/JointJog \
     '{header: \"auto\", joint_names: [elfin_joint1], velocities: [0.05], duration: 0.02}'"
```

`--times N` at `--rate R` bounds the jog to N/R seconds; omit it and Ctrl-C to stop manually.
On hardware start at `0.01` or lower and keep a hand on the e-stop.

Watch it in another terminal:

```bash
docker compose exec ros2_rapidcode bash -lc 'ros2 topic echo /joint_states'
docker compose exec ros2_rapidcode bash -lc 'ros2 topic echo /servo_node/status'
```

**Pass:**
- `elfin_joint1` moves smoothly and monotonically; the other five hold.
- Terminal 1 shows no `OUT_OF_FRAMES`, queue overflow, e-stop, or `DrainQueue blocked` storm.
- On stopping the publisher, the joint comes to rest promptly and stays there — the controller
  detects `online.producer_timeout` (0.05 s) of silence and synthesizes a quintic
  decel-to-rest tail bounded by `online.max_{velocity,acceleration,jerk}`.
- A **second** jog after the stop reopens and works.

> Stopping a jog = **stop publishing**. There is no stop service; the producer-timeout stop
> tail is the stop mechanism, and it is one of the least-exercised paths in the stack. Watch
> this transition carefully the first several times.

### 13.4 Low-speed Cartesian twist (only after §13.3 passes)

```bash
docker compose exec ros2_rapidcode bash -lc \
  'timeout 5s ros2 service call /servo_node/switch_command_type \
     moveit_msgs/srv/ServoCommandType "{command_type: 1}"'
docker compose exec ros2_rapidcode bash -lc \
  "ros2 topic pub --rate 50 --times 150 /servo_node/delta_twist_cmds \
     geometry_msgs/msg/TwistStamped \
     '{header: {stamp: \"now\", frame_id: \"elfin_base_link\"}, twist: {linear: {x: 0.005}}}'"
```

Because `command_in_type: speed_units`, `linear.x` **is** the EE speed in m/s (the yaml
`scale` block is inert in this mode). 0.005 = 5 mm/s.

A Servo safety halt (near-singularity, joint-limit margin, collision proximity) is **not** a
transport failure — read the `/servo_node/status` code; do not raise thresholds to mask it.

> ⚠️ **elfin5 wrist singularity.** With `elfin_joint5 ≈ 0` the wrist is singular and a
> Cartesian twist will e-stop. Move joint 5 away from zero before commanding twists.
>
> ⚠️ **Burst producers are unsafe for Cartesian jogging.** `teleop_twist_keyboard` (baked into
> the image) emits **one** twist per keystroke and then goes silent, which is a burst, not a
> continuous producer. A single 0.03 tap has been observed to produce a large wrong-direction
> runaway and wedge the session. The online pipeline is designed for a **continuous** producer
> (a clean 10 mm move with one is fine). Do not use keyboard teleop for Cartesian jogging until
> short-burst handling is fixed. Also do not use the upstream `servo_keyboard_input` demo — it
> hard-codes Panda joint/frame names and will not drive elfin5.

### 13.5 Scripted jog checks (phantom)

Two self-scoring scripts replace hand-eyeballing; both exit 0 only if all checks pass:

```bash
# online topic intake directly (no Servo): jog, stop, jog again
docker compose exec ros2_rapidcode bash -lc \
  'python3 /ros2_ws/src/rapidcode_bringup/scripts/jog_gate_demo.py'
# through Servo, JOINT_JOG then TWIST: scores net displacement, stall fraction,
# wrong-direction fraction, settle time
docker compose exec ros2_rapidcode bash -lc \
  'python3 /ros2_ws/src/rapidcode_bringup/scripts/servo_smoke_phase3.py'
```

(Neither is installed as an executable — invoke by path, see §7.3.)

`jog_gate_demo.py` asserts the joint reaches the commanded speed, comes to rest within 0.6 s
of producer silence (the bounded-lookahead promise), and that a second jog reopens.
`servo_smoke_phase3.py` allows a longer 2.5 s settle because Servo owns most of the stop
(0.1 s stale timeout plus its own Ruckig decel) before the controller's stop tail closes it.

---

## 14. Faults and recovery

The hardware `error_code` is a **latch**. It survives a `Begin`, and only a `Reset` clears it.

| `error_code` | Meaning |
|---|---|
| `0` | healthy |
| `1` | host chunk FIFO overflow (cap 4096 chunks) |
| `2` | a `std::exception` escaped the feed path in `write()` (an e-stop is issued) |
| `3` | the MultiAxis group was already in `ERROR`/`STOPPING_ERROR` with work in flight — a following-error trip, amp fault, or an external e-stop |

**How a latch presents:** the controller sees it in `update()`, aborts every pending /
in-flight / queued goal with `PATH_TOLERANCE_VIOLATED` ("hardware in error state (e-stop /
starvation / bad point)…"), drops any jog session, and then **holds `Cmd::None` every cycle**
until the error clears. It re-warns roughly every 5 s. Nothing you send will move the robot in
this state.

**Recovery, in order:**

1. Stop every publisher and sender.
2. Read the cause — Terminal 1 has the drained RapidCode error log (entry number, object,
   function, `file:line`, short + long text) and the MultiAxis diagnostics.
3. Clear the **drive-side** fault manually in RapidSetupX (`ClearFaults`). The controller
   deliberately does not do this for you.
4. Clear the transport latch:
   ```bash
   docker compose exec ros2_rapidcode bash -lc \
     'ros2 service call /rapidcode_passthrough_trajectory_controller/reset_fault std_srvs/srv/Trigger {}'
   ```
   Returns `success: false, "no fault latched"` if `error_code` is already 0; otherwise
   `success: true, "reset requested (error_code=N)"`. The actual clear happens
   asynchronously — the controller emits `Cmd::Reset` on the next cycle where the mailbox is
   acked, and the hardware's `HandleReset` zeroes the latch.
5. Re-verify with `ros2 control list_controllers` and a small `jog_single_joint.py` move.

> If you call `reset_fault` while the **drive** is still faulted, the reset is acknowledged and
> then `DrainQueue` immediately re-latches `error_code = 3` before any motion. Fix the drive
> first (step 3).

If the component itself dropped to `UNCONFIGURED` (an `on_configure` throw, or `on_error`),
restart the launch in Terminal 1 rather than trying to reset.

---

## 15. Inspection and diagnostics

```bash
# control graph
docker compose exec ros2_rapidcode bash -lc 'ros2 control list_hardware_components'
docker compose exec ros2_rapidcode bash -lc 'ros2 control list_controllers'
docker compose exec ros2_rapidcode bash -lc 'ros2 control list_hardware_interfaces'

# state
docker compose exec ros2_rapidcode bash -lc 'ros2 topic echo /joint_states'
docker compose exec ros2_rapidcode bash -lc \
  'timeout 25 ros2 run tf2_ros tf2_echo world elfin_end_link | grep Translation'

# controller parameters actually in force
docker compose exec ros2_rapidcode bash -lc \
  'ros2 param dump /rapidcode_passthrough_trajectory_controller'
```

Traces from §6: `/tmp/in.csv` (controller input waypoints + per-segment interpolation method
and coefficients) and `/tmp/fed.csv` (every point fed to `MovePVT`, with `chunk_id`).
Comparing the two shows whether the controller's interpolation, not the firmware, introduced
a spike.

RapidSetupX / WorkBench over `rapidserver` are useful **read-only** here (per-axis State,
PositionError, FramesToExecute). Do not command from them (§1).

> RMP Recorder 0 is configured in `on_configure` (4 values per axis: command/actual
> position and velocity, full rate, non-circular) but is not auto-started and cannot be
> started from ROS; start it from RapidSetupX / WorkBench instead.

---

## 16. Live-RMP manual tests (exclusive RMP ownership)

These need to be the **only** RMP owner, so stop Terminals 1–4 first and confirm the robot is
stationary.

```bash
docker compose exec ros2_rapidcode bash -lc '
  cd /ros2_ws &&
  ./build/rapidcode_system/test_rapidcode_system_motion --gtest_color=yes &&
  ./build/rapidcode_system/test_rapidcode_system_passthrough_controller_motion --gtest_color=yes
'
```

- `test_rapidcode_system_motion` — full lifecycle against a live phantom RMP; a 200-point /
  200 ms quintic 0.0 → 1.0 on both joints, fed as chunks 64/64/64/8; asserts both axes reach
  1.0 and settle. `RAPIDCODE_MOTION_HOLD=1` keeps rmp up for RapidSetupX scoping and
  ping-pongs 0↔1 on Enter.
- `test_rapidcode_system_passthrough_controller_motion` — the controller's output wired
  directly into the hardware on live phantom RMP: `ControllerChunksDriveMultiChunkMovePvajt`
  and `TwoGoalsPipelineCompletion` (the pipelined multi-goal path).
  `RAPIDCODE_PASSTHROUGH_MOTION_HOLD=1` gives a hold mode with auto-run fallback.

Both are finite-trajectory regressions; neither exercises the online jog path.

---

## 17. Shut down

```bash
./run.sh down                        # stops the stack, SIGINTs the bringup, waits, compose down
sudo systemctl stop lightdm          # host: turn the desktop back off
```

By hand, the equivalent is: stop RViz, Servo, the bridge and `move_group`; `Ctrl-C` the
bringup terminal and wait for it to exit; then

```bash
docker compose down                  # stops the container (the gui sidecar exits with --rm)
rm -f /dev/shm/RSI.*                 # only with no rmp running
```

> **Never SIGKILL during motion.** A clean stop lets `on_deactivate` / `on_cleanup` abort
> motion, disable amps, shut down the network, and `Shutdown()` then `Delete()` the
> `MotionController` in that mandated order, releasing RMP and shared memory. A hard kill
> skips all of it and leaves a non-IDLE axis plus stale `/dev/shm/RSI.*` (§2.3).

Roll the image back if a build went wrong:

```bash
docker tag rsi/ros2-rapidcode:backup-<date> rsi/ros2-rapidcode:latest
docker compose up -d      # recreate from the rolled-back image
```

---

## 18. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| No motion at all; `servo_node` logs `Command type has not been set, cannot accept input` | You didn't select an input type, or you restarted `servo_node` (which resets it). Re-run §13.2. Not a hardware or controller fault. |
| `ros2 control list_controllers` shows the controller `inactive` or missing | The spawner failed. Read Terminal 1: usually `on_activate` could not resolve a `trajectory_transfer` interface name (a `gpio_name` / joint-order mismatch between `elfin5_controllers.yaml` and the URDF `<ros2_control>` block). Joint order in the yaml **must** match the URDF joint order. |
| Cold boot segfaults; `UserUnitsSet` timeout | Stale `/dev/shm/RSI.*` from a hard-killed run. `rm -f /dev/shm/RSI.*` with no rmp running (§2.3). |
| `on_configure` throws with `NetworkStart` error 9 (`MASTER_STARTUP`) | The robot is unplugged or unpowered. Check the EtherCAT NIC's carrier. Not an ENI problem. |
| Phantom launch throws "shut it down or set `use_hardware:=true`" | The EtherCAT network is live. The plugin refuses to run phantom axes against a live network. |
| `use_hardware:=true` activates cleanly but nothing moves | The amps are not enabled. The plugin never enables physical amps; do it yourself (§7.2a). |
| Goal rejected immediately with `INVALID_GOAL` | First point is outside `first_point_tolerance` (0.05 rad) of the measured position or of a queued goal's seam. Re-plan from the current state. |
| Goal rejected: "online stream is active" | A jog session is streaming. Stop the jog, let it reach rest, retry (§1). |
| Jog windows ignored, throttled "action busy" warning | A finite goal is active. Same rule, other direction. |
| `OUT_OF_FRAMES` e-stop during a jog | The feed gate lost the drain race against the 32 ms firmware watchdog. Check `online.low_water` is still **above** 0.032 s. Also check for a 500 Hz control-loop stall in Terminal 1. |
| `OUT_OF_FRAMES` e-stop right after a streamed recipe sender died | Expected — that is the documented failure mode for an unattended producer with the move left open (§12.3). |
| Cartesian twist e-stops immediately | `elfin_joint5 ≈ 0` wrist singularity, or a burst producer (§13.4). |
| A steady stream of `DrainQueue blocked` lines | Backpressure: the firmware buffer has no room. Look at the reported `need/free/fte/qdepth/pendingFinal`. Sustained blockage usually means the ROS loop is outrunning the firmware — check `update_rate: 500` is still **below** `sample_rate: 1000`. |
| Second jog after a stop does nothing | The previous session may not have closed. Confirm `error_code == 0` and that the arm reached rest; a fresh window should reopen the move. |
| Source edits have no effect | The workspace is baked into the image. `docker compose build` **and** recreate (§3). |
| Fixes vanished after `docker compose down` | They were built inside the running container, not into the image (§3). |
| RViz renders with `llvmpipe` (slow) | GPU passthrough failed. See `RVIZ_SETUP.md` §5. |

---

## 19. Two-axis phantom smoke test (no robot model)

The quickest check that the plugin, the passthrough controller and rmp are healthy, with
no Elfin, no MoveIt and no bridge. Needs only `inputs/rsi.lic`. Do not run it while the
elfin5 bringup holds the RMP (§1).

```bash
docker compose exec ros2_rapidcode bash -lc 'ros2 launch rapidcode_bringup two_axis.launch.py'
```

`two_axis.launch.py` spawns `joint_state_broadcaster` + `rapidcode_passthrough_trajectory_controller`
+ `forward_position_controller --inactive` over two generic phantom joints (`joint1` → axis 0,
`joint2` → axis 1, one `MultiAxis`; config `rapidcode_two_axis_controllers.yaml`,
`update_rate: 500` Hz). In a second terminal:

```bash
docker compose exec ros2_rapidcode bash -lc 'ros2 run rapidcode_bringup send_passthrough_trajectory.py'
docker compose exec ros2_rapidcode bash -lc 'ros2 run rapidcode_bringup send_passthrough_trajectory.py 0.0 0.0'
```

`send_passthrough_trajectory.py [targets...] [seconds]` reads the current positions from
`/joint_states`, builds one rest-to-rest quintic from there to the absolute targets (default
`0.5 -0.3` over 2 s) and sends it as a `FollowJointTrajectory` goal. It exits 0 when the goal
succeeds and prints the controller's `error_code` otherwise. `--joints joint1 -- 0.5` drives
one joint. Watch `ros2 topic echo /joint_states` follow the profile.

> **Phantom note:** a phantom axis has no real feedback, so `read()` reports `CommandPosition`
> (`ActualPosition` stays 0). On a clean teardown the SystemInterface stops the rmp firmware
> (`MotionController::Shutdown()` then `Delete()`), so a following launch cold-boots rmp.
