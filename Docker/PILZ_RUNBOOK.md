# Pilz Cartesian Recipes — Runbook (elfin5, phantom or hardware)

End-to-end steps to build the container, bring up the elfin5 robot, start MoveIt's
`move_group`, show it in RViz, and run a Pilz motion recipe.

**Run every `docker compose` command from `Docker/`.** `./run.sh up phantom` at the
repository root does §0–§4 in one go; the steps below are the manual equivalent, then §5
runs the recipe.

Each launch is a **long-running foreground process** — give each its own terminal (each
`docker compose exec` opens a shell in the already-running container). Bring things up in
the order below; later steps depend on earlier ones.

---

## 0. Build (compile) the container image

Prereqs: the rmp `.deb` and `rsi.lic` in the repository's `inputs/` directory
(`inputs/README.md`, `RUNBOOK.md` §2.1). Optionally back up the current image first (so you
can roll back):

```bash
docker tag rsi/ros2-rapidcode:latest rsi/ros2-rapidcode:backup-$(date +%F)
./run.sh build            # = docker compose build, after checking inputs/
```

**What it does:** rebuilds `rsi/ros2-rapidcode:latest` — it copies `src/` into the image and
runs `colcon build`, so all packages (controllers, MoveIt config, bringup scripts, recipe
YAMLs) are baked in. The apt + rmp-deb layers are cached, so only the workspace layer
recompiles. Skip this step if you haven't changed anything under `src/`.

> If the container is already running an older image, recreate it to pick up the new build:
> `docker compose up -d` (a plain `docker compose restart` keeps the old image).

---

## 1. Start the container + the elfin5 robot

```bash
docker compose up -d && docker compose exec ros2_rapidcode bash -lc \
  'RAPIDCODE_TRAJ_INPUT_CSV=/tmp/in.csv RAPIDCODE_PVT_CSV=/tmp/fed.csv \
   ros2 launch rapidcode_bringup elfin5.launch.py use_hardware:=false'
```

**What it does:**
- `docker compose up -d` — starts the `ros2_rapidcode` container in the background.
- `ros2 launch rapidcode_bringup elfin5.launch.py` — starts the `controller_manager` + the
  RapidCode `rapidcode_system` plugin, the `joint_state_broadcaster`, the
  `rapidcode_passthrough_trajectory_controller` (active), and `robot_state_publisher`
  (publishes `/robot_description` + `/tf`).
- `use_hardware:=false` — RapidCode **phantom** axes (no EtherCAT hardware). Use `:=true`
  for real hardware.
- `RAPIDCODE_TRAJ_INPUT_CSV` / `RAPIDCODE_PVT_CSV` — *optional* trace files: the controller's
  input waypoints and the points fed to `MovePVT`, respectively. Drop them if you don't need
  traces.

This holds the terminal. Verify in another terminal:
```bash
docker compose exec ros2_rapidcode bash -lc 'ros2 control list_controllers'
# expect joint_state_broadcaster + rapidcode_passthrough_trajectory_controller both "active"
```

---

## 2. Start the MoveIt move_group (planner + Pilz sequence server)

```bash
docker compose exec ros2_rapidcode bash -lc 'ros2 launch rapidcode_moveit_config move_group.launch.py'
```

**What it does:** launches MoveIt's `move_group` with the OMPL **and** Pilz pipelines and the
`MoveGroupSequence` capability, so it serves `/sequence_move_group`. This is **required** for
the recipe scripts (they send Pilz sequences to that action) and for RViz's MotionPlanning
panel. It dispatches execution to the passthrough controller's `follow_joint_trajectory`
action from step 1.

Verify (another terminal):
```bash
docker compose exec ros2_rapidcode bash -lc 'ros2 action list | grep sequence_move_group'
```

---

## 3. Turn on the desktop (lightdm) so RViz can draw

RViz runs **inside** the container but draws on the **host's** X display. On this headless RT
host the desktop is off by default. At the **physical monitor** (or its session):

```bash
sudo systemctl start lightdm     # turn the Xfce desktop ON (on-demand; not enabled at boot)
# then, in a terminal INSIDE that desktop session:
xhost +local:                    # let local containers connect to the X server
echo $DISPLAY                    # note the value — usually :0
```

**What it does:** `lightdm` starts the on-demand desktop; `xhost +local:` authorizes the
container to draw on it. Turn it off later with `sudo systemctl stop lightdm`. (See
`RVIZ_SETUP.md` for the full X/GPU details and the real-time caution — the desktop perturbs
RT determinism, so keep it off during timing-critical hardware motion.)

---

## 4. Launch RViz (MoveIt view)

```bash
DISPLAY=:0 docker compose --profile gui run --rm rviz bash -lc \
  'ros2 launch rapidcode_moveit_config moveit_rviz.launch.py'
```

**What it does:** runs the on-demand `rviz` compose sidecar (it has the X socket + GPU access),
overriding its command to launch MoveIt's RViz config (`moveit.rviz` + semantic model +
kinematics). It connects to the `move_group` from step 2, and the robot follows the executed
motion via `/joint_states → /tf`. `--profile gui` keeps this out of a plain `up`; `--rm`
cleans up the one-off container on exit. `DISPLAY=:0` targets the monitor.

> A Pilz **sequence** can't be authored from the RViz panel — you *view* it here while a
> recipe script drives the motion (step 5). The panel is still usable for single-goal
> OMPL / Pilz-PTP planning.

---

## 5. Run a recipe

**File-driven recipe** (LIN / CIRC / PTP from a YAML file) — use
`send_pilz_cartesian_recipe.py`:

```bash
# by bundled name (resolved from the package's config/ dir):
docker compose exec ros2_rapidcode bash -lc \
  'ros2 run rapidcode_bringup send_pilz_cartesian_recipe.py recipe_box.yaml'

# or by absolute path (the baked source lives here in the container):
docker compose exec ros2_rapidcode bash -lc \
  'ros2 run rapidcode_bringup send_pilz_cartesian_recipe.py /ros2_ws/src/rapidcode_bringup/config/recipe_box.yaml'
```

**What it does:** reads the YAML recipe, builds one Pilz `MoveGroupSequence`
(PTP-to-`ready`, then the LIN/CIRC/PTP segments blended into a single trajectory), and
plans+executes it via `/sequence_move_group → move_group → passthrough controller →
MovePVT`. Watch it move in RViz. Optional flags override the file's defaults:
`--blend 0.0` (sharp corners / stop at each), `--vel 0.2 --acc 0.2`.

**Other senders:**
```bash
# Cartesian recipe hardcoded in source (LIN/CIRC), not from a file:
docker compose exec ros2_rapidcode bash -lc 'ros2 run rapidcode_bringup send_pilz_cartesian_sequence.py'
# joint-space PTP blended sequence:
docker compose exec ros2_rapidcode bash -lc 'ros2 run rapidcode_bringup send_pilz_sequence.py'
```

> Note: `send_pilz_cartesian_sequence.py` does **not** take a recipe-file argument — it runs
> its built-in `CART_RECIPE`. To run a **file**, use `send_pilz_cartesian_recipe.py`.

Inspect the executed motion:
```bash
docker compose exec ros2_rapidcode bash -lc \
  'timeout 25 ros2 run tf2_ros tf2_echo world elfin_end_link | grep Translation'
# or the traces from step 1: /tmp/in.csv (controller input) and /tmp/fed.csv (MovePVT points)
```

---

## Shut down / roll back

```bash
docker compose down                       # stop the container (GUI sidecar exits with --rm)
sudo systemctl stop lightdm               # turn the desktop back off (host)

# roll the image back to a tag you made in step 0, if needed:
docker tag rsi/ros2-rapidcode:backup-<date> rsi/ros2-rapidcode:latest
docker compose up -d                      # recreate from the rolled-back image
```
