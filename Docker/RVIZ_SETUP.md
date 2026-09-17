# Running RViz with the ros2-rapidcode container

How to bring up RViz (and any ROS 2 GUI) on a **headless real-time Linux host**,
while keeping ROS 2 + RViz **inside the Docker container** (nothing ROS installed
on the host). The host only provides an on-demand X desktop + GPU; the container
draws onto it. The reference host is Debian 13 with an AMD iGPU; other
distributions and GPUs work the same way with their own package names and
drivers.

> `./run.sh up` starts RViz inside the main container for you whenever an X
> socket exists on the host (`Docker/launch_draw_plane.sh`). Sections 1–3 here
> are the one-time host preparation it depends on. Section 4 is an alternative
> way to launch RViz as a separate sidecar container.

## Architecture

```
┌──────────────── Host (Debian 13, AMD iGPU) ────────────────┐
│  lightdm + Xfce session on the physical monitor (on demand) │
│  X server (DISPLAY=:0)  +  amdgpu  +  /dev/dri/renderD128    │
└───────────────┬─────────────────────────────────────────────┘
                │  X socket (/tmp/.X11-unix)  +  /dev/dri (GPU)
┌───────────────▼──────────── Docker (rsi/ros2-rapidcode) ────┐
│  ROS 2 Jazzy + rviz2  →  renders on host GPU, shows on monitor│
└──────────────────────────────────────────────────────────────┘
```

On the reference host the GPU is an AMD iGPU with the `amdgpu` driver and a
render node (`/dev/dri/renderD128`), so RViz runs **hardware-accelerated** (Mesa
`radeonsi`), not on software `llvmpipe`. Section 5 shows how to check yours.

---

## 1. Install the desktop (host)

`lightdm` is only the greeter — it needs a session to log into. Install a light
desktop (Xfce; avoid GNOME/KDE on an RT box):

```bash
sudo apt install xfce4 lightdm mesa-utils
```

Keep the machine booting **headless**, with the desktop **off by default** (so it
never steals CPU from real-time motion unless you ask for it):

```bash
sudo systemctl set-default multi-user.target   # boot to console, no GUI
sudo systemctl disable lightdm                  # do NOT autostart the desktop
```

## 2. Toggle the desktop ON / OFF (host)

```bash
# turn the desktop ON (appears on the physical monitor):
sudo systemctl start lightdm        # or: sudo systemctl isolate graphical.target

# turn it OFF (back to headless console):
sudo systemctl stop lightdm         # or: sudo systemctl isolate multi-user.target
```

Because `lightdm` is **not enabled**, this is purely on-demand and does not
survive a reboot — exactly what we want on the RT box.

## 3. Allow the container to use the host display (host)

Log in at the monitor, open a terminal **inside the Xfce session**, and run:

```bash
xhost +local:        # let local containers connect to the X server
echo $DISPLAY        # note the value — usually :0
```

> If you are SSH'd in rather than at the monitor, `xhost` needs the session's
> display/auth; easiest is to run it from a terminal on the monitor.

---

## 4. Connect the Docker container (docker compose)

The image and compose file are already wired for this:

- **Dockerfile** bakes in `ros-jazzy-rviz2`, `libgl1-mesa-dri` (GPU acceleration —
  without it Mesa falls back to software `llvmpipe`), and `mesa-utils`.
- **compose.yml** defines an on-demand `rviz` sidecar service, gated behind the
  `gui` profile so a plain `docker compose up` never starts it:

```yaml
  rviz:
    image: rsi/ros2-rapidcode:latest   # reuses the image the main service builds
    profiles: ["gui"]                  # NOT started by a plain `docker compose up`
    network_mode: host                 # shares the DDS graph with the workload
    privileged: true                   # GPU (/dev/dri) access
    environment:
      - DISPLAY=${DISPLAY:-:0}
      - QT_X11_NO_MITSHM=1
    volumes:
      - /dev/shm:/dev/shm
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    command: ["bash", "-lc", "rviz2"]
```

**Build once** so the image actually contains rviz2 (this does NOT disturb a
running workload container — it keeps its current image until the next `up -d`):

```bash
docker compose build
```

**Launch RViz on demand** (after the host desktop is on and `xhost +local:` has
run in its session). Force `DISPLAY=:0` so it targets the monitor, not an SSH
X-forward:

```bash
DISPLAY=:0 docker compose --profile gui run --rm rviz
```

`docker compose up -d` is unchanged — the GUI only comes up when you ask for it
with `--profile gui`.

> **Alternative (no compose):** a one-off standalone container, same effect —
> ```bash
> docker run --rm -it --network host --privileged \
>   -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
>   -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /dev/shm:/dev/shm \
>   rsi/ros2-rapidcode:latest bash -lc 'rviz2'
> ```

---

## 5. Verify GPU acceleration

```bash
docker compose --profile gui run --rm rviz bash -lc 'glxinfo -B | grep "OpenGL renderer"'
# expect:  OpenGL renderer string: <your GPU> (e.g. AMD Radeon ... (radeonsi ...))
# NOT:     ... llvmpipe ...   (that means software rendering)
```

---

## 6. Visualize and command motion

With the desktop on and `xhost +local:` run, `./run.sh up phantom` brings up the
robot, `move_group`, the bridge, Servo and RViz with the MoveIt displays already
configured (`rapidcode_moveit_config/launch/moveit_rviz.launch.py`). The manual
equivalents are `RUNBOOK.md` §6, §8 and §10; §11 shows how to plan and execute a
motion from RViz, and §19 the two-axis phantom smoke test that needs no robot
model.

If you launched the plain `rviz2` sidecar of Section 4 instead, set **Global
Options → Fixed Frame** to `world`, then **Add → RobotModel** with *Description
Topic* `/robot_description`, and optionally **Add → TF**. What makes it move:
axis → plugin `read()` → `/joint_states` → `robot_state_publisher` → `/tf` → RViz.

---

## Real-time caution

RViz + the desktop session consume CPU and **perturb real-time determinism**.
Keep `lightdm` disabled (toggle on only when needed), and treat the GUI as a
viz/dev tool — **not** something running during timing-critical motion. The GUI
runs on non-isolated cores; the rmp workload stays on its isolated `cpu_affinity`
core(s).
