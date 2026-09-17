# rapidcode_draw_plane

Two programs:

- **The bridge** (`ros2 run rapidcode_draw_plane bridge`) runs in the container
  with ROS 2. It serves a WebSocket on port 8765, turns the operator's strokes
  and recipes into Cartesian paths, and dispatches them to MoveIt and the
  passthrough trajectory controller. `./run.sh up` starts it.
- **The drawing surface client** (`rapidcode_draw_plane.surface_client`) is the
  operator window. It is pure Python: tkinter plus the `websockets` package, no
  ROS 2. The optional `sv-ttk` package gives it the Sun Valley look; without it
  the client uses the platform's native ttk theme.

## Start the client

From the repository root:

| Where | Command |
|---|---|
| Linux | `./run.sh client [ws://<linux-host>:8765]` |
| Windows | `run_client.bat ws://<linux-host>:8765` |

Both install the missing Python packages on first run. The Linux launcher may
ask for `sudo` once to install `python3-tk` and `python3-venv`. Windows needs
Python 3.10 or newer from python.org with the "tcl/tk and IDLE" component,
which is checked by default. Without a launcher:

```
python -m rapidcode_draw_plane.surface_client ws://<linux-host>:8765
```

run from this directory with `websockets` installed. The address defaults to
the local host.

## The window

The window shows a banner with the active mode, a red STOP button, and a side
panel of grouped sections: Tool (freehand, line, arc, chaining), Drawing (undo,
clear canvas, save, raise pen), Recipe (load), Plane calibration (the board's
origin in millimetres and its rotation in degrees), one Execute button, the
Live mode toggle, and Status (pointer coordinate readout in plane metres, stroke
counts, and the bridge's warnings). The theme in use is named in small print at
the bottom of the panel. The canvas maps one-to-one onto the configured plane
extent; the fine and coarse grids come from the bridge's configuration.

The canvas shows one thing at a time, and Execute runs that thing. Its label
says which: "Execute drawing" for your strokes, "Execute recipe: <name>" for a
loaded recipe, and it is disabled when the canvas is empty. "Load recipe…"
lists the bridge-side recipe files; loading one clears your drawing (the client
asks first when there are strokes) and shows the recipe's pen path as a dashed
purple preview. Drawing a new stroke, or "Clear canvas", drops the preview.

## Notes

- The client sends input at 100 Hz unconditionally; that cadence is the
  bridge's liveness signal. Closing the window, or losing the network, while a
  batch drawing executes stops the arm; during Live it brings the arm to rest.
- The drawing lives in the client while you compose: strokes, undo, clear, and
  the tool selection are instant and work even while disconnected or stopped.
  Execute and Save send the drawing to the bridge; an unsaved drawing is lost
  when the window closes.
- Entering Live mode asks for confirmation: in Live, the pointer moves the arm
  immediately and pen-down draws on the board.
- "Plane calibration" says where the physical board actually sits and how far
  it is turned, so it applies to Batch drawings and Live alike. Type the nudge
  in millimetres, the rotation in degrees counter-clockwise, and press Apply;
  all three go together as one command. It is a Batch command: Live and a
  running execution both refuse it, with the reason in Status, and the bridge
  forgets it on restart. The values are absolute rather than nudges on what is
  already set, so pressing Apply twice changes nothing the second time. The
  banner shows a marker while a calibration is in force, because the canvas is
  drawn in the plane's own frame and so looks the same either way.
- The rotation turns the drawing on the board; the pen's own orientation never
  changes. On a loaded recipe it adds to whatever rotation the recipe file
  authored, so a drawing authored at 30 degrees on a plane calibrated to 45
  comes out at 75.
- The bridge refuses an OFFSET that would put the arm out of reach; with the
  stock configuration that means any offset at all, so see the bridge
  parameters before expecting the x and y fields to move anything. The rotation
  is not bounded this way and works at the stock settings.

## Tests

The bridge and client component tests need no running RMP. Run them in the
container, where ROS 2 and pytest are installed:

```bash
./run.sh shell
cd /ros2_ws/src/rapidcode_draw_plane && python3 -m pytest test/ -q
```
