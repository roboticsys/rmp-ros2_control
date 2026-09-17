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

"""The drawing surface client: the operator-facing window.

Runs on any machine with Python 3, tkinter, and the ``websockets`` package --
no ROS. The optional ``sv_ttk`` package gives the window
the Sun Valley (Windows 11) look; without it the native ttk theme is used. The Windows operator station runs exactly this
module: ``python -m rapidcode_draw_plane.surface_client ws://<bridge>:8765``.

The client owns the drawing: capture, conditioning, primitives,
chaining, undo, clear, and ink rendering are local, through the same drawing
store and linear algebra library the bridge's validation uses. The bridge sees
the drawing only as the submitted drawing carried by an execute or save
control. Composing therefore never waits on the network; only motion commands
do.

Chaining: a surface toggle makes each new stroke
continue the previous one with no pen lift. A chained line or arc inherits its
first point from the previous stroke's end (one click for a line, two for an
arc); a chained freehand stroke must start within CHAIN_TOLERANCE of that end
and is snapped onto it. At execute/save, consecutive chained strokes merge
into one submitted stroke, so the bridge frames the whole sequence with a
single approach and retract and no travel inside it.

Three parts:

* ``SurfaceModel`` -- the headless view-model. It holds the drawing store,
  the active tool, the in-progress capture, the latest display state from the
  bridge, the local pointer/pen state, and the queued control events. No
  tkinter, no sockets: unit-testable anywhere (in spirit).
* ``Transport`` -- one background thread with its own asyncio loop. Sends one
  input message every ``SEND_PERIOD`` unconditionally (-- the cadence
  is the liveness signal), receives display states, reconnects on loss.
* ``run_gui`` -- the tkinter view: canvas mapped to the configured extent,
  grids, ink, mode banner, tool selector, stop control, readout, warnings.
  Ink redraws only when the drawing changes; the stroke in progress is drawn
  incrementally from the pointer events, so redraw cost never grows with the
  drawing. The window sizes itself to the display it opens on and picks a
  layout density to match (see ``pick_metrics``); the side panel scrolls
  whatever a short landscape screen still cannot hold, and STOP, Execute,
  the mode toggle and the status readout never scroll.

Conditioning values, the extent, and the sample bound arrive once from the
bridge in the display state's ``config`` block, during Configure plane.
"""

import asyncio
import math
import sys
import threading
import time
from collections import namedtuple
from typing import List, Optional

from . import linalg, protocol
from .drawing_store import ARC, LINE, POLYLINE, DrawingStore, Stroke

SEND_PERIOD = 0.01       # <= 0.01 s, unconditional
RECONNECT_DELAY = 1.0
STALL_REPORT_GAP = 0.25  # console-report threshold for send-loop stalls
DEFAULT_URL = 'ws://127.0.0.1:8765'

TOOLS = ('freehand', 'line', 'arc')
CLICKS_NEEDED = {'line': 2, 'arc': 3}
# Chaining: a freehand stroke chains only when it starts within
# this distance of the previous stroke's end; its start is then snapped there,
# so chained strokes share their seam point exactly. Primitives inherit their
# start point instead and need no tolerance.
CHAIN_TOLERANCE = 0.005  # metres
# One Execute button (stakeholder 2026-09-04): it runs whatever the canvas
# shows, the loaded recipe or the drawing, never both. A recipe load counts
# as loaded from the click until the bridge's preview arrives, or until this
# many seconds pass (a refused load never answers with a preview).
RECIPE_PENDING_TIMEOUT = 3.0


class SurfaceModel:
    """Thread-shared state between the view and the transport. All access
    goes through one lock; every method is safe from any thread."""

    def __init__(self):
        self._lock = threading.Lock()
        self._seq = 0
        self._pointer = (0.0, 0.0)       # canvas pixels, origin top-left
        self._surface = (800.0, 800.0)   # canvas size in pixels
        self._pen = False
        self._follow = False              # Live-mode engagement
        self._controls: List[dict] = []
        self._display: dict = {}
        self._connected = False
        # The client-owned drawing. The store is created when the
        # bridge's config block first arrives with the sample bound.
        self._store: Optional[DrawingStore] = None
        self._tool = 'freehand'
        self._capture: List[tuple] = []   # in-progress stroke, plane metres
        self._clicks: List[tuple] = []    # primitive click collection
        self._chain = False               # chain strokes: no pen lift
        self._clicks_seeded = False       # _clicks[0] is the inherited start
        self._revision = 0                # bumps on any drawing change
        self._status = ''                 # latest local notice (refusals)
        # Recipe exclusivity (see RECIPE_PENDING_TIMEOUT): a load in flight,
        # and a dismissal in flight (the preview still shows in the display
        # until the bridge answers the clear_recipe).
        self._recipe_pending: Optional[tuple] = None   # (name, deadline)
        self._recipe_dismissed = False

    # --------------------------------------------------------- configuration
    def _config_locked(self) -> Optional[dict]:
        return self._display.get('config')

    def _plane_of_pointer_locked(self) -> Optional[tuple]:
        config = self._config_locked()
        if not config:
            return None
        extent = (tuple(config['extent'][0]), tuple(config['extent'][1]))
        return linalg.clamp_to_extent(linalg.map_canvas_to_plane(
            self._pointer, self._surface, extent), extent)

    # ------------------------------------------------------------- view inputs
    def set_surface(self, width: float, height: float) -> None:
        with self._lock:
            self._surface = (float(width), float(height))

    def set_pointer(self, px: float, py: float) -> None:
        with self._lock:
            self._pointer = (float(px), float(py))
            if self._pen and self._capturing_locked():
                self._capture.append(self._plane_of_pointer_locked())

    def set_follow(self, engaged: bool) -> None:
        """Live-mode follow engagement; no effect on capture."""
        with self._lock:
            self._follow = bool(engaged)

    def set_pen(self, down: bool) -> None:
        with self._lock:
            was_down = self._pen
            self._pen = bool(down)
            if self._store is None or self._display.get('mode') == 'live':
                return  # live strokes land on the board, not in the store
            if down and not was_down:
                self._dismiss_recipe_locked()  # drawing replaces the recipe
            if self._tool == 'freehand':
                if down and self._capturing_locked():
                    self._capture.append(self._plane_of_pointer_locked())
                elif was_down and not down and self._capture:
                    self._terminate_stroke_locked()
            elif down and not was_down:
                self._collect_click_locked()

    def _capturing_locked(self) -> bool:
        return (self._store is not None and self._tool == 'freehand'
                and self._display.get('mode') != 'live')

    def _chain_anchor_locked(self):
        """The previous stroke's final point, when chaining can attach to it
        : chaining is on and the drawing holds at least one stroke."""
        if not self._chain or self._store is None or len(self._store) == 0:
            return None
        return self._store.strokes()[-1].points[-1]

    def _terminate_stroke_locked(self) -> None:
        config = self._config_locked()
        conditioned = linalg.resample_uniform(
            linalg.remove_duplicates(self._capture,
                                     config['duplicate_threshold']),
            config['resample_spacing'])
        self._capture = []
        if len(conditioned) < 2:
            self._status = 'stroke too short after conditioning; discarded'
        else:
            chained = False
            anchor = self._chain_anchor_locked()
            if anchor is not None:
                if math.dist(anchor, conditioned[0]) <= CHAIN_TOLERANCE:
                    conditioned[0] = tuple(anchor)  # share the seam exactly
                    chained = True
                else:
                    self._status = (
                        'stroke not chained: it starts more than '
                        f'{CHAIN_TOLERANCE * 1000:.0f} mm from the '
                        'previous end')
            if not self._store.append(
                    Stroke(POLYLINE, conditioned, chained=chained)):
                self._status = 'drawing is full; stroke refused (sample bound)'
        self._revision += 1  # also signals the view to drop the capture trail

    def _collect_click_locked(self) -> None:
        if not self._clicks:
            anchor = self._chain_anchor_locked()
            if anchor is not None:  # inherited start
                self._clicks.append(tuple(anchor))
                self._clicks_seeded = True
        self._clicks.append(self._plane_of_pointer_locked())
        if len(self._clicks) < CLICKS_NEEDED[self._tool]:
            return
        spacing = self._config_locked()['resample_spacing']
        if self._tool == 'line':
            points = linalg.sample_line(self._clicks[0], self._clicks[1],
                                        spacing)
            stroke = Stroke(LINE, points, params={'start': self._clicks[0],
                                                  'end': self._clicks[1]},
                            chained=self._clicks_seeded)
        else:
            points = linalg.sample_arc(
                self._clicks[0], self._clicks[1], self._clicks[2], spacing)
            stroke = Stroke(ARC, points, params={'first': self._clicks[0],
                                                 'second': self._clicks[1],
                                                 'third': self._clicks[2]},
                            chained=self._clicks_seeded)
        self._clicks = []
        self._clicks_seeded = False
        if not self._store.append(stroke):
            self._status = 'drawing is full; primitive refused (sample bound)'
        self._revision += 1

    # --------------------------------------------------------- drawing commands
    def select_tool(self, tool: str) -> None:
        if tool not in TOOLS:
            return
        with self._lock:
            self._tool = tool
            self._clicks = []
            self._clicks_seeded = False
            self._capture = []

    def set_chain(self, on: bool) -> None:
        """Toggle stroke chaining: while on, new strokes continue
        the previous stroke with no pen lift between them. A half-collected
        primitive is dropped — the toggle changes what its clicks mean."""
        with self._lock:
            self._chain = bool(on)
            self._clicks = []
            self._clicks_seeded = False

    def undo(self) -> None:
        with self._lock:
            if self._store is None or not self._store.undo_last():
                self._status = 'nothing to undo'
                return
            self._revision += 1

    def clear(self) -> None:
        """Clear the canvas: the local drawing and, when one is loaded, the
        bridge-side recipe preview too."""
        with self._lock:
            self._clear_drawing_locked()
            self._dismiss_recipe_locked()

    def _clear_drawing_locked(self) -> None:
        if self._store is not None:
            self._store.clear()
        self._clicks = []
        self._clicks_seeded = False
        self._capture = []
        self._revision += 1

    # ------------------------------------------------------- recipe exclusivity
    def _recipe_loaded_locked(self) -> Optional[str]:
        """The loaded recipe's name, or None. A load in flight counts until
        its preview arrives or its deadline passes; a dismissal in flight
        hides the preview until the bridge drops it."""
        pending = self._recipe_pending
        if pending is not None:
            if time.monotonic() < pending[1]:
                return pending[0]
            self._recipe_pending = None
        preview = self._display.get('preview')
        if preview and not self._recipe_dismissed:
            return preview.get('name') or 'recipe'
        return None

    def _dismiss_recipe_locked(self) -> None:
        """Drop the loaded recipe, queueing clear_recipe only when there is
        one to drop, so composing sends no needless controls."""
        if self._recipe_loaded_locked() is None:
            return
        self._recipe_pending = None
        self._recipe_dismissed = True
        self._controls.append({'action': 'clear_recipe'})

    def execute_target(self) -> Optional[tuple]:
        """What Execute would run: ('recipe', name), ('drawing', None), or
        None when the canvas is empty. The loaded recipe wins because loading
        cleared the drawing and drawing dismissed the recipe: at most one
        exists, and this only settles the moments in between."""
        with self._lock:
            return self._execute_target_locked()

    def _execute_target_locked(self) -> Optional[tuple]:
        name = self._recipe_loaded_locked()
        if name is not None:
            return ('recipe', name)
        if self._store is not None and len(self._store):
            return ('drawing', None)
        return None

    def _drawing_payload_locked(self) -> List[dict]:
        """The submitted drawing. Consecutive chained strokes merge into one
        submitted stroke: downstream framing then commands no lift,
        travel, or plunge inside the sequence. The shared seam point is
        dropped so the merged stroke carries no duplicate."""
        if self._store is None:
            return []
        threshold = self._config_locked()['duplicate_threshold']
        payload: List[dict] = []
        for stroke in self._store.strokes():
            points = [list(point) for point in stroke.points]
            if stroke.chained and payload:
                tail = payload[-1]['points']
                if points and math.dist(points[0], tail[-1]) <= threshold:
                    points = points[1:]
                tail.extend(points)
                payload[-1]['kind'] = 'polyline'  # a merged, mixed-kind run
            else:
                payload.append({'kind': stroke.kind, 'points': points})
        return payload

    def request_execute(self) -> bool:
        """The one Execute: run what the canvas shows. A loaded recipe queues
        execute_recipe; otherwise the submitted drawing queues execute; False
        (with a local notice) when there is nothing to execute."""
        with self._lock:
            target = self._execute_target_locked()
            if target is None:
                self._status = 'nothing to execute'
                return False
            if target[0] == 'recipe':
                self._controls.append({'action': 'execute_recipe'})
                return True
            drawing = self._drawing_payload_locked()
            if not drawing:
                self._status = 'nothing to execute'
                return False
            self._controls.append({'action': 'execute', 'drawing': drawing})
            return True

    def request_save(self, name: str) -> bool:
        with self._lock:
            drawing = self._drawing_payload_locked()
            if not drawing:
                self._status = 'nothing to save'
                return False
            self._controls.append({'action': 'save', 'name': name,
                                   'drawing': drawing})
            return True

    def request_mode(self, mode: str) -> None:
        with self._lock:
            self._controls.append({'action': 'mode', 'mode': mode})
            self._capture = []
            self._clicks = []
            self._clicks_seeded = False
            self._follow = False

    def requeue_controls(self, controls: List[dict]) -> None:
        """Put controls back at the queue's front after a failed send, so a
        command issued during a transport stall is delivered on reconnect
        instead of being lost."""
        if not controls:
            return
        with self._lock:
            self._controls[:0] = controls

    def request_park(self) -> None:
        """Raise the tool to the ready pose so the pen can be
        physically adjusted."""
        with self._lock:
            self._controls.append({'action': 'park'})

    def request_set_plane(self, dx_mm: float, dy_mm: float,
                          yaw_deg: float = 0.0) -> None:
        """Set the plane calibration: where the board actually sits
        and how far it is turned.

        Takes millimetres, because that is how a board gets measured, and
        sends metres, because the wire and the bridge are SI. The rotation is
        degrees on both sides -- it is the unit the operator reads off a
        protractor and the unit the recipe format already uses. Absolute, not
        deltas, so a control re-queued after a transport stall cannot double."""
        offset = [float(dx_mm) / 1000.0, float(dy_mm) / 1000.0]
        with self._lock:
            self._controls.append({'action': 'set_plane', 'offset': offset,
                                   'yaw': float(yaw_deg)})

    def plane_offset_mm(self):
        """The calibration the bridge reports, in millimetres; None before the
        first display arrives. What is in force, not what was typed."""
        with self._lock:
            offset = (self._config_locked() or {}).get('plane_offset')
        if offset is None:
            return None
        return (float(offset[0]) * 1000.0, float(offset[1]) * 1000.0)

    def plane_yaw_deg(self):
        """The plane rotation the bridge reports, degrees CCW; None before the
        first display arrives, or from a bridge that predates the rotation field."""
        with self._lock:
            yaw = (self._config_locked() or {}).get('plane_yaw_deg')
        return None if yaw is None else float(yaw)

    def request_load_recipe(self, name: str, scale=None,
                            fit: bool = False) -> None:
        """Queue a load of a bridge-side recipe (from config.recipes); the
        bridge answers with a preview in the display state. The recipe
        replaces the local drawing: the canvas shows one thing, and Execute
        runs that thing."""
        control = {'action': 'load_recipe', 'name': name}
        if scale is not None:
            control['scale'] = float(scale)
        if fit:
            control['fit'] = True
        with self._lock:
            if self._store is not None and len(self._store):
                self._clear_drawing_locked()
            self._recipe_pending = (name,
                                    time.monotonic() + RECIPE_PENDING_TIMEOUT)
            self._recipe_dismissed = False
            self._controls.append(control)

    def request_execute_recipe(self) -> None:
        """Run the loaded recipe outright (scripted clients); the window
        goes through ``request_execute`` instead."""
        with self._lock:
            self._controls.append({'action': 'execute_recipe'})

    def request_clear_recipe(self) -> None:
        with self._lock:
            self._recipe_pending = None
            self._recipe_dismissed = True
            self._controls.append({'action': 'clear_recipe'})

    def request_stop(self) -> None:
        with self._lock:
            self._controls.append({'action': 'stop'})

    def request_reset(self) -> None:
        with self._lock:
            self._controls.append({'action': 'reset'})

    # -------------------------------------------------------- transport inputs
    def apply_display(self, display: dict) -> None:
        with self._lock:
            self._display = display
            preview = display.get('preview')
            pending = self._recipe_pending
            if pending is not None and preview and \
                    preview.get('name') == pending[0]:
                self._recipe_pending = None  # the load answered
            if not preview:
                self._recipe_dismissed = False  # the bridge dropped it
            config = display.get('config')
            if self._store is None and config and 'max_samples' in config:
                self._store = DrawingStore(
                    max_samples=int(config['max_samples']))

    def set_connected(self, connected: bool) -> None:
        with self._lock:
            self._connected = connected

    # --------------------------------------------------------------- snapshots
    def snapshot(self):
        """One send tick's content: (seq, pointer, surface, pen, controls)."""
        with self._lock:
            controls, self._controls = self._controls, []
            seq = self._seq
            self._seq += 1
            return (seq, self._pointer, self._surface, self._pen,
                    self._follow, controls)

    def view_state(self) -> dict:
        """The cheap per-frame view: no drawing geometry. The view fetches
        ``ink_strokes()`` only when ``revision`` changes."""
        with self._lock:
            return {
                'display': dict(self._display),
                'connected': self._connected,
                'pointer': self._pointer,
                'surface': self._surface,
                'pen': self._pen,
                'follow': self._follow,
                'tool': self._tool,
                'chain': self._chain,
                'revision': self._revision,
                'pending': [list(click) for click in self._clicks],
                'status': self._status,
                'execute': self._execute_target_locked(),
                'strokes': len(self._store) if self._store else 0,
                'samples': self._store.sample_count if self._store else 0,
            }

    def ink_strokes(self) -> List[List[tuple]]:
        """The retained drawing's point lists, for rendering."""
        with self._lock:
            if self._store is None:
                return []
            return [list(stroke.points) for stroke in self._store.strokes()]

    def close_controls(self) -> List[dict]:
        """Best-effort final controls at client exit: stop a running
        execution; a live session rests through the loss watchdog instead."""
        with self._lock:
            if self._display.get('executing'):
                return [{'action': 'stop'}]
            return []

    def readout(self) -> Optional[tuple]:
        """The pointer's plane coordinates; None before config."""
        with self._lock:
            return self._plane_of_pointer_locked()


class Transport:
    """The client transport endpoint: one WebSocket, one background thread."""

    def __init__(self, url: str, model: SurfaceModel,
                 log=lambda text: print(text, flush=True)):
        self._url = url
        self._model = model
        self._log = log
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name='surface-transport', daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3.0)

    def _run(self) -> None:
        asyncio.run(self._session_loop())

    async def _session_loop(self) -> None:
        import websockets
        while not self._stop.is_set():
            try:
                async with websockets.connect(self._url) as websocket:
                    self._log(f'connected to {self._url}')
                    self._model.set_connected(True)
                    await self._session(websocket)
            except (OSError, asyncio.TimeoutError,
                    websockets.exceptions.WebSocketException) as error:
                self._log(f'connection lost: {error!r}')
            finally:
                self._model.set_connected(False)
            if not self._stop.is_set():
                await asyncio.sleep(RECONNECT_DELAY)

    async def _session(self, websocket) -> None:
        receiver = asyncio.ensure_future(self._receive(websocket))
        last_send = time.monotonic()
        try:
            while not self._stop.is_set():
                (seq, pointer, surface, pen, follow,
                 controls) = self._model.snapshot()
                try:
                    await websocket.send(protocol.encode_input(
                        seq, pointer, surface, pen, controls, follow))
                except Exception:
                    self._model.requeue_controls(controls)
                    raise
                now = time.monotonic()
                gap = now - last_send
                if gap > STALL_REPORT_GAP:
                    self._log(f'send gap {gap:.2f} s '
                              f'(transport stalled client-side)')
                last_send = now
                await asyncio.sleep(SEND_PERIOD)
            # Exit: one best-effort final message (stop while executing).
            controls = self._model.close_controls()
            seq, pointer, surface, pen, _, _ = self._model.snapshot()
            await websocket.send(protocol.encode_input(
                seq, pointer, surface, pen, controls))
        finally:
            receiver.cancel()

    async def _receive(self, websocket) -> None:
        async for text in websocket:
            try:
                display = protocol.decode_display(text)
            except protocol.ProtocolError:
                continue
            if display is not None:
                self._model.apply_display(display)


def live_button_state(left: bool, right: bool, armed: bool):
    """(pen, follow) from the held buttons and the marking arm (
    ): marking needs the primary button held AND armed — any button
    release clears the arm, so a release always returns the pen to the travel
    height; a fresh primary press re-arms. Either held button keeps follow
    engaged."""
    if left and armed:
        return True, True
    if left or right:
        return False, True
    return False, False


# --------------------------------------------------------------------- the view
BANNERS = {
    'disconnected': ('DISCONNECTED — reconnecting…', '#666666', 'white'),
    'batch': ('BATCH — composing', '#2e7d32', 'white'),
    'executing': ('EXECUTING — drawing runs on the arm', '#ef6c00', 'white'),
    'live': ('LIVE — POINTER MOVES THE ARM', '#c62828', 'white'),
    'stopped': ('STOPPED', '#b71c1c', 'white'),
}


# --------------------------------------------------------------- window metrics
# The operator station is a landscape panel: wide, but short. The Sun Valley
# theme pads its widgets more generously than the native ttk theme, so the
# fixed side panel ran off the bottom of a 720p or 768p screen and took the
# Recipe, Execute and Live-mode controls with it. Two mechanisms keep the
# window usable on any display:
#
# * a density -- ``pick_metrics`` reads the usable screen height and returns
#   the roomiest layout that fits it, tightening the spacing and the type
#   before it tightens anything else;
# * a scrolling side panel -- see ``run_gui``. STOP, Execute, the mode toggle
#   and the status readout hold fixed rows; only the compose groups scroll.
#
# The density shrinks padding and point sizes, never the buttons themselves:
# every control stays large enough to hit with a stylus or a gloved hand.
Metrics = namedtuple('Metrics', (
    'name',           # reported in the corner of the panel, for support
    'pad',            # one spacing unit inside a group, in pixels
    'gap',            # spacing between groups, in pixels
    'panel_width',    # side-panel column width, in pixels
    'banner_size',    # the rest are font point sizes
    'stop_size',
    'stop_lines',     # STOP button height, in text lines
    'heading_size',
    'body_size',
    'mono_size',
    'tiny_size',
    'list_rows',      # rows shown by the recipe list
    'min_width',      # smallest window width this density still reads in
    'min_height',     # usable screen height this density asks for
))

# Roomiest first: pick_metrics takes the first density the screen can hold.
DENSITIES = (
    Metrics(name='comfortable', pad=8, gap=16, panel_width=220,
            banner_size=14, stop_size=16, stop_lines=2, heading_size=11,
            body_size=10, mono_size=10, tiny_size=8, list_rows=16,
            min_width=760, min_height=860),
    Metrics(name='compact', pad=6, gap=10, panel_width=204,
            banner_size=12, stop_size=14, stop_lines=1, heading_size=10,
            body_size=9, mono_size=9, tiny_size=8, list_rows=12,
            min_width=700, min_height=680),
    Metrics(name='dense', pad=4, gap=6, panel_width=190,
            banner_size=11, stop_size=13, stop_lines=1, heading_size=9,
            body_size=8, mono_size=9, tiny_size=7, list_rows=9,
            min_width=640, min_height=0),
)

# What a window loses to the title bar, the border and the taskbar.
SCREEN_CHROME = (32, 96)     # width, height, in pixels
PREFERRED_SIZE = (1080, 900)  # the window never grows past this


def pick_metrics(usable_height: int) -> Metrics:
    """The roomiest density that fits ``usable_height`` pixels of screen."""
    for metrics in DENSITIES:
        if usable_height >= metrics.min_height:
            return metrics
    return DENSITIES[-1]


def window_size(screen_width: int, screen_height: int):
    """``(width, height, metrics)`` for a window opening on this screen.

    The window never exceeds the screen, so a low-resolution landscape
    display gets a window that fits it instead of one the taskbar clips."""
    usable_w = max(screen_width - SCREEN_CHROME[0], 480)
    usable_h = max(screen_height - SCREEN_CHROME[1], 360)
    return (min(PREFERRED_SIZE[0], usable_w),
            min(PREFERRED_SIZE[1], usable_h),
            pick_metrics(usable_h))


def apply_theme(root) -> str:
    """Theme the ttk widgets. Prefers the Sun Valley theme (``sv_ttk``, an
    optional pure-Python package); falls back to the platform's native ttk
    theme when it is missing. Styling only: nothing here touches the
    transport thread or the refresh loop. Returns the theme name in use."""
    from tkinter import ttk
    try:
        import sv_ttk
    except ImportError:
        sv_ttk = None
    if sv_ttk is not None:
        sv_ttk.set_theme('light')
        return 'sun-valley-light'
    style = ttk.Style(root)
    for name in ('vista', 'xpnative', 'clam'):
        if name in style.theme_names():
            style.theme_use(name)
            return name
    return style.theme_use()


def run_gui(url: str, auto_quit: float = 0.0) -> int:
    import tkinter as tk
    from tkinter import messagebox, simpledialog, ttk

    model = SurfaceModel()
    transport = Transport(url, model)
    transport.start()

    root = tk.Tk()
    root.title(f'rapidcode draw plane — {url}')
    # Size to the display, not to a fixed guess (the operator station is a
    # short landscape screen), and pick the matching density.
    width, height, metrics = window_size(root.winfo_screenwidth(),
                                         root.winfo_screenheight())
    pad, gap = metrics.pad, metrics.gap
    root.geometry(f'{width}x{height}')
    root.minsize(min(metrics.min_width, width), min(420, height))
    theme = apply_theme(root)
    style = ttk.Style(root)
    style.configure('Heading.TLabel',
                    font=('TkDefaultFont', metrics.heading_size, 'bold'))
    style.configure('Muted.TLabel', foreground='#5f6b73',
                    font=('TkDefaultFont', metrics.body_size))
    style.configure('Warning.TLabel', foreground='#b71c1c',
                    font=('TkDefaultFont', metrics.body_size))
    if theme != 'sun-valley-light':
        # Sun Valley ships an Accent.TButton; native themes need a stand-in.
        style.configure('Accent.TButton',
                        font=('TkDefaultFont', metrics.body_size, 'bold'))
    if metrics is not DENSITIES[0]:
        # A short screen buys its height back from the padding around the
        # controls. The controls themselves keep their size.
        style.configure('TButton', padding=(pad, pad // 2))
        style.configure('TLabelframe.Label',
                        font=('TkDefaultFont', metrics.body_size, 'bold'))
    frame_bg = style.lookup('TFrame', 'background') or root.cget('bg')
    root.configure(bg=frame_bg)

    # The banner changes colour with the mode, which plain ttk labels cannot
    # do per state, so it stays a classic label.
    banner = tk.Label(root, text=f'Connecting to {url}…',
                      font=('TkDefaultFont', metrics.banner_size, 'bold'),
                      pady=pad,
                      bg=BANNERS['disconnected'][1],
                      fg=BANNERS['disconnected'][2])
    banner.pack(side=tk.TOP, fill=tk.X)

    body = ttk.Frame(root)
    body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    canvas_holder = tk.Frame(body, bg=frame_bg)
    canvas_holder.pack(side=tk.LEFT, fill=tk.BOTH, expand=True,
                       padx=(gap, pad), pady=gap)
    canvas = tk.Canvas(canvas_holder, bg='white', highlightthickness=1,
                       highlightbackground='#c8ccd0')

    # The side panel is a grid, not a stack. STOP and Reset hold the top
    # rows, and Execute, the mode toggle and the status readout hold the
    # bottom ones, so a short screen can never push them out of sight. Only
    # row 2 -- the compose groups -- takes the leftover height, and it
    # scrolls when that height is not enough.
    panel = ttk.Frame(body, padding=(0, gap, gap, gap))
    panel.pack(side=tk.RIGHT, fill=tk.Y)
    panel.columnconfigure(0, weight=1)
    panel.rowconfigure(2, weight=1)
    # Text wraps at this width, so the column never resizes.
    panel_width = metrics.panel_width

    # --- stop and reset -------------------------------------------
    # STOP keeps a classic button: ttk buttons take no plain fill colour, and
    # this one must stay the largest, reddest thing on the screen.
    stop_button = tk.Button(
        panel, text='STOP', bg='#c62828', fg='white',
        activebackground='#8e0000', activeforeground='white',
        font=('TkDefaultFont', metrics.stop_size, 'bold'), relief=tk.FLAT,
        borderwidth=0, highlightthickness=0, height=metrics.stop_lines,
        cursor='hand2', command=model.request_stop)
    stop_button.grid(row=0, column=0, sticky=tk.EW, pady=(0, pad // 2))
    ttk.Button(panel, text='Reset', command=model.request_reset
               ).grid(row=1, column=0, sticky=tk.EW, pady=(0, gap))

    # --- the scrolling middle -------------------------------------------------
    # A tk canvas scrolls the compose groups; the scrollbar is packed only
    # while it is needed, so a roomy screen shows the plain panel. The
    # canvas asks for little height of its own, which leaves the pinned rows
    # first claim on the panel when the screen is very short.
    scroll_holder = ttk.Frame(panel)
    scroll_holder.grid(row=2, column=0, sticky=tk.NSEW)
    scroller = tk.Canvas(scroll_holder, bg=frame_bg, highlightthickness=0,
                         width=panel_width, height=80, takefocus=0)
    scroller.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    scrollbar = ttk.Scrollbar(scroll_holder, orient=tk.VERTICAL,
                              command=scroller.yview)
    scroller.configure(yscrollcommand=scrollbar.set)
    groups = ttk.Frame(scroller)
    groups_window = scroller.create_window((0, 0), window=groups,
                                           anchor=tk.NW)
    # The last width pushed onto the embedded frame. Writing it again would
    # fire another <Configure> for no gain.
    scroll_state = {'width': 0}

    def sync_scroll(_event=None):
        inner = max(scroller.winfo_width(), 1)
        if inner != scroll_state['width']:
            scroll_state['width'] = inner
            scroller.itemconfigure(groups_window, width=inner)
        needed = groups.winfo_reqheight()
        scroller.configure(scrollregion=(0, 0, inner, needed))
        if needed > scroller.winfo_height() + 1:
            if not scrollbar.winfo_ismapped():
                scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        elif scrollbar.winfo_ismapped():
            scrollbar.pack_forget()
            scroller.yview_moveto(0.0)

    groups.bind('<Configure>', sync_scroll)
    scroller.bind('<Configure>', sync_scroll)

    WHEEL_EVENTS = ('<MouseWheel>', '<Button-4>', '<Button-5>')

    def on_wheel(event):
        if not scrollbar.winfo_ismapped():
            return
        # X11 reports the wheel as buttons 4 and 5; Windows reports a delta.
        step = 1 if event.num == 5 or event.delta < 0 else -1
        scroller.yview_scroll(step * 3, tk.UNITS)

    def wheel_on(_event):
        # Bound while the pointer is over the panel only, so the wheel never
        # steals events from the drawing canvas.
        for sequence in WHEEL_EVENTS:
            scroller.bind_all(sequence, on_wheel)

    def wheel_off(event):
        # Moving onto a child raises Leave as well; that is not an exit.
        if getattr(event, 'detail', '') == 'NotifyInferior':
            return
        for sequence in WHEEL_EVENTS:
            scroller.unbind_all(sequence)

    scroll_holder.bind('<Enter>', wheel_on)
    scroll_holder.bind('<Leave>', wheel_off)

    def section(title):
        frame = ttk.LabelFrame(groups, text=title, padding=pad)
        frame.pack(fill=tk.X, pady=(0, gap))
        return frame

    # --- tool selector, client-local -------------------------------
    tool_frame = section('Tool')
    tool_var = tk.StringVar(value='freehand')

    for name, label in (('freehand', 'Freehand'), ('line', 'Line (2 clicks)'),
                        ('arc', 'Arc (3 clicks)')):
        ttk.Radiobutton(tool_frame, text=label, variable=tool_var, value=name,
                        command=lambda: model.select_tool(tool_var.get())
                        ).pack(anchor=tk.W, pady=1)

    # Chain strokes: while on, a new stroke continues the previous
    # one with no pen lift; a chained line needs 1 click and an arc 2.
    chain_var = tk.BooleanVar(value=False)
    ttk.Checkbutton(tool_frame, text='Chain strokes (no pen up)',
                    variable=chain_var,
                    command=lambda: model.set_chain(chain_var.get())
                    ).pack(anchor=tk.W, pady=(pad, 0))

    # --- drawing controls, client-local ---------------------------------------
    drawing_frame = section('Drawing')
    ttk.Button(drawing_frame, text='Undo stroke', command=model.undo
               ).pack(fill=tk.X, pady=1)
    ttk.Button(drawing_frame, text='Clear canvas', command=model.clear
               ).pack(fill=tk.X, pady=1)

    def save():
        name = simpledialog.askstring(
            'Save drawing', 'Recipe name:', initialvalue='drawing',
            parent=root)
        if name:
            model.request_save(name)

    ttk.Button(drawing_frame, text='Save…', command=save
               ).pack(fill=tk.X, pady=1)

    # --- pen adjustment: park at the retract height ----------------
    ttk.Button(drawing_frame, text='Raise pen (adjust)',
               command=model.request_park).pack(fill=tk.X, pady=(pad, 0))

    # --- bridge-side recipes (stakeholder 2026-08-26) -------------------------
    recipe_frame = section('Recipe')

    def load_recipe():
        view = model.view_state()
        recipes = view['display'].get('config', {}).get('recipes', [])
        # A recipe replaces the drawing (the canvas shows one thing).
        if view['strokes'] and not messagebox.askokcancel(
                'Load recipe',
                'Loading a recipe clears your drawing from the canvas.\n'
                'Save it first if you want to keep it. Continue?',
                parent=root):
            return
        dialog = tk.Toplevel(root)
        dialog.title('Load recipe')
        dialog.configure(bg=frame_bg)
        dialog.transient(root)
        dialog.grab_set()
        content = ttk.Frame(dialog, padding=pad)
        content.pack(fill=tk.BOTH, expand=True)
        # The list shortens with the density: the dialog must fit the same
        # short screen the main window does.
        listbox = tk.Listbox(content, width=44, relief=tk.FLAT,
                             highlightthickness=1,
                             highlightbackground='#c8ccd0',
                             font=('TkDefaultFont', metrics.body_size),
                             height=min(metrics.list_rows,
                                        max(4, len(recipes))))
        for name in recipes:
            listbox.insert(tk.END, name)
        listbox.pack(fill=tk.BOTH, expand=True, pady=(0, pad))
        row = ttk.Frame(content)
        row.pack(fill=tk.X)
        ttk.Label(row, text='Scale (blank = authored):').pack(side=tk.LEFT)
        scale_entry = ttk.Entry(row, width=8)
        scale_entry.pack(side=tk.LEFT, padx=(pad, 0))
        fit_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(content, text='Fit to canvas', variable=fit_var
                        ).pack(anchor=tk.W, pady=(pad, 0))

        def submit():
            selection = listbox.curselection()
            if not selection:
                return
            text = scale_entry.get().strip()
            try:
                scale = float(text) if text else None
            except ValueError:
                return  # an unreadable scale never crosses the wire
            model.request_load_recipe(recipes[selection[0]], scale=scale,
                                      fit=fit_var.get())
            dialog.destroy()

        buttons_row = ttk.Frame(content)
        buttons_row.pack(fill=tk.X, pady=(pad, 0))
        ttk.Button(buttons_row, text='Load', style='Accent.TButton',
                   command=submit).pack(side=tk.LEFT)
        ttk.Button(buttons_row, text='Cancel', command=dialog.destroy
                   ).pack(side=tk.RIGHT)
        listbox.bind('<Double-Button-1>', lambda _event: submit())
        # Keep the dialog on the screen the main window sits on.
        dialog.update_idletasks()
        dialog.maxsize(root.winfo_screenwidth(),
                       root.winfo_screenheight() - SCREEN_CHROME[1])

    ttk.Button(recipe_frame, text='Load recipe…', command=load_recipe
               ).pack(fill=tk.X, pady=1)

    # --- plane calibration ------------------------------------------
    # Where the board actually sits. Setup rather than composition, so it sits
    # below the drawing controls: it is touched once a session, not once a
    # drawing. Millimetres, because that is how a board gets measured.
    # Entries, not spinboxes: the panel grabs the mouse wheel while the
    # pointer is over it, and a spinbox would fight that.
    plane_frame = section('Plane calibration')
    plane_entries = {}
    # x and y in millimetres; the rotation in degrees CCW. The units are in the
    # labels because they differ between the rows, and mixing them silently is
    # how a board ends up 1000 times too far away or a thousandth of a turn off.
    for axis, label in (('x', 'x (mm):'), ('y', 'y (mm):'),
                        ('yaw', 'rot (deg):')):
        plane_row = ttk.Frame(plane_frame)
        plane_row.pack(fill=tk.X, pady=1)
        ttk.Label(plane_row, text=label).pack(side=tk.LEFT)
        plane_entries[axis] = ttk.Entry(plane_row, width=8)
        plane_entries[axis].pack(side=tk.LEFT, padx=(pad, 0))

    def apply_plane():
        try:
            values = [float(plane_entries[field].get().strip() or 0.0)
                      for field in ('x', 'y', 'yaw')]
        except ValueError:
            return  # an unreadable calibration never crosses the wire
        model.request_set_plane(*values)

    def reset_plane():
        for entry in plane_entries.values():
            entry.delete(0, tk.END)
        model.request_set_plane(0.0, 0.0, 0.0)

    ttk.Button(plane_frame, text='Apply', command=apply_plane
               ).pack(fill=tk.X, pady=(pad, 0))
    ttk.Button(plane_frame, text='Reset to configured', command=reset_plane
               ).pack(fill=tk.X, pady=1)
    # What is in force, echoed by the bridge -- not what is typed above. The
    # bridge refuses a calibration that would take the working area out of
    # reach, and this is where that refusal becomes visible.
    plane_readout = ttk.Label(plane_frame, text='in force: —',
                              style='Muted.TLabel')
    plane_readout.pack(anchor=tk.W, pady=(pad // 2, 0))

    # --- the one Execute (stakeholder 2026-09-04) -----------------------------
    # It runs what the canvas shows: the loaded recipe or the drawing. The
    # label names the target, and the button is disabled when there is none.
    # Pinned: the operator must reach it without scrolling.
    execute_button = ttk.Button(panel, text='Execute', style='Accent.TButton',
                                command=model.request_execute)
    execute_button.grid(row=3, column=0, sticky=tk.EW, ipady=pad // 2,
                        pady=(gap, 0))
    execute_button.state(['disabled'])

    # --- mode toggle (indication lives in the banner) ----------------
    def toggle_mode():
        mode = model.view_state()['display'].get('mode', 'batch')
        if mode == 'live':
            model.request_mode('batch')
        elif messagebox.askokcancel(
                'Enter Live mode',
                'In Live mode the arm follows the pointer immediately.\n'
                'Pen down draws on the board. Continue?', parent=root):
            model.request_mode('live')

    mode_button = ttk.Button(panel, text='Enter Live mode',
                             command=toggle_mode)
    mode_button.grid(row=4, column=0, sticky=tk.EW, pady=(pad, 0))

    # --- readout and warnings -------------------------------------------------
    status_frame = ttk.LabelFrame(panel, text='Status', padding=pad)
    status_frame.grid(row=5, column=0, sticky=tk.EW, pady=(gap, 0))
    readout = ttk.Label(status_frame, text='x —, y —',
                        font=('TkFixedFont', metrics.mono_size))
    readout.pack(anchor=tk.W)
    counts = ttk.Label(status_frame, text='strokes 0, samples 0',
                       style='Muted.TLabel')
    counts.pack(anchor=tk.W)
    status_box = ttk.Label(status_frame, text='', style='Muted.TLabel',
                           wraplength=panel_width - pad * 2, justify=tk.LEFT)
    status_box.pack(anchor=tk.W, pady=(pad // 2, 0))
    warnings_box = ttk.Label(status_frame, text='', style='Warning.TLabel',
                             wraplength=panel_width - pad * 2,
                             justify=tk.LEFT)
    warnings_box.pack(anchor=tk.W, pady=(pad // 2, 0))
    # The density and the window size go on the screen: a support call about
    # a cramped panel is then one photograph long.
    ttk.Label(panel, text=f'{theme} · {metrics.name} · {width}×{height}',
              style='Muted.TLabel',
              font=('TkDefaultFont', metrics.tiny_size)
              ).grid(row=6, column=0, sticky=tk.W)

    # Labels reconfigure only when their text changes: the refresh loop runs
    # at 50 Hz on the main thread, and an unchanged .config() is pure waste.
    shown = {}

    def set_text(widget, key, text, **extra):
        if shown.get(key) != (text, tuple(sorted(extra.items()))):
            shown[key] = (text, tuple(sorted(extra.items())))
            widget.config(text=text, **extra)

    # --- canvas geometry -------------------------------------------------------
    state = {'revision': None, 'surface': (0, 0), 'mode': None,
             'last_px': None, 'preview': None, 'started': time.monotonic()}

    def extent_of(display):
        config = display.get('config')
        if not config:
            return None
        return (tuple(config['extent'][0]), tuple(config['extent'][1]))

    def fit_canvas(display):
        extent = extent_of(display) or ((-0.1, -0.1), (0.1, 0.1))
        (x_min, y_min), (x_max, y_max) = extent
        aspect = (x_max - x_min) / (y_max - y_min)
        holder_w = max(canvas_holder.winfo_width(), 50)
        holder_h = max(canvas_holder.winfo_height(), 50)
        width = min(holder_w - 8, int((holder_h - 8) * aspect))
        height = int(width / aspect)
        return max(width, 40), max(height, 40), extent

    def to_px(plane, surface, extent):
        return linalg.map_plane_to_canvas(plane, surface, extent)

    def draw_grid(surface, extent, config):
        canvas.delete('grid')
        (x_min, y_min), (x_max, y_max) = extent
        for spacing, colour in ((config.get('fine_grid', 0.02), '#ececec'),
                                (config.get('coarse_grid', 0.04), '#a0a0a0')):
            if spacing <= 0:
                continue
            x = x_min - (x_min % spacing)
            while x <= x_max:
                px, _ = to_px((x, y_min), surface, extent)
                canvas.create_line(px, 0, px, surface[1],
                                   fill=colour, tags='grid')
                x += spacing
            y = y_min - (y_min % spacing)
            while y <= y_max:
                _, py = to_px((x_min, y), surface, extent)
                canvas.create_line(0, py, surface[0], py,
                                   fill=colour, tags='grid')
                y += spacing
        canvas.tag_lower('grid')

    def draw_ink(surface, extent):
        """Full ink redraw: only on a drawing revision change or a resize."""
        canvas.delete('ink')
        canvas.delete('capture')
        state['last_px'] = None
        for points in model.ink_strokes():
            if len(points) < 2:
                continue
            flat = []
            for point in points:
                flat.extend(to_px(point, surface, extent))
            canvas.create_line(*flat, fill='#1565c0', width=2, tags='ink')

    def preview_key(display):
        """What must change to earn a preview redraw."""
        preview = display.get('preview')
        if not preview:
            return None
        return (preview.get('name'), preview.get('scale'),
                sum(len(line) for line in preview.get('polylines', [])))

    def draw_preview(display, surface, extent):
        """The loaded recipe's pen-down path (bridge-supplied, plane frame),
        kept above the grid and below the operator's ink."""
        canvas.delete('preview')
        preview = display.get('preview')
        if not preview:
            return
        for polyline in preview.get('polylines', []):
            if len(polyline) < 2:
                continue
            flat = []
            for point in polyline:
                flat.extend(to_px(point, surface, extent))
            canvas.create_line(*flat, fill='#9c27b0', width=2,
                               dash=(6, 3), tags='preview')
        canvas.tag_lower('preview')
        canvas.tag_lower('grid')

    def draw_pending(view, surface, extent):
        canvas.delete('pending')
        for click in view['pending']:
            px, py = to_px(click, surface, extent)
            canvas.create_oval(px - 4, py - 4, px + 4, py + 4,
                               outline='#6a1b9a', width=2, tags='pending')

    # --- pointer events: the in-progress stroke draws incrementally ------------
    def trail_tag():
        display = model.view_state()['display']
        if display.get('mode') == 'live':
            return 'live' if not display.get('stopped') else None
        return 'capture' if tool_var.get() == 'freehand' else None

    def extend_trail(event):
        tag = trail_tag()
        if tag is None:
            state['last_px'] = None
            return
        last = state['last_px']
        if last is not None:
            colour = '#455a64' if tag == 'capture' else '#c62828'
            canvas.create_line(last[0], last[1], event.x, event.y,
                               fill=colour, width=2, dash=(3, 2), tags=tag)
        state['last_px'] = (event.x, event.y)

    def on_motion(event):
        model.set_pointer(event.x, event.y)

    def on_drag(event):
        model.set_pointer(event.x, event.y)
        extend_trail(event)

    buttons = {'left': False, 'right': False, 'armed': False}

    def in_live():
        return model.view_state()['display'].get('mode') == 'live'

    def apply_buttons():
        # left (armed) = follow at pen-down depth, right =
        # follow at the travel height, neither = disengaged. ANY release
        # clears the arm, so the pen always lifts on release; a fresh left
        # press re-arms marking.
        pen, follow = live_button_state(
            buttons['left'], buttons['right'], buttons['armed'])
        model.set_pen(pen)
        model.set_follow(follow)

    def on_press(event):
        model.set_pointer(event.x, event.y)
        state['last_px'] = (event.x, event.y)
        buttons['left'] = True
        buttons['armed'] = True
        if in_live():
            apply_buttons()
        else:
            model.set_pen(True)

    def on_release(event):
        model.set_pointer(event.x, event.y)
        buttons['left'] = False
        buttons['armed'] = False
        if in_live():
            apply_buttons()
        else:
            model.set_pen(False)
        state['last_px'] = None

    def on_press_right(event):
        model.set_pointer(event.x, event.y)
        buttons['right'] = True
        if in_live():
            apply_buttons()

    def on_release_right(event):
        model.set_pointer(event.x, event.y)
        buttons['right'] = False
        buttons['armed'] = False
        if in_live():
            apply_buttons()

    canvas.bind('<Motion>', on_motion)
    canvas.bind('<B1-Motion>', on_drag)
    canvas.bind('<B3-Motion>', on_motion)
    canvas.bind('<ButtonPress-1>', on_press)
    canvas.bind('<ButtonRelease-1>', on_release)
    canvas.bind('<ButtonPress-3>', on_press_right)
    canvas.bind('<ButtonRelease-3>', on_release_right)

    def refresh():
        view = model.view_state()
        display = view['display']
        config = display.get('config', {})

        width, height, extent = fit_canvas(display)
        surface = (float(width), float(height))
        resized = (width, height) != state['surface']
        if resized:
            state['surface'] = (width, height)
            canvas.config(width=width, height=height)
            canvas.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
            model.set_surface(width, height)
            draw_grid(surface, extent, config)

        if resized or view['revision'] != state['revision']:
            state['revision'] = view['revision']
            draw_ink(surface, extent)
        if resized or preview_key(display) != state['preview']:
            state['preview'] = preview_key(display)
            draw_preview(display, surface, extent)
        draw_pending(view, surface, extent)

        # The live trail marks what was drawn on the board this live session.
        mode = display.get('mode')
        if mode != state['mode']:
            state['mode'] = mode
            canvas.delete('live')

        # Banner: stopped wins, then mode.
        if not view['connected']:
            key = 'disconnected'
        elif display.get('stopped'):
            key = 'stopped'
        else:
            key = mode or 'batch'
        text, background, foreground = BANNERS.get(key, BANNERS['batch'])
        if key == 'stopped' and display.get('cause'):
            text = f"STOPPED: {display['cause']} — reset to continue"
        # A moved or turned plane is otherwise invisible: the canvas IS the
        # plane frame, so it looks identical however the board is calibrated
        #.
        offset_mm = model.plane_offset_mm()
        yaw_deg = model.plane_yaw_deg() or 0.0
        if offset_mm is not None and (any(offset_mm) or yaw_deg):
            text = (f'{text}   [plane {offset_mm[0]:+.1f}, '
                    f'{offset_mm[1]:+.1f} mm, {yaw_deg:+.1f}°]')
        set_text(banner, 'banner', text, bg=background, fg=foreground)
        set_text(plane_readout, 'plane', 'in force: —' if offset_mm is None
                 else (f'in force: {offset_mm[0]:+.1f}, {offset_mm[1]:+.1f} mm, '
                       f'{yaw_deg:+.1f}°'))
        set_text(mode_button, 'mode', 'Return to Batch' if mode == 'live'
                 else 'Enter Live mode')

        plane = model.readout()
        set_text(readout, 'readout', 'x —, y —' if plane is None else
                 f'x {plane[0]:+.3f} m, y {plane[1]:+.3f} m')
        set_text(counts, 'counts', f"strokes {view['strokes']}, "
                                   f"samples {view['samples']}")
        target = view['execute']
        if target is None:
            set_text(execute_button, 'execute', 'Execute (canvas empty)',
                     state=tk.DISABLED)
        elif target[0] == 'recipe':
            name = target[1] if len(target[1]) <= 24 else target[1][:23] + '…'
            set_text(execute_button, 'execute', f'Execute recipe: {name}',
                     state=tk.NORMAL)
        else:
            set_text(execute_button, 'execute', 'Execute drawing',
                     state=tk.NORMAL)
        set_text(status_box, 'status', view['status'])
        set_text(warnings_box, 'warnings',
                 '\n'.join(display.get('warnings', [])[-3:]))

        if auto_quit and time.monotonic() - state['started'] > auto_quit:
            root.destroy()
            return
        root.after(20, refresh)

    def on_close():
        transport.stop()   # sends the best-effort final controls
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    root.after(50, refresh)
    root.mainloop()
    transport.stop()
    return 0


def smoke(url: str, seconds: float) -> int:
    """Headless transport check: connect, hold the cadence, drag a short
    freehand stroke into the local store, and report the outcome. No GUI."""
    model = SurfaceModel()
    transport = Transport(url, model)
    transport.start()
    deadline = time.monotonic() + seconds
    time.sleep(1.0)
    if not model.view_state()['connected']:
        print('smoke: never connected', flush=True)
        transport.stop()
        return 1
    model.set_surface(800, 800)
    model.set_pointer(200, 400)
    time.sleep(0.2)
    model.set_pen(True)
    for step in range(40):
        model.set_pointer(200 + step * 10, 400)
        time.sleep(0.02)
    model.set_pen(False)
    while time.monotonic() < deadline:
        time.sleep(0.1)
    view = model.view_state()
    display = view['display']
    print(f"smoke: connected={view['connected']} mode={display.get('mode')} "
          f"strokes={view['strokes']} samples={view['samples']} "
          f"config={'yes' if display.get('config') else 'no'}", flush=True)
    transport.stop()
    ok = view['connected'] and display.get('config') and view['strokes'] >= 1
    return 0 if ok else 1


def bare(url: str, seconds: float) -> int:
    """Transport-plus-window bisect test: the real transport thread under an
    EMPTY tkinter window -- no canvas, no refresh loop, no bindings. If the
    send loop stalls here (watch the console for 'send gap' lines and the
    bridge log for 'client input lost'), the tkinter mainloop itself starves
    the transport on this machine; if it stays clean, the fault is in the
    full GUI's rendering path."""
    import tkinter as tk
    model = SurfaceModel()
    transport = Transport(url, model)
    transport.start()
    root = tk.Tk()
    root.title('bare transport test')
    tk.Label(root, text='bare transport test - watch the console',
             padx=20, pady=20).pack()
    root.after(int(seconds * 1000), root.destroy)
    root.mainloop()
    view = model.view_state()
    print(f"bare: connected={view['connected']} "
          f"config={'yes' if view['display'].get('config') else 'no'}",
          flush=True)
    transport.stop()
    return 0 if view['connected'] else 1


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    url = DEFAULT_URL
    auto_quit = 0.0
    smoke_seconds = 0.0
    bare_seconds = 0.0
    while argv:
        argument = argv.pop(0)
        if argument == '--auto-quit' and argv:
            auto_quit = float(argv.pop(0))
        elif argument == '--smoke' and argv:
            smoke_seconds = float(argv.pop(0))
        elif argument == '--bare' and argv:
            bare_seconds = float(argv.pop(0))
        elif argument.startswith('ws://'):
            url = argument
        else:
            print(f'usage: surface_client [ws://host:port] '
                  f'[--auto-quit S] [--smoke S] [--bare S]', flush=True)
            return 2
    if smoke_seconds:
        return smoke(url, smoke_seconds)
    if bare_seconds:
        return bare(url, bare_seconds)
    return run_gui(url, auto_quit=auto_quit)


if __name__ == '__main__':
    raise SystemExit(main())
