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

"""Dispatcher: the bridge's mode state machine and per-sample coordinate path.

Pure Python -- no ROS imports. The bridge node injects the route components
(batch executor, live target streamer), the file-manager functions, and the
display callback; every coordinate decision is delegated to the linear algebra
library. All methods run on the coordination thread (the bridge node's rclpy
executor); the routes cross to their own threads inside their own components.

The client owns the drawing: capture, conditioning, undo, and clear
never reach the bridge. The dispatcher sees the drawing only as the submitted
drawing carried by an execute or save control, validates it -- sample bound,
extent clamp, working-area split -- and never retains it past the command.

Modes: ``batch`` (composing), ``executing``, ``live``, all shadowed by the
``stopped`` latch: while stopped, no motion is admitted until the
operator's reset, which clears the controller's latch through the returning
route before any motion is commanded.

Live sub-states (stakeholder 2026-08-25, fidelity over transition latency):
``follow`` -- pen-up pointer tracking on the Servo route (LiveTargetStreamer);
``draw`` -- pen-down strokes on the stroke route (StrokeChainStreamer), which
executes them as sized trajectory goals, including the pen-up lift / travel /
plunge between quick successive strokes; ``draining`` -- the pen came up and
the chain is finishing the drawn strokes, after which the route's
``chain_drained`` event hands control back to Servo. Without an injected
stroke route the dispatcher keeps the older behavior (both pen states on
Servo, depth-selected).
"""

from typing import Callable, List, NamedTuple, Optional, Tuple

from . import file_manager, linalg
from .recipe_loader import RecipeLoadError

Coordinate = Tuple[float, float]

BATCH = 'batch'
EXECUTING = 'executing'
LIVE = 'live'

# Live sub-states.
FOLLOW = 'follow'
DRAW = 'draw'
DRAINING = 'draining'


class DispatcherConfig(NamedTuple):
    extent: linalg.Extent
    working_centre: Coordinate
    working_radius: float
    duplicate_threshold: float
    resample_spacing: float      #  and primitive sampling
    min_run: int                 #  short-fragment exclusion
    plane: file_manager.PlaneConfig
    recipe_directory: str
    fine_grid: float = 0.02      # rendered by the client
    coarse_grid: float = 0.04
    max_samples: int = 20000     #  submitted-drawing bound
    live_queue_limit: int = 2000  #  waypoint-queue bound, points
    # The arm's reach band in the planning frame, the annulus the
    # plane's working disc must stay inside once the operator can move it.
    # Defaults are the 0.30...0.62 m band the plane radius derives from.
    reach_inner: float = 0.30
    reach_outer: float = 0.62


class Dispatcher:
    def __init__(self, config: DispatcherConfig, batch_route, live_route,
                 send_display: Callable[[dict], None],
                 log: Callable[[str], None] = lambda text: None,
                 stroke_route=None, recipe_loader=None,
                 on_plane_change: Callable = None):
        self._config = config
        self._batch = batch_route
        self._live = live_route
        self._stroke = stroke_route
        self._recipe_loader = recipe_loader
        self._send_display = send_display
        self._log = log
        # The bridge keeps its own reference to the configuration, so a plane
        # change has to reach it too; without this it would go stale.
        self._on_plane_change = on_plane_change or (lambda plane: None)

        self._mode = BATCH
        self._stopped = False
        self._live_following = False
        self._live_sub = FOLLOW
        self._cause = ''
        self._warnings: List[str] = []
        self._echo: Optional[Coordinate] = None
        self._loaded_recipe = None  # LoadedRecipe held for execute_recipe

    # ------------------------------------------------------------------ startup
    @staticmethod
    def check_startup(config: DispatcherConfig) -> Optional[str]:
        """The startup refusal check; None when the config is sound."""
        if not linalg.extent_fits_working_area(
                config.extent, config.working_centre, config.working_radius):
            return (f'extent {config.extent} does not fit the working area '
                    f'(centre {config.working_centre}, '
                    f'radius {config.working_radius})')
        if config.plane.draw_z >= config.plane.safe_z:
            return (f'draw_z {config.plane.draw_z} must lie below safe_z '
                    f'{config.plane.safe_z}')
        if config.plane.retract_z <= config.plane.safe_z:
            return (f'retract_z {config.plane.retract_z} must lie above '
                    f'safe_z {config.plane.safe_z}')
        return Dispatcher.reach_refusal(config, config.plane)

    @staticmethod
    def reach_refusal(config: DispatcherConfig, plane) -> Optional[str]:
        """The plane's working disc must sit inside the reach band;
        None when it does.

        Shared by startup and the runtime control, so a launch parameter and an
        operator command are judged by one rule. Nothing else bounds the plane
        origin against reach: the working area is plane-relative and travels
        with it, and the live route has no downstream reachability gate at all
        (the batch route at least fails its anchor IK).

        The plane's rotation turns the working centre about the plane
        origin, so it is applied here too. With the default working centre at
        the origin this is a no-op and a rotation costs no reach budget at any
        angle -- a disc turned about its own centre is the same disc. It stops
        being a no-op the moment somebody moves the working centre off the
        origin, which is exactly when a silent omission would bite.

        The check stays purely planar: two vertical cylinders. It says nothing
        about wrist range, and a large plane rotation changes the wrist
        excursion across the extent considerably. Prove a new rotation on the
        batch route, which at least fails its anchor IK cleanly.
        """
        anchor = plane.effective_anchor()
        turned = linalg.rotate_in_plane(config.working_centre,
                                        plane.plane_yaw_deg)
        centre = (anchor[0] + turned[0], anchor[1] + turned[1])
        if linalg.disc_fits_reach_band(centre, config.working_radius,
                                       config.reach_inner, config.reach_outer):
            return None
        return (f'working area at planning-frame {centre[0]:.3f}, '
                f'{centre[1]:.3f} (radius {config.working_radius}) leaves the '
                f'reach band {config.reach_inner}...{config.reach_outer} m')

    # ------------------------------------------------------------- input events
    def on_input_event(self, event: dict) -> None:
        """One timestamped canvas sample: map, clamp, and route by mode."""
        plane = linalg.map_canvas_to_plane(
            event['pointer'], event['surface'], self._config.extent)
        plane = linalg.clamp_to_extent(plane, self._config.extent)
        pen = event['pen']
        self._echo = plane

        for control in event.get('controls', []):
            self.on_control(control)

        if self._stopped or self._mode != LIVE:
            return  # composing is client-local; no motion while held

        # Follow engagement: the arm tracks the pointer only
        # while a follow control is held. Disengaging lifts to the travel
        # height at the last position and settles (the streamer's release).
        if event.get('follow', False):
            plane = linalg.clamp_to_working_area(
                plane, self._config.working_centre, self._config.working_radius)
            self._live_following = True
            if self._stroke is None:
                depth = (self._config.plane.draw_z if pen
                         else self._config.plane.safe_z)
                self._live.set_intent((plane[0], plane[1], depth))
            elif pen:
                self._route_pen_down(plane)
            else:
                self._route_pen_up(plane)
        elif self._live_following:
            self._live_following = False
            if self._live_sub == FOLLOW:
                self._live.release()
            else:
                self._finish_strokes()

    def _route_pen_down(self, plane: Coordinate) -> None:
        """A pen-down sample: strokes belong to the goal-chain route. Entering
        draw rests the Servo route by instant silence (never release: the
        chain's own plunge replaces the lift); a pen-down while the chain is
        still draining keeps it (travel + plunge to the new stroke)."""
        if self._live_sub == DRAW:
            self._stroke.add_point(plane)
            return
        if self._live_sub == FOLLOW:
            self._live.rest()
        self._stroke.begin_stroke(plane)
        self._live_sub = DRAW

    def _route_pen_up(self, plane: Coordinate) -> None:
        """A pen-up sample while following: Servo tracks at the travel height.
        Samples while the chain drains are dropped -- the arm is finishing the
        drawn strokes; Servo re-engages on the chain_drained event."""
        if self._live_sub == FOLLOW:
            self._live.set_intent(
                (plane[0], plane[1], self._config.plane.safe_z))
        elif self._live_sub == DRAW:
            self._finish_strokes()

    def _finish_strokes(self) -> None:
        """Close the open stroke as drawn and ask the chain to drain."""
        self._stroke.end_stroke()
        self._stroke.request_handback()
        self._live_sub = DRAINING

    # ----------------------------------------------------------------- controls
    def on_control(self, control: dict) -> None:
        action = control.get('action')
        if action == 'stop':
            self._enter_stopped('operator stop')
        elif action == 'reset':
            self._reset()
        elif action == 'save':
            self._save(control)  # commands no motion; available while stopped
        elif action == 'load_recipe':
            self._load_recipe(control)  # commands no motion either
        elif action == 'clear_recipe':
            self._loaded_recipe = None
        elif action == 'set_plane':
            self._set_plane(control)  # commands no motion either
        elif self._stopped:
            self._warn(f'{action} ignored: stopped ({self._cause}); reset first')
        elif action == 'execute':
            self._execute(control)
        elif action == 'execute_recipe':
            self._execute_recipe()
        elif action == 'park':
            self._park()
        elif action == 'mode':
            self._switch_mode(control['mode'])
        self._push_display()

    def _submitted_runs(self, control: dict) -> Optional[List[List[Coordinate]]]:
        """The submitted drawing's in-area runs (admission checks 7-10), or
        None when the submission is refused. Points are re-clamped to the
        extent: the bridge validates, it does not trust the client."""
        drawing = control.get('drawing') or []
        total = sum(len(stroke.get('points', [])) for stroke in drawing)
        if total > self._config.max_samples:
            self._warn(f'drawing refused: {total} samples exceed the bound '
                       f'({self._config.max_samples})')
            return None
        runs: List[List[Coordinate]] = []
        for stroke in drawing:
            points = [linalg.clamp_to_extent((float(p[0]), float(p[1])),
                                             self._config.extent)
                      for p in stroke.get('points', [])]
            in_runs, excluded = linalg.split_at_working_area(
                points, self._config.working_centre,
                self._config.working_radius, self._config.min_run)
            runs.extend(in_runs)
            if excluded and any(len(run) > 0 for run in excluded):
                self._warn(f'{sum(len(r) for r in excluded)} point(s) outside the '
                           'working area excluded from execution')
        return runs

    def _execute(self, control: dict) -> None:
        if self._mode != BATCH:
            self._warn('execute is a Batch command')
            return
        runs = self._submitted_runs(control)
        if runs is None:
            return
        if not runs:
            self._warn('nothing to execute')
            return
        structure = file_manager.to_recipe_structure(runs, self._config.plane)
        self._mode = EXECUTING
        self._batch.execute(structure)

    def _set_plane(self, control: dict) -> None:
        """Adopt the operator's plane calibration: where the board actually
        sits, as an absolute planning-frame x/y offset in metres and
        an absolute in-plane rotation in degrees CCW.

        Absolute, not a delta, so a repeated or re-queued control cannot
        accumulate. It commands no motion, which is why it sits above the
        stopped pivot -- but it is still a Batch command. In Live the intent is
        held in plane coordinates and remapped every tick, so moving the frame
        under it would jump the arm; while Executing it would change the frame
        mid-drawing. A stop does not leave Executing, so after a stopped
        execution the operator resets first, then calibrates.

        ``yaw`` is optional so a client that predates the rotation still speaks
        this control; absent, the rotation in force is left as it is rather
        than zeroed, because an old client cannot know it exists to resend it.
        """
        if self._mode != BATCH:
            self._warn(f'set plane is a Batch command (mode is {self._mode})')
            return
        offset = (float(control['offset'][0]), float(control['offset'][1]))
        yaw = self._config.plane.plane_yaw_deg
        if control.get('yaw') is not None:
            yaw = float(control['yaw'])
        plane = self._config.plane._replace(plane_offset=offset,
                                            plane_yaw_deg=yaw)
        refusal = self.reach_refusal(self._config, plane)
        if refusal is not None:
            self._warn(f'plane calibration refused: {refusal}')
            return
        self._config = self._config._replace(plane=plane)
        # Every holder of the configuration, or a route and the dispatcher
        # disagree about where the board is. Safe without a lock only because
        # the Batch-mode guard above means neither route is streaming.
        self._live.set_plane(plane)
        if self._stroke is not None:
            self._stroke.set_plane(plane)
        self._on_plane_change(plane)
        # The loaded recipe was re-anchored and bound-checked against the old
        # plane; drop it rather than execute a stale anchor.
        self._loaded_recipe = None
        self._log(f'plane offset {offset[0]:+.4f}, {offset[1]:+.4f} m, '
                  f'rotation {yaw:+.2f} deg')

    def _park(self) -> None:
        """Raise the tool to the ready pose so the operator can
        physically adjust the pen tool.

        A joint PTP, not a recipe (2026-09-10): the ready pose already sits at
        the clearance the operator wants, and every execution opens by moving
        there anyway. Routing park through a recipe made the arm approach the
        plane origin first -- a descent to the board between two raised poses,
        for no gain -- because the sender approaches a recipe's anchor."""
        if self._mode != BATCH:
            self._warn('park is a Batch command')
            return
        plane = self._config.plane
        self._mode = EXECUTING
        self._batch.park(plane.ready_joints, plane.velocity, plane.acceleration)

    def _load_recipe(self, control: dict) -> None:
        """Load a bridge-side recipe file: verbatim, re-anchored to the plane
        (stakeholder 2026-08-26). The working area refuses; the extent only
        warns. The loaded doc is held (and previewed) until execute_recipe,
        clear_recipe, or a replacing load; a failed load keeps the previous
        one. Commands no motion, so it is available while stopped."""
        if self._mode != BATCH:
            self._warn('load recipe is a Batch command')
            return
        if self._recipe_loader is None:
            self._warn('recipe loading unavailable')
            return
        scale = control.get('scale')
        try:
            loaded = self._recipe_loader.load(
                control.get('name', ''), self._config.plane,
                self._config.extent, self._config.working_centre,
                self._config.working_radius,
                scale=None if scale is None else float(scale),
                fit=bool(control.get('fit', False)))
        except RecipeLoadError as error:
            self._warn(f'recipe load failed: {error}')
            return
        self._loaded_recipe = loaded
        for note in loaded.notes:
            self._warn(note)
        self._log(f'loaded recipe {loaded.name} at scale {loaded.scale:.4g}')

    def _execute_recipe(self) -> None:
        """Execute the loaded recipe doc through the batch route, exactly like
        a submitted drawing's structure but with no clamping -- the loader
        already validated the verbatim footprint."""
        if self._mode != BATCH:
            self._warn('execute recipe is a Batch command')
            return
        if self._loaded_recipe is None:
            self._warn('no recipe loaded')
            return
        self._mode = EXECUTING
        self._batch.execute(self._loaded_recipe.doc)

    def _save(self, control: dict) -> None:
        runs = self._submitted_runs(control)
        if runs is None:
            return
        structure = file_manager.to_recipe_structure(runs, self._config.plane)
        try:
            path = file_manager.save_recipe(
                structure, control.get('name') or 'drawing',
                self._config.recipe_directory)
        except (OSError, ValueError) as error:
            self._warn(f'save failed: {error}')
            return
        self._log(f'saved {path}')

    def _switch_mode(self, mode: str) -> None:
        if self._mode == EXECUTING:
            self._warn('mode is locked while a drawing executes')
            return
        if mode == LIVE and self._mode != LIVE:
            if self._live.activate():
                self._mode = LIVE
                self._live_sub = FOLLOW
            else:
                self._warn('live mode unavailable (Cartesian streaming node '
                           'or controller not ready)')
        elif mode == BATCH and self._mode == LIVE:
            if self._stroke is not None and self._stroke.active:
                self._warn('mode is locked while strokes execute')
                return
            self._live.rest()
            self._live_following = False
            self._live_sub = FOLLOW
            self._mode = BATCH

    def _reset(self) -> None:
        if not self._stopped:
            self._warn('nothing to reset')
            return
        # In Live the stroke route clears the shared controller latch (and its
        # own commanded-stop flag); it refuses while a chain still winds down.
        if self._mode != LIVE:
            route = self._batch
        else:
            route = self._stroke if self._stroke is not None else self._live
        if not route.reset():
            self._warn('reset failed: the controller did not clear its latch')
            return
        self._stopped = False
        self._cause = ''
        if self._mode == EXECUTING:
            self._mode = BATCH  # the stopped execution is abandoned, not resumed

    # -------------------------------------------------------------- route events
    def on_route_event(self, event: dict) -> None:
        kind = event.get('kind')
        if kind == 'result':
            if self._mode == EXECUTING:
                self._mode = BATCH
            if not event.get('ok'):
                self._warn(f"execution failed: {event.get('message', '')}")
        elif kind == 'warning':
            self._warn(event.get('message', ''))
        elif kind == 'health' and not event.get('ok'):
            self._enter_stopped(f"interface lost: {event.get('message', '')}")
        elif kind == 'stroke':
            self._on_stroke_event(event)
        self._push_display()

    def _on_stroke_event(self, event: dict) -> None:
        name = event.get('event')
        if name == 'chain_failed':
            self._warn(f"stroke chain failed: {event.get('message', '')}")
        elif (name == 'chain_drained' and self._mode == LIVE
                and self._live_sub == DRAINING):
            # The arm is at rest: hand control back to Servo if the operator
            # is still following (and nothing stopped us meanwhile). A stale
            # drain report after the pen re-engaged (sub back to draw) is
            # ignored -- the new stroke owns the route.
            self._live_sub = FOLLOW
            if (not self._stopped and self._live_following
                    and not self._live.activate()):
                self._warn('live follow unavailable after the stroke chain')

    def on_input_loss(self) -> None:
        """Client silence or disconnect."""
        if self._mode == EXECUTING:
            self._enter_stopped('client input lost during execution')
        elif self._mode == LIVE:
            if self._stroke is not None and self._live_sub == DRAW:
                self._finish_strokes()  # the drawn strokes finish (fidelity)
            self._live.rest()
            self._live_following = False
        self._push_display()

    def _enter_stopped(self, cause: str) -> None:
        already = self._stopped
        self._stopped = True
        self._cause = cause
        if not already:
            if self._mode == LIVE:
                # Both live routes: each stop is idempotent, and either engine
                # may hold the open motion when the operator hits stop.
                self._live.stop()
                if self._stroke is not None:
                    self._stroke.stop()
                self._live_sub = FOLLOW
            else:
                self._batch.stop()
        self._push_display()

    # ------------------------------------------------------------------ display
    def display_state(self) -> dict:
        state = {
            'mode': self._mode,
            'stopped': self._stopped,
            'cause': self._cause,
            'echo': list(self._echo) if self._echo else None,
            'executing': self._mode == EXECUTING,
            'warnings': self._warnings[-5:],
            # The loaded recipe's pen-down path, plane frame, for the client's
            # preview overlay (decimated by the loader; display-only).
            'preview': ({
                'name': self._loaded_recipe.name,
                'scale': self._loaded_recipe.scale,
                'polylines': [[list(point) for point in polyline]
                              for polyline in self._loaded_recipe.polylines],
            } if self._loaded_recipe is not None else None),
            # Surface and capture configuration for the client (
            # the client conditions and bounds with these values):
            'config': {
                'extent': [list(self._config.extent[0]),
                           list(self._config.extent[1])],
                'fine_grid': self._config.fine_grid,
                'coarse_grid': self._config.coarse_grid,
                'working_centre': list(self._config.working_centre),
                'working_radius': self._config.working_radius,
                'duplicate_threshold': self._config.duplicate_threshold,
                'resample_spacing': self._config.resample_spacing,
                'max_samples': self._config.max_samples,
                # The plane calibration in force, and the band bounding it
                #, so the client can show and refuse
                # sensibly. Degrees for the rotation, matching the field the
                # operator types into and the recipe format's own `rotate`.
                'plane_offset': list(self._config.plane.plane_offset),
                'plane_yaw_deg': self._config.plane.plane_yaw_deg,
                'reach_inner': self._config.reach_inner,
                'reach_outer': self._config.reach_outer,
                'recipes': (self._recipe_loader.list_names()
                            if self._recipe_loader is not None else []),
            },
        }
        return state

    def push_display(self) -> None:
        """Send the full display state now (e.g. to a newly connected client)."""
        self._push_display()

    def _push_display(self) -> None:
        self._send_display(self.display_state())

    def _warn(self, message: str) -> None:
        self._warnings.append(message)
        self._log(message)

    # Introspection for tests and the bridge node.
    @property
    def mode(self) -> str:
        return self._mode

    @property
    def stopped(self) -> bool:
        return self._stopped

    @property
    def plane(self):
        """The plane configuration in force, calibration included."""
        return self._config.plane

    @property
    def live_sub(self) -> str:
        return self._live_sub
