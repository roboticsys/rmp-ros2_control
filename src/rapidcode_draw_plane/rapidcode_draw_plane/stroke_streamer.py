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

"""Stroke chain streamer: pen-down strokes as streamed sized trajectory goals.

The third route (stakeholder 2026-08-25): pen-DOWN drawing executes as sized
FollowJointTrajectory goals streamed through the passthrough controller --
full trajectory fidelity -- while pen-UP following stays on the Servo route
(LiveTargetStreamer). Owns the stroke thread: planning calls block
for whole windows, so they run here, never on the coordination or live
threads, on a dedicated rclpy node this component owns and spins.

The chain contract (controller + hardware, verified 2026-08-25):
* Goals queue behind each other while the firmware move stays open; the move
  stays open only while each goal's last chunk carries finalize=false, and
  the controller reads that from its `finalize_last_chunk` parameter AT GOAL
  ACCEPT. The policy is therefore this client's responsibility.
* SAFETY: a finalize=false goal whose frames drain with no successor fed
  e-stops the firmware in ~32 ms (OUT_OF_FRAMES). A goal is submitted with
  finalize=false ONLY when its successor is already planned (the streamed
  sender's one-goal emission lag, reused verbatim); every other exit from
  streaming submits finalize=true or rides the controller's own ~/stop.
* The controller rejects goals while a Servo (online) session is open, and
  `online_busy_` clears at stop-tail EMISSION, not at rest; goal 0 is planned
  from measured joints and continuity-checked against live state (0.05 rad).
  So a chain opens only after the settle probe (position-delta quiescence on
  /joint_states) says the arm is at rest.
* Mid-stroke catch-up (the intake runs dry while goals execute) closes the
  chain at rest at draw depth -- a dwell mark, accepted (stakeholder
  2026-08-25: fidelity over transition latency; no auto-lift). The next point
  reopens a chain from measured joints.

Between strokes the chain stays open: end_stroke appends the lift to the
travel height and the next begin_stroke appends travel + plunge, all as
ordinary chain waypoints (the recipe structure's per-run shape), so quick
successive strokes flow with no Servo hand-back and no reopen dwell.

Reuses the streamed sender's internals (GoalPlanner, GoalPipeline,
FinalizeSwitch, retime_goal_window, build_goal_message) as a library, exactly
like the batch route reuses run_streamed.
"""

import math
import threading
import time
from typing import Callable, List, Optional, Tuple

from . import linalg
from .live_streamer import plane_to_world

Command = Tuple[float, float, float]

CONTROLLER_NODE = 'rapidcode_passthrough_trajectory_controller'
STOP_SERVICE = f'/{CONTROLLER_NODE}/stop'
RESET_SERVICE = f'/{CONTROLLER_NODE}/reset_fault'


class StrokeIntake:
    """The locked intake buffer: plane-frame waypoints plus session flags.

    Pure Python (no ROS). The coordination thread appends through
    begin/add/end; the stroke thread cuts windows. One polyline in plane
    commands: stroke points at draw_z, with the lift / travel / plunge
    synthesis appended in place, so a window cut anywhere is a valid path.
    """

    def __init__(self, plane, spacing: float, limit: int,
                 warn: Callable[[str], None]):
        self._plane = plane
        self._spacing = spacing
        self._limit = limit
        self._warn = warn
        self._lock = threading.Lock()
        self._points: List[Command] = []
        self._tail: Optional[Command] = None   # newest appended (survives cuts)
        self._stroke_open = False
        self._dropped_warned = False

    def _append_locked(self, point: Command, bounded: bool) -> None:
        if bounded and len(self._points) >= self._limit:
            if not self._dropped_warned:
                self._dropped_warned = True
                self._warn(f'stroke intake full ({self._limit}); new points '
                           'dropped until the arm catches up')
            return
        self._points.append(point)
        self._tail = point

    def begin_stroke(self, plane_xy) -> None:
        """Pen down at (x, y): travel at the safe height, then plunge.
        Synthesis points are never dropped (they close and open pen contact)."""
        with self._lock:
            travel = (plane_xy[0], plane_xy[1], self._plane.safe_z)
            plunge = (plane_xy[0], plane_xy[1], self._plane.draw_z)
            self._append_locked(travel, bounded=False)
            self._append_locked(plunge, bounded=False)
            self._stroke_open = True
            self._dropped_warned = False

    def add_point(self, plane_xy) -> bool:
        """One pen-down sample, decimated to the resample spacing (the
         rule). Returns False when no stroke is open (the caller
        should begin one instead)."""
        with self._lock:
            if not self._stroke_open:
                return False
            point = (plane_xy[0], plane_xy[1], self._plane.draw_z)
            if self._tail is not None and math.dist(self._tail, point) < self._spacing:
                return True  # within the decimation spacing of the tail
            self._append_locked(point, bounded=True)
            return True

    def end_stroke(self) -> None:
        """Pen up: lift to the travel height at the stroke's final position."""
        with self._lock:
            if not self._stroke_open:
                return
            self._stroke_open = False
            if self._tail is None:
                return
            lift = (self._tail[0], self._tail[1], self._plane.safe_z)
            if lift != self._tail:
                self._append_locked(lift, bounded=False)

    def take_window(self, seconds: float, speed: float,
                    flush: bool) -> Optional[List[Command]]:
        """Cut the next goal window: points whose estimated path time reaches
        ``seconds`` at the (rough) cartesian ``speed``. With ``flush`` the
        remainder is taken regardless of time. None when nothing (or, without
        flush, not yet enough) is available."""
        with self._lock:
            if not self._points:
                return None
            elapsed, count = 0.0, 0
            previous = None
            for point in self._points:
                if previous is not None:
                    elapsed += math.dist(previous, point) / max(speed, 1e-9)
                previous = point
                count += 1
                if elapsed >= seconds:
                    break
            if elapsed < seconds and not flush:
                return None
            window = self._points[:count]
            del self._points[:count]
            return window

    def set_plane(self, plane) -> None:
        """Adopt a new plane calibration; see the streamer's setter."""
        self._plane = plane

    def clear(self) -> None:
        with self._lock:
            self._points.clear()
            self._tail = None
            self._stroke_open = False

    @property
    def pending(self) -> int:
        with self._lock:
            return len(self._points)

    @property
    def stroke_open(self) -> bool:
        with self._lock:
            return self._stroke_open


class StrokeChainStreamer:
    """``begin_stroke``/``add_point``/``end_stroke``/``request_handback``/
    ``stop``/``reset``/``health`` -- the pen-down goal-chain route.

    ``work_node`` belongs to the stroke thread (the planner and pipeline spin
    it for whole windows); ``control_node`` belongs to the coordination
    thread's stop/reset/health calls, shared with the other routes.
    ``on_event`` may be called FROM THE STROKE THREAD -- the bridge marshals
    it back to the coordination thread. ``ros_factory`` is injectable so unit
    tests drive the chain loop with fakes and no ROS.
    """

    def __init__(self, config, work_node=None, control_node=None,
                 on_event: Callable[[dict], None] = None, sender_module=None,
                 goal_seconds: float = 2.0, max_inflight: int = 2,
                 chain_close_margin: float = 0.75,
                 settle_rate: float = 0.005, settle_hold: float = 0.2,
                 settle_timeout: float = 3.0,
                 control_executor=None, control_lock: threading.Lock = None,
                 clock: Callable[[], float] = time.monotonic,
                 ros_factory=None, start_thread: bool = True):
        self._config = config
        self._work_node = work_node
        self._control_node = control_node
        self._on_event = on_event or (lambda event: None)
        self._sender = sender_module
        self._goal_seconds = goal_seconds
        self._max_inflight = max_inflight
        self._chain_close_margin = chain_close_margin
        self._settle_rate = settle_rate
        self._settle_hold = settle_hold
        self._settle_timeout = settle_timeout
        self._control_executor = control_executor
        self._control_lock = control_lock or threading.Lock()
        self._clock = clock
        self._ros_factory = ros_factory or self._make_ros
        self._ros = None  # built lazily on the stroke thread

        plane = config.plane
        # Goal sizing speed: the same rough constant the streamed sender uses
        # (cut PLACEMENT only, never real timing).
        self._cartesian_speed = max(1e-3, 0.25 * float(plane.velocity))
        self._quaternion = linalg.rpy_to_quaternion(plane.anchor_rpy)
        self._intake = StrokeIntake(
            plane, getattr(config, 'resample_spacing', 0.002),
            getattr(config, 'live_queue_limit', 2000), self._warn)

        self._wake = threading.Event()
        self._shutdown_flag = threading.Event()
        self._stop_flag = threading.Event()
        self._session_requested = threading.Event()
        self._busy = threading.Event()
        self._handback = False
        self._retry_after = 0.0   # cool-down after a soft plan failure: the
                                  # pen may still be down and streaming points

        self._stop_client = None
        self._reset_client = None
        if control_node is not None:
            from std_srvs.srv import Trigger
            self._stop_client = control_node.create_client(Trigger, STOP_SERVICE)
            self._reset_client = control_node.create_client(Trigger, RESET_SERVICE)

        self._thread = threading.Thread(
            target=self._run, name='draw-plane-stroke', daemon=True)
        if start_thread:  # tests drive _run_session on their own thread
            self._thread.start()

    # ------------------------------------------------------------------- intake
    def begin_stroke(self, plane_xy) -> None:
        """Pen down (coordination thread): queue travel + plunge and make sure
        a chain session is running. Also cancels a pending hand-back -- a new
        stroke keeps the chain."""
        if self._stop_flag.is_set():
            return  # stopped: no motion is admitted until reset
        self._handback = False
        self._intake.begin_stroke(plane_xy)
        self._session_requested.set()
        self._wake.set()

    def add_point(self, plane_xy) -> None:
        """One pen-down sample (coordination thread)."""
        if self._stop_flag.is_set():
            return
        if not self._intake.add_point(plane_xy):
            self.begin_stroke(plane_xy)
            return
        self._session_requested.set()
        self._wake.set()

    def end_stroke(self) -> None:
        """Pen up: the stroke finishes as drawn, then lifts (coordination
        thread). The chain stays open for a quick next stroke."""
        self._intake.end_stroke()
        self._wake.set()

    def request_handback(self) -> None:
        """Drain the chain and return control (the route answers with a
        ``chain_drained`` event once the arm is at rest). An open stroke is
        closed first -- a hand-back with the pen still down must not hang."""
        self._intake.end_stroke()
        self._handback = True
        self._wake.set()

    # ----------------------------------------------------------------- controls
    def set_plane(self, plane) -> None:
        """Adopt a new plane calibration (offset,  rotation).

        The intake holds its own plane reference, so both are replaced: one
        component must never carry two divergent copies. ``_quaternion`` stays
        as built -- the tool orientation is the anchor's rpy, which neither the
        offset nor the rotation changes. Turning the board turns the drawing,
        not the wrist."""
        self._config = self._config._replace(plane=plane)
        self._intake.set_plane(plane)

    def stop(self) -> None:
        """Latch the controller stopped (same service as the other
        routes; idempotent) and collapse the session: in-flight goals abort
        against the latch on their own."""
        self._stop_flag.set()
        self._intake.clear()
        self._session_requested.clear()
        self._wake.set()
        self._call_trigger(self._stop_client, 'stop')

    def reset(self) -> bool:
        """Clear the controller's latch through ``~/reset_fault``.
        Refused while the session is still collapsing."""
        if self._busy.is_set():
            self._warn('reset refused: stroke chain still winding down')
            return False
        if not self._call_trigger(self._reset_client, 'reset_fault'):
            return False
        self._stop_flag.clear()
        return True

    def health(self) -> bool:
        """Controller availability, same watch as the batch route."""
        return (self._stop_client is not None
                and self._stop_client.service_is_ready())

    def shutdown(self) -> None:
        self._shutdown_flag.set()
        self._wake.set()
        if self._thread.ident is not None:
            self._thread.join(timeout=5.0)

    @property
    def active(self) -> bool:
        """A chain is open, draining, or has material waiting."""
        return (self._busy.is_set() or self._session_requested.is_set()
                or self._intake.pending > 0)

    # ------------------------------------------------------------ stroke thread
    def _run(self) -> None:
        while not self._shutdown_flag.is_set():
            if not self._wake.wait(timeout=0.2):
                continue
            self._wake.clear()
            if self._shutdown_flag.is_set():
                return
            if not self._session_requested.is_set() or self._stop_flag.is_set():
                continue
            if self._clock() < self._retry_after:
                continue  # cooling down; live points keep re-waking us
            # Cleared BEFORE the session: material arriving while the session
            # winds down re-arms the flag and the next loop pass serves it.
            self._session_requested.clear()
            self._busy.set()
            try:
                self._run_session()
            except Exception as error:  # the route must report, never die
                self._warn(f'stroke chain: {type(error).__name__}: {error}')
                self._intake.clear()
            finally:
                self._busy.clear()
            self._on_event({'kind': 'stroke', 'event': 'chain_drained'})

    def _run_session(self) -> None:
        """One session: settle in, run chains until hand-back or stop.
        A chain that closes at rest (catch-up dwell) reopens within the
        session when new points arrive."""
        ros = self._ensure_ros()
        if ros is None:
            self._intake.clear()
            return
        if not ros['settle'](self._settle_timeout, self._settle_hold):
            self._warn('stroke chain: settle probe timed out; proceeding')
        self._on_event({'kind': 'stroke', 'event': 'chain_opened'})
        while not self._stop_flag.is_set() and not self._shutdown_flag.is_set():
            if self._intake.pending > 0:
                self._run_chain(ros)
                continue
            if self._handback and not self._intake.stroke_open:
                return  # drained; the caller emits chain_drained
            ros['poll'](0.05)

    def _run_chain(self, ros) -> None:
        """One open firmware move: goal k goes out finalize=false only when
        goal k+1 is already planned; every exit runs through _close_chain."""
        seed = ros['read_joints']()
        if seed is None:
            self._warn('stroke chain: could not read joint state')
            self._intake.clear()
            return
        window = self._take_window(flush=True)  # goal 0: whatever has arrived
        pending = ros['planner'].plan_goal(self._world(window), seed)
        if pending is None:
            # Nothing was submitted and no move is open: a soft failure, not
            # a stop -- the arm is simply still at rest where it settled.
            # Cool down before retrying: the pen may still be streaming
            # points, and hammering the planner at input rate helps nobody.
            self._retry_after = self._clock() + 1.0
            self._intake.clear()
            self._on_event({'kind': 'stroke', 'event': 'chain_failed',
                            'message': 'goal 0 failed to plan'})
            return
        entry_speed, entry_velocity = 0.0, None
        index, horizon = 0, None
        while not self._stop_flag.is_set():
            window = self._next_window(ros, index, horizon)
            if window is None:  # catch-up dwell, hand-back drain, or stop
                break
            following = ros['planner'].plan_goal(self._world(window),
                                                 pending.end_joints)
            if following is None:
                self._warn('stroke chain: a window failed to plan; '
                           'closing at the previous goal')
                break
            seam = self._submit(ros, index, pending, following.positions,
                                entry_speed, entry_velocity, final=False)
            if seam is None:
                return  # submit failure: _submit already forced a stop
            entry_speed, entry_velocity, duration = seam
            horizon = max(self._clock(), horizon or 0.0) + duration
            pending, index = following, index + 1
        self._close_chain(ros, index, pending, entry_speed, entry_velocity)

    def _next_window(self, ros, index: int, horizon) -> Optional[List[Command]]:
        """Wait for the successor window. None means: close the chain now
        (hand-back with an empty intake, catch-up deadline, or stop). Goal 0
        (nothing executing yet) waits indefinitely -- no starvation risk."""
        while not self._stop_flag.is_set() and not self._shutdown_flag.is_set():
            deadline_near = (
                index > 0 and horizon is not None
                and self._clock() > horizon - self._chain_close_margin)
            drain = self._handback and not self._intake.stroke_open
            window = self._take_window(flush=deadline_near or drain)
            if window is not None:
                return window
            if deadline_near or (drain and self._intake.pending == 0):
                return None
            ros['poll'](0.02)
        return None

    def _take_window(self, flush: bool) -> Optional[List[Command]]:
        return self._intake.take_window(
            self._goal_seconds, self._cartesian_speed, flush)

    def _submit(self, ros, index: int, planned, next_positions,
                entry_speed: float, entry_velocity, final: bool):
        """Retime one goal across its seam and send it. Returns the carried
        (seam_speed, seam_velocity, duration), or None on failure -- in which
        case the controller has been stopped (never a starved open move)."""
        sender = self._sender
        times, velocities, seam_speed, seam_velocity = sender.retime_goal_window(
            planned.positions, next_positions, self._limits(),
            entry_speed, entry_velocity)
        message = sender.build_goal_message(times, planned.positions, velocities)
        ok = (ros['finalize'].set_finalize(final)
              and ros['pipeline'].submit(index, message))
        if not ok:
            self._fail_chain(f'goal {index} could not be submitted')
            return None
        return seam_speed, seam_velocity, float(times[-1])

    def _close_chain(self, ros, index: int, pending, entry_speed: float,
                     entry_velocity) -> None:
        """The single choke point that ends an open move: the final goal goes
        out finalize=true retimed to rest, then the pipeline drains and the
        arm settles. On stop the controller's own latch already collapsed the
        chain; the pipeline is only drained of its aborted results."""
        if not self._stop_flag.is_set():
            seam = self._submit(ros, index, pending, None,
                                entry_speed, entry_velocity, final=True)
            if seam is None:
                return
        drained = ros['pipeline'].drain(timeout_sec=120.0)
        if not drained and not self._stop_flag.is_set():
            self._warn('stroke chain: a goal did not succeed')
        ros['settle'](0.5, self._settle_hold)
        self._on_event({'kind': 'stroke', 'event': 'chain_closed'})

    def _fail_chain(self, message: str) -> None:
        """A chain broke in a way the finalize discipline cannot repair
        (submit/parameter failure with a goal possibly open): fall back to the
        controller's ~/stop -- firmware-strong, never a starved open move --
        and latch the route stopped so nothing retries against the latch.
        The health event drives the dispatcher's stopped latch,
        exactly like the live route's guard trip."""
        already_stopped = self._stop_flag.is_set()
        self._stop_flag.set()
        if not already_stopped:
            self._call_trigger(self._stop_client, 'stop')
        self._intake.clear()
        self._on_event({'kind': 'stroke', 'event': 'chain_failed',
                        'message': message})
        if not already_stopped:
            self._on_event({'kind': 'health', 'ok': False,
                            'message': f'stroke chain stopped: {message}'})

    # -------------------------------------------------------------- ROS plumbing
    def _world(self, window: List[Command]) -> list:
        plane = self._config.plane
        yaw = plane.plane_yaw_deg
        return [(plane_to_world(plane.effective_anchor(), plane.anchor_rpy,
                                point, yaw), self._quaternion)
                for point in window]

    def _limits(self):
        plane = self._config.plane
        sender = self._sender
        return (sender.JOINT_VELOCITY_LIMIT * float(plane.velocity),
                sender.JOINT_ACCEL_LIMIT * float(plane.acceleration),
                sender.CORNER_DELTA_V)

    def _ensure_ros(self):
        if self._ros is None:
            try:
                self._ros = self._ros_factory()
            except Exception as error:
                self._warn(f'stroke chain unavailable: {error}')
                return None
        return self._ros

    def _make_ros(self):
        """Build the planner/pipeline/finalize/probe set on the work node
        (stroke thread only). Mirrors run_streamed's client setup."""
        import rclpy
        from rclpy.action import ActionClient
        from control_msgs.action import FollowJointTrajectory
        from geometry_msgs.msg import Point, Pose, Quaternion
        from moveit_msgs.srv import GetCartesianPath
        from rcl_interfaces.srv import SetParameters

        if self._sender is None:
            from .batch_executor import load_sender_module
            self._sender = load_sender_module()
        sender = self._sender
        node = self._work_node
        plane = self._config.plane

        def to_pose(position, quaternion):
            qw, qx, qy, qz = quaternion
            return Pose(position=Point(x=position[0], y=position[1],
                                       z=position[2]),
                        orientation=Quaternion(w=qw, x=qx, y=qy, z=qz))

        cart_client = node.create_client(GetCartesianPath,
                                         sender.monolithic.CART_SERVICE)
        param_client = node.create_client(SetParameters,
                                          sender.SET_PARAM_SERVICE)
        fjt_client = ActionClient(node, FollowJointTrajectory,
                                  sender.ACTION_NAME)
        for client, name in ((cart_client, sender.monolithic.CART_SERVICE),
                             (param_client, sender.SET_PARAM_SERVICE)):
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f'{name} unavailable')
        if not fjt_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f'{sender.ACTION_NAME} unavailable')

        template = {'frame': sender.monolithic.PLANNING_FRAME,
                    'group': sender.monolithic.GROUP,
                    'link': sender.monolithic.TIP_LINK,
                    'eef_step': float(plane.eef_step), 'jump': 0.0,
                    'collisions': True, 'vel': float(plane.velocity),
                    'acc': float(plane.acceleration), 'to_pose': to_pose}
        pipeline = sender.GoalPipeline(node, fjt_client, self._max_inflight)

        def poll(timeout_sec):
            pipeline._poll(timeout_sec=timeout_sec)

        return {'planner': sender.GoalPlanner(node, cart_client, template),
                'pipeline': pipeline,
                'finalize': sender.FinalizeSwitch(node, param_client),
                'read_joints': lambda: sender.current_joint_state(node),
                'settle': self._wait_settled,
                'poll': poll}

    def _wait_settled(self, timeout: float, hold: float) -> bool:
        """Position-delta quiescence on /joint_states: every joint moving
        slower than the settle rate, held for ``hold`` seconds. There is no
        controller-side settled signal (online_busy_ clears at stop-tail
        emission), so the arm's own state is the probe."""
        import rclpy
        from sensor_msgs.msg import JointState

        node = self._work_node
        samples = []

        def receive(message):
            samples.append((self._clock(), dict(zip(message.name,
                                                    message.position))))

        subscription = node.create_subscription(JointState, '/joint_states',
                                                receive, 10)
        try:
            deadline = self._clock() + timeout
            quiet_since = None
            previous = None
            while self._clock() < deadline and not self._stop_flag.is_set():
                rclpy.spin_once(node, timeout_sec=0.05)
                if not samples:
                    continue
                stamp, positions = samples[-1]
                samples.clear()
                if previous is not None:
                    rate = self._position_rate(previous, (stamp, positions))
                    if rate is not None and rate < self._settle_rate:
                        quiet_since = quiet_since or stamp
                        if stamp - quiet_since >= hold:
                            return True
                    else:
                        quiet_since = None
                previous = (stamp, positions)
            return False
        finally:
            node.destroy_subscription(subscription)

    @staticmethod
    def _position_rate(previous, current) -> Optional[float]:
        """Max per-joint |delta position| / delta time between two samples."""
        (stamp_a, joints_a), (stamp_b, joints_b) = previous, current
        elapsed = stamp_b - stamp_a
        if elapsed <= 0.0:
            return None
        shared = set(joints_a) & set(joints_b)
        if not shared:
            return None
        return max(abs(joints_b[name] - joints_a[name])
                   for name in shared) / elapsed

    def _call_trigger(self, client, label: str) -> bool:
        if client is None:
            return False
        import rclpy
        if not client.wait_for_service(timeout_sec=2.0):
            self._warn(f'{label}: controller service unavailable')
            return False
        with self._control_lock:
            future = client.call_async(client.srv_type.Request())
            rclpy.spin_until_future_complete(self._control_node, future,
                                             executor=self._control_executor,
                                             timeout_sec=5.0)
        result = future.result()
        if result is None:
            self._warn(f'{label}: no response')
            return False
        if not result.success and label == 'stop':
            return True  # stop is idempotent (success either way)
        return result.success

    def _warn(self, message: str) -> None:
        self._on_event({'kind': 'warning', 'message': message})
