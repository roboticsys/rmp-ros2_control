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

"""Live target streamer: the pose-stream route to the Cartesian streaming node.

Owns the live thread: a dedicated tick loop at <= 0.02 s
that publishes the next rate-limited, guard-checked target as a planning-frame
``PoseStamped`` on the Cartesian streaming node's pose command topic.
The dispatcher stores intents (``set_intent``); intents land in a bounded
waypoint queue (stakeholder 2026-08-24) decimated to the resample
spacing, and the tick drains it in order at the displacement rate limit -- the
arm replays the drawn path at its own speed instead of shortcutting to the
latest pointer position. A depth change (pen up/down) always queues, so pen
transitions happen where they were made. ``release`` appends the lift to the
travel height AFTER the queued path, so a released stroke finishes as drawn
before the pen rises at the release position (as amended).

The guard here is the independent stop-check: the clamp already ran in
the dispatcher; this component re-judges every outgoing command with
``bound_verdicts`` and stops -- it never re-clamps.

Verified E2E on phantom (2026-08-21): activate unpauses Servo and selects POSE,
the arm tracks the pointer through hover, pen-down draw, and pen-up, and client
loss rests it through the controller's producer-silence stop-tail.
"""

import collections
import math
import threading
import time
from typing import Callable, Optional, Tuple

from . import linalg

Command = Tuple[float, float, float]

SERVO_NODE = '/servo_node'
POSE_TOPIC = f'{SERVO_NODE}/pose_target_cmds'
SWITCH_SERVICE = f'{SERVO_NODE}/switch_command_type'
PAUSE_SERVICE = f'{SERVO_NODE}/pause_servo'  # SetBool: false = run (Jazzy has no start_servo)
CONTROLLER_NODE = 'rapidcode_passthrough_trajectory_controller'
POSE_COMMAND_TYPE = 2  # moveit_msgs/srv/ServoCommandType: POSE
SETTLE_SECONDS = 1.2   # >= 0.2 s: hold the reached target so the
                       # streaming node settles onto it before publication ceases
STEP_LIMIT = 0.0005    #  (TBR): max distance between consecutive targets,
                       # matched to the streaming node's 0.05 m/s linear scale so the
                       # command never outruns the arm


def plane_to_world(anchor_xyz, anchor_rpy, command: Command,
                   plane_yaw_deg: float = 0.0):
    """A plane-frame command into the planning frame: rotate, then anchor + offset.

    The batch sender's convention (send_cartesian_path.py ``build_waypoints``):
    the in-plane (x, y) turns about the anchor, then relative coordinates
    translate from the anchor along the PLANNING frame's axes. z never turns
    — it is the pen depth, a physical quantity.

    ``plane_yaw_deg`` is the operator's plane rotation, degrees CCW,
    and it is the ONLY rotation here. The anchor rpy orients the TOOL and never
    rotates positions — deliberately, so a plane rotation turns the drawing
    without turning the wrist. (An earlier revision rotated offsets by the rpy
    — with the tool-down (pi, 0, pi) anchor that mirrored x and inverted the
    pen depth, 2026-08-24.)

    At zero yaw this is the pure translation it has always been, on the same
    arithmetic path: the live route calls it up to 100 times a second.
    """
    del anchor_rpy  # tool orientation only; positions do not rotate with it
    x, y, z = command
    if plane_yaw_deg:
        x, y = linalg.rotate_in_plane((x, y), plane_yaw_deg)
    return (anchor_xyz[0] + x, anchor_xyz[1] + y, anchor_xyz[2] + z)


class LiveTargetStreamer:
    """``tick``/``set_intent``/``rest``/``stop``/``activate``/``health``.

    ``publish_pose`` is injected (the bridge wires the real publisher; tests a
    fake): called with the planning-frame position tuple. ``control_node`` is
    the coordination-thread service node shared with the batch route -- never
    the node the batch thread spins.
    """

    def __init__(self, config, publish_pose: Callable[[Command], None],
                 control_node=None, on_event: Callable[[dict], None] = None,
                 tick_period: float = 0.02, clock: Callable[[], float] = time.monotonic,
                 control_executor=None, control_lock: threading.Lock = None):
        self._config = config          # DispatcherConfig (extent, working area, plane)
        self._publish_pose = publish_pose
        self._control_node = control_node
        self._control_executor = control_executor
        # Serializes every spin of the control executor: the guard's stop() runs
        # on the LIVE thread while activate/reset run on the coordination thread,
        # and one executor must never be spun from two threads at once.
        self._control_lock = control_lock or threading.Lock()
        self._on_event = on_event or (lambda event: None)
        self._tick_period = tick_period
        self._clock = clock

        limits_speed = getattr(config.plane, 'velocity', 0.2)
        self._limits = linalg.Limits(
            extent=config.extent,
            working_centre=config.working_centre,
            working_radius=config.working_radius,
            depth_min=min(config.plane.draw_z, config.plane.safe_z),
            depth_max=max(config.plane.draw_z, config.plane.safe_z),
            max_speed=limits_speed)
        # Per-tick bound: the absolute step limit, and
        # never faster than the configured plane velocity allows.
        self._max_step = min(limits_speed * tick_period, STEP_LIMIT)

        # Waypoint queue: decimation spacing and bound.
        self._queue_spacing = getattr(config, 'resample_spacing', 0.002)
        self._queue_limit = getattr(config, 'live_queue_limit', 2000)

        self._lock = threading.Lock()
        self._queue: 'collections.deque[Command]' = collections.deque()
        self._target: Optional[Command] = None    # waypoint being driven now
        self._last: Optional[Command] = None
        self._active = False
        self._at_rest = True
        self._releasing = False           # lift-and-settle after disengagement
        self._settle_started: Optional[float] = None
        self._queue_warned = False        # one overflow warning per engagement
        self._switch_client = None
        self._reset_client = None
        if control_node is not None:
            self._make_clients(control_node)

    def _make_clients(self, node):
        from moveit_msgs.srv import ServoCommandType
        from std_srvs.srv import SetBool, Trigger
        self._switch_client = node.create_client(ServoCommandType, SWITCH_SERVICE)
        self._pause_client = node.create_client(SetBool, PAUSE_SERVICE)
        self._reset_client = node.create_client(
            Trigger, f'/{CONTROLLER_NODE}/reset_fault')
        self._stop_client = node.create_client(
            Trigger, f'/{CONTROLLER_NODE}/stop')

    def _call(self, client, request, timeout: float = 5.0):
        """One serialized service round trip on the control executor."""
        import rclpy
        with self._control_lock:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(
                self._control_node, future, executor=self._control_executor,
                timeout_sec=timeout)
        return future.result()

    # ------------------------------------------------------------------- intents
    def _tail_locked(self) -> Optional[Command]:
        """The newest queued or driven position (caller holds the lock)."""
        if self._queue:
            return self._queue[-1]
        return self._target or self._last

    def set_intent(self, target: Command) -> None:
        """Queue one pointer-derived waypoint: decimated to the
        resample spacing, except a depth change (pen up/down) always queues;
        dropped with one warning when the bound is reached."""
        with self._lock:
            self._at_rest = False
            self._releasing = False
            self._settle_started = None
            tail = self._tail_locked()
            if tail is not None:
                depth_changed = tail[2] != target[2]
                distance = math.dist(tail, target)
                if not depth_changed and distance < self._queue_spacing:
                    return  # within the decimation spacing of the tail
            if len(self._queue) >= self._queue_limit:
                if not self._queue_warned:
                    self._queue_warned = True
                    self._on_event({'kind': 'warning', 'message':
                                    f'live waypoint queue full '
                                    f'({self._queue_limit}); new points dropped '
                                    f'until the arm catches up'})
                return
            self._queue.append(target)

    def release(self) -> None:
        """Follow disengaged: finish the queued path as drawn,
        then lift to the travel height at its final position, keep publishing
        through the settling interval, then cease. Publication
        cessation is what lets the controller's producer-silence stop-tail
        bring the arm to rest."""
        with self._lock:
            if not self._active or self._at_rest:
                return
            tail = self._tail_locked()
            if tail is None:
                self._at_rest = True
                return
            lift = (tail[0], tail[1], self._config.plane.safe_z)
            if lift != tail:
                # The lift is never dropped: it may exceed the queue bound.
                self._queue.append(lift)
            self._releasing = True
            self._settle_started = None
            self._queue_warned = False

    def rest(self) -> None:
        """Cease publication immediately (input loss)."""
        with self._lock:
            self._queue.clear()
            self._target = None
            self._at_rest = True
            self._releasing = False
            self._settle_started = None

    # ---------------------------------------------------------------------- tick
    def tick(self) -> Optional[Command]:
        """One live-thread tick; returns the command it published (tests), or
        None when it held. The controller's producer-silence stop-tail brings
        the arm to rest when publication ceases -- resting is by silence."""
        with self._lock:
            active, at_rest, last = self._active, self._at_rest, self._last
            if self._target is None and self._queue:
                self._target = self._queue.popleft()
            target, releasing = self._target, self._releasing
        if not active or at_rest:
            return None
        if target is None:
            # Engaged (or settling) with nothing new to drive: hold position.
            # The producer must keep its cadence or the controller's
            # silence watchdog stops the arm.
            if last is None:
                return None
            command = last
        else:
            command = self._advance(target, last)
            if command is None:
                return None  # the guard tripped; the route stopped
        # One load of the plane, not two: set_plane swaps the config from the
        # coordination thread, and two attribute chains could straddle it.
        plane = self._config.plane
        self._publish_pose(
            plane_to_world(plane.effective_anchor(), plane.anchor_rpy, command,
                           plane.plane_yaw_deg))
        with self._lock:
            self._last = command
            queue_drained = self._target is None and not self._queue
            if self._releasing and queue_drained:
                now = self._clock()
                if self._settle_started is None:
                    self._settle_started = now
                elif now - self._settle_started >= SETTLE_SECONDS:
                    self._at_rest = True
                    self._releasing = False
                    self._settle_started = None
        return command

    def _advance(self, target: Command, last: Optional[Command]) -> Optional[Command]:
        """One rate-limited, guard-checked step along the queued path. Spends
        the whole tick's displacement budget across waypoint boundaries
        (the limit bounds the tick's TOTAL displacement, not each piece).
        Returns the command to publish, or None when the guard tripped."""
        # Bound the FULL 3D step (a pen up/down z hop must ramp like any other
        # move, or the speed guard below would rightly trip on it). 0.95 keeps
        # a saturated step strictly inside max_speed against rounding.
        budget = 0.95 * self._max_step
        if last is None:
            last = target  # first tick of a session: no displacement to limit
        command = last
        while True:
            delta = tuple(target[axis] - command[axis] for axis in range(3))
            norm = math.sqrt(sum(d * d for d in delta))
            if norm > budget:
                scale = budget / norm
                command = tuple(command[axis] + delta[axis] * scale
                                for axis in range(3))
                break
            command = target
            budget -= norm
            with self._lock:
                self._target = self._queue.popleft() if self._queue else None
                next_target = self._target
            if next_target is None:
                break
            target = next_target
        verdicts = linalg.bound_verdicts(command, last, self._tick_period,
                                         self._limits)
        if not verdicts.ok:
            # Independent guard: a violating command is never sent;
            # the route stops and reports.
            self.stop()
            self._on_event({'kind': 'health', 'ok': False,
                            'message': f'live guard tripped: {verdicts}'})
            return None
        return command

    # ------------------------------------------------------------------ commands
    def set_plane(self, plane) -> None:
        """Adopt a new plane calibration (offset,  rotation).

        Called on the coordination thread by the dispatcher, which admits the
        change only in Batch mode -- so this route is idle and no in-flight
        target is re-based mid-motion. ``_limits`` and ``_max_step`` derive from
        the extent, the working area and the depths, none of which the
        calibration touches, so they need no rebuild. The step limiter works in
        PLANE coordinates, which turning the frame leaves alone."""
        self._config = self._config._replace(plane=plane)

    def activate(self) -> bool:
        """Clear a standing latch, select POSE command type, go active."""
        if self._switch_client is None:
            return False
        if self._reset_client.wait_for_service(timeout_sec=2.0):
            self._call(self._reset_client, self._reset_client.srv_type.Request())
        if not self._switch_client.wait_for_service(timeout_sec=2.0):
            return False
        request = self._switch_client.srv_type.Request()
        request.command_type = POSE_COMMAND_TYPE
        result = self._call(self._switch_client, request)
        if result is None or not result.success:
            return False
        # Servo starts paused on Jazzy; unpause AFTER the command type is set.
        if not self._pause_client.wait_for_service(timeout_sec=2.0):
            return False
        unpause = self._pause_client.srv_type.Request()
        unpause.data = False
        result = self._call(self._pause_client, unpause)
        if result is None or not result.success:
            return False
        with self._lock:
            self._active = True
            self._queue.clear()
            self._target = None
            self._last = None
            self._at_rest = True
            self._queue_warned = False
        return True

    def deactivate(self) -> None:
        with self._lock:
            self._active = False
            self._queue.clear()
            self._target = None
            self._at_rest = True
            self._last = None
            self._releasing = False
            self._settle_started = None

    def stop(self) -> None:
        """`~/stop` on the controller; publication ceases."""
        self.deactivate()
        if self._control_node is None:
            return
        try:
            if self._stop_client.wait_for_service(timeout_sec=2.0):
                self._call(self._stop_client, self._stop_client.srv_type.Request())
        except Exception as error:  # the tick thread must survive a failed call
            self._on_event({'kind': 'warning', 'message': f'live stop: {error}'})

    def reset(self) -> bool:
        if self._reset_client is None:
            return False
        if not self._reset_client.wait_for_service(timeout_sec=2.0):
            return False
        result = self._call(self._reset_client, self._reset_client.srv_type.Request())
        return result is not None and result.success

    def health(self) -> bool:
        """Both watched interfaces must be up during Live (stakeholder,
        2026-08-21): the Cartesian streaming node and the controller."""
        if self._switch_client is None:
            return False
        return (self._switch_client.service_is_ready()
                and self._stop_client.service_is_ready())

    @property
    def active(self) -> bool:
        return self._active
