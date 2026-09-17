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

"""Batch executor: the finite-goal route to the passthrough trajectory controller.

Owns the batch thread: the streamed Cartesian sender's planning calls
block for whole slices, so they run here, never on the coordination or live
threads. The sender is imported as a library and driven with the recipe
structure in memory on a dedicated rclpy node this component owns and
spins -- the bridge's executor must never spin that node.

Stop and reset ride the controller's services: ``stop()`` latches the
controller faulted; ``reset()`` clears the latch through ``~/reset_fault``.
``execute()`` clears any standing latch first, so an execute after a stop can
never be silently rejected.

``park()`` is the one route that commands motion without a recipe: a
joint PTP to the ready pose through ``/move_action``. It runs on the same batch
thread, behind the same latch clear, and reports through the same result event.
"""

import importlib.util
import os
import queue
import threading
from typing import Callable, Optional

CONTROLLER_NODE = 'rapidcode_passthrough_trajectory_controller'
STOP_SERVICE = f'/{CONTROLLER_NODE}/stop'
RESET_SERVICE = f'/{CONTROLLER_NODE}/reset_fault'


def load_sender_module():
    """Import the streamed Cartesian sender (and its monolithic half) from the
    installed rapidcode_bringup scripts, without touching sys.path for good."""
    import sys

    from ament_index_python.packages import get_package_prefix
    scripts = os.path.join(
        get_package_prefix('rapidcode_bringup'), 'lib', 'rapidcode_bringup')
    sys.path.insert(0, scripts)
    try:
        spec = importlib.util.spec_from_file_location(
            'send_cartesian_path_streamed',
            os.path.join(scripts, 'send_cartesian_path_streamed.py'))
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)  # imports send_cartesian_path from scripts
        return module
    finally:
        sys.path.remove(scripts)


class BatchExecutor:
    """``execute``/``stop``/``reset``/``health`` per the architecture table.

    Two dedicated rclpy nodes, neither spun by the bridge's executor:
    ``work_node`` belongs to the batch thread (the sender spins it for whole
    executions); ``control_node`` belongs to the coordination thread's stop,
    reset, and health calls, which must never touch the node the batch thread
    is spinning. ``on_event`` receives route events (results, warnings) and may
    be called FROM THE BATCH THREAD -- the bridge marshals it back to the
    coordination thread.
    """

    def __init__(self, work_node, control_node, on_event: Callable[[dict], None],
                 sender_module=None, goal_seconds: float = 4.0,
                 max_inflight: int = 2, control_executor=None,
                 control_lock: threading.Lock = None):
        import rclpy.action
        from control_msgs.action import FollowJointTrajectory
        from moveit_msgs.action import MoveGroup
        from std_srvs.srv import Trigger

        self._work_node = work_node
        self._control_node = control_node
        # Every spin here needs an executor that is NOT the bridge's and NOT the
        # global default (the batch thread's sender spins that one): control-node
        # calls get their own, shared with the live route.
        self._control_executor = control_executor
        self._control_lock = control_lock or threading.Lock()
        self._on_event = on_event
        self._sender = sender_module or load_sender_module()
        self._goal_seconds = goal_seconds
        self._max_inflight = max_inflight

        self._stop_client = control_node.create_client(Trigger, STOP_SERVICE)
        self._reset_client = control_node.create_client(Trigger, RESET_SERVICE)
        self._work_reset_client = work_node.create_client(Trigger, RESET_SERVICE)
        self._health_client = rclpy.action.ActionClient(
            control_node, FollowJointTrajectory,
            f'/{CONTROLLER_NODE}/follow_joint_trajectory')
        # The park route plans through MoveGroup, like the sender's own approach
        # does; it belongs to the work node, because the batch thread drives it.
        self._move_client = rclpy.action.ActionClient(
            work_node, MoveGroup, self._sender.monolithic.MOVE_ACTION)

        self._work: 'queue.Queue[Optional[tuple]]' = queue.Queue()
        self._busy = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name='draw-plane-batch', daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------- routes
    def execute(self, structure: dict) -> None:
        """Queue the drawing for the batch thread; results arrive via on_event."""
        self._work.put(('execute', structure))

    def park(self, ready_joints, velocity: float, acceleration: float) -> None:
        """Queue the pen-adjustment park move: a joint PTP to the
        ready pose. Results arrive via on_event, exactly as an execution's do.

        The ready pose is a joint vector, so the move needs no IK and no
        Cartesian plan: it cannot fail for reachability, and it leaves the arm
        where the next execution's own approach already starts."""
        self._work.put(('park', (list(ready_joints), float(velocity),
                                 float(acceleration))))

    def stop(self) -> None:
        """Latch the controller stopped. Fire-and-report: the batch
        thread's in-flight goals collapse against the latch on their own."""
        self._call_trigger(self._stop_client, 'stop')

    def reset(self) -> bool:
        """Clear the controller's latch through ``~/reset_fault``.
        Refused while an execution is still collapsing -- resetting under it
        would re-arm motion the stop just killed."""
        if self._busy.is_set():
            self._on_event({'kind': 'warning',
                            'message': 'reset refused: execution still winding down'})
            return False
        return self._call_trigger(self._reset_client, 'reset_fault')

    def health(self) -> bool:
        """Controller availability."""
        return self._health_client.server_is_ready()

    def compute_unreachable_mask(self, extent, orientation):
        """Secondary-tier feature: not implemented in this increment."""
        return None

    def shutdown(self) -> None:
        self._work.put(None)
        self._thread.join(timeout=5.0)

    # ---------------------------------------------------------------- internals
    def _call_trigger(self, client, label: str) -> bool:
        import rclpy
        if not client.wait_for_service(timeout_sec=2.0):
            self._on_event({'kind': 'warning',
                            'message': f'{label}: controller service unavailable'})
            return False
        with self._control_lock:
            future = client.call_async(client.srv_type.Request())
            rclpy.spin_until_future_complete(self._control_node, future,
                                             executor=self._control_executor,
                                             timeout_sec=5.0)
        result = future.result()
        if result is None:
            self._on_event({'kind': 'warning', 'message': f'{label}: no response'})
            return False
        if not result.success and label == 'stop':
            # stop is idempotent (success either way); reset can genuinely refuse
            return True
        return result.success

    def _run(self) -> None:
        while True:
            item = self._work.get()
            if item is None:
                return
            kind, payload = item
            self._busy.set()
            try:
                if kind == 'park':
                    ok, message = self._park(*payload)
                else:
                    ok, message = self._execute_structure(payload)
            except Exception as error:  # the route must report, never die
                ok, message = False, f'{type(error).__name__}: {error}'
            finally:
                self._busy.clear()
            self._on_event({'kind': 'result', 'ok': ok, 'message': message})

    def _clear_latch(self) -> None:
        """Drop any standing stop latch from the batch thread. A latch would
        reject every goal; a refusal here is fine -- it means none was set."""
        if self._work_reset_client.wait_for_service(timeout_sec=2.0):
            import rclpy
            future = self._work_reset_client.call_async(
                self._work_reset_client.srv_type.Request())
            rclpy.spin_until_future_complete(self._work_node, future, timeout_sec=5.0)

    def _park(self, ready_joints, velocity: float, acceleration: float):
        """Joint PTP to the ready pose, planned through MoveGroup by
        the sender's own approach helper -- the same call, the same planner and
        the same Pilz PTP that opens every execution."""
        self._clear_latch()
        action = self._sender.monolithic.MOVE_ACTION
        if not self._move_client.wait_for_server(timeout_sec=10.0):
            return False, f'park: {action} unavailable'
        moved = self._sender.ptp_to_joints(
            self._work_node, self._move_client, ready_joints, 'ready',
            velocity, acceleration)
        return moved, '' if moved else 'park: the ready-pose move failed'

    def _execute_structure(self, structure: dict):
        self._clear_latch()
        sender = self._sender
        coords = str(structure.get('coordinates', 'relative')).lower()
        defaults = dict(sender.monolithic.DEFAULTS,
                        **(structure.get('defaults') or {}))
        segments = structure.get('segments') or []
        if not segments:
            return False, 'empty recipe structure'
        args = sender.monolithic.parse_args([])
        args['eef_step'] = float(structure.get('eef_step',
                                               sender.monolithic.EEF_STEP))
        args['scale'] = float(structure.get('scale', 1.0))
        args['rotate'] = float(structure.get('rotate', 0.0))
        # The plane's own rotation, added to `rotate` by the sender.
        # Resolved here because `run_streamed` is the in-process entry point
        # and takes the argument dict already built: the sender's own doc
        # resolution runs in its CLI path, which this route never enters.
        args['plane_rotate'] = float(structure.get('plane_rotate', 0.0))
        options = {'goal_seconds': self._goal_seconds,
                   'max_inflight': self._max_inflight,
                   'rest_seams': False, 'self_test': False}
        code = sender.run_streamed(options, args, structure, coords, defaults,
                                   segments, node=self._work_node)
        return code == 0, f'sender exit code {code}'
