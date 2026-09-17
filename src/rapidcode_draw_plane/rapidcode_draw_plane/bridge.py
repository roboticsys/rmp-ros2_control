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

"""The draw-plane bridge process: wiring, thread groups, and entry point.

Thread groups:
  coordination -- a dedicated drain/watchdog thread (NOT rclpy timers: rcl
                  timers proved able to stop firing mid-session, 2026-08-24;
                  the node's executor now serves only parameter services);
  transport    -- UiComms's asyncio thread (WebSocket server);
  batch        -- BatchExecutor's worker thread (blocking planning calls);
  live         -- a dedicated tick thread driving LiveTargetStreamer.

Crossings: transport->coordination through UiComms's queue (drained by the
coordination thread);
batch->coordination through the route-event queue; coordination->live through
LiveTargetStreamer's locked waypoint queue.
"""

import os
import queue
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from . import file_manager, linalg, live_streamer
from .batch_executor import BatchExecutor, load_sender_module
from .dispatcher import Dispatcher, DispatcherConfig
from .live_streamer import LiveTargetStreamer
from .recipe_loader import RecipeLoader
from .stroke_streamer import StrokeChainStreamer
from .ui_comms import UiComms

PLANNING_FRAME = 'world'


class DrawPlaneBridge(Node):
    def __init__(self):
        super().__init__('draw_plane_bridge')
        declare = self.declare_parameter
        declare('port', 8765)
        declare('host', '0.0.0.0')
        declare('input_timeout', 0.5)          #  loss timeout
        declare('extent_min', [-0.11, -0.11])  # plane window, metres
        declare('extent_max', [0.11, 0.11])
        declare('working_centre', [0.0, 0.0])   # plane frame; origin = centre
        declare('working_radius', 0.16)
        declare('duplicate_threshold', 0.0005)  # metres
        declare('resample_spacing', 0.002)      # metres
        declare('min_run', 3)                   # points
        declare('max_samples', 20000)           #  submitted-drawing bound
        declare('fine_grid', 0.02)              # metres
        declare('coarse_grid', 0.04)            # metres
        declare('recipe_directory', '/tmp')
        # Plane pose and motion defaults.
        declare('anchor_xyz', [-0.46, 0.0, 0.271])
        declare('anchor_rpy', [3.141592653589793, 0.0, 3.141592653589793])
        declare('plane_offset', [0.0, 0.0])     # the operator's plane
                                                # calibration, planning-frame
                                                # x/y metres, session-only
        declare('plane_yaw_deg', 0.0)           # the rest of it -- how
                                                # far the board is turned, deg
                                                # CCW about the plane origin.
                                                # Degrees (anchor_rpy is radians)
        declare('reach_inner', 0.30)            # the band the plane's
        declare('reach_outer', 0.65)            # working disc must stay inside
        declare('ready_joints', [0.0, 0.0, -1.5708, 0.0, -1.5708, 0.0])
                                                # approached before the anchor,
                                                # and the pen-adjustment park
                                                # pose
        declare('draw_z', -0.02)
        declare('safe_z', 0.0)
        declare('retract_z', 0.12)              # end-of-drawing
                                                # height
        declare('velocity', 0.2)
        declare('acceleration', 0.2)
        declare('eef_step', 0.004)
        declare('goal_seconds', 4.0)
        declare('max_inflight', 2)
        declare('live_tick', 0.02)
        declare('live_queue_limit', 2000)       #  waypoint-queue bound
        # Pen-down stroke chain (stakeholder 2026-08-25): sized-goal streaming.
        declare('stroke_goal_seconds', 0.25)     # target motion per stroke goal
        declare('stroke_max_inflight', 2)       # goals submitted but unfinished
        declare('chain_close_margin', 0.75)     # close the chain this long
                                                # before the submitted motion
                                                # would starve the open move
        declare('settle_position_rate', 0.005)  # rad/s: at-rest threshold
        declare('settle_hold', 0.2)             # s the threshold must hold
        declare('settle_timeout', 3.0)          # s: producer timeout + max
                                                # stop-tail + margin

        value = lambda name: self.get_parameter(name).value
        plane = file_manager.PlaneConfig(
            anchor_xyz=tuple(value('anchor_xyz')),
            anchor_rpy=tuple(value('anchor_rpy')),
            plane_offset=tuple(value('plane_offset')),
            plane_yaw_deg=float(value('plane_yaw_deg')),
            ready_joints=tuple(value('ready_joints')),
            draw_z=float(value('draw_z')),
            safe_z=float(value('safe_z')),
            velocity=float(value('velocity')),
            acceleration=float(value('acceleration')),
            eef_step=float(value('eef_step')),
            retract_z=float(value('retract_z')))
        self.config = DispatcherConfig(
            extent=(tuple(value('extent_min')), tuple(value('extent_max'))),
            working_centre=tuple(value('working_centre')),
            working_radius=float(value('working_radius')),
            duplicate_threshold=float(value('duplicate_threshold')),
            resample_spacing=float(value('resample_spacing')),
            min_run=int(value('min_run')),
            plane=plane,
            recipe_directory=str(value('recipe_directory')),
            fine_grid=float(value('fine_grid')),
            coarse_grid=float(value('coarse_grid')),
            max_samples=int(value('max_samples')),
            live_queue_limit=int(value('live_queue_limit')),
            reach_inner=float(value('reach_inner')),
            reach_outer=float(value('reach_outer')))

        # Startup refusal before anything else runs.
        refusal = Dispatcher.check_startup(self.config)
        if refusal is not None:
            raise RuntimeError(f'startup refused: {refusal}')

        self._route_events: 'queue.Queue[dict]' = queue.Queue()
        self._work_node = Node('draw_plane_batch_work')
        self._stroke_work_node = Node('draw_plane_stroke_work')
        self._control_node = Node('draw_plane_route_control')
        # Executor separation: the bridge spins on its own executor in
        # main(); the batch thread's sender and the stroke thread's planner both
        # spin the GLOBAL default executor -- safe because Batch execution and
        # Live stroking are mutually exclusive dispatcher modes (leaving Live is
        # refused while a chain drains); control-node service calls spin this one.
        self._control_executor = rclpy.executors.SingleThreadedExecutor()
        self._control_lock = threading.Lock()  # one spinner at a time

        self.ui = UiComms(str(value('host')), int(value('port')),
                          float(value('input_timeout')),
                          log=lambda text: self.get_logger().info(text))
        sender = load_sender_module()  # shared by the batch and stroke routes
        self.batch = BatchExecutor(
            self._work_node, self._control_node, self._route_events.put,
            sender_module=sender,
            goal_seconds=float(value('goal_seconds')),
            max_inflight=int(value('max_inflight')),
            control_executor=self._control_executor,
            control_lock=self._control_lock)
        self.stroke = StrokeChainStreamer(
            self.config, work_node=self._stroke_work_node,
            control_node=self._control_node, on_event=self._route_events.put,
            sender_module=sender,
            goal_seconds=float(value('stroke_goal_seconds')),
            max_inflight=int(value('stroke_max_inflight')),
            chain_close_margin=float(value('chain_close_margin')),
            settle_rate=float(value('settle_position_rate')),
            settle_hold=float(value('settle_hold')),
            settle_timeout=float(value('settle_timeout')),
            control_executor=self._control_executor,
            control_lock=self._control_lock)
        self._pose_pub = self.create_publisher(
            PoseStamped, live_streamer.POSE_TOPIC, 10)
        self.live = LiveTargetStreamer(
            self.config, self._publish_pose, control_node=self._control_node,
            on_event=self._route_events.put,
            tick_period=float(value('live_tick')),
            control_executor=self._control_executor,
            control_lock=self._control_lock)
        self.dispatcher = Dispatcher(
            self.config, self.batch, self.live,
            send_display=self.ui.send_display_state,
            log=lambda text: self.get_logger().info(text),
            stroke_route=self.stroke,
            recipe_loader=RecipeLoader(sender.monolithic,
                                       self._recipe_directories()),
            on_plane_change=self._adopt_plane)

        self._input_was_lost = False
        self._loss_started = 0.0
        self._session_was_open = False
        # Coordination runs on its own thread, NOT on rclpy timers: mid-session
        # (2026-08-24) the node's rcl timers stopped becoming ready on their
        # own — the executor's wait woke only on external guard events — so the
        # UI queue backed up by minutes. The drain must keep input cadence
        # regardless of executor health.
        self._coord_thread_stop = threading.Event()
        self._coord_thread = threading.Thread(
            target=self._coordination_loop, name='draw-plane-coordination',
            daemon=True)

        self._live_thread_stop = threading.Event()
        self._live_thread = threading.Thread(
            target=self._live_loop, name='draw-plane-live', daemon=True)

    def _recipe_directories(self):
        """Loadable recipe locations: the operator's save directory first (so
        saved drawings shadow the bundled recipes), then the bringup package's
        bundled config directory when the workspace provides it."""
        directories = [self.config.recipe_directory]
        try:
            from ament_index_python.packages import get_package_share_directory
            directories.append(os.path.join(
                get_package_share_directory('rapidcode_bringup'), 'config'))
        except Exception as error:  # bundled recipes are optional
            self.get_logger().warning(
                f'bundled recipes unavailable: {error}')
        return directories

    # ------------------------------------------------------------------- wiring
    def start(self) -> None:
        self.ui.start()
        self._coord_thread.start()
        self._live_thread.start()
        self.get_logger().info(
            f"bridge up: ws://{self.get_parameter('host').value}:"
            f"{self.get_parameter('port').value}, extent {self.config.extent}")

    def _publish_pose(self, world_xyz) -> None:
        message = PoseStamped()
        message.header.frame_id = PLANNING_FRAME
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = world_xyz[0]
        message.pose.position.y = world_xyz[1]
        message.pose.position.z = world_xyz[2]
        # Tool orientation: the anchor's (pen normal to the plane), constant.
        qw, qx, qy, qz = self._anchor_quaternion()
        message.pose.orientation.w = qw
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        self._pose_pub.publish(message)

    def _anchor_quaternion(self):
        return linalg.rpy_to_quaternion(self.config.plane.anchor_rpy)

    def _adopt_plane(self, plane) -> None:
        """Keep the bridge's own configuration copy current.

        The bridge holds a fourth reference to the configuration, beside the
        dispatcher's and the two streamers'. It reads only ``anchor_rpy`` from
        it today, which neither the calibration offset nor the plane rotation
        changes -- so this is a trap disarmed rather than a bug fixed, and it
        stays that way only while nobody adds a second read."""
        self.config = self.config._replace(plane=plane)

    def _coordination_loop(self) -> None:
        """The coordination thread: drain at 200 Hz, watchdog at 50 Hz."""
        ticks = 0
        while not self._coord_thread_stop.wait(timeout=0.005):
            self._drain()
            ticks += 1
            if ticks % 4 == 0:
                self._watchdog_tick()

    def _drain(self) -> None:
        session_open = self.ui.session_open
        if session_open and not self._session_was_open:
            # A fresh client needs the full state (config, ink) immediately.
            self.dispatcher.push_display()
        self._session_was_open = session_open
        while True:
            events = self.ui.drain()
            for event in events:
                self.dispatcher.on_input_event(event)
            if len(events) < 64:
                break
        while True:
            try:
                event = self._route_events.get_nowait()
            except queue.Empty:
                break
            self.dispatcher.on_route_event(event)

    def _watchdog_tick(self) -> None:
        lost = self.ui.input_lost()
        if lost and not self._input_was_lost:
            self._loss_started = time.monotonic()
            self.get_logger().warning('client input lost')
            self.dispatcher.on_input_loss()
        elif not lost and self._input_was_lost:
            self.get_logger().info(
                f'client input recovered after '
                f'{time.monotonic() - self._loss_started:.1f} s')
        self._input_was_lost = lost

    def _live_loop(self) -> None:
        period = float(self.get_parameter('live_tick').value)
        while not self._live_thread_stop.wait(timeout=period):
            self.live.tick()

    def shutdown(self) -> None:
        self._coord_thread_stop.set()
        self._live_thread_stop.set()
        self.ui.shutdown()
        self.batch.shutdown()
        self.stroke.shutdown()
        self._work_node.destroy_node()
        self._stroke_work_node.destroy_node()
        self._control_node.destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    try:
        bridge = DrawPlaneBridge()
    except RuntimeError as error:
        print(f'draw_plane_bridge: {error}')
        rclpy.shutdown()
        return 1
    bridge.start()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(bridge)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.shutdown()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
