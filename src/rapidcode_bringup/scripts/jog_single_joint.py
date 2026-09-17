#!/usr/bin/env python3
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

"""Safely jog ONE joint a small relative amount through the passthrough controller.

Bring-up probe for real hardware: exercises the rapidcode_system plugin's full
chunk-streaming path (PassthroughTrajectoryController -> trajectory_transfer gpio ->
MovePVT over the 6-axis MultiAxis) WITHOUT MoveIt. send_passthrough_trajectory.py is
the generic-joint equivalent for the two-axis smoke test; this one knows the elfin5
joint names, moves a single joint by a small delta and holds the others.

Safety design (this is a real arm -- see the monorepo AGENTS.md):
  * Reads the CURRENT position of all six joints from /joint_states.
  * Moves only the target joint, by a small RELATIVE delta, and returns to start.
  * Holds the other five joints at their current position for the whole move
    (constant position, zero velocity/accel on every point).
  * The trajectory's FIRST point is exactly the current state at t=0 (at rest), so
    it passes the controller's first_point_tolerance and the firmware never has to
    step from a stale command position -- no first-frame following-error spike.
  * A rest-to-rest quintic => C2, starts/ends at zero velocity. With a small delta
    over a long duration the peak speed is tiny, so any following-error trip that
    still happens is a direction/seed/limit problem, not a speed problem.

Usage (run INSIDE the controller_manager container; ROS + ws sourced):
  # default: joint index 4 (elfin_joint5), +0.02 rad (~1.15 deg), 3 s each way
  python3 jog_single_joint.py
  python3 jog_single_joint.py --joint 4 --delta 0.02 --duration 3.0
  python3 jog_single_joint.py --joint 4 --delta -0.02          # other direction
  python3 jog_single_joint.py --joint 4 --delta 0.02 --dry-run  # print, don't send

--joint accepts an index (0-5) or a name (elfin_joint5). Watch the arm AND RViz:
confirm the joint moves the way RViz shows (else `invert` is wrong for that axis),
and watch PositionError per axis in RapidSetupX while it runs.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Full joint set in the controller's configured order (elfin5_controllers.yaml).
# The whole group is streamed as one MultiAxis, so we command every joint every
# point -- the target joint moves, the rest explicitly hold.
JOINTS = [f"elfin_joint{i}" for i in range(1, 7)]
ACTION = "/rapidcode_passthrough_trajectory_controller/follow_joint_trajectory"


def quintic(start, end, tau):
    """Rest-to-rest quintic position/velocity/accel at phase tau in [0, 1]."""
    d = end - start
    pos = start + d * (10 * tau**3 - 15 * tau**4 + 6 * tau**5)
    vel = d * (30 * tau**2 - 60 * tau**3 + 30 * tau**4)
    acc = d * (60 * tau - 180 * tau**2 + 120 * tau**3)
    return pos, vel, acc


def resolve_joint(spec):
    """--joint may be an index (0-5) or a joint name; return (index, name)."""
    try:
        idx = int(spec)
    except ValueError:
        if spec not in JOINTS:
            raise SystemExit(f"unknown joint '{spec}'; use 0-5 or one of {JOINTS}")
        return JOINTS.index(spec), spec
    if not 0 <= idx < len(JOINTS):
        raise SystemExit(f"joint index {idx} out of range 0-{len(JOINTS) - 1}")
    return idx, JOINTS[idx]


class JogSingleJoint(Node):
    def __init__(self):
        super().__init__("jog_single_joint")
        self.current = {}
        self.sub = self.create_subscription(JointState, "/joint_states", self._on_state, 10)

    def _on_state(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current[name] = pos

    def wait_for_state(self, timeout_s=5.0):
        deadline = int(timeout_s / 0.1)
        for _ in range(deadline):
            if all(j in self.current for j in JOINTS):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False


def build_trajectory(start_positions, target_idx, delta, duration, num_points):
    """current -> current+delta -> current on the target joint; others held.

    First point is at t=0 == current state (at rest). Two rest-to-rest quintic legs
    back to back: [0, duration] out, [duration, 2*duration] back.
    """
    traj = JointTrajectory()
    traj.joint_names = list(JOINTS)
    goal_val = start_positions[target_idx] + delta

    for leg, (a, b) in enumerate(((start_positions[target_idx], goal_val),
                                  (goal_val, start_positions[target_idx]))):
        # leg 0 includes its t=0 endpoint; leg 1 skips k=0 (== leg 0's last point).
        for k in range(0 if leg == 0 else 1, num_points):
            tau = k / (num_points - 1)
            t = leg * duration + tau * duration
            pt = JointTrajectoryPoint()
            for j in range(len(JOINTS)):
                if j == target_idx:
                    p, v, acc = quintic(a, b, tau)
                    p, v, acc = p, v / duration, acc / (duration**2)
                else:
                    p, v, acc = start_positions[j], 0.0, 0.0  # hold
                pt.positions.append(p)
                pt.velocities.append(v)
                pt.accelerations.append(acc)
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
            traj.points.append(pt)
    return traj


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", default="4", help="joint index 0-5 or name (default 4 = elfin_joint5)")
    ap.add_argument("--delta", type=float, default=0.02, help="relative move in rad (default 0.02 ~ 1.15 deg)")
    ap.add_argument("--duration", type=float, default=3.0, help="seconds per leg (default 3.0)")
    ap.add_argument("--points", type=int, default=41, help="waypoints per leg (default 41)")
    ap.add_argument("--dry-run", action="store_true", help="print the trajectory; do not send")
    args = ap.parse_args()

    target_idx, target_name = resolve_joint(args.joint)
    if abs(args.delta) > 0.2:
        raise SystemExit(f"--delta {args.delta} rad is large for a bring-up probe; "
                         "keep |delta| <= 0.2 (~11 deg) or edit this guard deliberately.")

    rclpy.init()
    node = JogSingleJoint()
    if not node.wait_for_state():
        node.get_logger().error("no /joint_states for all 6 joints; is the stack up?")
        return 1
    start = [node.current[j] for j in JOINTS]

    peak_vel = 1.875 * abs(args.delta) / args.duration  # rest-to-rest quintic peak
    node.get_logger().info(
        f"jog {target_name} (idx {target_idx}): {start[target_idx]:.5f} -> "
        f"{start[target_idx] + args.delta:.5f} -> {start[target_idx]:.5f} rad, "
        f"{args.duration}s/leg, peak ~{peak_vel:.4f} rad/s. Others held.")

    traj = build_trajectory(start, target_idx, args.delta, args.duration, args.points)

    if args.dry_run:
        p0, pT = traj.points[0], traj.points[args.points - 1]
        node.get_logger().info(f"{len(traj.points)} points. "
                               f"first={ [round(x, 4) for x in p0.positions] } "
                               f"peak={ [round(x, 4) for x in pT.positions] }")
        rclpy.shutdown()
        return 0

    client = ActionClient(node, FollowJointTrajectory, ACTION)
    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error(f"action server {ACTION} not available "
                                "(is the passthrough controller active?)")
        return 1

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    handle = send_future.result()
    if handle is None or not handle.accepted:
        node.get_logger().error("goal REJECTED (first_point_tolerance? controller inactive?)")
        return 1
    node.get_logger().info("goal accepted; executing...")

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=2.0 * args.duration + 15.0)
    result = result_future.result()
    if result is None:
        node.get_logger().error("no result (timeout)")
        return 1
    node.get_logger().info(
        f"result: status={result.status} error_code={result.result.error_code} "
        f"'{result.result.error_string}'")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
