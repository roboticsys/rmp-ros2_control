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

"""Send a smooth quintic FollowJointTrajectory goal to the passthrough controller.

The generic smoke test for the streaming path, meant for the two-axis phantom
bringup (two_axis.launch.py): the rapidcode_passthrough_trajectory_controller
streams the waypoints to the rapidcode_system hardware over the
trajectory_transfer gpio in chunks, and RapidCode interpolates between them with
MovePVT. No MoveIt, no robot model.

The trajectory is a rest-to-rest quintic from each joint's CURRENT position (read
from /joint_states) to its target over T seconds, sampled at N points with
velocities and accelerations. Its first point is the measured state at t=0, so it
passes the controller's first_point_tolerance and never commands a step, and a
second run continues from wherever the first one ended.

Usage:
  ros2 run rapidcode_bringup send_passthrough_trajectory.py                 # j1->0.5, j2->-0.3, 2 s
  ros2 run rapidcode_bringup send_passthrough_trajectory.py 0.8 -0.5 3.0    # targets + duration
  ros2 run rapidcode_bringup send_passthrough_trajectory.py 0.0 0.0         # back to zero
  ros2 run rapidcode_bringup send_passthrough_trajectory.py --joints joint1 -- 0.5  # one joint

Targets are absolute positions in the joints' units (radians for the URDF's
revolute joints). Use jog_single_joint.py for the elfin5 on real hardware.
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def quintic(start, end, phase):
    """Position/velocity/acceleration of a rest-to-rest quintic at phase in [0,1]."""

    distance = end - start
    pos = start + distance * (10 * phase**3 - 15 * phase**4 + 6 * phase**5)
    vel = distance * (30 * phase**2 - 60 * phase**3 + 30 * phase**4)
    acc = distance * (60 * phase - 180 * phase**2 + 120 * phase**3)
    return pos, vel, acc


def parse_args(argv):
    joints = ["joint1", "joint2"]
    if "--joints" in argv:
        i = argv.index("--joints")
        j = argv.index("--") if "--" in argv else len(argv)
        joints = argv[i + 1:j]
        argv = argv[:i] + argv[j + 1:] if "--" in argv else argv[:i]
    nums = [float(a) for a in argv]
    # Trailing number is the duration if there's one more value than joints.
    if len(nums) == len(joints) + 1:
        duration = nums[-1]
        targets = nums[:-1]
    else:
        duration = 2.0
        targets = nums if nums else [0.5, -0.3][:len(joints)]
    # Pad/truncate targets to the joint count.
    targets = (targets + [0.0] * len(joints))[:len(joints)]
    return joints, targets, duration


class Sender(Node):
    def __init__(self, joints):
        super().__init__("send_passthrough_trajectory")
        self.joints = list(joints)
        self.current = {}
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)

    def _on_state(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current[name] = pos

    def wait_for_state(self, timeout_s=5.0):
        for _ in range(int(timeout_s / 0.1)):
            if all(j in self.current for j in self.joints):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False


def build_trajectory(joints, starts, targets, duration, num_points):
    traj = JointTrajectory()
    traj.joint_names = list(joints)
    for k in range(num_points):
        tau = k / (num_points - 1)
        t = tau * duration
        pt = JointTrajectoryPoint()
        for start, target in zip(starts, targets):
            p, v, a = quintic(start, target, tau)
            pt.positions.append(p)
            pt.velocities.append(v / duration)
            pt.accelerations.append(a / (duration ** 2))
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
        traj.points.append(pt)
    return traj


def main():
    joints, targets, duration = parse_args(sys.argv[1:])
    num_points = 21

    rclpy.init()
    node = Sender(joints)
    log = node.get_logger()

    if not node.wait_for_state():
        log.error(f"no /joint_states carrying {joints} within 5 s "
                  "(is the bringup running with these joint names?)")
        return 1
    starts = [node.current[j] for j in joints]

    action_name = "/rapidcode_passthrough_trajectory_controller/follow_joint_trajectory"
    client = ActionClient(node, FollowJointTrajectory, action_name)
    if not client.wait_for_server(timeout_sec=5.0):
        log.error(f"action server {action_name} not available "
                  "(is the passthrough controller active?)")
        return 1

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = build_trajectory(joints, starts, targets, duration, num_points)
    log.info(f"sending {num_points}-pt quintic over {duration}s: " +
             ", ".join(f"{j}: {s:.3f} -> {t:.3f}" for j, s, t in zip(joints, starts, targets)))

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    handle = send_future.result()
    if handle is None or not handle.accepted:
        log.error("goal rejected")
        return 1
    log.info("goal accepted; executing...")

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=duration + 30.0)
    result = result_future.result()
    if result is None:
        log.error(f"no result within {duration + 30.0:.0f} s (timeout)")
        return 1
    log.info(f"result: status={result.status} error_code={result.result.error_code} "
             f"'{result.result.error_string}'")
    rclpy.shutdown()
    return 0 if result.result.error_code == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
