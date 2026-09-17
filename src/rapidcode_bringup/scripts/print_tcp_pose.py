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

"""Print the TCP (elfin_end_link) pose in the `world` frame as a paste-ready
`anchor:` block for the send_*_recipe.py recipes.

Uses the SAME machinery the recipe senders use to resolve their start pose --
MoveIt's /compute_fk, frame 'world', tip 'elfin_end_link' -- so the printed pose
is exactly what a recipe would treat as its anchor. No motion is commanded.

Modes:
  - Give 6 joint values -> FK for THAT joint config (e.g. your `ready`), no move.
  - Give none          -> read the CURRENT /joint_states and FK the live pose.

Usage (ROS env sourced in the container):
  ros2 run rapidcode_bringup print_tcp_pose.py                              # current pose
  ros2 run rapidcode_bringup print_tcp_pose.py 0 0 -1.5708 0 -1.5708 3.1415 # a `ready` config
  # (or, without a rebuild:)
  python3 /ros2_ws/src/rapidcode_bringup/scripts/print_tcp_pose.py 0 0 -1.5708 0 -1.5708 3.1415
"""
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK

PLANNING_FRAME = "world"
TIP_LINK = "elfin_end_link"
FK_SERVICE = "/compute_fk"
JOINTS = ["elfin_joint1", "elfin_joint2", "elfin_joint3",
          "elfin_joint4", "elfin_joint5", "elfin_joint6"]


def rpy_from_quat(w, x, y, z):
    """roll(X), pitch(Y), yaw(Z) in radians from a (w,x,y,z) quaternion."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sp) if abs(sp) >= 1.0 else math.asin(sp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def read_current_joints(node):
    got = {}
    sub = node.create_subscription(
        JointState, "/joint_states",
        lambda m: got.update({n: p for n, p in zip(m.name, m.position)}), 10)
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(j in got for j in JOINTS):
            break
    node.destroy_subscription(sub)
    if not all(j in got for j in JOINTS):
        node.get_logger().error("could not read /joint_states"); return None
    return [got[j] for j in JOINTS]


def fk(node, joints):
    cli = node.create_client(GetPositionFK, FK_SERVICE)
    if not cli.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(f"{FK_SERVICE} unavailable -- is move_group up?"); return None
    req = GetPositionFK.Request()
    req.header.frame_id = PLANNING_FRAME
    req.fk_link_names = [TIP_LINK]
    req.robot_state.joint_state = JointState(name=list(JOINTS), position=list(joints))
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    resp = fut.result()
    if resp is None or resp.error_code.val != 1 or not resp.pose_stamped:
        node.get_logger().error("/compute_fk failed"); return None
    return resp.pose_stamped[0].pose


def main():
    args = sys.argv[1:]
    joints = None
    if args:
        if len(args) != len(JOINTS):
            print(f"error: give exactly {len(JOINTS)} joint values, or none for the "
                  f"current pose (got {len(args)})", file=sys.stderr)
            return 2
        try:
            joints = [float(v) for v in args]
        except ValueError:
            print("error: joint values must be numbers", file=sys.stderr); return 2

    rclpy.init()
    node = Node("print_tcp_pose")
    if joints is None:
        joints = read_current_joints(node)
        src = "current /joint_states"
    else:
        src = "given joint values"
    if joints is None:
        rclpy.shutdown(); return 1

    pose = fk(node, joints)
    rclpy.shutdown()
    if pose is None:
        return 1

    p, q = pose.position, pose.orientation
    roll, pitch, yaw = rpy_from_quat(q.w, q.x, q.y, q.z)
    print(f"# TCP ({TIP_LINK}) in '{PLANNING_FRAME}' from {src}")
    print(f"# joints: {[round(v, 5) for v in joints]}")
    print(f"# quaternion (w,x,y,z): "
          f"[{q.w:.6f}, {q.x:.6f}, {q.y:.6f}, {q.z:.6f}]")
    print("anchor:")
    print(f"  xyz: [{p.x:.5f}, {p.y:.5f}, {p.z:.5f}]")
    print(f"  rpy: [{roll:.6f}, {pitch:.6f}, {yaw:.6f}]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
