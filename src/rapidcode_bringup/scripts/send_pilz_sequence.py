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

"""Send a BLENDED Pilz sequence of joint-space poses to move_group.

This is the "recipe of poses, no stops between them" path. It builds a
moveit_msgs/MotionSequenceRequest -- an ordered list of PTP (joint-space
point-to-point) segments, each carrying a blend_radius -- and sends it to the Pilz
MoveGroupSequence action (/sequence_move_group). Pilz plans the whole list into ONE
continuous trajectory: with blend_radius > 0 the arm blends THROUGH each interior
waypoint without decelerating to a stop. move_group then executes it, dispatching to
the rapidcode_passthrough_trajectory_controller's follow_joint_trajectory action,
which streams the sampled points to the RapidCode MultiAxis (MovePVT).

blend_radius is a Cartesian distance (m) around each waypoint; the last item MUST be
0, and adjacent blend spheres must not overlap. blend_radius=0 everywhere falls back
to stop-at-each-pose (always valid) -- useful to prove the recipe before tuning blend.

Prereqs (separate exec sessions):
  1. ros2 launch rapidcode_bringup elfin5.launch.py             # controllers + phantom axes
  2. ros2 launch rapidcode_moveit_config move_group.launch.py   # move_group + Pilz + sequence cap
  (optional, for viewing) ros2 launch rapidcode_moveit_config moveit_rviz.launch.py

Usage:
  ros2 run rapidcode_bringup send_pilz_sequence.py                  # default recipe, blend=0.1
  ros2 run rapidcode_bringup send_pilz_sequence.py --blend 0.05     # tighter blend
  ros2 run rapidcode_bringup send_pilz_sequence.py --blend 0.0      # stop at each pose (always valid)
  ros2 run rapidcode_bringup send_pilz_sequence.py --vel 0.2 --acc 0.2
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import MoveGroupSequence
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MotionSequenceItem,
    MotionSequenceRequest,
    PlanningOptions,
)

GROUP = "elfin_arm"
JOINTS = [
    "elfin_joint1", "elfin_joint2", "elfin_joint3",
    "elfin_joint4", "elfin_joint5", "elfin_joint6",
]
ACTION = "/sequence_move_group"
PIPELINE = "pilz_industrial_motion_planner"

# A "recipe" of joint-space poses (radians, one value per JOINT). The arm blends
# through the interior poses without stopping and ends back at home. Keep the angles
# modest and the poses well separated so the Cartesian blend spheres do not overlap.
DEFAULT_POSES = [
    [ 0.0, 0.0, 1.5708, 0.0, 1.5708, 0.0],
    [ -0.7854, 0.0, 1.5708, 0.0, 1.5708, 0.0],
    [ 0.7854, 0.0, 1.5708, 0.0, 1.5708, 0.0],
    [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
]


def parse_args(argv):
    opts = {"blend": 0.1, "vel": 0.1, "acc": 0.1}
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in ("--blend", "--vel", "--acc") and i + 1 < len(argv):
            opts[a[2:]] = float(argv[i + 1])
            i += 2
        else:
            i += 1
    return opts


def make_item(pose, blend, vel, acc):
    """One PTP segment to a joint-space pose, with a trailing blend_radius."""
    item = MotionSequenceItem()
    item.blend_radius = blend

    req = MotionPlanRequest()
    req.pipeline_id = PIPELINE
    req.planner_id = "PTP"
    req.group_name = GROUP
    req.max_velocity_scaling_factor = vel
    req.max_acceleration_scaling_factor = acc
    req.allowed_planning_time = 5.0
    # Leave start_state empty: the FIRST item starts from the current state, and Pilz
    # requires every subsequent item to start where the previous one ended.

    goal = Constraints()
    for name, position in zip(JOINTS, pose):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = position
        jc.tolerance_above = 1e-4
        jc.tolerance_below = 1e-4
        jc.weight = 1.0
        goal.joint_constraints.append(jc)
    req.goal_constraints.append(goal)

    item.req = req
    return item


def main():
    opts = parse_args(sys.argv[1:])
    poses = DEFAULT_POSES

    rclpy.init()
    node = Node("send_pilz_sequence")
    client = ActionClient(node, MoveGroupSequence, ACTION)
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error(
            f"action server {ACTION} not available -- is move_group up with the Pilz "
            "MoveGroupSequence capability? (see move_group.launch.py)")
        return 1

    seq = MotionSequenceRequest()
    for idx, pose in enumerate(poses):
        last = idx == len(poses) - 1
        seq.items.append(make_item(pose, 0.0 if last else opts["blend"], opts["vel"], opts["acc"]))

    goal = MoveGroupSequence.Goal()
    goal.request = seq
    goal.planning_options = PlanningOptions()
    goal.planning_options.plan_only = False  # plan AND execute

    node.get_logger().info(
        f"sending Pilz sequence: {len(poses)} PTP poses, blend={opts['blend']} m, "
        f"vel_scale={opts['vel']}, acc_scale={opts['acc']} (plan+execute)")

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    handle = send_future.result()
    if handle is None or not handle.accepted:
        node.get_logger().error("sequence goal REJECTED by move_group")
        return 1
    node.get_logger().info("sequence accepted; planning + executing...")

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=120.0)
    result = result_future.result()
    if result is None:
        node.get_logger().error("no result within 120 s (timeout)")
        return 1

    resp = result.result.response
    ok = resp.error_code.val == 1  # MoveItErrorCodes.SUCCESS == 1
    node.get_logger().info(
        f"done: error_code={resp.error_code.val} ({'SUCCESS' if ok else 'FAILURE'}), "
        f"planned_trajectories={len(resp.planned_trajectories)}, "
        f"planning_time={resp.planning_time:.3f}s")
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
