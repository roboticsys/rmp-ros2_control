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

"""Send a BLENDED Pilz CARTESIAN sequence (straight LINes + a CIRC arc) to move_group.

Companion to send_pilz_sequence.py. That script sends PTP (joint-space) segments;
this one sends the Cartesian trajectory generators -- LIN (straight-line TCP motion)
and CIRC (circular arc) -- as a single MoveGroupSequence. Pilz interpolates each
segment in Cartesian space, solves IK internally (the KDL solver from kinematics.yaml)
at every sample, and blends the segments into ONE continuous trajectory. With
blend_radius > 0 the TCP flows through the corners without stopping. move_group then
executes it via the rapidcode_passthrough_trajectory_controller's follow_joint_trajectory
action -> RapidCode MultiAxis (MovePVT).

You do NOT run IK yourself: LIN/CIRC ARE the Cartesian generators and the IK layer.
You only supply Cartesian goal poses + the segment type. The arc (CIRC) additionally
needs a third point -- an "interim" point ON the arc (used here), or the circle
"center" -- passed via the request's path_constraints (Pilz's message convention).

To guarantee the small Cartesian moves are reachable, the sequence starts with a PTP
to a dexterous READY joint pose; its TCP pose is obtained WITHOUT moving via the
/compute_fk service, and all Cartesian waypoints are offsets (in the `world` frame)
from that pose, holding orientation fixed (pure translation -- a clear, IK-friendly
demo). Use --skip-ready to instead build the loop around the CURRENT TCP pose (looked
up from TF); only do that when the arm is already in a dexterous, non-singular pose.

Prereqs (separate exec sessions):
  1. ros2 launch rapidcode_bringup elfin5.launch.py             # controllers + phantom axes
  2. ros2 launch rapidcode_moveit_config move_group.launch.py   # move_group + Pilz + sequence cap
  (optional, for viewing) ros2 launch rapidcode_moveit_config moveit_rviz.launch.py

Usage:
  ros2 run rapidcode_bringup send_pilz_cartesian_sequence.py                 # ready + LIN/LIN/CIRC/LIN loop
  ros2 run rapidcode_bringup send_pilz_cartesian_sequence.py --blend 0.0     # stop at each corner (always valid)
  ros2 run rapidcode_bringup send_pilz_cartesian_sequence.py --blend 0.05    # more blending
  ros2 run rapidcode_bringup send_pilz_cartesian_sequence.py --skip-ready    # loop around the CURRENT pose
  ros2 run rapidcode_bringup send_pilz_cartesian_sequence.py --vel 0.2 --acc 0.2
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from moveit_msgs.action import MoveGroupSequence
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MotionSequenceItem,
    MotionSequenceRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
)

GROUP = "elfin_arm"
JOINTS = [
    "elfin_joint1", "elfin_joint2", "elfin_joint3",
    "elfin_joint4", "elfin_joint5", "elfin_joint6",
]
PLANNING_FRAME = "world"      # URDF root == MoveIt model frame
TIP_LINK = "elfin_end_link"   # elfin_arm solver tip / TCP (SRDF end-effector parent)
ACTION = "/sequence_move_group"
FK_SERVICE = "/compute_fk"
PIPELINE = "pilz_industrial_motion_planner"

# Goal pose tolerances (checked at the segment endpoint only).
GOAL_POS_TOL = 0.01   # m
GOAL_ORI_TOL = 0.01   # rad
# The CIRC interim/center point is carried in path_constraints (Pilz's convention),
# but the ValidateSolution response adapter treats path_constraints as "must hold at
# EVERY waypoint" -- a tight region there rejects the whole arc. Pilz reads only the
# point's position and ignores the region size, so make the region large enough that
# validation always passes while Pilz still gets the exact interim point.
AUX_REGION = 10.0     # m (sphere radius; intentionally huge, see above)

# Dexterous starting configuration (elbow + wrist bent, away from the all-zeros
# shoulder singularity) so the small Cartesian moves below are always solvable.
READY_POSE = [0.0, 0.0, 1.5708, 0.0, 1.5708, 0.0]

# Cartesian recipe: each entry is (planner_id, goal_offset, aux) where goal_offset is
# (dx, dy, dz) in the `world` frame relative to the start TCP, and aux (CIRC only) is
# (kind, (dx,dy,dz)) giving an "interim" point ON the arc (also relative to start).
# Orientation is held at the start orientation for every point (pure translation).
# Shape: right -> up -> arc over the top to the left -> straight back to start.
CART_RECIPE = [
    # # BOX
    # ("LIN",  (0.20, 0.00, 0.00), None),
    # ("LIN",  (0.20, 0.20, 0.00), None),
    # ("LIN",  (0.00, 0.20, 0.00), None),
    # ("LIN",  (0.00, 0.00, 0.00), None),

    # # CIRCLE
    # ("CIRC", (0.20, 0.00, 0.00), ("interim", (0.10, 0.10, 0.0))),
    # ("CIRC", (0.00, 0.00, 0.00), ("interim", (0.10, -0.10, 0.0))),

    # HELIX
    ("LIN",  (0.00, 0.00, -0.10), None),
    ("CIRC", (0.10, 0.00, -0.05), ("interim", (0.05,  0.05, -0.075))),
    ("CIRC", (0.00, 0.00,  0.00), ("interim", (0.05, -0.05, -0.025))),
    ("CIRC", (0.10, 0.00,  0.05), ("interim", (0.05,  0.05,  0.025))),
    ("CIRC", (0.00, 0.00,  0.10), ("interim", (0.05, -0.05,  0.075))),
    ("CIRC", (0.10, 0.00,  0.15), ("interim", (0.05,  0.05,  0.125))),
    ("CIRC", (0.00, 0.00,  0.20), ("interim", (0.05, -0.05,  0.175))),
    ("LIN",  (0.00, 0.00,  0.00), None),
]


def parse_args(argv):
    opts = {"blend": 0.03, "vel": 0.15, "acc": 0.1, "skip_ready": False}
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--skip-ready":
            opts["skip_ready"] = True
            i += 1
        elif a in ("--blend", "--vel", "--acc") and i + 1 < len(argv):
            opts[a[2:]] = float(argv[i + 1])
            i += 2
        else:
            i += 1
    return opts


def offset_pose(base, d):
    """A copy of `base` translated by d=(dx,dy,dz), orientation unchanged."""
    p = Pose()
    p.position.x = base.position.x + d[0]
    p.position.y = base.position.y + d[1]
    p.position.z = base.position.z + d[2]
    p.orientation = base.orientation
    return p


def offset_point(base, d):
    return Point(x=base.position.x + d[0], y=base.position.y + d[1], z=base.position.z + d[2])


def position_constraint(point, tol=1e-3):
    """A PositionConstraint pinning TIP_LINK's origin at `point` (a geometry Point)."""
    pc = PositionConstraint()
    pc.header.frame_id = PLANNING_FRAME
    pc.link_name = TIP_LINK
    pc.constraint_region.primitives.append(
        SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[tol]))
    region_pose = Pose()
    region_pose.position = point
    region_pose.orientation.w = 1.0
    pc.constraint_region.primitive_poses.append(region_pose)
    pc.weight = 1.0
    return pc


def pose_goal_constraints(pose, pos_tol=GOAL_POS_TOL, ang_tol=GOAL_ORI_TOL):
    """goal_constraints for a Cartesian pose = position + orientation on TIP_LINK
    (the message form constructGoalConstraints() builds; Pilz reads the target from it)."""
    c = Constraints()
    c.position_constraints.append(position_constraint(pose.position, pos_tol))
    oc = OrientationConstraint()
    oc.header.frame_id = PLANNING_FRAME
    oc.link_name = TIP_LINK
    oc.orientation = pose.orientation
    oc.absolute_x_axis_tolerance = ang_tol
    oc.absolute_y_axis_tolerance = ang_tol
    oc.absolute_z_axis_tolerance = ang_tol
    oc.weight = 1.0
    c.orientation_constraints.append(oc)
    return c


def base_request(planner_id, vel, acc):
    req = MotionPlanRequest()
    req.pipeline_id = PIPELINE
    req.planner_id = planner_id
    req.group_name = GROUP
    req.max_velocity_scaling_factor = vel
    req.max_acceleration_scaling_factor = acc
    req.allowed_planning_time = 5.0
    return req


def ptp_item(joint_positions, blend, vel, acc):
    item = MotionSequenceItem()
    item.blend_radius = blend
    req = base_request("PTP", vel, acc)
    goal = Constraints()
    for name, position in zip(JOINTS, joint_positions):
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


def cart_item(planner_id, goal_pose, aux, blend, vel, acc):
    item = MotionSequenceItem()
    item.blend_radius = blend
    req = base_request(planner_id, vel, acc)
    req.goal_constraints.append(pose_goal_constraints(goal_pose))
    if planner_id == "CIRC" and aux is not None:
        kind, point = aux
        # Pilz reads the arc's interim/center point from path_constraints: the name
        # selects "interim" (a point on the arc) vs "center" (the circle centre); the
        # point is the position-constraint region's pose position.
        path = Constraints()
        path.name = kind
        path.position_constraints.append(position_constraint(point, AUX_REGION))
        req.path_constraints = path
    item.req = req
    return item


def compute_fk(node, joint_positions):
    """TCP pose of TIP_LINK at `joint_positions`, via /compute_fk (no motion)."""
    cli = node.create_client(GetPositionFK, FK_SERVICE)
    if not cli.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(f"{FK_SERVICE} not available -- is move_group up?")
        return None
    req = GetPositionFK.Request()
    req.header.frame_id = PLANNING_FRAME
    req.fk_link_names = [TIP_LINK]
    req.robot_state = RobotState()
    req.robot_state.joint_state = JointState(name=list(JOINTS), position=list(joint_positions))
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    resp = future.result()
    if resp is None or resp.error_code.val != 1 or not resp.pose_stamped:
        node.get_logger().error(f"/compute_fk failed (error_code="
                                f"{getattr(resp, 'error_code', None) and resp.error_code.val})")
        return None
    return resp.pose_stamped[0].pose


def current_tcp(node):
    """Current TCP pose of TIP_LINK in PLANNING_FRAME, from TF."""
    buffer = Buffer()
    TransformListener(buffer, node)
    for _ in range(50):  # up to ~5 s for the TF tree to populate
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            t = buffer.lookup_transform(PLANNING_FRAME, TIP_LINK, rclpy.time.Time()).transform
        except Exception:
            continue
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (
            t.translation.x, t.translation.y, t.translation.z)
        pose.orientation = t.rotation
        return pose
    node.get_logger().error(f"could not look up TF {PLANNING_FRAME} -> {TIP_LINK}")
    return None


def send_sequence(node, client, items, label):
    goal = MoveGroupSequence.Goal()
    goal.request = MotionSequenceRequest(items=items)
    goal.planning_options = PlanningOptions()
    goal.planning_options.plan_only = False  # plan AND execute
    node.get_logger().info(f"sending {label}: {len(items)} items")

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    handle = send_future.result()
    if handle is None or not handle.accepted:
        node.get_logger().error("sequence goal REJECTED by move_group")
        return False
    node.get_logger().info("accepted; planning + executing...")

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=120.0)
    result = result_future.result()
    if result is None:
        node.get_logger().error("no result within 120 s (timeout)")
        return False
    resp = result.result.response
    ok = resp.error_code.val == 1  # MoveItErrorCodes.SUCCESS
    node.get_logger().info(
        f"done: error_code={resp.error_code.val} ({'SUCCESS' if ok else 'FAILURE'}), "
        f"planned_trajectories={len(resp.planned_trajectories)}, "
        f"planning_time={resp.planning_time:.3f}s")
    return ok


def main():
    opts = parse_args(sys.argv[1:])
    rclpy.init()
    node = Node("send_pilz_cartesian_sequence")
    client = ActionClient(node, MoveGroupSequence, ACTION)
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error(
            f"action server {ACTION} not available -- is move_group up with the Pilz "
            "MoveGroupSequence capability?")
        return 1

    items = []
    if opts["skip_ready"]:
        start = current_tcp(node)
    else:
        start = compute_fk(node, READY_POSE)
        if start is not None:
            # Move to the dexterous ready pose first (blend 0 = clean arrival), then
            # the Cartesian loop blends around that pose.
            items.append(ptp_item(READY_POSE, 0.0, opts["vel"], opts["acc"]))
    if start is None:
        return 1
    node.get_logger().info(
        f"start TCP ({TIP_LINK} in {PLANNING_FRAME}): "
        f"({start.position.x:.3f}, {start.position.y:.3f}, {start.position.z:.3f})")

    for planner_id, goal_off, aux in CART_RECIPE:
        goal_pose = offset_pose(start, goal_off)
        aux_abs = (aux[0], offset_point(start, aux[1])) if aux else None
        items.append(cart_item(planner_id, goal_pose, aux_abs, opts["blend"], opts["vel"], opts["acc"]))
    items[-1].blend_radius = 0.0  # last item must not blend

    ok = send_sequence(node, client, items, f"Pilz Cartesian sequence (blend={opts['blend']} m)")
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
