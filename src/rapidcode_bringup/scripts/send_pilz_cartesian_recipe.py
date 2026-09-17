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

"""Send a Pilz sequence read from a YAML RECIPE FILE (data-driven send_pilz_cartesian_sequence.py).

Same engine as send_pilz_cartesian_sequence.py -- builds a Pilz MoveGroupSequence of
PTP/LIN/CIRC segments, blends them into one continuous trajectory, and plans+executes
via move_group -> rapidcode_passthrough_trajectory_controller -> MovePVT. The only
difference is the recipe comes from a file instead of the source, so you can iterate on
paths without editing Python.

RECIPE FILE FORMAT (YAML)
-------------------------
  coordinates: relative | absolute     # how xyz / interim / center are interpreted:
                                        #   relative (default) = offsets (m) from the start TCP
                                        #   absolute           = positions in the `world` frame
  ready: [j1, j2, j3, j4, j5, j6]      # optional: PTP to this joint pose first; its TCP (via
                                        #   /compute_fk, no motion) becomes the start for
                                        #   'relative' coords. Omit / null -> start = CURRENT TCP.
  anchor: {xyz: [x, y, z], rpy: [r, p, y]}  # optional, for 'relative' coords: an ABSOLUTE
                                        #   world pose. The sequence LINs to it (the one absolute
                                        #   positioning move) and it -- not the ready/current TCP --
                                        #   becomes the start that offsets are measured from. Edit
                                        #   `anchor` alone to relocate the whole path. rpy optional
                                        #   (default identity); set it to the drawing orientation so
                                        #   the approach doesn't reorient into the first segment.
  defaults: {blend: 0.02, vel: 0.1, acc: 0.1}   # applied to every segment unless overridden
  segments:                            # ordered list; each is one of:
    - {type: lin,  xyz: [dx, dy, dz]}                         # straight line to a position
    - {type: lin,  xyz: [dx, dy, dz], rpy: [r, p, y]}         # + reorient (rpy in radians)
    - {type: circ, xyz: [...], interim: [ix, iy, iz]}         # arc through a point ON it
    - {type: circ, xyz: [...], center:  [cx, cy, cz]}         # arc around a circle centre
    - {type: ptp,  joints: [j1..j6]}                          # joint-space move
  # per-segment overrides: blend / vel / acc, and for orientation: rpy + rpy_frame.
  # rpy_frame: start (default, rotation relative to the start orientation) | world (absolute).
  # A segment with no rpy holds the start orientation. The CIRC interim/center is position-only.
  # The LAST segment's blend is always forced to 0 (Pilz requires it).

Pass the recipe as the first argument; a bare name (no '/') is looked up in this package's
config/ dir. CLI --blend/--vel/--acc override the file's `defaults` block.

Prereqs (separate exec sessions):
  1. ros2 launch rapidcode_bringup elfin5.launch.py
  2. ros2 launch rapidcode_moveit_config move_group.launch.py
  (optional) ros2 launch rapidcode_moveit_config moveit_rviz.launch.py

Usage:
  ros2 run rapidcode_bringup send_pilz_cartesian_recipe.py recipe_box.yaml         # bundled example
  ros2 run rapidcode_bringup send_pilz_cartesian_recipe.py /abs/path/mypath.yaml
  ros2 run rapidcode_bringup send_pilz_cartesian_recipe.py recipe_box.yaml --blend 0.02
"""

import math
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point, Quaternion
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
TIP_LINK = "elfin_end_link"   # elfin_arm solver tip / TCP
ACTION = "/sequence_move_group"
FK_SERVICE = "/compute_fk"
PIPELINE = "pilz_industrial_motion_planner"

GOAL_POS_TOL = 0.01   # m  (goal pose tolerance, checked at the endpoint only)
GOAL_ORI_TOL = 0.01   # rad
# The CIRC interim/center is carried in path_constraints, which the ValidateSolution
# response adapter checks at EVERY waypoint. Pilz reads only the point's position and
# ignores the region size, so a large region lets validation pass. (See v1 for details.)
AUX_REGION = 10.0     # m
DEFAULTS = {"blend": 0.02, "vel": 0.1, "acc": 0.1}


# ---- orientation helpers ----------------------------------------------------
def quat_from_rpy(roll, pitch, yaw):
    """geometry_msgs Quaternion from roll(X)/pitch(Y)/yaw(Z) in radians."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy)


def quat_mul(a, b):
    """Quaternion product a ∘ b (apply b, then a)."""
    return Quaternion(
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w)


# ---- constraint / item builders (shared with v1) ----------------------------
def position_constraint(point, tol):
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


def pose_goal_constraints(pose):
    c = Constraints()
    c.position_constraints.append(position_constraint(pose.position, GOAL_POS_TOL))
    oc = OrientationConstraint()
    oc.header.frame_id = PLANNING_FRAME
    oc.link_name = TIP_LINK
    oc.orientation = pose.orientation
    oc.absolute_x_axis_tolerance = GOAL_ORI_TOL
    oc.absolute_y_axis_tolerance = GOAL_ORI_TOL
    oc.absolute_z_axis_tolerance = GOAL_ORI_TOL
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
        path = Constraints()
        path.name = kind
        path.position_constraints.append(position_constraint(point, AUX_REGION))
        req.path_constraints = path
    item.req = req
    return item


# ---- start-pose lookup (shared with v1) -------------------------------------
def compute_fk(node, joint_positions):
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
        node.get_logger().error("/compute_fk failed")
        return None
    return resp.pose_stamped[0].pose


def current_tcp(node):
    buffer = Buffer()
    TransformListener(buffer, node)
    for _ in range(50):
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


# ---- recipe parsing ---------------------------------------------------------
def point_from(xyz, start, coords):
    x, y, z = (float(v) for v in xyz)
    if coords == "absolute":
        return Point(x=x, y=y, z=z)
    return Point(x=start.position.x + x, y=start.position.y + y, z=start.position.z + z)


def pose_from_anchor(anchor):
    """Absolute world Pose from an `anchor: {xyz:[x,y,z], rpy:[r,p,y]}` mapping.

    rpy is optional (default: identity). This pose is both the target of the one
    absolute positioning move and the start that 'relative' offsets add to.
    """
    if not isinstance(anchor, dict):
        raise ValueError("anchor must be a mapping with 'xyz' (and optional 'rpy')")
    xyz = anchor.get("xyz")
    if not xyz or len(xyz) != 3:
        raise ValueError("anchor needs 'xyz' [x, y, z]")
    rpy = anchor.get("rpy")
    if rpy is not None and len(rpy) != 3:
        raise ValueError("anchor 'rpy' must be [roll, pitch, yaw]")
    orientation = (quat_from_rpy(*(float(v) for v in rpy)) if rpy is not None
                   else Quaternion(w=1.0))
    return Pose(position=Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])),
                orientation=orientation)


def orientation_from(seg, start):
    rpy = seg.get("rpy")
    if rpy is None:
        return start.orientation
    if len(rpy) != 3:
        raise ValueError("rpy must be [roll, pitch, yaw]")
    q = quat_from_rpy(*(float(v) for v in rpy))
    frame = seg.get("rpy_frame", "start")
    if frame == "world":
        return q
    if frame == "start":
        return quat_mul(start.orientation, q)
    raise ValueError(f"rpy_frame must be 'world' or 'start', got '{frame}'")


def build_item(seg, idx, start, coords, defaults):
    if not isinstance(seg, dict) or "type" not in seg:
        raise ValueError(f"segment {idx}: must be a mapping with a 'type'")
    stype = str(seg["type"]).lower()
    blend = float(seg.get("blend", defaults["blend"]))
    vel = float(seg.get("vel", defaults["vel"]))
    acc = float(seg.get("acc", defaults["acc"]))

    if stype == "ptp":
        joints = seg.get("joints")
        if not joints or len(joints) != len(JOINTS):
            raise ValueError(f"segment {idx}: ptp needs 'joints' with {len(JOINTS)} values")
        return ptp_item([float(v) for v in joints], blend, vel, acc)

    if stype in ("lin", "circ"):
        xyz = seg.get("xyz")
        if not xyz or len(xyz) != 3:
            raise ValueError(f"segment {idx}: {stype} needs 'xyz' [x, y, z]")
        goal_pose = Pose(position=point_from(xyz, start, coords),
                         orientation=orientation_from(seg, start))
        aux = None
        if stype == "circ":
            if ("interim" in seg) == ("center" in seg):
                raise ValueError(
                    f"segment {idx}: circ needs exactly one of 'interim' or 'center'")
            kind = "interim" if "interim" in seg else "center"
            pt = seg[kind]
            if not pt or len(pt) != 3:
                raise ValueError(f"segment {idx}: circ '{kind}' needs [x, y, z]")
            aux = (kind, point_from(pt, start, coords))
        return cart_item(stype.upper(), goal_pose, aux, blend, vel, acc)

    raise ValueError(f"segment {idx}: unknown type '{stype}' (use lin | circ | ptp)")


def resolve_recipe_path(arg):
    if os.path.isfile(arg):
        return arg
    try:
        cand = os.path.join(get_package_share_directory("rapidcode_bringup"), "config", arg)
        if os.path.isfile(cand):
            return cand
    except Exception:
        pass
    return None


def parse_args(argv):
    out = {"recipe": None, "overrides": {}}
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in ("--blend", "--vel", "--acc") and i + 1 < len(argv):
            out["overrides"][a[2:]] = float(argv[i + 1])
            i += 2
        elif not a.startswith("-") and out["recipe"] is None:
            out["recipe"] = a
            i += 1
        else:
            i += 1
    return out


# ---- send (shared with v1) --------------------------------------------------
def send_sequence(node, client, items, label):
    goal = MoveGroupSequence.Goal()
    goal.request = MotionSequenceRequest(items=items)
    goal.planning_options = PlanningOptions()
    goal.planning_options.plan_only = False
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
    ok = resp.error_code.val == 1
    node.get_logger().info(
        f"done: error_code={resp.error_code.val} ({'SUCCESS' if ok else 'FAILURE'}), "
        f"planned_trajectories={len(resp.planned_trajectories)}, "
        f"planning_time={resp.planning_time:.3f}s")
    return ok


def main():
    args = parse_args(sys.argv[1:])
    if not args["recipe"]:
        print(__doc__)
        print("error: a recipe file (path or bundled name) is required", file=sys.stderr)
        return 2
    path = resolve_recipe_path(args["recipe"])
    if path is None:
        print(f"error: recipe '{args['recipe']}' not found (not a file, not in package config/)",
              file=sys.stderr)
        return 2
    with open(path) as f:
        doc = yaml.safe_load(f) or {}

    coords = str(doc.get("coordinates", "relative")).lower()
    if coords not in ("relative", "absolute"):
        print(f"error: coordinates must be 'relative' or 'absolute', got '{coords}'",
              file=sys.stderr)
        return 2
    defaults = dict(DEFAULTS, **(doc.get("defaults") or {}))
    defaults.update(args["overrides"])   # CLI overrides the file's defaults block
    ready = doc.get("ready")
    anchor = doc.get("anchor")
    segments = doc.get("segments") or []
    if not segments:
        print("error: recipe has no 'segments'", file=sys.stderr)
        return 2
    if anchor is not None and coords != "relative":
        print("warning: 'anchor' only affects 'relative' coordinates; ignoring for "
              f"'{coords}' coords (segment positions are already absolute)", file=sys.stderr)
        anchor = None

    rclpy.init()
    node = Node("send_pilz_cartesian_recipe")
    client = ActionClient(node, MoveGroupSequence, ACTION)
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error(f"action server {ACTION} not available -- is move_group up "
                                "with the Pilz MoveGroupSequence capability?")
        return 1

    items = []
    # Optional joint-space gross approach (PTP): reach a good arm configuration
    # before any Cartesian move. Also the relative-coords start when no `anchor`.
    ready_tcp = None
    if ready is not None:
        if len(ready) != len(JOINTS):
            node.get_logger().error(f"'ready' needs {len(JOINTS)} joint values")
            return 1
        ready_tcp = compute_fk(node, [float(v) for v in ready])
        if ready_tcp is not None:
            items.append(ptp_item([float(v) for v in ready], 0.0, defaults["vel"], defaults["acc"]))

    # `anchor` (absolute world pose) is the single absolute positioning move: LIN
    # to it, and it becomes the start that 'relative' offsets are measured from --
    # so editing `anchor` alone relocates the whole path. Falls back to the ready
    # TCP, then the current TCP.
    if anchor is not None:
        try:
            start = pose_from_anchor(anchor)
        except ValueError as exc:
            node.get_logger().error(f"recipe error: {exc}")
            return 2
        items.append(cart_item("LIN", start, None,
                               defaults["blend"], defaults["vel"], defaults["acc"]))
    elif ready is not None:
        start = ready_tcp
    else:
        start = current_tcp(node)
    if start is None:
        return 1
    node.get_logger().info(
        f"recipe '{os.path.basename(path)}' ({coords} coords, {len(segments)} segments); "
        f"start TCP ({start.position.x:.3f}, {start.position.y:.3f}, {start.position.z:.3f})")

    try:
        for idx, seg in enumerate(segments):
            items.append(build_item(seg, idx, start, coords, defaults))
    except ValueError as exc:
        node.get_logger().error(f"recipe error: {exc}")
        return 2
    items[-1].blend_radius = 0.0  # Pilz requires the last item not to blend

    ok = send_sequence(node, client, items, f"Pilz recipe '{os.path.basename(path)}'")
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
