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

"""Draw ARBITRARY Cartesian curves (B-splines, splines-through-points, parametric
functions) that Pilz LIN/CIRC can't express -- one code path, one recipe format.

Every curve mode reduces to the SAME pipeline (there is no "arbitrary curve" motion
primitive anywhere in the stack -- the robot always executes joint trajectories):

    evaluate the curve  ->  dense Cartesian poses  ->  MoveIt /compute_cartesian_path
    (per-waypoint KDL IK + time-parameterization)  ->  /execute_trajectory
    ->  rapidcode_passthrough_trajectory_controller  ->  MovePVT

So the "spline" lives HERE, in the sender: we sample the curve into dense waypoints
and hand them to MoveIt, which interpolates LINEARLY between them (sample densely!)
and runs IK at each. The gross approach to `ready`/`anchor` is a Pilz PTP (joint
space, no straight-line constraint) so it can't trip the anchor-approach singularity.

This complements send_pilz_cartesian_recipe.py (LIN/CIRC/PTP) -- same recipe
conventions (coordinates / ready / anchor / relative offsets / world tool-down),
different segment vocabulary for the curved bits.

RECIPE FILE FORMAT (YAML)
-------------------------
  coordinates: relative | absolute     # how points are interpreted:
                                        #   relative (default) = offsets (m) from `anchor`
                                        #   absolute           = positions in the `world` frame
  ready: [j1..j6]                       # optional: Pilz-PTP to this joint pose first (good arm
                                        #   config / IK seed). Its TCP seeds 'relative' when no anchor.
  anchor: {xyz:[x,y,z], rpy:[r,p,y]}    # optional ABSOLUTE world pose. We PTP the TCP to it (the
                                        #   one absolute positioning move) and every 'relative' point
                                        #   is an offset from it. EDIT `anchor` ALONE to relocate.
  defaults: {vel: 0.1, acc: 0.1, z: 0.0, samples: 120}   # per-segment fallbacks
  eef_step: 0.002                       # OPTIONAL top-level: MoveIt IK/interp step (m) for
                                        #   compute_cartesian_path. Bigger = fewer IK solves =
                                        #   faster planning (coarser Cartesian interp on straight
                                        #   travel; curve fidelity is set by `samples`). --eef-step wins.
  scale: 1.0                            # OPTIONAL top-level: multiply every point's in-plane
                                        #   (x, y) by this, leaving z (pen depth/retract) alone.
                                        #   >1 enlarges, <1 shrinks; scales about the anchor
                                        #   (relative coords) / world origin (absolute). --scale wins.
  rotate: 0                             # OPTIONAL top-level: rotate the drawing in-plane by this
                                        #   many degrees CCW about the anchor/origin (z & tool rpy
                                        #   unchanged). Applied after `scale`. --rotate wins.
  plane_rotate: 0                       # OPTIONAL top-level: the DRAWING PLANE's own rotation, deg
                                        #   CCW about the anchor. Written by the draw-plane bridge
                                        #   from the operator's calibration; says the
                                        #   board itself sits rotated. ADDS to `rotate`, which stays
                                        #   the artwork's own rotation: 30 + 45 = 75 deg total. Kept
                                        #   a separate key so reloading a saved drawing under a new
                                        #   plane rotation replaces this and leaves `rotate` alone.
                                        #   No CLI override -- it is calibration, not a knob.
  segments:                             # ordered; each expands to >=1 dense waypoint:
    - {type: lin, xyz:[dx,dy,dz]}                          # a straight move / pen up-down / travel
    - {type: func, t:[t0,t1], x:"...", y:"...", z:"..."}   # parametric curve; x/y (z optional)
                                        #   are expressions in t (math fns: sin cos sqrt pi ...)
    - {type: bspline, points:[[x,y],...], degree:3, closed:false}   # CONTROL points; the TCP is
                                        #   pulled toward them but only touches the first & last
                                        #   (clamped). Approximating -- how fonts/SVG store curves.
    - {type: through, points:[[x,y],...], closed:false}    # INTERPOLATION points; the TCP passes
                                        #   THROUGH every one (centripetal Catmull-Rom).
  # Curve points are 2-D [x,y] in the drawing plane; a bspline/through segment draws at ONE
  #   `z` (the segment's `z`, else defaults.z) -- per-point z is NOT honored for curves, so to
  #   vary depth within a curve split it into constant-z segments. `lin.xyz`/`func` z are per-point.
  # `samples` (per curve segment) sets how many points we evaluate along it (denser = truer curve,
  #   since MoveIt connects them with straight lines). Orientation: `rpy` + `rpy_frame`
  #   (world|start) like the Pilz recipe; omit to HOLD the anchor's (tool-down) orientation for the
  #   whole segment (constant orientation along the curve).

Pass the recipe as the first arg; a bare name (no '/') is looked up in this package's
config/ dir. --dry-run evaluates + transforms the whole recipe and prints the generated
waypoints WITHOUT ROS (offline geometry check -- exercises the exact same code path).
--plan-only calls /compute_cartesian_path and reports the solved fraction WITHOUT moving
the robot (a reachability pre-check; the anchor is planned as the first Cartesian leg).

Prereqs (ROS mode; separate exec sessions):
  1. ros2 launch rapidcode_bringup elfin5.launch.py
  2. ros2 launch rapidcode_moveit_config move_group.launch.py

Usage:
  ros2 run rapidcode_bringup send_cartesian_path.py recipe_japanese_woman_simple.yaml
  ros2 run rapidcode_bringup send_cartesian_path.py recipe_japanese_woman_simple.yaml --dry-run
  ros2 run rapidcode_bringup send_cartesian_path.py /abs/path/mycurves.yaml --eef-step 0.001
  ros2 run rapidcode_bringup send_cartesian_path.py recipe_rsi_logo_svg.yaml --scale 1.5 --dry-run
  ros2 run rapidcode_bringup send_cartesian_path.py recipe_rsi_logo_svg.yaml --rotate 90 --dry-run
"""

import math
import os
import sys

import numpy as np
import yaml

# ROS imports are deferred so --dry-run (pure geometry) runs without a ROS env.
try:
    from ament_index_python.packages import get_package_share_directory
    _HAVE_AMENT = True
except ImportError:
    _HAVE_AMENT = False


GROUP = "elfin_arm"
JOINTS = [
    "elfin_joint1", "elfin_joint2", "elfin_joint3",
    "elfin_joint4", "elfin_joint5", "elfin_joint6",
]
PLANNING_FRAME = "world"      # URDF root == MoveIt model frame
TIP_LINK = "elfin_end_link"   # elfin_arm solver tip / TCP
MOVE_ACTION = "/move_action"
CART_SERVICE = "/compute_cartesian_path"
EXEC_ACTION = "/execute_trajectory"
FK_SERVICE = "/compute_fk"
IK_SERVICE = "/compute_ik"
PIPELINE = "pilz_industrial_motion_planner"

DEFAULTS = {"vel": 0.1, "acc": 0.1, "z": 0.0, "samples": 120}
EEF_STEP = 0.002        # m -- MoveIt's IK/interp granularity between our waypoints
JUMP_THRESHOLD = 0.0    # 0 disables IK-jump detection (fine for phantom; tighten near singularities)
MIN_FRACTION = 0.99     # abort if compute_cartesian_path solves less than this (unless --force)


# ---- orientation helpers (plain (w,x,y,z) tuples; no ROS types) -------------
def quat_from_rpy(roll, pitch, yaw):
    """(w,x,y,z) from roll(X)/pitch(Y)/yaw(Z) in radians."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def quat_mul(a, b):
    """Quaternion product a ∘ b (apply b, then a); (w,x,y,z) tuples."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def orientation_from(seg, start_quat):
    """Segment orientation as (w,x,y,z): `rpy`+`rpy_frame`, else HOLD start_quat."""
    rpy = seg.get("rpy")
    if rpy is None:
        return start_quat
    if len(rpy) != 3:
        raise ValueError("rpy must be [roll, pitch, yaw]")
    q = quat_from_rpy(*(float(v) for v in rpy))
    frame = seg.get("rpy_frame", "start")
    if frame == "world":
        return q
    if frame == "start":
        return quat_mul(start_quat, q)
    raise ValueError(f"rpy_frame must be 'world' or 'start', got '{frame}'")


# ---- curve generators (numpy only; no scipy in the container) ---------------
def _clamped_knots(num_ctrl, degree):
    """Clamped (open-uniform) knot vector, length num_ctrl + degree + 1.
    The curve is clamped so it starts at the first and ends at the last control point."""
    p = degree
    interior = num_ctrl - p - 1                 # count of interior knots (>= 0)
    knots = [0.0] * (p + 1)
    for i in range(1, interior + 1):
        knots.append(i / (interior + 1))
    knots += [1.0] * (p + 1)
    return knots


def _find_span(u, knots, num_ctrl, p):
    """Knot span k with knots[k] <= u < knots[k+1] (clamped at both ends)."""
    n = num_ctrl - 1
    if u >= knots[n + 1]:
        return n
    if u <= knots[p]:
        return p
    lo, hi = p, n + 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if u < knots[mid]:
            hi = mid
        else:
            lo = mid
    return lo


def _deboor(u, knots, ctrl, p):
    """Evaluate a B-spline at parameter u via De Boor's algorithm."""
    k = _find_span(u, knots, len(ctrl), p)
    d = [ctrl[k - p + j].astype(float).copy() for j in range(p + 1)]
    for r in range(1, p + 1):
        for j in range(p, r - 1, -1):
            i = k - p + j
            denom = knots[i + p - r + 1] - knots[i]
            a = 0.0 if denom == 0.0 else (u - knots[i]) / denom
            d[j] = (1.0 - a) * d[j - 1] + a * d[j]
    return d[p]


def bspline_curve(points, degree, samples, closed):
    """Dense samples of a B-spline through its CONTROL points (approximating).

    Open: clamped -> hits first & last control point only. Closed: periodic (wraps
    `degree` control points, uniform knots) -> a smooth loop that hits none of them.
    """
    ctrl = [np.asarray(p, dtype=float) for p in points]
    if len(ctrl) < 2:
        raise ValueError("bspline needs at least 2 control points")
    p = min(int(degree), len(ctrl) - 1)         # degree can't exceed #ctrl-1
    if p < 1:
        raise ValueError("bspline degree must be >= 1")
    if closed:
        cp = ctrl + ctrl[:p]                     # wrap p control points
        m = len(cp)
        knots = [float(i) for i in range(m + p + 1)]   # uniform integer knots
        u0, u1 = knots[p], knots[m]              # valid periodic domain
        us = np.linspace(u0, u1, samples)
        return [_deboor(u, knots, cp, p) for u in us]
    knots = _clamped_knots(len(ctrl), p)
    us = np.linspace(0.0, 1.0, samples)
    return [_deboor(u, knots, ctrl, p) for u in us]


def _cr_span(p0, p1, p2, p3, n, alpha):
    """Centripetal Catmull-Rom samples on the [p1, p2) span (Barry-Goldman form)."""
    def next_t(t, a, b):
        d = float(np.linalg.norm(b - a)) ** alpha
        return t + (d if d > 1e-12 else 1e-12)   # guard coincident points
    t0 = 0.0
    t1 = next_t(t0, p0, p1)
    t2 = next_t(t1, p1, p2)
    t3 = next_t(t2, p2, p3)
    out = []
    for s in range(n):
        t = t1 + (t2 - t1) * (s / n)
        a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1
        a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
        a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
        b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
        b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3
        out.append((t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2)
    return out


def catmull_rom(points, samples, closed, alpha=0.5):
    """Dense samples of a centripetal Catmull-Rom spline through EVERY input point."""
    pts = [np.asarray(p, dtype=float) for p in points]
    if len(pts) < 2:
        raise ValueError("through-points needs at least 2 points")
    if closed:
        ext = [pts[-1]] + pts + [pts[0], pts[1]]
        spans = len(pts)                          # wrap back to the start
    else:
        ext = [pts[0]] + pts + [pts[-1]]          # phantom endpoints
        spans = len(pts) - 1
    per = max(2, int(round(samples / spans)))
    curve = []
    for i in range(spans):
        curve.extend(_cr_span(ext[i], ext[i + 1], ext[i + 2], ext[i + 3], per, alpha))
    curve.append(ext[-2] if not closed else pts[0])   # include the final endpoint
    return curve


_FUNC_NS = {name: getattr(math, name) for name in (
    "sin cos tan asin acos atan atan2 sinh cosh tanh sqrt exp log log10 "
    "pow fabs floor ceil hypot degrees radians pi e tau").split()}
_FUNC_NS["abs"] = abs


def sample_func(seg, samples):
    """Sample a parametric func segment: x(t), y(t), optional z(t) over t in [t0,t1]."""
    trange = seg.get("t")
    if not trange or len(trange) != 2:
        raise ValueError("func needs 't': [t0, t1]")
    if "x" not in seg or "y" not in seg:
        raise ValueError("func needs 'x' and 'y' expressions (strings in t)")
    t0, t1 = float(trange[0]), float(trange[1])
    codes = {k: compile(str(seg[k]), f"<func {k}>", "eval")
             for k in ("x", "y", "z") if k in seg}
    out = []
    for t in np.linspace(t0, t1, samples):
        ns = dict(_FUNC_NS, t=float(t))
        vals = {k: float(eval(c, {"__builtins__": {}}, ns)) for k, c in codes.items()}
        out.append(np.array([vals["x"], vals["y"], vals.get("z", None)], dtype=object))
    return out


# ---- segment -> local drawing-plane points ----------------------------------
def _resolve_z(pt, seg_z):
    """Point may be 2-D (fill z from the segment) or 3-D (use as-is)."""
    if len(pt) == 3 and pt[2] is not None:
        return float(pt[0]), float(pt[1]), float(pt[2])
    return float(pt[0]), float(pt[1]), float(seg_z)


def segment_local_points(seg, idx, defaults):
    """Expand one segment to a list of local (x, y, z) tuples in the drawing plane."""
    stype = str(seg["type"]).lower()
    seg_z = float(seg.get("z", defaults["z"]))
    samples = int(seg.get("samples", defaults["samples"]))

    if stype == "lin":
        xyz = seg.get("xyz")
        if not xyz or len(xyz) != 3:
            raise ValueError(f"segment {idx}: lin needs 'xyz' [dx, dy, dz]")
        return [(float(xyz[0]), float(xyz[1]), float(xyz[2]))]

    if stype == "func":
        return [_resolve_z(p, seg_z) for p in sample_func(seg, samples)]

    if stype in ("bspline", "through"):
        raw = seg.get("points")
        if not raw or len(raw) < 2:
            raise ValueError(f"segment {idx}: {stype} needs 'points' (>= 2)")
        pts2 = [(float(p[0]), float(p[1])) for p in raw]     # sample in the XY plane
        closed = bool(seg.get("closed", False))
        if stype == "bspline":
            curve = bspline_curve(pts2, seg.get("degree", 3), samples, closed)
        else:
            curve = catmull_rom(pts2, samples, closed)
        return [(float(c[0]), float(c[1]), seg_z) for c in curve]

    raise ValueError(f"segment {idx}: unknown type '{stype}' "
                     "(use lin | func | bspline | through)")


# ---- recipe -> world waypoints (pos3, quat4); pure geometry -----------------
def world_point(local_xyz, start_pos, coords):
    x, y, z = local_xyz
    if coords == "absolute":
        return (x, y, z)
    return (start_pos[0] + x, start_pos[1] + y, start_pos[2] + z)


def build_waypoints(segments, start_pos, start_quat, coords, defaults,
                    scale=1.0, rotate_deg=0.0, plane_rotate_deg=0.0):
    """Flatten all segments into a list of ((x,y,z), (w,x,y,z)) world waypoints.

    `scale` multiplies the in-plane (x, y) of every point; the rotation then turns
    (x, y) CCW by `rotate_deg` + `plane_rotate_deg` degrees. Both leave z (pen depth
    / retract height -- physical) untouched, and act in the drawing plane about the
    anchor ('relative' coords) / world origin ('absolute'). NOTE: rotating turns the
    drawing IN THE PLANE only; the tool orientation (rpy) is unchanged, which is what
    you want for a flat pen on a flat board.

    The two angles ADD because both turn about the same point, so composing them is
    exact. They stay separate arguments because they mean different things and are
    replaced independently: `rotate_deg` is the artwork's own rotation, authored in
    the recipe; `plane_rotate_deg` is where the board sits, written by
    the draw-plane bridge and replaced wholesale every time it re-anchors a recipe.
    Summing them into one stored field would make a reload double-count.
    """
    th = math.radians(rotate_deg + plane_rotate_deg)
    ct, st = math.cos(th), math.sin(th)
    waypoints = []
    for idx, seg in enumerate(segments):
        if not isinstance(seg, dict) or "type" not in seg:
            raise ValueError(f"segment {idx}: must be a mapping with a 'type'")
        quat = orientation_from(seg, start_quat)
        for local in segment_local_points(seg, idx, defaults):
            lx, ly, lz = local
            lx, ly = lx * scale, ly * scale
            rx, ry = lx * ct - ly * st, lx * st + ly * ct   # CCW rotation in-plane
            waypoints.append((world_point((rx, ry, lz), start_pos, coords), quat))
    return waypoints


def anchor_pose(anchor):
    """(pos3, quat4) from `anchor: {xyz:[..], rpy:[..]}` (rpy optional -> identity)."""
    if not isinstance(anchor, dict) or "xyz" not in anchor or len(anchor["xyz"]) != 3:
        raise ValueError("anchor needs 'xyz' [x, y, z]")
    xyz = tuple(float(v) for v in anchor["xyz"])
    rpy = anchor.get("rpy")
    if rpy is not None and len(rpy) != 3:
        raise ValueError("anchor 'rpy' must be [roll, pitch, yaw]")
    quat = quat_from_rpy(*(float(v) for v in rpy)) if rpy is not None else (1.0, 0.0, 0.0, 0.0)
    return xyz, quat


# ---- recipe loading + arg parsing -------------------------------------------
def resolve_recipe_path(arg):
    if os.path.isfile(arg):
        return arg
    if _HAVE_AMENT:
        try:
            cand = os.path.join(get_package_share_directory("rapidcode_bringup"), "config", arg)
            if os.path.isfile(cand):
                return cand
        except Exception:
            pass
    return None


def parse_args(argv):
    out = {"recipe": None, "dry_run": False, "plan_only": False, "force": False,
           "no_collision": False, "eef_step": None, "jump": JUMP_THRESHOLD,
           "min_fraction": MIN_FRACTION, "scale": None, "rotate": None,
           # 0.0, not None: `plane_rotate` has no CLI flag, so there is no
           # "not overridden" state to represent. A sentinel here would reach
           # build_waypoints unresolved from any caller that builds this dict
           # itself rather than going through the CLI path -- which is how the
           # in-process batch route broke on 2026-09-11.
           "plane_rotate": 0.0, "overrides": {}}
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--dry-run":
            out["dry_run"] = True; i += 1
        elif a == "--plan-only":
            out["plan_only"] = True; i += 1
        elif a == "--force":
            out["force"] = True; i += 1
        elif a == "--no-collision":
            out["no_collision"] = True; i += 1
        elif a in ("--vel", "--acc") and i + 1 < len(argv):
            out["overrides"][a[2:]] = float(argv[i + 1]); i += 2
        elif a == "--eef-step" and i + 1 < len(argv):
            out["eef_step"] = float(argv[i + 1]); i += 2
        elif a == "--jump" and i + 1 < len(argv):
            out["jump"] = float(argv[i + 1]); i += 2
        elif a == "--min-fraction" and i + 1 < len(argv):
            out["min_fraction"] = float(argv[i + 1]); i += 2
        elif a == "--scale" and i + 1 < len(argv):
            out["scale"] = float(argv[i + 1]); i += 2
        elif a == "--rotate" and i + 1 < len(argv):
            out["rotate"] = float(argv[i + 1]); i += 2
        elif not a.startswith("-") and out["recipe"] is None:
            out["recipe"] = a; i += 1
        else:
            i += 1
    return out


def load_recipe(path):
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    coords = str(doc.get("coordinates", "relative")).lower()
    if coords not in ("relative", "absolute"):
        raise ValueError(f"coordinates must be 'relative' or 'absolute', got '{coords}'")
    defaults = dict(DEFAULTS, **(doc.get("defaults") or {}))
    segments = doc.get("segments") or []
    if not segments:
        raise ValueError("recipe has no 'segments'")
    return doc, coords, defaults, segments


def summarize(waypoints):
    pts = np.array([w[0] for w in waypoints], dtype=float)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    print(f"  waypoints: {len(waypoints)}")
    print(f"  bbox  x[{lo[0]:+.4f},{hi[0]:+.4f}]  "
          f"y[{lo[1]:+.4f},{hi[1]:+.4f}]  z[{lo[2]:+.4f},{hi[2]:+.4f}] (world, m)")
    print(f"  first {tuple(round(v, 4) for v in pts[0])}   last {tuple(round(v, 4) for v in pts[-1])}")


# ---- ROS mode ---------------------------------------------------------------
def run_ros(args, doc, coords, defaults, segments):
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from geometry_msgs.msg import Pose, Point, Quaternion
    from sensor_msgs.msg import JointState
    from builtin_interfaces.msg import Duration
    from moveit_msgs.action import MoveGroup, ExecuteTrajectory
    from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK
    from moveit_msgs.msg import (
        Constraints, JointConstraint, MotionPlanRequest, PlanningOptions, RobotState)

    def to_pose(pos, quat):
        w, x, y, z = quat
        return Pose(position=Point(x=pos[0], y=pos[1], z=pos[2]),
                    orientation=Quaternion(w=w, x=x, y=y, z=z))

    def base_request(planner_id):
        req = MotionPlanRequest()
        req.pipeline_id = PIPELINE
        req.planner_id = planner_id
        req.group_name = GROUP
        req.max_velocity_scaling_factor = defaults["vel"]
        req.max_acceleration_scaling_factor = defaults["acc"]
        req.allowed_planning_time = 5.0
        return req

    def joint_goal(joints):
        c = Constraints()
        for name, position in zip(JOINTS, joints):
            c.joint_constraints.append(JointConstraint(
                joint_name=name, position=position,
                tolerance_above=1e-4, tolerance_below=1e-4, weight=1.0))
        return c

    def ptp(node, client, constraints, label):
        goal = MoveGroup.Goal()
        goal.request = base_request("PTP")
        goal.request.goal_constraints.append(constraints)
        goal.planning_options = PlanningOptions(plan_only=False)
        node.get_logger().info(f"PTP -> {label}")
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            node.get_logger().error(f"PTP to {label} REJECTED"); return False
        rfut = handle.get_result_async()
        rclpy.spin_until_future_complete(node, rfut, timeout_sec=60.0)
        res = rfut.result()
        ok = res is not None and res.result.error_code.val == 1
        if not ok:
            node.get_logger().error(f"PTP to {label} FAILED "
                                    f"(error_code={None if res is None else res.result.error_code.val})")
        return ok

    def current_joint_state(node):
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

    def fk_tcp(node, joints):
        cli = node.create_client(GetPositionFK, FK_SERVICE)
        if not cli.wait_for_service(timeout_sec=10.0):
            node.get_logger().error(f"{FK_SERVICE} unavailable"); return None
        req = GetPositionFK.Request()
        req.header.frame_id = PLANNING_FRAME
        req.fk_link_names = [TIP_LINK]
        req.robot_state.joint_state = JointState(name=list(JOINTS), position=list(joints))
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        resp = fut.result()
        if resp is None or resp.error_code.val != 1 or not resp.pose_stamped:
            node.get_logger().error("/compute_fk failed"); return None
        p = resp.pose_stamped[0].pose
        return ((p.position.x, p.position.y, p.position.z),
                (p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z))

    def ik_solve(node, pos, quat, seed):
        """Joint solution for a world pose (seeded by `seed` joints), or None."""
        cli = node.create_client(GetPositionIK, IK_SERVICE)
        if not cli.wait_for_service(timeout_sec=10.0):
            node.get_logger().error(f"{IK_SERVICE} unavailable"); return None
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = GROUP
        r.ik_link_name = TIP_LINK
        r.pose_stamped.header.frame_id = PLANNING_FRAME
        r.pose_stamped.pose = to_pose(pos, quat)
        if seed is not None:
            r.robot_state.joint_state = JointState(name=list(JOINTS), position=list(seed))
        r.avoid_collisions = True
        r.timeout = Duration(sec=2)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        resp = fut.result()
        if resp is None or resp.error_code.val != 1:
            return None
        got = {n: p for n, p in zip(resp.solution.joint_state.name,
                                    resp.solution.joint_state.position)}
        if not all(j in got for j in JOINTS):
            return None
        return [got[j] for j in JOINTS]

    plan_only = args["plan_only"]
    rclpy.init()
    node = Node("send_cartesian_path")
    cart_cli = node.create_client(GetCartesianPath, CART_SERVICE)
    if not cart_cli.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(f"{CART_SERVICE} unavailable -- is move_group up?"); return 1
    move_cli = exec_cli = None
    if not plan_only:
        move_cli = ActionClient(node, MoveGroup, MOVE_ACTION)
        exec_cli = ActionClient(node, ExecuteTrajectory, EXEC_ACTION)
        if not move_cli.wait_for_server(timeout_sec=10.0):
            node.get_logger().error(f"{MOVE_ACTION} unavailable -- is move_group up?"); return 1
        if not exec_cli.wait_for_server(timeout_sec=10.0):
            node.get_logger().error(f"{EXEC_ACTION} unavailable -- is move_group up?"); return 1

    ready = doc.get("ready")
    anchor = doc.get("anchor")
    if anchor is not None and coords != "relative":
        node.get_logger().warn("'anchor' only affects 'relative' coords; ignoring")
        anchor = None
    if ready is not None and len(ready) != len(JOINTS):
        node.get_logger().error(f"'ready' needs {len(JOINTS)} joint values"); return 1

    # Resolve the start pose that 'relative' offsets add to (anchor / ready TCP / current TCP).
    if anchor is not None:
        try:
            start_pos, start_quat = anchor_pose(anchor)
        except ValueError as exc:
            node.get_logger().error(f"recipe error: {exc}"); return 2
    else:
        seed = fk_tcp(node, [float(v) for v in ready]) if ready is not None else None
        if seed is None:
            js = current_joint_state(node)
            seed = fk_tcp(node, js) if js is not None else None
        if seed is None:
            return 1
        start_pos, start_quat = seed

    # 1) gross approach (SKIPPED in --plan-only): joint PTP to `ready`, then solve the anchor
    #    pose IK via the robust /compute_ik service (random restarts) and joint-PTP to THAT
    #    config. A joint goal is always plannable, sidestepping Pilz-PTP's single-seed IK
    #    failure when `ready` is a far-away seed for the anchor pose.
    if not plan_only:
        seed = [float(v) for v in ready] if ready is not None else None
        if ready is not None and not ptp(node, move_cli, joint_goal(seed), "ready"):
            return 1
        if anchor is not None:
            if seed is None:
                seed = current_joint_state(node)
            anchor_js = ik_solve(node, start_pos, start_quat, seed)
            if anchor_js is None:
                node.get_logger().error("IK for the anchor pose failed; adjust `anchor`/`ready`.")
                return 1
            if not ptp(node, move_cli, joint_goal(anchor_js), "anchor"):
                return 1

    # 2) build the dense Cartesian waypoints (the "one code path").
    try:
        waypoints = build_waypoints(segments, start_pos, start_quat, coords,
                                    defaults, args["scale"], args["rotate"],
                                    args["plane_rotate"])
    except ValueError as exc:
        node.get_logger().error(f"recipe error: {exc}"); return 2
    if args["scale"] != 1.0:
        node.get_logger().info(f"scale: {args['scale']}x (in-plane; z/pen-depth unchanged)")
    if args["rotate"] != 0.0:
        node.get_logger().info(f"rotate: {args['rotate']} deg CCW (in-plane; z/rpy unchanged)")
    if args["plane_rotate"] != 0.0:
        node.get_logger().info(
            f"plane_rotate: {args['plane_rotate']} deg CCW (drawing plane calibration)")

    # Start state for compute_cartesian_path. --plan-only doesn't move, so seed from the
    # anchor's IK solution (the config the real flow reaches by PTP) -- validates the DRAW
    # exactly as executed, not the singular straight-line approach.
    if plan_only:
        seed = [float(v) for v in ready] if ready is not None else current_joint_state(node)
        if anchor is not None:
            js = ik_solve(node, start_pos, start_quat, seed)
            if js is None:
                node.get_logger().error(
                    "IK for the anchor pose failed, so the draw can't be pre-checked in "
                    "isolation. Run without --plan-only (the anchor is reached by PTP).")
                return 1
        else:
            js = seed
        if js is None:
            return 1
    else:
        js = current_joint_state(node)   # CURRENT (post-approach) state
        if js is None:
            return 1
    node.get_logger().info(f"{len(segments)} segments -> {len(waypoints)} Cartesian waypoints")

    # 3) compute_cartesian_path.
    req = GetCartesianPath.Request()
    req.header.frame_id = PLANNING_FRAME
    req.start_state = RobotState()
    req.start_state.joint_state = JointState(name=list(JOINTS), position=list(js))
    req.group_name = GROUP
    req.link_name = TIP_LINK
    req.waypoints = [to_pose(pos, quat) for pos, quat in waypoints]
    req.max_step = args["eef_step"]
    req.jump_threshold = args["jump"]
    req.avoid_collisions = not args["no_collision"]
    req.max_velocity_scaling_factor = defaults["vel"]
    req.max_acceleration_scaling_factor = defaults["acc"]
    fut = cart_cli.call_async(req)
    # Dense/large drawings do thousands of serial KDL IK solves here; 60 s was too
    # tight for the heaviest recipes (they solve 100% but slower). 300 s headroom.
    rclpy.spin_until_future_complete(node, fut, timeout_sec=300.0)
    resp = fut.result()
    if resp is None or resp.error_code.val != 1:
        node.get_logger().error(
            f"/compute_cartesian_path failed "
            f"(error_code={None if resp is None else resp.error_code.val})"); return 1
    n_traj = len(resp.solution.joint_trajectory.points)
    node.get_logger().info(f"cartesian path: fraction={resp.fraction:.3f}, "
                           f"{n_traj} trajectory points")
    if resp.fraction < args["min_fraction"] and not args["force"]:
        node.get_logger().error(
            f"only {resp.fraction:.1%} of the path was solvable (< {args['min_fraction']:.0%}); "
            "IK likely failed near a singularity. Adjust `ready`/`anchor` or pass --force.")
        return 1
    if n_traj == 0:
        node.get_logger().error("empty trajectory; nothing to execute"); return 1

    if plan_only:
        ok = resp.fraction >= args["min_fraction"]
        node.get_logger().info(f"plan-only: {'REACHABLE' if ok else 'NOT fully reachable'} "
                               f"(fraction={resp.fraction:.3f}); not executing.")
        rclpy.shutdown()
        return 0 if (ok or args["force"]) else 1

    # 4) execute the time-parameterized joint trajectory via MoveIt -> passthrough ctrl.
    goal = ExecuteTrajectory.Goal(trajectory=resp.solution)
    node.get_logger().info("executing...")
    efut = exec_cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, efut)
    handle = efut.result()
    if handle is None or not handle.accepted:
        node.get_logger().error("execute goal REJECTED"); return 1
    rfut = handle.get_result_async()
    # Scale the execution wait to the planned trajectory duration (big drawings take
    # minutes) so heavy recipes get enough time WITHOUT making small ones wait long to
    # report a truly-hung execution. 1.5x + 30 s margin, floored at the old 180 s.
    last_pt = resp.solution.joint_trajectory.points[-1].time_from_start
    traj_dur = last_pt.sec + last_pt.nanosec * 1e-9
    exec_timeout = max(180.0, traj_dur * 1.5 + 30.0)
    node.get_logger().info(f"executing (trajectory {traj_dur:.0f} s; "
                           f"result timeout {exec_timeout:.0f} s)...")
    rclpy.spin_until_future_complete(node, rfut, timeout_sec=exec_timeout)
    res = rfut.result()
    if res is None:
        node.get_logger().error(f"no execution result within {exec_timeout:.0f} s"); return 1
    ok = res.result.error_code.val == 1
    node.get_logger().info(f"done: error_code={res.result.error_code.val} "
                           f"({'SUCCESS' if ok else 'FAILURE'})")
    rclpy.shutdown()
    return 0 if ok else 1


def main():
    args = parse_args(sys.argv[1:])
    if not args["recipe"]:
        print(__doc__)
        print("error: a recipe file (path or bundled name) is required", file=sys.stderr)
        return 2
    path = resolve_recipe_path(args["recipe"])
    if path is None:
        print(f"error: recipe '{args['recipe']}' not found", file=sys.stderr)
        return 2
    try:
        doc, coords, defaults, segments = load_recipe(path)
        defaults.update(args["overrides"])
        # eef_step precedence: CLI --eef-step > recipe `eef_step` > built-in default.
        if args["eef_step"] is None:
            args["eef_step"] = float(doc.get("eef_step", EEF_STEP))
        # scale precedence: CLI --scale > recipe `scale` > 1.0 (no scaling).
        if args["scale"] is None:
            args["scale"] = float(doc.get("scale", 1.0))
        # rotate precedence: CLI --rotate > recipe `rotate` > 0.0 (deg, CCW in-plane).
        if args["rotate"] is None:
            args["rotate"] = float(doc.get("rotate", 0.0))
        # plane_rotate has no CLI override on purpose: it is the bridge's plane
        # calibration, and --rotate must not be able to silently
        # discard it on a standalone replay. Unconditional for the
        # same reason -- with no flag to preserve, the file is the only source.
        args["plane_rotate"] = float(doc.get("plane_rotate", 0.0))
    except (ValueError, yaml.YAMLError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    if args["dry_run"]:
        # Offline geometry check: exercise the exact evaluate+transform pipeline with
        # the anchor as the start (or origin if none), no ROS.
        anchor = doc.get("anchor")
        if anchor is not None and coords == "relative":
            start_pos, start_quat = anchor_pose(anchor)
        else:
            start_pos, start_quat = (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)
        try:
            waypoints = build_waypoints(segments, start_pos, start_quat, coords,
                                        defaults, args["scale"], args["rotate"],
                                        args["plane_rotate"])
        except ValueError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 2
        if args["scale"] != 1.0:
            print(f"  scale: {args['scale']}x (in-plane; z unchanged)")
        if args["rotate"] != 0.0:
            print(f"  rotate: {args['rotate']} deg CCW (in-plane; z/rpy unchanged)")
        if args["plane_rotate"] != 0.0:
            print(f"  plane_rotate: {args['plane_rotate']} deg CCW (plane calibration)")
        print(f"[dry-run] recipe '{os.path.basename(path)}' ({coords} coords, "
              f"{len(segments)} segments)")
        print(f"  start (anchor/origin): {tuple(round(v, 4) for v in start_pos)}")
        summarize(waypoints)
        return 0

    return run_ros(args, doc, coords, defaults, segments)


if __name__ == "__main__":
    sys.exit(main())
