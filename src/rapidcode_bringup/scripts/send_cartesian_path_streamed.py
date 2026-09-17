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

"""Streamed variant of send_cartesian_path.py: start drawing after planning only the
FIRST few seconds of the recipe, and keep planning while the robot executes.

The monolithic sender solves IK for EVERY dense waypoint before the robot moves
(up to ~60 s for heavy recipes). This sender splits the same dense waypoint list
into GOALS (~--goal-seconds of motion each, cut at recipe segment boundaries when
possible), plans each goal with /compute_cartesian_path, and sends the planned
goals DIRECTLY to the passthrough controller's FollowJointTrajectory action server
(bypassing MoveIt's serializing /execute_trajectory). Planning goal k+1 overlaps
execution of goals <= k.

Nomenclature (see context/ros2-streaming-recipe-execution-proposal.md):
  segment = one recipe YAML segment; goal = one FollowJointTrajectory action goal
  (a slice of the drawing); chunk = controller->hardware transport unit (not ours).

Seam continuity (Phase 2): MoveIt's per-goal timing is DISCARDED. A sender-side
retimer (accel-limited forward/backward pass with grbl-style junction speed caps)
re-times each goal over a window that spans the seam into the NEXT goal, so the
shared seam waypoint carries one consistent nonzero velocity -- the pen does not
dwell at seams. Goal k is therefore only emitted after goal k+1 is planned (one-goal
emission lag). Interior goals are sent with the controller's `finalize_last_chunk`
parameter set false (firmware move stays open across seams); it is set true for the
chain's final goal. --rest-seams disables all of this (Phase 1 behavior: per-goal
rest-to-rest MoveIt timing, every goal finalizes; a dwell at every seam).

Failure story: if a goal fails to plan (fraction < 1, IK failure, timeout) the
previous goal is re-timed to END AT REST and sent as final -- the pen lands at a
known point on the artwork, never a smear. If this sender DIES mid-chain with the
move open, the firmware out-of-frames watchdog e-stops (~32 ms) -- the documented
correct failure mode for an unattended producer.

Usage (same recipe files and shared flags as send_cartesian_path.py):
  ros2 run rapidcode_bringup send_cartesian_path_streamed.py recipe_man_throwing_discus_simple.yaml
  ... --goal-seconds 3.0 --max-inflight 2         # pipeline tuning
  ... --rest-seams                                # Phase 1 mode (dwell at seams)
  ... --dry-run                                   # offline: goal split table, no ROS
  ... --plan-only                                 # plan every goal, report, no motion
  ... --self-test                                 # unit-test the pure helpers, no ROS

Prereqs (ROS mode): elfin5.launch.py + move_group.launch.py (see PILZ_RUNBOOK.md).
"""

import math
import os
import sys
import time

import numpy as np

# Co-located import: the monolithic sender is installed/run from the same directory
# and its ROS imports are deferred, so importing it is side-effect-free. It supplies
# the recipe format, curve evaluation, and waypoint transforms (one code path).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import send_cartesian_path as monolithic

CONTROLLER_NODE = "rapidcode_passthrough_trajectory_controller"
ACTION_NAME = f"/{CONTROLLER_NODE}/follow_joint_trajectory"
SET_PARAM_SERVICE = f"/{CONTROLLER_NODE}/set_parameters"
FINALIZE_PARAM = "finalize_last_chunk"
JOINTS = monolithic.JOINTS

# Joint limits from rapidcode_moveit_config/config/joint_limits.yaml (all six joints
# identical); scaled by the recipe's vel/acc scaling factors exactly as MoveIt scales
# its own time parameterization.
JOINT_VELOCITY_LIMIT = 1.57   # rad/s
JOINT_ACCEL_LIMIT = 3.0       # rad/s^2

DEFAULT_GOAL_SECONDS = 3.0    # target motion per goal
DEFAULT_MAX_INFLIGHT = 2      # goals submitted but unfinished (controller cap is 32)
SEAM_TOLERANCE = 0.01         # rad; sender-side seam gate (controller net is 0.05)
JUNCTION_ANGLE_FREE = math.radians(15.0)  # joins below this are smooth curvature: no cap
CORNER_DELTA_V = 0.05         # rad/s; per-joint velocity step allowed through a corner
MIN_POINT_DT = 0.002          # s; floor for degenerate (zero-length) intervals
MIN_PLAN_FRACTION = 0.999     # per-goal /compute_cartesian_path fraction required


# ============================ pure helpers (no ROS) ===========================
def cartesian_step_lengths(waypoints):
    """Per-interval XYZ distance (m) between consecutive dense waypoints
    [((x,y,z), quat), ...]; used only for goal sizing, never for real timing."""
    points = np.array([wp[0] for wp in waypoints], dtype=float)
    return np.linalg.norm(np.diff(points, axis=0), axis=1)


def estimate_waypoint_times(waypoints, cartesian_speed):
    """Rough cumulative time (s) at each waypoint assuming constant Cartesian speed.
    Output length == len(waypoints); starts at 0."""
    steps = cartesian_step_lengths(waypoints)
    return np.concatenate(([0.0], np.cumsum(steps / max(cartesian_speed, 1e-9))))


def segment_start_indices(segments, defaults):
    """Waypoint index where each recipe segment starts (same expansion
    build_waypoints performs). Returns a list of len(segments) indices."""
    starts, cursor = [], 0
    for index, segment in enumerate(segments):
        starts.append(cursor)
        cursor += len(monolithic.segment_local_points(segment, index, defaults))
    return starts


def split_cut_indices(est_times, segment_starts, goal_seconds):
    """Waypoint indices where the dense list is cut into goals.

    Cuts land on the estimated-time grid, each snapped to the nearest recipe
    segment start within half a goal (segment joins are corner points -- the
    natural seams). Returns a strictly increasing list of interior indices
    (possibly empty for a short recipe)."""
    total_time = float(est_times[-1])
    if total_time <= goal_seconds:
        return []
    goal_count = int(math.ceil(total_time / goal_seconds))
    boundary_times = est_times[segment_starts] if len(segment_starts) else np.array([])
    cuts = []
    for goal_index in range(1, goal_count):
        target_time = total_time * goal_index / goal_count
        cut = int(np.searchsorted(est_times, target_time))
        if len(boundary_times):
            nearest = int(np.argmin(np.abs(boundary_times - target_time)))
            if abs(boundary_times[nearest] - target_time) <= goal_seconds / 2.0:
                cut = segment_starts[nearest]
        if 0 < cut < len(est_times) - 1 and (not cuts or cut > cuts[-1]):
            cuts.append(cut)
    return cuts


def slice_goal_waypoints(waypoints, cuts):
    """Slice waypoints into per-goal lists with the boundary waypoint DUPLICATED
    (goal k+1 starts exactly at goal k's last waypoint -- the seam contract)."""
    bounds = [0] + list(cuts) + [len(waypoints) - 1]
    return [waypoints[bounds[k]:bounds[k + 1] + 1] for k in range(len(bounds) - 1)]


def joint_path_geometry(positions):
    """Per-interval joint-space geometry of a dense joint path [N, J].
    Returns (lengths [N-1], unit_dirs [N-1, J]); zero-length intervals keep a zero
    direction vector (callers must guard)."""
    steps = np.diff(positions, axis=0)
    lengths = np.linalg.norm(steps, axis=1)
    unit_dirs = np.zeros_like(steps)
    moving = lengths > 1e-12
    unit_dirs[moving] = steps[moving] / lengths[moving, None]
    return lengths, unit_dirs


def junction_speed_cap(dir_in, dir_out, corner_delta_v):
    """Corner speed cap between two interval directions. Joins gentler than
    JUNCTION_ANGLE_FREE get NO cap -- dense sampling means smooth curvature, which
    the acceleration pass already shapes (a deviation-style cap here throttles the
    whole drawing to a crawl). Sharper joins are capped so no joint's velocity
    steps more than corner_delta_v through the corner."""
    cos_theta = float(np.clip(np.dot(dir_in, dir_out), -1.0, 1.0))
    if cos_theta >= math.cos(JUNCTION_ANGLE_FREE):
        return math.inf
    velocity_step = float(np.max(np.abs(dir_out - dir_in)))  # per-joint, unit speed
    return corner_delta_v / max(velocity_step, 1e-9)


def point_speed_caps(lengths, unit_dirs, velocity_limit, corner_delta_v):
    """Path-speed cap at every point [N]: per-joint velocity limit (path speed *
    |dir component| <= joint limit) plus corner caps at interior direction changes.
    Zero-length and micro intervals (IK jitter) contribute no corner cap."""
    point_count = len(lengths) + 1
    caps = np.full(point_count, math.inf)
    for interval in range(len(lengths)):
        if lengths[interval] <= 1e-12:
            continue
        biggest_component = float(np.max(np.abs(unit_dirs[interval])))
        interval_cap = velocity_limit / max(biggest_component, 1e-9)
        caps[interval] = min(caps[interval], interval_cap)
        caps[interval + 1] = min(caps[interval + 1], interval_cap)
    micro = 1e-5  # rad; direction of a micro interval is IK noise, not geometry
    for point in range(1, point_count - 1):
        before, after = point - 1, point
        if lengths[before] <= micro or lengths[after] <= micro:
            continue
        corner = junction_speed_cap(unit_dirs[before], unit_dirs[after], corner_delta_v)
        caps[point] = min(caps[point], corner)
    return caps


def forward_backward_speeds(caps, lengths, path_accel, entry_speed, exit_speed):
    """Classic accel-limited two-pass speed profile along the path.
    Returns per-point path speeds [N] honoring entry/exit speeds and all caps.
    entry_speed is clipped to the feasible profile (a warning-worthy event the
    caller detects by comparing speeds[0] to entry_speed)."""
    speeds = caps.copy()
    speeds[0] = min(speeds[0], entry_speed)
    for point in range(1, len(speeds)):  # forward: acceleration limit
        reachable = math.sqrt(speeds[point - 1] ** 2 + 2.0 * path_accel * lengths[point - 1])
        speeds[point] = min(speeds[point], reachable)
    speeds[-1] = min(speeds[-1], exit_speed)
    for point in range(len(speeds) - 2, -1, -1):  # backward: deceleration limit
        reachable = math.sqrt(speeds[point + 1] ** 2 + 2.0 * path_accel * lengths[point])
        speeds[point] = min(speeds[point], reachable)
    return speeds


def speeds_to_durations(speeds, lengths, path_accel):
    """Per-interval durations [N-1] from the speed profile (trapezoid rule),
    floored at MIN_POINT_DT. A rest-to-rest interval with real length (both
    endpoint speeds ~0, e.g. between duplicated dwell points) gets the triangular
    accel+decel time 2*sqrt(length/accel) instead of the (infinite) trapezoid."""
    durations = np.empty(len(lengths))
    for interval in range(len(lengths)):
        pair_speed = speeds[interval] + speeds[interval + 1]
        if lengths[interval] <= 1e-12:
            durations[interval] = MIN_POINT_DT
        elif pair_speed <= 1e-9:
            durations[interval] = max(MIN_POINT_DT,
                                      2.0 * math.sqrt(lengths[interval] / path_accel))
        else:
            durations[interval] = max(MIN_POINT_DT, 2.0 * lengths[interval] / pair_speed)
    return durations


def point_joint_velocities(lengths, unit_dirs, speeds):
    """Per-point joint velocity vectors [N, J]: point direction is the
    length-weighted mean of adjacent interval directions (single-sided at the
    ends), scaled by the point's path speed."""
    point_count = len(lengths) + 1
    velocities = np.zeros((point_count, unit_dirs.shape[1]))
    for point in range(point_count):
        before = unit_dirs[point - 1] * lengths[point - 1] if point > 0 else None
        after = unit_dirs[point] * lengths[point] if point < len(lengths) else None
        blended = (before if after is None else after if before is None else before + after)
        norm = float(np.linalg.norm(blended))
        if norm > 1e-12:
            velocities[point] = blended / norm * speeds[point]
    return velocities


def retime_positions(positions, limits, entry_speed, exit_speed):
    """Retime a dense joint path [N, J]: returns (durations [N-1], speeds [N],
    velocities [N, J]). limits = (velocity_limit, accel_limit, corner_delta_v).
    Pure; the seam-window slicing is done by the caller."""
    velocity_limit, path_accel, corner_delta_v = limits
    lengths, unit_dirs = joint_path_geometry(positions)
    caps = point_speed_caps(lengths, unit_dirs, velocity_limit, corner_delta_v)
    speeds = forward_backward_speeds(caps, lengths, path_accel, entry_speed, exit_speed)
    durations = speeds_to_durations(speeds, lengths, path_accel)
    velocities = point_joint_velocities(lengths, unit_dirs, speeds)
    return durations, speeds, velocities


def retime_goal_window(goal_positions, next_positions, limits, entry_speed, entry_velocity):
    """Retime goal k across the seam into goal k+1 (Phase 2 seam continuity).

    The window [goal k | goal k+1] is profiled with exit speed 0 at the WINDOW end
    (always decelerable); only goal k's slice is returned -- goal k+1 is re-timed
    again later inside its own window. next_positions=None retimes goal k alone to
    rest (last goal / clean-stop path). entry_velocity (vector or None) overrides
    point 0's velocity verbatim so the emitted seam is C1 with the previous goal.

    Returns (times [Nk] from 0, velocities [Nk, J], seam_speed, seam_velocity)."""
    goal_points = len(goal_positions)
    if next_positions is not None:
        window = np.vstack([goal_positions, next_positions[1:]])  # seam point deduped
    else:
        window = np.asarray(goal_positions)
    durations, speeds, velocities = retime_positions(window, limits, entry_speed, 0.0)
    if entry_velocity is not None:
        velocities[0] = entry_velocity
    times = np.concatenate(([0.0], np.cumsum(durations[:goal_points - 1])))
    seam_speed = float(speeds[goal_points - 1])
    seam_velocity = velocities[goal_points - 1].copy()
    return times, velocities[:goal_points], seam_speed, seam_velocity


def seam_error(previous_end_joints, first_point_positions):
    """Max absolute per-joint delta between a goal's first planned point and the
    previous goal's planned end -- the sender-side seam gate."""
    previous = np.asarray(previous_end_joints, dtype=float)
    current = np.asarray(first_point_positions, dtype=float)
    return float(np.max(np.abs(current - previous)))


def goal_split_summary(slices, est_times, cuts):
    """Human-readable --dry-run table of the goal split (index, waypoints, est s)."""
    bounds = [0] + list(cuts) + [len(est_times) - 1]
    lines = [f"  {len(slices)} goal(s):"]
    for index, piece in enumerate(slices):
        begin, end = bounds[index], bounds[index + 1]
        lines.append(f"    goal {index}: {len(piece):5d} waypoints, "
                     f"est {est_times[end] - est_times[begin]:6.2f} s "
                     f"[wp {begin}..{end}]")
    return "\n".join(lines)


# ============================ self-test (no ROS) ==============================
def run_self_tests():
    """Assert the pure helpers' core invariants; returns 0 on success. Prints one
    line per check; any failure raises AssertionError (non-zero exit)."""
    limits = (1.0, 2.0, CORNER_DELTA_V)  # vel 1 rad/s, acc 2 rad/s^2

    line = np.linspace([0.0, 0.0], [1.0, 0.0], 101)  # straight 1 rad path, 2 joints
    durations, speeds, velocities = retime_positions(line, limits, 0.0, 0.0)
    assert speeds[0] == 0.0 and speeds[-1] == 0.0, "rest-to-rest boundary speeds"
    assert np.max(speeds) <= 1.0 + 1e-9, "velocity limit respected"
    assert np.max(speeds) > 0.9, "trapezoid reaches near the velocity limit"
    assert np.all(durations > 0.0), "strictly positive durations"
    accels = np.abs(np.diff(speeds)) / durations
    assert np.max(accels) <= 2.0 + 1e-6, "acceleration limit respected"
    print("  ok: straight-line trapezoid profile")

    corner = np.vstack([np.linspace([0, 0], [1, 0], 51),
                        np.linspace([1, 0], [1, 1], 51)[1:]])  # 90-degree corner
    _, corner_speeds, _ = retime_positions(corner, limits, 0.0, 0.0)
    assert corner_speeds[50] < 0.15, f"corner forces slowdown (got {corner_speeds[50]:.3f})"
    theta = np.linspace(0.0, math.pi / 2.0, 200)  # smooth quarter arc, ~0.45 deg/join
    arc = np.column_stack([np.cos(theta), np.sin(theta)])
    _, arc_speeds, _ = retime_positions(arc, limits, 0.0, 0.0)
    assert np.max(arc_speeds) > 0.9, \
        f"smooth curvature must NOT be corner-capped (got {np.max(arc_speeds):.3f})"
    print("  ok: junction cap slows a 90-degree corner, leaves smooth arcs alone")

    goal_a, goal_b = line[:60], line[59:]  # split with duplicated seam point
    times_a, vels_a, seam_speed, seam_velocity = retime_goal_window(
        goal_a, goal_b, limits, 0.0, None)
    assert seam_speed > 0.5, f"seam keeps speed through the window (got {seam_speed:.3f})"
    assert np.allclose(vels_a[-1], seam_velocity), "emitted tail velocity == carried seam"
    times_b, vels_b, _, _ = retime_goal_window(goal_b, None, limits, seam_speed, seam_velocity)
    assert np.allclose(vels_b[0], seam_velocity), "next goal entry velocity == carried seam"
    assert np.allclose(vels_b[-1], 0.0), "final goal ends at rest"
    assert np.all(np.diff(times_a) > 0) and np.all(np.diff(times_b) > 0), "monotone times"
    print("  ok: cross-seam window carries a C1 seam velocity")

    waypoints = [((x, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)) for x in np.linspace(0.0, 0.1, 201)]
    est_times = estimate_waypoint_times(waypoints, 0.01)  # 10 s of motion
    cuts = split_cut_indices(est_times, [0, 100], 3.0)
    assert cuts and all(0 < c < 200 for c in cuts), "interior cuts produced"
    assert 100 in cuts, "cut snapped to the segment boundary"
    slices = slice_goal_waypoints(waypoints, cuts)
    for first, second in zip(slices, slices[1:]):
        assert first[-1] == second[0], "seam waypoint duplicated"
    total = sum(len(piece) for piece in slices) - (len(slices) - 1)
    assert total == len(waypoints), "no waypoint lost or invented"
    print("  ok: goal split snaps to segment bounds and duplicates seams")

    dwell = np.array([[0.0, 0.0], [0.0, 0.0], [0.5, 0.0], [0.5, 0.0]])  # duplicates
    dwell_durations, _, _ = retime_positions(dwell, limits, 0.0, 0.0)
    assert np.all(dwell_durations >= MIN_POINT_DT - 1e-12), "degenerate intervals floored"
    rest_to_rest_time = 2.0 * math.sqrt(0.5 / 2.0)  # triangular profile over 0.5 rad
    assert dwell_durations[1] >= rest_to_rest_time - 1e-9, \
        "real-length rest-to-rest interval gets the triangular time, not the floor"
    print("  ok: duplicate-point and rest-to-rest interval guards")
    print("self-test PASSED")
    return 0


# ============================ ROS-facing pieces ===============================
def parse_streamed_args(argv):
    """Split this script's extra flags from the shared monolithic ones.
    Returns (streamed_options dict, remaining argv for monolithic.parse_args)."""
    options = {"goal_seconds": DEFAULT_GOAL_SECONDS, "max_inflight": DEFAULT_MAX_INFLIGHT,
               "rest_seams": False, "self_test": False}
    remaining = []
    index = 0
    while index < len(argv):
        argument = argv[index]
        if argument == "--goal-seconds" and index + 1 < len(argv):
            options["goal_seconds"] = float(argv[index + 1]); index += 2
        elif argument == "--max-inflight" and index + 1 < len(argv):
            options["max_inflight"] = max(1, int(argv[index + 1])); index += 2
        elif argument == "--rest-seams":
            options["rest_seams"] = True; index += 1
        elif argument == "--self-test":
            options["self_test"] = True; index += 1
        else:
            remaining.append(argument); index += 1
    return options, remaining


class PlannedGoal:
    """One planned goal: joint positions [N, J] (MoveIt timing discarded), the end
    joint state (seam seed for the next plan), solved fraction, and plan wall time."""

    def __init__(self, positions, response_points, fraction, plan_seconds):
        self.positions = positions
        self.response_points = response_points  # raw msg points (for --rest-seams)
        self.end_joints = positions[-1].tolist()
        self.fraction = fraction
        self.plan_seconds = plan_seconds


class GoalPlanner:
    """Owns the /compute_cartesian_path client; plans one goal slice at a time,
    seeding each request with the previous goal's planned end state."""

    def __init__(self, node, cart_client, request_template):
        self.node = node
        self.cart_client = cart_client
        self.template = request_template  # (frame, group, link, eef_step, jump, collisions, vel, acc)

    def plan_goal(self, waypoint_slice, start_joints):
        """Plan one slice from start_joints. Returns PlannedGoal, or None on any
        failure (service error, fraction below MIN_PLAN_FRACTION, seam mismatch)."""
        import rclpy
        from moveit_msgs.srv import GetCartesianPath
        from sensor_msgs.msg import JointState
        from moveit_msgs.msg import RobotState

        request = GetCartesianPath.Request()
        request.header.frame_id = self.template["frame"]
        request.start_state = RobotState()
        request.start_state.joint_state = JointState(name=list(JOINTS),
                                                     position=list(start_joints))
        request.group_name = self.template["group"]
        request.link_name = self.template["link"]
        request.waypoints = [self.template["to_pose"](pos, quat)
                             for pos, quat in waypoint_slice]
        request.max_step = self.template["eef_step"]
        request.jump_threshold = self.template["jump"]
        request.avoid_collisions = self.template["collisions"]
        request.max_velocity_scaling_factor = self.template["vel"]
        request.max_acceleration_scaling_factor = self.template["acc"]

        started = time.perf_counter()
        future = self.cart_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=300.0)
        response = future.result()
        elapsed = time.perf_counter() - started
        if response is None or response.error_code.val != 1:
            self.node.get_logger().error(
                f"plan_goal: /compute_cartesian_path failed "
                f"(error_code={None if response is None else response.error_code.val})")
            return None
        if response.fraction < MIN_PLAN_FRACTION:
            self.node.get_logger().error(
                f"plan_goal: fraction {response.fraction:.3f} < {MIN_PLAN_FRACTION} "
                "(IK hole mid-goal); stopping cleanly")
            return None
        points = response.solution.joint_trajectory.points
        if not points:
            self.node.get_logger().error("plan_goal: empty trajectory")
            return None
        positions = np.array([point.positions for point in points], dtype=float)
        gate = seam_error(start_joints, positions[0])
        if gate > SEAM_TOLERANCE:
            self.node.get_logger().error(
                f"plan_goal: seam gate {gate:.4f} rad > {SEAM_TOLERANCE} (IK flip?)")
            return None
        return PlannedGoal(positions, points, response.fraction, elapsed)


class FinalizeSwitch:
    """Flips the controller's `finalize_last_chunk` parameter (the per-goal
    finalize hint). Caches the last value so repeat sets are no-ops."""

    def __init__(self, node, client):
        self.node = node
        self.client = client
        self.current = None  # unknown until first set

    def set_finalize(self, value):
        """Synchronously set the parameter; returns False on service failure.
        Must be called BEFORE sending the goal whose accept should read `value`."""
        import rclpy
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        from rcl_interfaces.srv import SetParameters

        if self.current == value:
            return True
        request = SetParameters.Request()
        request.parameters = [Parameter(
            name=FINALIZE_PARAM,
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value))]
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not all(res.successful for res in response.results):
            self.node.get_logger().error(f"set_finalize({value}) FAILED")
            return False
        self.current = value
        return True


class GoalPipeline:
    """Owns the FollowJointTrajectory client to the passthrough controller.
    submit() paces to --max-inflight; any rejection/abort latches `failed`."""

    def __init__(self, node, action_client, max_inflight):
        self.node = node
        self.client = action_client
        self.max_inflight = max_inflight
        self.pending_results = []  # (goal_index, result_future)
        self.failed = False

    def _poll(self, timeout_sec=0.05):
        """One spin + prune finished results; latches failure on bad status."""
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=timeout_sec)
        still_pending = []
        for goal_index, future in self.pending_results:
            if not future.done():
                still_pending.append((goal_index, future))
                continue
            result = future.result()
            code = result.result.error_code
            if result.status != 4 or code != 0:  # 4 = STATUS_SUCCEEDED
                self.node.get_logger().error(
                    f"goal {goal_index}: status={result.status} error_code={code} "
                    f"'{result.result.error_string}'")
                self.failed = True
            else:
                self.node.get_logger().info(f"goal {goal_index}: SUCCEEDED")
        self.pending_results = still_pending

    def submit(self, goal_index, goal_message, timeout_sec=30.0):
        """Send one goal (blocking only while >= max_inflight are unfinished or
        the server hasn't accepted). Returns False on rejection/failure."""
        import rclpy
        deadline = time.monotonic() + timeout_sec
        while len(self.pending_results) >= self.max_inflight and not self.failed:
            self._poll()
            if time.monotonic() > deadline:
                self.node.get_logger().error("submit: pacing timeout"); return False
        if self.failed:
            return False
        send_future = self.client.send_goal_async(goal_message)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.node.get_logger().error(
                f"goal {goal_index}: REJECTED (jog active? controller down?)")
            self.failed = True
            return False
        self.pending_results.append((goal_index, handle.get_result_async()))
        return True

    def drain(self, timeout_sec):
        """Spin until every submitted goal reports a result (or timeout).
        Returns True when all succeeded."""
        deadline = time.monotonic() + timeout_sec
        while self.pending_results and time.monotonic() < deadline:
            self._poll(timeout_sec=0.1)
        if self.pending_results:
            self.node.get_logger().error(
                f"drain: {len(self.pending_results)} goal(s) unresolved after "
                f"{timeout_sec:.0f} s")
            return False
        return not self.failed


def build_goal_message(times, positions, velocities):
    """FollowJointTrajectory goal from retimed arrays: every point carries
    positions AND velocities (the controller's `quadratic` interpolation requires
    velocities on every point). First point is at time_from_start = 0."""
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(JOINTS)
    for stamp, position, velocity in zip(times, positions, velocities):
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in position]
        point.velocities = [float(value) for value in velocity]
        point.time_from_start = Duration(sec=int(stamp),
                                         nanosec=int((stamp % 1.0) * 1e9))
        goal.trajectory.points.append(point)
    return goal


def moveit_timed_goal_message(response_points):
    """Phase 1 (--rest-seams) goal: forward the /compute_cartesian_path solution
    verbatim (MoveIt rest-to-rest timing, P/V/A as returned)."""
    from control_msgs.action import FollowJointTrajectory

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(JOINTS)
    goal.trajectory.points = list(response_points)
    return goal


# ============================ approach (PTP) helpers ==========================
# Small copies of the monolithic sender's nested closures (they are not importable);
# same services, same behavior: joint-PTP to `ready`, IK + joint-PTP to `anchor`.
def current_joint_state(node):
    """Read one /joint_states sample covering all managed joints, or None."""
    import rclpy
    from sensor_msgs.msg import JointState

    seen = {}
    subscription = node.create_subscription(
        JointState, "/joint_states",
        lambda msg: seen.update({name: pos for name, pos in zip(msg.name, msg.position)}), 10)
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(joint in seen for joint in JOINTS):
            break
    node.destroy_subscription(subscription)
    if not all(joint in seen for joint in JOINTS):
        node.get_logger().error("could not read /joint_states")
        return None
    return [seen[joint] for joint in JOINTS]


def solve_ik(node, ik_client, pose_builder, position, quaternion, seed):
    """Joint solution for a world pose via /compute_ik (seeded), or None."""
    import rclpy
    from moveit_msgs.srv import GetPositionIK
    from sensor_msgs.msg import JointState
    from builtin_interfaces.msg import Duration

    if not ik_client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(f"{monolithic.IK_SERVICE} unavailable")
        return None
    request = GetPositionIK.Request()
    ik = request.ik_request
    ik.group_name = monolithic.GROUP
    ik.ik_link_name = monolithic.TIP_LINK
    ik.pose_stamped.header.frame_id = monolithic.PLANNING_FRAME
    ik.pose_stamped.pose = pose_builder(position, quaternion)
    if seed is not None:
        ik.robot_state.joint_state = JointState(name=list(JOINTS), position=list(seed))
    ik.avoid_collisions = True
    ik.timeout = Duration(sec=2)
    future = ik_client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    response = future.result()
    if response is None or response.error_code.val != 1:
        return None
    solved = {name: pos for name, pos in zip(response.solution.joint_state.name,
                                             response.solution.joint_state.position)}
    if not all(joint in solved for joint in JOINTS):
        return None
    return [solved[joint] for joint in JOINTS]


def ptp_to_joints(node, move_client, joints, label, vel, acc):
    """Pilz joint-PTP via /move_action; True on success. Blocks up to 60 s."""
    import rclpy
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                                 PlanningOptions)

    request = MotionPlanRequest()
    request.pipeline_id = monolithic.PIPELINE
    request.planner_id = "PTP"
    request.group_name = monolithic.GROUP
    request.max_velocity_scaling_factor = vel
    request.max_acceleration_scaling_factor = acc
    request.allowed_planning_time = 5.0
    constraints = Constraints()
    for name, position in zip(JOINTS, joints):
        constraints.joint_constraints.append(JointConstraint(
            joint_name=name, position=float(position),
            tolerance_above=1e-4, tolerance_below=1e-4, weight=1.0))
    request.goal_constraints.append(constraints)
    goal = MoveGroup.Goal()
    goal.request = request
    goal.planning_options = PlanningOptions(plan_only=False)

    node.get_logger().info(f"PTP -> {label}")
    send_future = move_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    handle = send_future.result()
    if handle is None or not handle.accepted:
        node.get_logger().error(f"PTP to {label} REJECTED")
        return False
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=60.0)
    result = result_future.result()
    ok = result is not None and result.result.error_code.val == 1
    if not ok:
        node.get_logger().error(f"PTP to {label} FAILED")
    return ok


# ============================ streaming main flow =============================
def stream_goals(node, planner, pipeline, finalize, slices, start_joints, options,
                 limits, est_total):
    """The Phase 2 pipeline: plan k+1, retime k across the seam, submit k.
    One-goal emission lag; a failed plan closes the chain at rest. Returns exit
    code (0 ok). Side-effects: sends action goals, flips the finalize parameter."""
    logger = node.get_logger()
    stream_started = time.perf_counter()
    pending = planner.plan_goal(slices[0], start_joints)  # goal 0
    if pending is None:
        logger.error("goal 0 failed to plan; nothing sent")
        return 1
    entry_speed, entry_velocity = 0.0, None
    plan_seconds_total = pending.plan_seconds

    for index in range(len(slices)):
        following = None
        if index + 1 < len(slices) and not pipeline.failed:
            following = planner.plan_goal(slices[index + 1], pending.end_joints)
            plan_seconds_total += following.plan_seconds if following else 0.0
            if following is None:
                logger.warning(f"goal {index + 1} failed to plan; "
                               f"closing the chain at goal {index}")
        is_final = following is None
        times, velocities, seam_speed, seam_velocity = retime_goal_window(
            pending.positions, None if is_final else following.positions,
            limits, entry_speed, entry_velocity)
        if not finalize.set_finalize(is_final):
            return 1
        message = build_goal_message(times, pending.positions, velocities)
        logger.info(f"goal {index}: {len(pending.positions)} pts, "
                    f"retimed {float(times[-1]):.1f} s, seam {seam_speed:.3f} rad/s"
                    f"{' FINAL' if is_final else ''}")
        pacing_timeout = max(120.0, 4.0 * float(times[-1]) * pipeline.max_inflight)
        if not pipeline.submit(index, message, timeout_sec=pacing_timeout):
            return 1
        if index == 0:
            logger.info(f"first goal submitted {time.perf_counter() - stream_started:.2f} s "
                        f"after streaming start (est total motion {est_total:.1f} s)")
        if is_final:
            break
        pending, entry_speed, entry_velocity = following, seam_speed, seam_velocity

    ok = pipeline.drain(timeout_sec=max(180.0, est_total * 1.5 + 30.0))
    logger.info(f"planning total {plan_seconds_total:.1f} s across goals "
                f"(overlapped with execution)")
    return 0 if ok else 1


def stream_goals_rest_seams(node, planner, pipeline, finalize, slices, start_joints):
    """Phase 1 (--rest-seams): every goal keeps MoveIt's rest-to-rest timing and
    finalizes (a dwell at each seam; the Gate-6 back-to-back pattern). Returns exit
    code. Side-effects: sends action goals; forces finalize=true."""
    logger = node.get_logger()
    if not finalize.set_finalize(True):
        return 1
    seed = start_joints
    for index, piece in enumerate(slices):
        planned = planner.plan_goal(piece, seed)
        if planned is None:
            logger.error(f"goal {index} failed to plan; stopping after goal {index - 1}")
            return 1
        if not pipeline.submit(index, moveit_timed_goal_message(planned.response_points)):
            return 1
        seed = planned.end_joints
    return 0 if pipeline.drain(timeout_sec=600.0) else 1


def resolve_recipe(argv):
    """Parse args (streamed + shared), load the recipe, resolve eef/scale/rotate
    precedence exactly like the monolithic sender. Returns (options, args, doc,
    coords, defaults, segments) or (exit_code, None, ...) on error."""
    options, remaining = parse_streamed_args(argv)
    args = monolithic.parse_args(remaining)
    if options["self_test"]:
        return options, args, None, None, None, None
    if not args["recipe"]:
        print(__doc__)
        print("error: a recipe file (path or bundled name) is required", file=sys.stderr)
        return None, None, None, None, None, None
    path = monolithic.resolve_recipe_path(args["recipe"])
    if path is None:
        print(f"error: recipe '{args['recipe']}' not found", file=sys.stderr)
        return None, None, None, None, None, None
    doc, coords, defaults, segments = monolithic.load_recipe(path)
    defaults.update(args["overrides"])
    if args["eef_step"] is None:
        args["eef_step"] = float(doc.get("eef_step", monolithic.EEF_STEP))
    if args["scale"] is None:
        args["scale"] = float(doc.get("scale", 1.0))
    if args["rotate"] is None:
        args["rotate"] = float(doc.get("rotate", 0.0))
    # No CLI override: the plane calibration travels in the file so a
    # standalone replay lands where the bridge put it. Unconditional --
    # with no flag to preserve, the file is the only source.
    args["plane_rotate"] = float(doc.get("plane_rotate", 0.0))
    args["recipe_path"] = path
    return options, args, doc, coords, defaults, segments


def prepare_split(doc, coords, defaults, segments, args, options):
    """Evaluate the recipe geometry and split it into goal slices.
    Returns (waypoints, slices, cuts, est_times) -- pure, no ROS."""
    anchor = doc.get("anchor")
    if anchor is not None and coords == "relative":
        start_pos, start_quat = monolithic.anchor_pose(anchor)
    else:
        start_pos, start_quat = (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)
    waypoints = monolithic.build_waypoints(segments, start_pos, start_quat, coords,
                                           defaults, args["scale"], args["rotate"],
                                           args["plane_rotate"])
    # Cartesian speed estimate for goal sizing: the drawing feed is roughly
    # vel_scaling * a nominal 0.25 m/s Cartesian full speed; only cut PLACEMENT
    # depends on this, so a rough constant is fine.
    cartesian_speed = max(1e-3, 0.25 * float(defaults["vel"]))
    est_times = estimate_waypoint_times(waypoints, cartesian_speed)
    starts = segment_start_indices(segments, defaults)
    cuts = split_cut_indices(est_times, starts, options["goal_seconds"])
    slices = slice_goal_waypoints(waypoints, cuts)
    return waypoints, slices, cuts, est_times


def run_streamed(options, args, doc, coords, defaults, segments, node=None):
    """ROS mode: approach, then plan/stream the goal slices. Returns exit code.

    Pass an existing rclpy ``node`` to run inside another process (the draw-plane
    bridge imports this module and calls it on its batch thread): the function
    then neither inits nor shuts down rclpy, and the caller must not spin the
    node concurrently -- this function spins it internally.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from geometry_msgs.msg import Pose, Point, Quaternion
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.srv import GetCartesianPath, GetPositionIK
    from rcl_interfaces.srv import SetParameters
    from control_msgs.action import FollowJointTrajectory

    def to_pose(position, quaternion):
        qw, qx, qy, qz = quaternion
        return Pose(position=Point(x=position[0], y=position[1], z=position[2]),
                    orientation=Quaternion(w=qw, x=qx, y=qy, z=qz))

    owns_context = node is None
    if owns_context:
        rclpy.init()
        node = Node("send_cartesian_path_streamed")
    cart_client = node.create_client(GetCartesianPath, monolithic.CART_SERVICE)
    ik_client = node.create_client(GetPositionIK, monolithic.IK_SERVICE)
    param_client = node.create_client(SetParameters, SET_PARAM_SERVICE)
    move_client = ActionClient(node, MoveGroup, monolithic.MOVE_ACTION)
    fjt_client = ActionClient(node, FollowJointTrajectory, ACTION_NAME)
    for client, name in ((cart_client, monolithic.CART_SERVICE),
                         (param_client, SET_PARAM_SERVICE)):
        if not client.wait_for_service(timeout_sec=10.0):
            node.get_logger().error(f"{name} unavailable"); return 1
    if not fjt_client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error(f"{ACTION_NAME} unavailable"); return 1
    plan_only = args["plan_only"]
    if not plan_only and not move_client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error(f"{monolithic.MOVE_ACTION} unavailable"); return 1

    # Start pose (anchor) + approach, mirroring the monolithic sender.
    ready = doc.get("ready")
    anchor = doc.get("anchor")
    if anchor is not None and coords != "relative":
        anchor = None
    if anchor is not None:
        start_pos, start_quat = monolithic.anchor_pose(anchor)
    else:
        node.get_logger().error("streamed sender requires an 'anchor' (relative recipe)")
        return 1
    seed = [float(value) for value in ready] if ready is not None else current_joint_state(node)
    if seed is None:
        return 1
    if not plan_only and ready is not None:
        if not ptp_to_joints(node, move_client, seed, "ready",
                             defaults["vel"], defaults["acc"]):
            return 1
    anchor_joints = solve_ik(node, ik_client, to_pose, start_pos, start_quat, seed)
    if anchor_joints is None:
        node.get_logger().error("IK for the anchor pose failed; adjust `anchor`/`ready`")
        return 1
    if not plan_only:
        if not ptp_to_joints(node, move_client, anchor_joints, "anchor",
                             defaults["vel"], defaults["acc"]):
            return 1

    waypoints, slices, cuts, est_times = prepare_split(
        doc, coords, defaults, segments, args, options)
    node.get_logger().info(
        f"{len(segments)} segments -> {len(waypoints)} waypoints -> {len(slices)} goal(s)")

    template = {"frame": monolithic.PLANNING_FRAME, "group": monolithic.GROUP,
                "link": monolithic.TIP_LINK, "eef_step": args["eef_step"],
                "jump": args["jump"], "collisions": not args["no_collision"],
                "vel": defaults["vel"], "acc": defaults["acc"], "to_pose": to_pose}
    planner = GoalPlanner(node, cart_client, template)

    if plan_only:
        seed_joints = anchor_joints
        for index, piece in enumerate(slices):
            planned = planner.plan_goal(piece, seed_joints)
            if planned is None:
                node.get_logger().error(f"plan-only: goal {index} NOT plannable"); return 1
            node.get_logger().info(
                f"plan-only: goal {index}: fraction={planned.fraction:.3f} "
                f"{len(planned.positions)} pts plan={planned.plan_seconds:.2f} s")
            seed_joints = planned.end_joints
        if owns_context:
            rclpy.shutdown()
        return 0

    limits = (JOINT_VELOCITY_LIMIT * float(defaults["vel"]),
              JOINT_ACCEL_LIMIT * float(defaults["acc"]),
              CORNER_DELTA_V)
    pipeline = GoalPipeline(node, fjt_client, options["max_inflight"])
    finalize = FinalizeSwitch(node, param_client)
    if options["rest_seams"]:
        code = stream_goals_rest_seams(node, planner, pipeline, finalize,
                                       slices, anchor_joints)
    else:
        code = stream_goals(node, planner, pipeline, finalize, slices, anchor_joints,
                            options, limits, float(est_times[-1]))
    # Leave the controller parameter as the default (true) for whoever runs next.
    finalize.set_finalize(True)
    if owns_context:
        rclpy.shutdown()
    return code


def main():
    options, args, doc, coords, defaults, segments = resolve_recipe(sys.argv[1:])
    if options is None:
        return 2
    if options["self_test"]:
        return run_self_tests()
    if args["dry_run"]:
        waypoints, slices, cuts, est_times = prepare_split(
            doc, coords, defaults, segments, args, options)
        print(f"[dry-run] recipe '{os.path.basename(args['recipe_path'])}' "
              f"({len(segments)} segments, {len(waypoints)} waypoints, "
              f"est {est_times[-1]:.1f} s at goal_seconds={options['goal_seconds']})")
        print(goal_split_summary(slices, est_times, cuts))
        return 0
    return run_streamed(options, args, doc, coords, defaults, segments)


if __name__ == "__main__":
    sys.exit(main())
