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

"""Linear algebra library: every coordinate decision of the draw-plane product.

Pure functions only. Every caller passes all inputs; nothing is read from the
environment. The client and the bridge both import these, so the
canvas-to-plane convention (y-flip, no x-mirror) is pinned in exactly one
place.

Conventions
-----------
* A canvas position is ``(px, py)`` in device pixels, origin top-left, y down.
* A plane coordinate is ``(x, y)`` in metres on the drawing plane, y up.
* A command is ``(x, y, z)`` plane-frame metres; z is the pen depth axis.
* An extent is ``((x_min, y_min), (x_max, y_max))``.
* A stroke is a list of plane coordinates.
"""

import math
from typing import List, NamedTuple, Sequence, Tuple

Coordinate = Tuple[float, float]
Command = Tuple[float, float, float]
Extent = Tuple[Coordinate, Coordinate]
Stroke = List[Coordinate]


class Limits(NamedTuple):
    """The configured motion bounds ``bound_verdicts`` checks against."""

    extent: Extent
    working_centre: Coordinate
    working_radius: float
    depth_min: float
    depth_max: float
    max_speed: float  # metres per second along the commanded path


class BoundVerdicts(NamedTuple):
    """Per-bound verdicts for one outgoing command."""

    extent_ok: bool
    depth_ok: bool
    working_area_ok: bool
    speed_ok: bool

    @property
    def ok(self) -> bool:
        return self.extent_ok and self.depth_ok and self.working_area_ok and self.speed_ok


def rpy_to_quaternion(rpy: Tuple[float, float, float]) -> Tuple[float, float, float, float]:
    """Roll-pitch-yaw (radians, extrinsic xyz) to a ``(w, x, y, z)`` quaternion.

    The anchor rpy orients the TOOL only (see ``plane_to_world``); the bridge
    and the stroke route both stamp every outgoing pose with this constant.
    """
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def rotate_in_plane(point: Coordinate, degrees: float) -> Coordinate:
    """Rotate a plane coordinate counter-clockwise about the plane origin.

    Degrees, not radians: that is the unit the recipe format's ``rotate`` uses,
    the unit the operator's field takes, and the unit that crosses the wire.
    The conversion happens here and nowhere else, so the neighbouring
    ``anchor_rpy`` (radians) can never be confused for it.

    The expression is deliberately identical to the sender's ``build_waypoints``
    (``send_cartesian_path.py``), down to the order of operations: the live
    route rotates here and the batch route rotates there, and the two must land
    on the same world point for the same drawing.
    """
    theta = math.radians(degrees)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    x, y = point
    return (x * cos_t - y * sin_t, x * sin_t + y * cos_t)


def map_canvas_to_plane(
        canvas: Coordinate, surface: Coordinate, extent: Extent) -> Coordinate:
    """Map a canvas position onto the plane extent, y-flipped.

    ``surface`` is the drawing surface size ``(width, height)`` in the canvas
    units. The full surface spans the full extent; the y axis flips so canvas-up
    is plane-up (no x mirror).
    """
    (x_min, y_min), (x_max, y_max) = extent
    width, height = surface
    if width <= 0.0 or height <= 0.0:
        raise ValueError(f'surface must be positive, got {surface}')
    x = x_min + (canvas[0] / width) * (x_max - x_min)
    y = y_min + (1.0 - canvas[1] / height) * (y_max - y_min)
    return (x, y)


def map_plane_to_canvas(
        plane: Coordinate, surface: Coordinate, extent: Extent) -> Coordinate:
    """The inverse of ``map_canvas_to_plane``: plane metres to canvas pixels.

    The client renders retained ink (plane coordinates from the display state)
    onto its canvas with this, so both directions of the mapping live here.
    """
    (x_min, y_min), (x_max, y_max) = extent
    width, height = surface
    if x_max <= x_min or y_max <= y_min:
        raise ValueError(f'extent must be positive, got {extent}')
    px = (plane[0] - x_min) / (x_max - x_min) * width
    py = (1.0 - (plane[1] - y_min) / (y_max - y_min)) * height
    return (px, py)


def clamp_to_extent(plane: Coordinate, extent: Extent) -> Coordinate:
    """Clamp a plane coordinate into the configured extent."""
    (x_min, y_min), (x_max, y_max) = extent
    return (min(max(plane[0], x_min), x_max), min(max(plane[1], y_min), y_max))


def clamp_to_working_area(
        plane: Coordinate, centre: Coordinate, radius: float) -> Coordinate:
    """Clamp a plane coordinate into the reachable working-area disc."""
    dx = plane[0] - centre[0]
    dy = plane[1] - centre[1]
    distance = math.hypot(dx, dy)
    if distance <= radius or distance == 0.0:
        return plane
    scale = radius / distance
    return (centre[0] + dx * scale, centre[1] + dy * scale)


def limit_displacement(
        desired: Coordinate, last: Coordinate, max_step: float) -> Coordinate:
    """Bound one tick's target displacement to ``max_step``."""
    dx = desired[0] - last[0]
    dy = desired[1] - last[1]
    distance = math.hypot(dx, dy)
    if distance <= max_step or distance == 0.0:
        return desired
    scale = max_step / distance
    return (last[0] + dx * scale, last[1] + dy * scale)


def bound_verdicts(
        command: Command, previous: Command, period: float,
        limits: Limits) -> BoundVerdicts:
    """Judge one outgoing command against every configured bound.

    The guard call sites (batch executor, live target streamer) act on the
    verdicts; this function only judges (the design keeps the clamp and the guard
    at separate call sites).
    """
    x, y, z = command
    (x_min, y_min), (x_max, y_max) = limits.extent
    extent_ok = x_min <= x <= x_max and y_min <= y <= y_max
    depth_ok = limits.depth_min <= z <= limits.depth_max
    working_area_ok = (
        math.hypot(x - limits.working_centre[0], y - limits.working_centre[1])
        <= limits.working_radius)
    if period <= 0.0:
        speed_ok = False
    else:
        distance = math.dist(command, previous)
        speed_ok = (distance / period) <= limits.max_speed
    return BoundVerdicts(extent_ok, depth_ok, working_area_ok, speed_ok)


def sample_line(start: Coordinate, end: Coordinate, spacing: float) -> Stroke:
    """Sample the line from ``start`` to ``end`` at ``spacing``.

    Both endpoints are always included; interior points sit on a uniform grid
    no coarser than ``spacing``.
    """
    if spacing <= 0.0:
        raise ValueError(f'spacing must be positive, got {spacing}')
    length = math.dist(start, end)
    if length == 0.0:
        return [start]
    steps = max(1, math.ceil(length / spacing))
    return [
        (start[0] + (end[0] - start[0]) * i / steps,
         start[1] + (end[1] - start[1]) * i / steps)
        for i in range(steps + 1)
    ]


def _circumcentre(a: Coordinate, b: Coordinate, c: Coordinate):
    """Centre of the circle through three points, or None when collinear."""
    d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
    if abs(d) < 1e-12:
        return None
    a2 = a[0] * a[0] + a[1] * a[1]
    b2 = b[0] * b[0] + b[1] * b[1]
    c2 = c[0] * c[0] + c[1] * c[1]
    ux = (a2 * (b[1] - c[1]) + b2 * (c[1] - a[1]) + c2 * (a[1] - b[1])) / d
    uy = (a2 * (c[0] - b[0]) + b2 * (a[0] - c[0]) + c2 * (b[0] - a[0])) / d
    return (ux, uy)


def sample_arc(
        first: Coordinate, second: Coordinate, third: Coordinate,
        spacing: float) -> Stroke:
    """Sample the arc from ``first`` to ``third`` through ``second``.

    Three (nearly) collinear points define no circle; the operator's intent is
    then a flat curve, so the sampling degrades to the line first-to-third.
    """
    if spacing <= 0.0:
        raise ValueError(f'spacing must be positive, got {spacing}')
    centre = _circumcentre(first, second, third)
    if centre is None:
        return sample_line(first, third, spacing)
    radius = math.dist(centre, first)

    def angle(p: Coordinate) -> float:
        return math.atan2(p[1] - centre[1], p[0] - centre[0])

    a1, a2, a3 = angle(first), angle(second), angle(third)
    # Sweep counter-clockwise from first to third; if the middle point does not
    # lie on that sweep, the operator drew the arc the other way round.
    ccw_total = (a3 - a1) % (2.0 * math.pi)
    ccw_mid = (a2 - a1) % (2.0 * math.pi)
    if ccw_mid <= ccw_total:
        sweep = ccw_total
    else:
        sweep = ccw_total - 2.0 * math.pi  # clockwise
    arc_length = abs(sweep) * radius
    if arc_length == 0.0:
        return [first]
    steps = max(1, math.ceil(arc_length / spacing))
    return [
        (centre[0] + radius * math.cos(a1 + sweep * i / steps),
         centre[1] + radius * math.sin(a1 + sweep * i / steps))
        for i in range(steps + 1)
    ]


def remove_duplicates(stroke: Sequence[Coordinate], threshold: float) -> Stroke:
    """Drop points closer than ``threshold`` to the last kept point.

    The first point is always kept. The final point is always kept too -- a
    stroke must end where the operator ended it -- unless it duplicates the
    kept point before it exactly.
    """
    if not stroke:
        return []
    kept: Stroke = [tuple(stroke[0])]
    for point in stroke[1:-1]:
        if math.dist(point, kept[-1]) >= threshold:
            kept.append(tuple(point))
    if len(stroke) > 1 and tuple(stroke[-1]) != kept[-1]:
        kept.append(tuple(stroke[-1]))
    return kept


def resample_uniform(stroke: Sequence[Coordinate], spacing: float) -> Stroke:
    """Resample a polyline at uniform arc-length ``spacing``.

    Both endpoints are preserved; interior samples sit at exact multiples of
    the resulting (evenly divided) spacing along the polyline.
    """
    if spacing <= 0.0:
        raise ValueError(f'spacing must be positive, got {spacing}')
    if len(stroke) < 2:
        return [tuple(p) for p in stroke]
    lengths = [math.dist(stroke[i], stroke[i + 1]) for i in range(len(stroke) - 1)]
    total = sum(lengths)
    if total == 0.0:
        return [tuple(stroke[0])]
    steps = max(1, math.ceil(total / spacing))
    step = total / steps
    out: Stroke = [tuple(stroke[0])]
    segment = 0
    consumed = 0.0  # arc length before the current segment
    for i in range(1, steps):
        target = i * step
        while segment < len(lengths) and consumed + lengths[segment] < target:
            consumed += lengths[segment]
            segment += 1
        fraction = (target - consumed) / lengths[segment]
        a, b = stroke[segment], stroke[segment + 1]
        out.append((a[0] + (b[0] - a[0]) * fraction, a[1] + (b[1] - a[1]) * fraction))
    out.append(tuple(stroke[-1]))
    return out


def split_at_working_area(
        stroke: Sequence[Coordinate], centre: Coordinate, radius: float,
        min_run: int) -> Tuple[List[Stroke], List[Stroke]]:
    """Split a stroke into in-area runs and excluded runs.

    Consecutive points inside the working-area disc form an in-area run; an
    in-area run shorter than ``min_run`` points is excluded too (a fragment not
    worth a pen touch). Returns ``(in_runs, excluded_runs)`` in stroke order.
    """
    in_runs: List[Stroke] = []
    excluded: List[Stroke] = []
    current: Stroke = []
    current_inside = None
    for point in stroke:
        inside = math.hypot(point[0] - centre[0], point[1] - centre[1]) <= radius
        if inside == current_inside:
            current.append(tuple(point))
            continue
        if current:
            if current_inside and len(current) >= min_run:
                in_runs.append(current)
            else:
                excluded.append(current)
        current = [tuple(point)]
        current_inside = inside
    if current:
        if current_inside and len(current) >= min_run:
            in_runs.append(current)
        else:
            excluded.append(current)
    return in_runs, excluded


REACH_BAND_TOLERANCE = 1e-9   # metres; see disc_fits_reach_band


def extent_fits_working_area(
        extent: Extent, centre: Coordinate, radius: float) -> bool:
    """True when the whole extent lies inside the working-area disc.

    The extent is a rectangle; it fits exactly when all four corners do.
    """
    (x_min, y_min), (x_max, y_max) = extent
    corners = ((x_min, y_min), (x_min, y_max), (x_max, y_min), (x_max, y_max))
    return all(
        math.hypot(cx - centre[0], cy - centre[1]) <= radius for cx, cy in corners)


def disc_fits_reach_band(
        centre: Coordinate, radius: float, inner: float, outer: float) -> bool:
    """True when a disc lies wholly inside the arm's reach band.

    ``centre`` is in the PLANNING frame, unlike every other bound here. The
    working area is plane-relative, so it travels with the plane origin; once
    the operator can move that origin, something has to judge where the disc
    ends up in the frame the arm actually lives in. This is that check, and it
    is the only one that crosses out of plane coordinates.

    The comparison is tolerant by a nanometre because the DEFAULT plane sits
    exactly on both edges:  derives the 0.16 m radius as the half
    width of the 0.30...0.62 m band and puts the origin on its
    midline at 0.46 m, so 0.46 -/+ 0.16 lands on each limit to the bit. A
    strict comparison would leave the stock configuration's ability to start
    depending on floating-point rounding.
    """
    distance = math.hypot(centre[0], centre[1])
    return (distance - radius >= inner - REACH_BAND_TOLERANCE and
            distance + radius <= outer + REACH_BAND_TOLERANCE)
