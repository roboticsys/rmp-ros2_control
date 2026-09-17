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

"""Recipe loader: existing recipe YAML files into batch mode, verbatim.

Loads a recipe file, re-anchors it onto the configured drawing plane, and
returns both the executable structure and a plane-frame preview of its
pen-down path. The doc executes VERBATIM (stakeholder 2026-08-26): segments,
per-segment depths, vel/acc, eef_step, and rotate stay as authored; only
``anchor``, ``ready`` and ``plane_rotate`` are replaced with the plane's
values, and ``scale`` with the effective scale, so the drawing lands on the
plane the operator calibrated. The anchor written is ``effective_anchor()``
and the rotation written is the plane's own, so a load picks up the operator's
calibration whatever the file itself carries.

The recipe's local frame IS the drawing-plane frame: the sender maps it with
the same rotate-then-translate the live route's ``plane_to_world`` uses, so no
other transformation is needed. Note that the sender ADDS the recipe's
authored ``rotate`` to the plane's ``plane_rotate`` -- an artwork rotated 30
degrees on a plane calibrated to 45 lands at 75 -- which is why the two are
separate keys and only one of them is replaced here.

Validation samples the recipe's footprint: a pen-down path that leaves the
working area is refused (the motion bound); one that overruns the canvas
extent only warns (the extent is the operator's window, and verbatim recipes
may legitimately exceed it). ``fit=True`` computes the largest scale that
satisfies both bounds instead.

Pure Python, no ROS. The geometry module (``send_cartesian_path``, whose
imports are math/os/sys/numpy/yaml only) is injected, mirroring the way
``BatchExecutor`` receives the sender module, so tests run without a
workspace.
"""

import math
import os
from typing import List, NamedTuple, Optional, Sequence, Tuple

import yaml

from .file_manager import PlaneConfig
from .linalg import Coordinate, Extent

Polyline = List[Coordinate]

PREVIEW_POLYLINE_CAP = 200    # points kept per preview polyline
PREVIEW_TOTAL_CAP = 1500      # points kept across the whole preview


class RecipeLoadError(Exception):
    """The recipe cannot be loaded or must not be executed on this plane."""


class LoadedRecipe(NamedTuple):
    """One loaded, re-anchored recipe ready for the batch route."""

    name: str
    doc: dict                   # executable structure (anchor/ready/scale replaced)
    polylines: List[Polyline]   # pen-down preview, plane frame, decimated
    scale: float                # effective in-plane scale written into doc
    notes: List[str]            # non-fatal findings (extent overrun, depth)


class RecipeLoader:
    """``list_names``/``load`` over an ordered list of recipe directories.

    The first directory that holds a name wins (the operator's save directory
    shadows the bundled recipes). ``geometry`` supplies ``load_recipe`` and
    ``segment_local_points``; the scale-then-rotate transform matches its
    ``build_waypoints`` exactly, so the preview shows what will execute.
    """

    def __init__(self, geometry, directories: Sequence[str]):
        self._geometry = geometry
        self._directories = list(directories)

    def list_names(self) -> List[str]:
        """Loadable recipe names: ``*.yaml`` basenames without the suffix."""
        names = set()
        for directory in self._directories:
            try:
                entries = os.listdir(directory)
            except OSError:
                continue
            names.update(entry[:-len('.yaml')] for entry in entries
                         if entry.endswith('.yaml'))
        return sorted(names)

    def load(self, name: str, plane: PlaneConfig, extent: Extent,
             working_centre: Coordinate, working_radius: float,
             scale: Optional[float] = None, fit: bool = False) -> LoadedRecipe:
        """Load ``name``, validate its footprint, and re-anchor it.

        ``scale`` overrides the authored scale; ``fit`` (when no override is
        given) picks the largest scale that keeps the pen-down path inside
        both the extent and the working area. Raises ``RecipeLoadError`` on
        anything that must not execute.
        """
        path = self._resolve(name)
        doc, coords, defaults, segments = self._parse(path)
        if coords != 'relative':
            raise RecipeLoadError(
                f"{name}: only 'relative' recipes can run on the drawing "
                f"plane (got coordinates: '{coords}')")

        rotate = float(doc.get('rotate', 0.0))
        try:
            unit_points = self._sample(segments, defaults, rotate)
        except (ValueError, TypeError, KeyError) as error:
            raise RecipeLoadError(f'{name}: {error}') from error

        pen_down = [point for point in unit_points if point[2] < plane.safe_z]
        fit_scale = self._fit_scale(pen_down, extent,
                                    working_centre, working_radius)
        if scale is not None:
            effective = float(scale)
        elif fit:
            if fit_scale is None:
                raise RecipeLoadError(
                    f'{name}: no scale can fit the pen-down path inside the '
                    'working area')
            effective = fit_scale
        else:
            effective = float(doc.get('scale', 1.0))

        notes: List[str] = []
        polylines = self._pen_down_polylines(unit_points, effective,
                                             plane.safe_z)
        depths = [z for _x, _y, z in pen_down]
        self._validate(name, polylines, depths, plane, extent, working_centre,
                       working_radius, effective, fit_scale, notes)

        executable = dict(doc)
        executable['anchor'] = {'xyz': list(plane.effective_anchor()),
                                'rpy': list(plane.anchor_rpy)}
        # REPLACED, never composed with what the file carries. The
        # recipe's own `rotate` is the artwork's rotation and is passed through
        # untouched; `plane_rotate` is where the board sits, and the sender adds
        # the two. Summing them here would make a drawing saved under one plane
        # rotation and reloaded under another come out at their sum.
        executable['plane_rotate'] = plane.plane_yaw_deg
        executable['ready'] = list(plane.ready_joints)
        executable['scale'] = effective
        return LoadedRecipe(name=name, doc=executable,
                            polylines=self._decimate(polylines),
                            scale=effective, notes=notes)

    # ------------------------------------------------------------- resolution
    def _resolve(self, name: str) -> str:
        """Find ``name`` in the directories; refuse anything path-like
        (discipline, mirroring ``save_recipe``)."""
        if not name or os.sep in name or (os.altsep and os.altsep in name) \
                or '..' in name:
            raise RecipeLoadError(f'recipe name {name!r} is not a plain name')
        filename = name if name.endswith('.yaml') else f'{name}.yaml'
        for directory in self._directories:
            path = os.path.join(directory, filename)
            if os.path.isfile(path):
                return path
        raise RecipeLoadError(f'recipe {name!r} not found')

    def _parse(self, path: str):
        try:
            return self._geometry.load_recipe(path)
        except (OSError, ValueError, yaml.YAMLError) as error:
            raise RecipeLoadError(
                f'{os.path.basename(path)}: {error}') from error

    # --------------------------------------------------------------- geometry
    def _sample(self, segments, defaults, rotate_deg: float):
        """Every segment's local points at scale 1, rotation applied -- the
        in-plane transform of the sender's ``build_waypoints``, minus the
        scale (isotropic, so it commutes with the rotation and is applied
        later per candidate scale)."""
        theta = math.radians(rotate_deg)
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        points = []
        for index, segment in enumerate(segments):
            if not isinstance(segment, dict) or 'type' not in segment:
                raise ValueError(
                    f"segment {index}: must be a mapping with a 'type'")
            for x, y, z in self._geometry.segment_local_points(
                    segment, index, defaults):
                points.append((x * cos_t - y * sin_t,
                               x * sin_t + y * cos_t, z))
        return points

    @staticmethod
    def _pen_down_polylines(unit_points, scale: float,
                            safe_z: float) -> List[Polyline]:
        """Consecutive below-travel samples form one polyline; a sample at or
        above the travel height splits (framing read backwards)."""
        polylines: List[Polyline] = []
        current: Polyline = []
        for x, y, z in unit_points:
            if z < safe_z:
                current.append((x * scale, y * scale))
            elif current:
                polylines.append(current)
                current = []
        if current:
            polylines.append(current)
        return polylines

    @staticmethod
    def _fit_scale(pen_down, extent: Extent, working_centre: Coordinate,
                   working_radius: float) -> Optional[float]:
        """Largest scale keeping every pen-down point (sampled at scale 1)
        inside the extent rectangle and the working-area disc; None when no
        positive scale can satisfy the working area."""
        (x_min, y_min), (x_max, y_max) = extent
        centre_x, centre_y = working_centre
        best = math.inf
        for x, y, _z in pen_down:
            for coordinate, low, high in ((x, x_min, x_max),
                                          (y, y_min, y_max)):
                if coordinate > 0.0:
                    best = min(best, high / coordinate)
                elif coordinate < 0.0:
                    best = min(best, low / coordinate)
            # Largest s with |s*p - c| <= R: the positive root of
            # |p|^2 s^2 - 2(p.c) s + |c|^2 - R^2 = 0.
            radial = x * x + y * y
            if radial <= 0.0:
                continue
            half_b = -(x * centre_x + y * centre_y)
            constant = centre_x ** 2 + centre_y ** 2 - working_radius ** 2
            discriminant = half_b * half_b - radial * constant
            if discriminant < 0.0:
                return None
            root = (-half_b + math.sqrt(discriminant)) / radial
            if root <= 0.0:
                return None
            best = min(best, root)
        if not math.isfinite(best):
            return 1.0  # nothing constrains the scale (no or origin-only points)
        # Shrink by a hair (0.1 um at 0.1 m) so the boundary-binding point
        # cannot land outside its own bound through rounding.
        best *= 1.0 - 1e-6
        return best if best > 0.0 else None

    def _validate(self, name: str, polylines: List[Polyline],
                  depths: List[float], plane: PlaneConfig, extent: Extent,
                  working_centre: Coordinate, working_radius: float,
                  effective: float, fit_scale: Optional[float],
                  notes: List[str]) -> None:
        (x_min, y_min), (x_max, y_max) = extent
        centre_x, centre_y = working_centre
        outside_extent = 0
        for polyline in polylines:
            for x, y in polyline:
                if math.hypot(x - centre_x, y - centre_y) > working_radius:
                    hint = (f'; largest fitting scale is {fit_scale:.4g}'
                            if fit_scale else '')
                    raise RecipeLoadError(
                        f'{name}: pen-down path leaves the working area '
                        f'(centre {working_centre}, radius {working_radius}) '
                        f'at scale {effective:.4g}{hint}')
                if not (x_min <= x <= x_max and y_min <= y <= y_max):
                    outside_extent += 1
        if outside_extent:
            notes.append(
                f'{name}: {outside_extent} pen-down point(s) extend beyond '
                'the canvas extent; the preview is clipped at the canvas edge')
        if not polylines:
            notes.append(f'{name}: the recipe has no pen-down path below the '
                         'travel height')
            return
        if depths and min(depths) < plane.draw_z:
            notes.append(
                f'{name}: recipe draws to z {min(depths):.4g}, below the '
                f'configured draw_z {plane.draw_z:.4g} -- verify pen '
                'clearance')

    # ------------------------------------------------------------- decimation
    @staticmethod
    def _decimate(polylines: List[Polyline]) -> List[Polyline]:
        """Bound the preview payload: stride-decimate, endpoints kept. The
        executed doc is never decimated -- this is display-only."""
        total = sum(len(polyline) for polyline in polylines)
        budget = min(PREVIEW_TOTAL_CAP, PREVIEW_POLYLINE_CAP * len(polylines))
        result = []
        for polyline in polylines:
            cap = PREVIEW_POLYLINE_CAP
            if total > PREVIEW_TOTAL_CAP:
                cap = max(2, budget * len(polyline) // total)
            if len(polyline) <= cap:
                result.append(list(polyline))
                continue
            stride = -(-len(polyline) // cap)  # ceiling division
            kept = list(polyline[::stride])
            if kept[-1] != polyline[-1]:
                kept.append(polyline[-1])
            result.append(kept)
        return result
