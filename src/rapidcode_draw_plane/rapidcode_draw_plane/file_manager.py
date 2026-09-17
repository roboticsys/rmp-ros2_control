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

"""File manager: drawings out to the recipe structure and recipe YAML files.

``to_recipe_structure`` converts an ordered drawing (in-area runs from the
dispatcher) into the recipe structure the streamed Cartesian sender executes
directly in memory -- the same structure its file layer parses from recipe YAML,
so a saved drawing replays standalone byte-for-byte the way it executed live
.

Framing per run: travel at the safe height to the run's start,
approach straight down to the drawing depth, draw the run at depth, retract
straight up to the safe height; after the final run, rise to the retract
height so a hand can reach the artwork.

The pen-adjustment park move is not framed here: it is a joint PTP to
the ready pose, commanded straight on the batch route, so it needs no recipe.

The anchor written into a structure is ``PlaneConfig.effective_anchor()``, not
the configured ``anchor_xyz``: it carries the operator's plane calibration
. A saved recipe therefore replays standalone where it executed
, and reloading it through the bridge re-anchors it anyway. The
plane's rotation rides in the structure's own ``plane_rotate`` key
for the same reason, and stays separate from a recipe's authored ``rotate``:
the sender adds the two, the bridge replaces only this one.
"""

import os
import re
from typing import List, NamedTuple, Sequence, Tuple

import yaml

Coordinate = Tuple[float, float]
Run = Sequence[Coordinate]


class PlaneConfig(NamedTuple):
    """The plane and motion configuration a recipe structure is framed with."""

    anchor_xyz: Tuple[float, float, float]
    anchor_rpy: Tuple[float, float, float]
    ready_joints: Tuple[float, ...]
    draw_z: float      # pen-down depth, relative to the anchor (negative = into the plane)
    safe_z: float      # travel height, relative to the anchor
    velocity: float
    acceleration: float
    eef_step: float
    retract_z: float = 0.12   # end-of-drawing height
    # Operator plane calibration: planning-frame x/y metres added to
    # the anchor, session-only. Every route maps through effective_anchor().
    plane_offset: Tuple[float, float] = (0.0, 0.0)
    # The rest of the calibration: how far the board itself is turned,
    # degrees CCW about the plane origin. Degrees, and named so, because the
    # neighbouring anchor_rpy is radians. Session-only, like the offset.
    plane_yaw_deg: float = 0.0

    def effective_anchor(self) -> Tuple[float, float, float]:
        """Where plane (0, 0, 0) actually sits: the configured anchor plus the
        operator's calibration offset.

        The rotation is NOT folded in here: it turns points about
        this anchor, so it has to be applied to the point, not to the origin
        the point is measured from. ``plane_to_world`` in the live streamer
        does that; the batch route hands ``plane_rotate`` to the sender."""
        return (self.anchor_xyz[0] + self.plane_offset[0],
                self.anchor_xyz[1] + self.plane_offset[1],
                self.anchor_xyz[2])


def to_recipe_structure(runs: Sequence[Run], plane_config: PlaneConfig) -> dict:
    """Frame the drawing's runs into the sender's recipe structure.

    ``runs`` are conditioned, in-area plane runs in drawing order. Runs of one
    or two points draw as straight ``lin`` segments; longer runs draw as one
    ``through`` spline sampled at their own point count.
    """
    if plane_config.draw_z >= plane_config.safe_z:
        raise ValueError(
            f'draw_z ({plane_config.draw_z}) must lie below safe_z '
            f'({plane_config.safe_z})')
    segments = []
    for run in runs:
        if not run:
            continue
        x0, y0 = run[0]
        segments.append({'type': 'lin', 'xyz': [x0, y0, plane_config.safe_z]})
        segments.append({'type': 'lin', 'xyz': [x0, y0, plane_config.draw_z]})
        if len(run) >= 3:
            segments.append({
                'type': 'through',
                'closed': False,
                'z': plane_config.draw_z,
                'samples': len(run),
                'points': [[x, y] for x, y in run],
            })
        else:
            for x, y in run[1:]:
                segments.append({'type': 'lin', 'xyz': [x, y, plane_config.draw_z]})
        xn, yn = run[-1]
        segments.append({'type': 'lin', 'xyz': [xn, yn, plane_config.safe_z]})
    if segments:
        # Retract after the final stroke: the finished pose leaves
        # room for a hand, not just pen clearance.
        xn, yn = segments[-1]['xyz'][0], segments[-1]['xyz'][1]
        segments.append({'type': 'lin', 'xyz': [xn, yn, plane_config.retract_z]})
    return {
        'coordinates': 'relative',
        'ready': list(plane_config.ready_joints),
        'anchor': {
            'xyz': list(plane_config.effective_anchor()),
            'rpy': list(plane_config.anchor_rpy),
        },
        # The plane's own rotation. A drawing carries no authored
        # `rotate` of its own, so this is its whole in-plane rotation; the
        # sender adds the two and this one stands alone.
        'plane_rotate': plane_config.plane_yaw_deg,
        'defaults': {
            'z': plane_config.draw_z,
            'vel': plane_config.velocity,
            'acc': plane_config.acceleration,
            'samples': 120,
        },
        'eef_step': plane_config.eef_step,
        'segments': segments,
    }


def save_recipe(structure: dict, name: str, directory: str) -> str:
    """Write the recipe structure as recipe YAML; return the written path.

    ``name`` is the operator-supplied (or dispatcher-defaulted) drawing name; it
    becomes ``recipe_<name>.yaml`` with unsafe filename characters replaced, so
    an operator name can never escape the drawing file directory (
    ).
    """
    safe = re.sub(r'[^A-Za-z0-9_-]+', '_', name).strip('_')
    if not safe:
        raise ValueError(f'name {name!r} leaves no usable filename')
    path = os.path.join(directory, f'recipe_{safe}.yaml')
    with open(path, 'w', encoding='utf-8') as handle:
        yaml.safe_dump(structure, handle, sort_keys=False, default_flow_style=None)
    return path
