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

"""Unit tests for the batch executor's seam with the streamed Cartesian sender.

The executor drives the sender through ``run_streamed``, the IN-PROCESS entry
point, which takes an argument dict already built. The sender's own resolution
of a recipe's top-level keys (``scale``, ``rotate``, ``plane_rotate``) runs in
its CLI path, which this route never enters -- so every key the sender reads has
to be put into that dict here, and nothing else in the suite checks it.

This file exists because that seam broke on 2026-09-11: ``plane_rotate``
 reached ``build_waypoints`` as the ``None`` sentinel ``parse_args``
seeded, and every batch drawing died with ``unsupported operand type(s) for +:
'float' and 'NoneType'`` after the arm had already gone to the pen-down height.
The geometry tests all passed, because they called ``build_waypoints``
themselves rather than through the dict the executor builds.
"""

import importlib.util
import math
import os
import types

import pytest

from rapidcode_draw_plane import file_manager
from rapidcode_draw_plane.batch_executor import BatchExecutor

_HERE = os.path.dirname(os.path.abspath(__file__))
_GEOMETRY_PATH = os.path.normpath(os.path.join(
    _HERE, '..', '..', 'rapidcode_bringup', 'scripts',
    'send_cartesian_path.py'))


def _load_geometry():
    if not os.path.isfile(_GEOMETRY_PATH):
        pytest.skip(f'geometry module not found at {_GEOMETRY_PATH}')
    spec = importlib.util.spec_from_file_location('send_cartesian_path',
                                                  _GEOMETRY_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


GEOMETRY = _load_geometry()

PLANE = file_manager.PlaneConfig(
    anchor_xyz=(-0.46, 0.0, 0.271),
    anchor_rpy=(math.pi, 0.0, math.pi),
    ready_joints=(0.0, 0.0, -1.5708, 0.0, -1.5708, 0.0),
    draw_z=-0.02,
    safe_z=0.0,
    velocity=0.2,
    acceleration=0.2,
    eef_step=0.004,
)
# A plus sign: two crossing strokes, the drawing that found the defect.
PLUS = [[(-0.03, 0.0), (0.03, 0.0)], [(0.0, -0.03), (0.0, 0.03)]]


class FakeSender:
    """Stands in for the streamed sender, but resolves the recipe through the
    REAL geometry exactly as ``prepare_split`` does -- so a key the executor
    forgot to put in ``args`` fails here the way it fails on the arm."""

    def __init__(self):
        self.monolithic = GEOMETRY
        self.waypoints = None
        self.args = None

    def run_streamed(self, options, args, doc, coords, defaults, segments,
                     node=None):
        self.args = dict(args)
        start_pos, start_quat = self.monolithic.anchor_pose(doc['anchor'])
        self.waypoints = self.monolithic.build_waypoints(
            segments, start_pos, start_quat, coords, defaults,
            args['scale'], args['rotate'], args['plane_rotate'])
        return 0


def make_executor(sender):
    """A BatchExecutor with only what ``_execute_structure`` touches.

    The real constructor imports rclpy and creates nodes and action clients;
    this method needs none of that, and going through __init__ would make the
    test a ROS test for no gain.
    """
    executor = object.__new__(BatchExecutor)
    executor._sender = sender
    executor._goal_seconds = 4.0
    executor._max_inflight = 2
    executor._work_node = None
    executor._clear_latch = lambda: None
    return executor


class TestSenderArguments:
    def test_a_plain_drawing_executes(self):
        # The regression: this raised TypeError once `plane_rotate` existed.
        sender = FakeSender()
        structure = file_manager.to_recipe_structure(PLUS, PLANE)
        ok, message = make_executor(sender)._execute_structure(structure)
        assert ok, message
        assert len(sender.waypoints) > 4

    def test_every_key_the_sender_reads_is_resolved_to_a_number(self):
        # A None sentinel reaches arithmetic and kills the execution after the
        # arm has already descended, so assert the type, not just the presence.
        sender = FakeSender()
        structure = file_manager.to_recipe_structure(PLUS, PLANE)
        make_executor(sender)._execute_structure(structure)
        for key in ('eef_step', 'scale', 'rotate', 'plane_rotate'):
            assert isinstance(sender.args[key], float), key

    def test_the_plane_rotation_reaches_the_sender(self):
        sender = FakeSender()
        structure = file_manager.to_recipe_structure(
            PLUS, PLANE._replace(plane_yaw_deg=45.0))
        make_executor(sender)._execute_structure(structure)
        assert sender.args['plane_rotate'] == 45.0

    def test_the_plane_rotation_turns_the_executed_waypoints(self):
        turned = FakeSender()
        flat = FakeSender()
        make_executor(turned)._execute_structure(
            file_manager.to_recipe_structure(
                PLUS, PLANE._replace(plane_yaw_deg=90.0)))
        make_executor(flat)._execute_structure(
            file_manager.to_recipe_structure(PLUS, PLANE))
        # Plane +x is world +y after a quarter turn, about the plane origin.
        first_flat = flat.waypoints[0][0]
        first_turned = turned.waypoints[0][0]
        assert first_flat[:2] == pytest.approx((-0.49, 0.0))
        assert first_turned[:2] == pytest.approx((-0.46, -0.03))

    def test_a_structure_without_the_key_defaults_to_no_rotation(self):
        # A recipe written before the rotation field existed, or any hand-authored file.
        sender = FakeSender()
        structure = file_manager.to_recipe_structure(PLUS, PLANE)
        del structure['plane_rotate']
        ok, message = make_executor(sender)._execute_structure(structure)
        assert ok, message
        assert sender.args['plane_rotate'] == 0.0

    def test_an_empty_structure_is_refused_before_the_sender(self):
        sender = FakeSender()
        ok, message = make_executor(sender)._execute_structure(
            file_manager.to_recipe_structure([], PLANE))
        assert not ok and 'empty' in message
        assert sender.waypoints is None
