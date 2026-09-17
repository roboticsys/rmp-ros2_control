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

"""Unit tests for the file manager (recipe structure + recipe YAML output)."""

import math

import pytest
import yaml

from rapidcode_draw_plane import file_manager

CONFIG = file_manager.PlaneConfig(
    anchor_xyz=(-0.4, 0.15, 0.271),
    anchor_rpy=(math.pi, 0.0, math.pi),
    ready_joints=(0.0, 0.0, -1.5708, 0.0, -1.5708, 0.0),
    draw_z=-0.02,
    safe_z=0.0,
    velocity=0.2,
    acceleration=0.2,
    eef_step=0.004,
)


class TestPlaneOffset:
    """The operator's calibration says where the board actually sits.

    It is a property of the plane, not of the drawing, so it moves the anchor
    and leaves every segment in plane coordinates.
    """

    def test_no_offset_is_the_configured_anchor(self):
        assert CONFIG.effective_anchor() == CONFIG.anchor_xyz

    def test_offset_translates_x_and_y_and_leaves_z(self):
        moved = CONFIG._replace(plane_offset=(0.01, -0.02))
        assert moved.effective_anchor() == pytest.approx((-0.39, 0.13, 0.271))

    def test_structure_carries_the_effective_anchor(self):
        # A saved recipe replays standalone where it executed, so
        # the calibration belongs in the file's own anchor.
        moved = CONFIG._replace(plane_offset=(0.01, -0.02))
        doc = file_manager.to_recipe_structure(
            [[(0.0, 0.0), (0.05, 0.0)]], moved)
        assert doc['anchor']['xyz'] == pytest.approx([-0.39, 0.13, 0.271])
        assert doc['anchor']['rpy'] == list(CONFIG.anchor_rpy)

    def test_segments_stay_in_plane_coordinates(self):
        # The offset moves the frame, not the drawing inside it: the same
        # drawing conditioned the same way yields byte-identical segments.
        moved = CONFIG._replace(plane_offset=(0.01, -0.02))
        run = [(0.0, 0.0), (0.01, 0.0), (0.02, 0.01)]
        assert (file_manager.to_recipe_structure([run], moved)['segments'] ==
                file_manager.to_recipe_structure([run], CONFIG)['segments'])


class TestPlaneRotation:
    """The other half of the calibration -- how far the board is
    turned. Like the offset it is a property of the plane, not of the drawing,
    so it leaves every segment in plane coordinates.
    """

    def test_the_default_is_no_rotation(self):
        assert CONFIG.plane_yaw_deg == 0.0
        doc = file_manager.to_recipe_structure([[(0.0, 0.0), (0.05, 0.0)]],
                                               CONFIG)
        assert doc['plane_rotate'] == 0.0

    def test_the_structure_carries_the_rotation(self):
        # A saved drawing replays standalone where it executed, so
        # the rotation belongs in the file the same way the anchor does.
        turned = CONFIG._replace(plane_yaw_deg=45.0)
        doc = file_manager.to_recipe_structure(
            [[(0.0, 0.0), (0.05, 0.0)]], turned)
        assert doc['plane_rotate'] == 45.0

    def test_a_drawing_carries_no_authored_rotation_of_its_own(self):
        # The sender ADDS `rotate` to `plane_rotate`. A freehand drawing has
        # no artwork rotation, so the key must stay absent rather than arrive
        # as a second copy of the plane's.
        turned = CONFIG._replace(plane_yaw_deg=45.0)
        doc = file_manager.to_recipe_structure(
            [[(0.0, 0.0), (0.05, 0.0)]], turned)
        assert 'rotate' not in doc

    def test_the_rotation_never_moves_the_anchor(self):
        # It turns points ABOUT the anchor, so folding it into the anchor
        # would be the one wrong place to put it.
        turned = CONFIG._replace(plane_yaw_deg=90.0)
        assert turned.effective_anchor() == CONFIG.anchor_xyz

    def test_the_rotation_composes_with_the_offset_independently(self):
        both = CONFIG._replace(plane_offset=(0.01, -0.02), plane_yaw_deg=30.0)
        assert both.effective_anchor() == pytest.approx((-0.39, 0.13, 0.271))
        doc = file_manager.to_recipe_structure(
            [[(0.0, 0.0), (0.05, 0.0)]], both)
        assert doc['plane_rotate'] == 30.0
        assert doc['anchor']['xyz'] == pytest.approx([-0.39, 0.13, 0.271])

    def test_segments_stay_in_plane_coordinates(self):
        # The rotation turns the frame, not the drawing inside it: the same
        # drawing yields byte-identical segments however the plane is turned.
        turned = CONFIG._replace(plane_yaw_deg=45.0)
        run = [(0.0, 0.0), (0.01, 0.0), (0.02, 0.01)]
        assert (file_manager.to_recipe_structure([run], turned)['segments'] ==
                file_manager.to_recipe_structure([run], CONFIG)['segments'])


class TestToRecipeStructure:
    def test_run_is_framed_travel_approach_draw_retract(self):
        run = [(0.0, 0.0), (0.01, 0.0), (0.02, 0.01), (0.03, 0.01)]
        doc = file_manager.to_recipe_structure([run], CONFIG)
        segments = doc['segments']
        assert segments[0] == {'type': 'lin', 'xyz': [0.0, 0.0, 0.0]}     # travel
        assert segments[1] == {'type': 'lin', 'xyz': [0.0, 0.0, -0.02]}   # approach
        assert segments[2]['type'] == 'through'                            # draw
        assert segments[2]['z'] == -0.02
        assert segments[2]['closed'] is False
        assert segments[2]['samples'] == 4
        assert segments[2]['points'] == [[x, y] for x, y in run]
        assert segments[3] == {'type': 'lin', 'xyz': [0.03, 0.01, 0.0]}   # lift
        # the drawing ends risen to the retract height.
        assert segments[4] == {'type': 'lin', 'xyz': [0.03, 0.01, CONFIG.retract_z]}
        assert len(segments) == 5

    def test_short_runs_draw_as_lin_segments(self):
        doc = file_manager.to_recipe_structure([[(0.0, 0.0), (0.05, 0.0)]], CONFIG)
        types = [seg['type'] for seg in doc['segments']]
        # travel, approach, draw, lift, final retract
        assert types == ['lin', 'lin', 'lin', 'lin', 'lin']
        assert doc['segments'][2]['xyz'] == [0.05, 0.0, -0.02]
        assert doc['segments'][4]['xyz'] == [0.05, 0.0, CONFIG.retract_z]

    def test_header_matches_sender_schema(self):
        doc = file_manager.to_recipe_structure([], CONFIG)
        assert doc['coordinates'] == 'relative'
        assert doc['anchor'] == {'xyz': [-0.4, 0.15, 0.271],
                                 'rpy': [math.pi, 0.0, math.pi]}
        assert doc['ready'] == [0.0, 0.0, -1.5708, 0.0, -1.5708, 0.0]
        assert doc['defaults']['z'] == -0.02
        assert doc['defaults']['vel'] == 0.2
        assert doc['eef_step'] == 0.004
        assert doc['segments'] == []

    def test_runs_keep_drawing_order(self):
        runs = [[(0.0, 0.0), (0.01, 0.0)], [(0.1, 0.1), (0.11, 0.1)]]
        doc = file_manager.to_recipe_structure(runs, CONFIG)
        travels = [seg for seg in doc['segments']
                   if seg['type'] == 'lin' and seg['xyz'][2] == CONFIG.safe_z][::2]
        assert travels[0]['xyz'][:2] == [0.0, 0.0]
        assert travels[1]['xyz'][:2] == [0.1, 0.1]

    def test_final_retract_is_above_every_travel(self):
        runs = [[(0.0, 0.0), (0.01, 0.0)], [(0.1, 0.1), (0.11, 0.1)]]
        doc = file_manager.to_recipe_structure(runs, CONFIG)
        last = doc['segments'][-1]
        assert last['xyz'][2] == CONFIG.retract_z
        assert all(seg['xyz'][2] < CONFIG.retract_z
                   for seg in doc['segments'][:-1] if seg['type'] == 'lin')

    def test_draw_above_safe_refused(self):
        bad = CONFIG._replace(draw_z=0.01)
        with pytest.raises(ValueError):
            file_manager.to_recipe_structure([], bad)


class TestSaveRecipe:
    def test_round_trips_through_yaml(self, tmp_path):
        doc = file_manager.to_recipe_structure([[(0.0, 0.0), (0.01, 0.0), (0.02, 0.0)]],
                                               CONFIG)
        path = file_manager.save_recipe(doc, 'demo drawing', str(tmp_path))
        assert path.endswith('recipe_demo_drawing.yaml')
        with open(path, encoding='utf-8') as handle:
            loaded = yaml.safe_load(handle)
        assert loaded == doc

    def test_unsafe_names_are_sanitized(self, tmp_path):
        doc = file_manager.to_recipe_structure([], CONFIG)
        path = file_manager.save_recipe(doc, '../../etc/passwd', str(tmp_path))
        assert path == str(tmp_path / 'recipe_etc_passwd.yaml')
        with pytest.raises(ValueError):
            file_manager.save_recipe(doc, '///', str(tmp_path))
