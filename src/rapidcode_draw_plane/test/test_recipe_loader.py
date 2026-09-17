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

"""Unit tests for the recipe loader (bridge-side recipe files into batch).

The geometry dependency is the real ``send_cartesian_path.py``, imported by
file path from the sibling ``rapidcode_bringup`` package (its top-level
imports are math/os/sys/numpy/yaml only, so no ROS is needed). Tests skip if
the sibling package is absent (an unusual checkout).
"""

import importlib.util
import math
import os
import shutil

import pytest
import yaml

from rapidcode_draw_plane import file_manager
from rapidcode_draw_plane.live_streamer import plane_to_world
from rapidcode_draw_plane.recipe_loader import (
    PREVIEW_POLYLINE_CAP, PREVIEW_TOTAL_CAP, RecipeLoader, RecipeLoadError)

_HERE = os.path.dirname(os.path.abspath(__file__))
_GEOMETRY_PATH = os.path.normpath(os.path.join(
    _HERE, '..', '..', 'rapidcode_bringup', 'scripts',
    'send_cartesian_path.py'))
_LOGO_PATH = os.path.normpath(os.path.join(
    _HERE, '..', '..', 'rapidcode_bringup', 'config',
    'recipe_rsi_logo_svg.yaml'))


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
EXTENT = ((-0.11, -0.11), (0.11, 0.11))
CENTRE = (0.0, 0.0)
RADIUS = 0.16

# Two pen-down runs separated by a travel lin at the safe height.
SYNTHETIC = {
    'coordinates': 'relative',
    'ready': [0.1] * 6,
    'anchor': {'xyz': [-0.39, -0.13, 0.271], 'rpy': [math.pi, 0.0, math.pi]},
    'defaults': {'z': -0.02, 'vel': 0.3, 'acc': 0.3, 'samples': 8},
    'eef_step': 0.003,
    'scale': 0.5,
    'segments': [
        {'type': 'lin', 'xyz': [0.0, 0.0, 0.0]},
        {'type': 'lin', 'xyz': [0.0, 0.0, -0.02]},
        {'type': 'lin', 'xyz': [0.05, 0.0, -0.02]},
        {'type': 'lin', 'xyz': [0.05, 0.0, 0.0]},      # travel: splits runs
        {'type': 'lin', 'xyz': [0.05, 0.05, -0.02]},   # plunge starts run 2
        {'type': 'through', 'z': -0.02, 'samples': 12,
         'points': [[0.05, 0.05], [0.06, 0.06], [0.08, 0.05]]},
        {'type': 'lin', 'xyz': [0.08, 0.05, 0.12]},
    ],
}


def write_recipe(directory, name, doc):
    path = os.path.join(str(directory), f'{name}.yaml')
    with open(path, 'w', encoding='utf-8') as handle:
        yaml.safe_dump(doc, handle, sort_keys=False)
    return path


@pytest.fixture
def recipe_dir(tmp_path):
    write_recipe(tmp_path, 'recipe_synthetic', SYNTHETIC)
    return tmp_path


def make_loader(*directories):
    return RecipeLoader(GEOMETRY, [str(d) for d in directories])


def load(loader, name='recipe_synthetic', **kwargs):
    return loader.load(name, PLANE, EXTENT, CENTRE, RADIUS, **kwargs)


class TestListNames:
    def test_lists_yaml_basenames_sorted(self, tmp_path):
        write_recipe(tmp_path, 'recipe_b', SYNTHETIC)
        write_recipe(tmp_path, 'recipe_a', SYNTHETIC)
        (tmp_path / 'notes.txt').write_text('not a recipe')
        assert make_loader(tmp_path).list_names() == ['recipe_a', 'recipe_b']

    def test_merges_directories_and_tolerates_missing(self, tmp_path):
        first = tmp_path / 'first'
        second = tmp_path / 'second'
        first.mkdir()
        second.mkdir()
        write_recipe(first, 'recipe_shared', SYNTHETIC)
        write_recipe(second, 'recipe_shared', SYNTHETIC)
        write_recipe(second, 'recipe_extra', SYNTHETIC)
        loader = make_loader(first, second, tmp_path / 'missing')
        assert loader.list_names() == ['recipe_extra', 'recipe_shared']


class TestLoadVerbatim:
    def test_segments_and_motion_parameters_are_untouched(self, recipe_dir):
        loaded = load(make_loader(recipe_dir))
        assert loaded.doc['segments'] == SYNTHETIC['segments']
        assert loaded.doc['defaults'] == SYNTHETIC['defaults']
        assert loaded.doc['eef_step'] == SYNTHETIC['eef_step']
        assert loaded.doc['coordinates'] == 'relative'

    def test_anchor_and_ready_are_replaced_with_plane_values(self, recipe_dir):
        loaded = load(make_loader(recipe_dir))
        assert loaded.doc['anchor'] == {'xyz': list(PLANE.anchor_xyz),
                                        'rpy': list(PLANE.anchor_rpy)}
        assert loaded.doc['ready'] == list(PLANE.ready_joints)

    def test_authored_scale_is_kept_without_override(self, recipe_dir):
        loaded = load(make_loader(recipe_dir))
        assert loaded.scale == SYNTHETIC['scale']
        assert loaded.doc['scale'] == SYNTHETIC['scale']

    def test_manual_scale_overrides_authored(self, recipe_dir):
        loaded = load(make_loader(recipe_dir), scale=0.25)
        assert loaded.scale == 0.25
        assert loaded.doc['scale'] == 0.25

    def test_first_directory_shadows_the_second(self, tmp_path):
        first = tmp_path / 'first'
        second = tmp_path / 'second'
        first.mkdir()
        second.mkdir()
        shadowing = dict(SYNTHETIC, scale=0.123)
        write_recipe(first, 'recipe_synthetic', shadowing)
        write_recipe(second, 'recipe_synthetic', SYNTHETIC)
        loaded = load(make_loader(first, second))
        assert loaded.scale == 0.123


class TestPolylines:
    def test_travel_splits_and_plunge_joins_runs(self, recipe_dir):
        loaded = load(make_loader(recipe_dir))
        assert len(loaded.polylines) == 2
        # Run 2 starts at the plunge and carries the spline's own samples.
        spline = GEOMETRY.segment_local_points(
            SYNTHETIC['segments'][5], 5, dict(GEOMETRY.DEFAULTS,
                                              **SYNTHETIC['defaults']))
        assert len(loaded.polylines[1]) == 1 + len(spline)

    def test_polylines_are_scaled(self, recipe_dir):
        loaded = load(make_loader(recipe_dir), scale=0.5)
        assert loaded.polylines[0][-1] == pytest.approx((0.025, 0.0))

    def test_decimation_caps_points_and_keeps_endpoints(self, tmp_path):
        dense = dict(SYNTHETIC, scale=0.5)
        dense['segments'] = [
            {'type': 'lin', 'xyz': [0.1, 0.0, -0.02]},
            {'type': 'func', 't': [0.0, 2.0 * math.pi],
             'x': '0.1*cos(t)', 'y': '0.1*sin(t)', 'z': -0.02,
             'samples': 4000},
        ]
        write_recipe(tmp_path, 'recipe_dense', dense)
        loaded = load(make_loader(tmp_path), name='recipe_dense')
        total = sum(len(line) for line in loaded.polylines)
        assert total <= PREVIEW_TOTAL_CAP + len(loaded.polylines)
        line = loaded.polylines[0]
        assert len(line) <= PREVIEW_POLYLINE_CAP + 1
        assert line[0] == pytest.approx((0.05, 0.0))
        assert line[-1] == pytest.approx((0.05, 0.0), abs=1e-6)


class TestFit:
    def test_fit_on_the_logo_matches_the_extent_bound(self, tmp_path):
        if not os.path.isfile(_LOGO_PATH):
            pytest.skip(f'logo recipe not found at {_LOGO_PATH}')
        shutil.copy(_LOGO_PATH, os.path.join(str(tmp_path),
                                             'recipe_rsi_logo_svg.yaml'))
        loaded = load(make_loader(tmp_path), name='recipe_rsi_logo_svg',
                      fit=True)
        # The circle (R = 0.14375) binds on the extent half-width before the
        # working radius: fit = 0.11 / 0.14375.
        assert loaded.scale == pytest.approx(0.11 / 0.14375, rel=1e-3)
        assert loaded.doc['scale'] == loaded.scale
        assert not any('working area' in note for note in loaded.notes)

    def test_fit_respects_the_working_radius(self, recipe_dir):
        tight = 0.03
        loaded = make_loader(recipe_dir).load(
            'recipe_synthetic', PLANE, EXTENT, CENTRE, tight, fit=True)
        reach = max(math.hypot(x, y) for line in loaded.polylines
                    for x, y in line)
        assert reach <= tight + 1e-9


class TestRefusals:
    def test_unknown_name(self, recipe_dir):
        with pytest.raises(RecipeLoadError, match='not found'):
            load(make_loader(recipe_dir), name='recipe_missing')

    @pytest.mark.parametrize('name', ['../evil', 'a/b', '', 'recipe..yaml'])
    def test_path_like_names(self, recipe_dir, name):
        with pytest.raises(RecipeLoadError, match='plain name|not found'):
            load(make_loader(recipe_dir), name=name)

    def test_absolute_coordinates(self, tmp_path):
        write_recipe(tmp_path, 'recipe_abs',
                     dict(SYNTHETIC, coordinates='absolute'))
        with pytest.raises(RecipeLoadError, match='relative'):
            load(make_loader(tmp_path), name='recipe_abs')

    def test_empty_segments(self, tmp_path):
        write_recipe(tmp_path, 'recipe_empty', dict(SYNTHETIC, segments=[]))
        with pytest.raises(RecipeLoadError, match='segments'):
            load(make_loader(tmp_path), name='recipe_empty')

    def test_unparseable_yaml(self, tmp_path):
        path = os.path.join(str(tmp_path), 'recipe_bad.yaml')
        with open(path, 'w', encoding='utf-8') as handle:
            handle.write('{unbalanced: [')
        with pytest.raises(RecipeLoadError):
            load(make_loader(tmp_path), name='recipe_bad')

    def test_working_area_violation_is_refused_with_a_fit_hint(
            self, recipe_dir):
        with pytest.raises(RecipeLoadError,
                           match='working area.*largest fitting scale'):
            make_loader(recipe_dir).load(
                'recipe_synthetic', PLANE, EXTENT, CENTRE, 0.01, scale=1.0)


class TestNotes:
    def test_extent_overrun_warns_but_loads(self, recipe_dir):
        # 0.12 m reach: past the 0.11 extent, inside the 0.16 working radius.
        loaded = load(make_loader(recipe_dir), scale=1.5)
        assert any('canvas extent' in note for note in loaded.notes)
        assert loaded.scale == 1.5

    def test_depth_below_draw_z_warns(self, tmp_path):
        deep = dict(SYNTHETIC)
        deep['segments'] = list(SYNTHETIC['segments'])
        deep['segments'][2] = {'type': 'lin', 'xyz': [0.05, 0.0, -0.024]}
        write_recipe(tmp_path, 'recipe_deep', deep)
        loaded = load(make_loader(tmp_path), name='recipe_deep')
        assert any('draw_z' in note for note in loaded.notes)

    def test_clean_load_has_no_notes(self, recipe_dir):
        loaded = load(make_loader(recipe_dir))
        assert loaded.notes == []


class TestPlaneOffset:
    """across the two halves of the plane-to-world mapping."""

    def test_a_loaded_recipe_is_re_anchored_with_the_offset(self, recipe_dir):
        # The loader replaces the file's anchor unconditionally, so the
        # calibration lands once however the file was authored -- and a file
        # saved under a different calibration cannot double-count.
        plane = PLANE._replace(plane_offset=(0.01, -0.02))
        loaded = make_loader(recipe_dir).load(
            'recipe_synthetic', plane, EXTENT, CENTRE, RADIUS)
        assert loaded.doc['anchor']['xyz'] == pytest.approx(
            [-0.45, -0.02, 0.271])
        assert SYNTHETIC['anchor']['xyz'] == [-0.39, -0.13, 0.271]  # untouched

    @staticmethod
    def _routes_agree(plane):
        """World points for the same drawing down both routes.

        The mapping exists twice: `plane_to_world` on the live and stroke
        routes, and scale/rotate/`world_point` inside the sender's
        `build_waypoints` on the batch route. They are mirrored by hand, so a
        calibration that reached one and not the other would draw in two
        different places depending on the mode.
        """
        run = [(0.0, 0.0), (0.02, 0.01), (0.04, -0.015), (0.01, -0.02)]
        doc = file_manager.to_recipe_structure([run], plane)
        defaults = dict(GEOMETRY.DEFAULTS, **doc['defaults'])
        start_pos, start_quat = GEOMETRY.anchor_pose(doc['anchor'])

        batch = [position for position, _quaternion in GEOMETRY.build_waypoints(
            doc['segments'], start_pos, start_quat, doc['coordinates'],
            defaults, 1.0, doc.get('rotate', 0.0), doc['plane_rotate'])]
        locals_ = []
        for index, segment in enumerate(doc['segments']):
            locals_.extend(
                GEOMETRY.segment_local_points(segment, index, defaults))
        live = [plane_to_world(plane.effective_anchor(), plane.anchor_rpy,
                               point, plane.plane_yaw_deg)
                for point in locals_]
        return batch, live

    def test_the_batch_and_live_routes_land_on_the_same_world_point(self):
        """This is the test that catches the two mappings drifting apart."""
        batch, live = self._routes_agree(
            PLANE._replace(plane_offset=(0.013, -0.007)))
        assert len(batch) == len(live) > 4
        assert batch == pytest.approx(live)

    @pytest.mark.parametrize('degrees', [0.0, 15.0, 45.0, 90.0, -30.0, 180.0])
    def test_the_routes_agree_under_a_plane_rotation(self, degrees):
        """reaches both routes, at every angle. The live route turns
        the point in `linalg.rotate_in_plane`; the batch route turns it inside
        the sender. Two implementations, one required answer."""
        batch, live = self._routes_agree(
            PLANE._replace(plane_offset=(0.013, -0.007),
                           plane_yaw_deg=degrees))
        assert len(batch) == len(live) > 4
        assert batch == pytest.approx(live)

    def test_a_plane_rotation_actually_moves_the_drawing(self):
        # Guards the parametrized test above against passing because both
        # routes ignore the rotation identically.
        flat, _ = self._routes_agree(PLANE)
        turned, _ = self._routes_agree(PLANE._replace(plane_yaw_deg=90.0))
        assert flat != pytest.approx(turned)


class TestPlaneRotationInLoadedRecipes:
    """through the loader: `plane_rotate` is REPLACED, `rotate` is
    passed through, and the sender adds them."""

    ROTATED = dict(SYNTHETIC, rotate=30.0)

    @pytest.fixture
    def rotated_dir(self, tmp_path):
        write_recipe(tmp_path, 'recipe_rotated', self.ROTATED)
        return tmp_path

    def test_the_plane_rotation_is_written_into_the_loaded_doc(self,
                                                              recipe_dir):
        plane = PLANE._replace(plane_yaw_deg=45.0)
        loaded = make_loader(recipe_dir).load(
            'recipe_synthetic', plane, EXTENT, CENTRE, RADIUS)
        assert loaded.doc['plane_rotate'] == 45.0

    def test_the_authored_rotation_is_left_alone(self, rotated_dir):
        # The artwork's own rotation is the operator's, not the bridge's.
        plane = PLANE._replace(plane_yaw_deg=45.0)
        loaded = make_loader(rotated_dir).load(
            'recipe_rotated', plane, EXTENT, CENTRE, RADIUS)
        assert loaded.doc['rotate'] == 30.0
        assert loaded.doc['plane_rotate'] == 45.0

    def test_the_sender_adds_them(self, rotated_dir):
        # 30 on the artwork, 45 on the plane, 75 in the world. Both turn about
        # the same point, so composing them is exact addition.
        plane = PLANE._replace(plane_yaw_deg=45.0)
        loaded = make_loader(rotated_dir).load(
            'recipe_rotated', plane, EXTENT, CENTRE, RADIUS)
        defaults = dict(GEOMETRY.DEFAULTS, **loaded.doc['defaults'])
        start_pos, start_quat = GEOMETRY.anchor_pose(loaded.doc['anchor'])

        def world(rotate, plane_rotate):
            return [position for position, _q in GEOMETRY.build_waypoints(
                loaded.doc['segments'], start_pos, start_quat,
                loaded.doc['coordinates'], defaults, loaded.doc['scale'],
                rotate, plane_rotate)]

        assert world(30.0, 45.0) == pytest.approx(world(75.0, 0.0))

    def test_a_reload_replaces_the_plane_rotation_and_does_not_compose(
            self, tmp_path):
        # The failure this design exists to prevent: a drawing saved under one
        # plane rotation, reloaded under another, must land at the NEW one --
        # not at their sum. The loader replaces `plane_rotate` the same way it
        # replaces the anchor.
        saved = file_manager.to_recipe_structure(
            [[(0.0, 0.0), (0.05, 0.0)]], PLANE._replace(plane_yaw_deg=30.0))
        write_recipe(tmp_path, 'recipe_saved', saved)
        assert saved['plane_rotate'] == 30.0

        loaded = make_loader(tmp_path).load(
            'recipe_saved', PLANE._replace(plane_yaw_deg=45.0), EXTENT,
            CENTRE, RADIUS)
        assert loaded.doc['plane_rotate'] == 45.0
        assert 'rotate' not in loaded.doc

    def test_the_preview_ignores_the_plane_rotation(self, rotated_dir):
        # The canvas IS the plane frame, so it is board-fixed: turning the
        # board changes nothing about how the drawing sits on it. Folding the
        # plane rotation into the preview would turn the picture on screen and
        # mis-judge the extent fit.
        flat = make_loader(rotated_dir).load(
            'recipe_rotated', PLANE, EXTENT, CENTRE, RADIUS)
        turned = make_loader(rotated_dir).load(
            'recipe_rotated', PLANE._replace(plane_yaw_deg=45.0), EXTENT,
            CENTRE, RADIUS)
        assert flat.polylines == turned.polylines

    def test_the_bound_check_ignores_the_plane_rotation(self, rotated_dir):
        # Same reason: the working-area disc is plane-relative and turns with
        # the board, so the footprint that fits it is unchanged.
        flat = make_loader(rotated_dir).load(
            'recipe_rotated', PLANE, EXTENT, CENTRE, RADIUS, fit=True)
        turned = make_loader(rotated_dir).load(
            'recipe_rotated', PLANE._replace(plane_yaw_deg=45.0), EXTENT,
            CENTRE, RADIUS, fit=True)
        assert flat.scale == pytest.approx(turned.scale)
        assert flat.notes == turned.notes
