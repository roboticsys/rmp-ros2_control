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

"""Unit tests for the drawing surface client's view-model (no tkinter, no
sockets). The client owns the drawing: capture, conditioning,
primitives, undo, and clear happen here, and execute/save queue controls
carrying the submitted drawing."""

import math

import pytest

from rapidcode_draw_plane import linalg
from rapidcode_draw_plane.surface_client import SurfaceModel

EXTENT = [[-0.1, -0.1], [0.1, 0.1]]
SURFACE = (800.0, 800.0)


def display(mode='batch', executing=False, stopped=False, max_samples=10000,
            plane_offset=(0.0, 0.0), plane_yaw_deg=0.0):
    return {'mode': mode, 'executing': executing, 'stopped': stopped,
            'config': {'extent': EXTENT, 'fine_grid': 0.02,
                       'coarse_grid': 0.04, 'duplicate_threshold': 0.0005,
                       'resample_spacing': 0.002, 'max_samples': max_samples,
                       'plane_offset': list(plane_offset),
                       'plane_yaw_deg': plane_yaw_deg}}


def make_model(**kwargs):
    model = SurfaceModel()
    model.apply_display(display(**kwargs))
    model.set_surface(*SURFACE)
    return model


def drag(model, path):
    model.set_pointer(*path[0])
    model.set_pen(True)
    for point in path[1:]:
        model.set_pointer(*point)
    model.set_pen(False)


def click(model, pointer):
    model.set_pointer(*pointer)
    model.set_pen(True)
    model.set_pen(False)


class TestPlaneToCanvas:
    def test_round_trip(self):
        extent = (tuple(EXTENT[0]), tuple(EXTENT[1]))
        for canvas in [(0.0, 0.0), (400.0, 400.0), (123.0, 700.0)]:
            plane = linalg.map_canvas_to_plane(canvas, SURFACE, extent)
            back = linalg.map_plane_to_canvas(plane, SURFACE, extent)
            assert back == pytest.approx(canvas)

    def test_extent_corners(self):
        extent = (tuple(EXTENT[0]), tuple(EXTENT[1]))
        (x_min, y_min), (x_max, y_max) = extent
        # Plane top-left (x_min, y_max) is canvas origin (y-flip).
        assert linalg.map_plane_to_canvas(
            (x_min, y_max), SURFACE, extent) == pytest.approx((0.0, 0.0))
        assert linalg.map_plane_to_canvas(
            (x_max, y_min), SURFACE, extent) == pytest.approx(SURFACE)


class TestLocalCapture:
    def test_freehand_stroke_is_conditioned_and_stored(self):
        model = make_model()
        drag(model, [(400.0 + i * 4.0, 400.0) for i in range(40)])
        view = model.view_state()
        assert view['strokes'] == 1
        points = model.ink_strokes()[0]
        gaps = [math.dist(a, b) for a, b in zip(points, points[1:])]
        assert max(gaps) <= 0.002 + 1e-12
        # Geometry is plane metres inside the extent.
        assert all(-0.1 <= x <= 0.1 and -0.1 <= y <= 0.1 for x, y in points)

    def test_too_short_stroke_discarded_with_notice(self):
        model = make_model()
        drag(model, [(400.0, 400.0)])
        view = model.view_state()
        assert view['strokes'] == 0
        assert 'too short' in view['status']

    def test_line_tool_two_clicks_and_pending(self):
        model = make_model()
        model.select_tool('line')
        click(model, (200.0, 400.0))
        assert len(model.view_state()['pending']) == 1
        click(model, (600.0, 400.0))
        view = model.view_state()
        assert view['pending'] == []
        assert view['strokes'] == 1

    def test_arc_tool_three_clicks(self):
        model = make_model()
        model.select_tool('arc')
        for point in ((300.0, 400.0), (400.0, 300.0), (500.0, 400.0)):
            click(model, point)
        assert model.view_state()['strokes'] == 1

    def test_undo_and_clear_bump_revision(self):
        model = make_model()
        drag(model, [(400.0 + i * 4.0, 400.0) for i in range(20)])
        drag(model, [(400.0, 400.0 + i * 4.0) for i in range(20)])
        revision = model.view_state()['revision']
        model.undo()
        assert model.view_state()['strokes'] == 1
        model.clear()
        view = model.view_state()
        assert view['strokes'] == 0
        assert view['revision'] > revision
        model.undo()
        assert 'nothing to undo' in model.view_state()['status']

    def test_sample_bound_refuses_with_notice(self):
        model = make_model(max_samples=10)
        drag(model, [(100.0 + i * 4.0, 400.0) for i in range(100)])
        view = model.view_state()
        assert view['strokes'] == 0
        assert 'sample bound' in view['status']

    def test_capture_works_while_stopped_and_disconnected(self):
        model = make_model(stopped=True)
        model.set_connected(False)
        drag(model, [(400.0 + i * 4.0, 400.0) for i in range(40)])
        assert model.view_state()['strokes'] == 1

    def test_no_capture_before_config_or_in_live(self):
        bare = SurfaceModel()
        drag(bare, [(400.0 + i * 4.0, 400.0) for i in range(40)])
        assert bare.view_state()['strokes'] == 0
        live = make_model(mode='live')
        drag(live, [(400.0 + i * 4.0, 400.0) for i in range(40)])
        assert live.view_state()['strokes'] == 0


class TestChaining:
    """Stroke chaining: chained strokes inherit their
    start point and merge into one submitted stroke at execute/save."""

    def canvas_of(self, plane):
        extent = (tuple(EXTENT[0]), tuple(EXTENT[1]))
        return linalg.map_plane_to_canvas(plane, SURFACE, extent)

    def base_line(self, model):
        model.select_tool('line')
        click(model, (200.0, 400.0))
        click(model, (600.0, 400.0))

    def submitted(self, model):
        assert model.request_execute()
        _, _, _, _, _, controls = model.snapshot()
        return controls[0]['drawing']

    def test_chained_line_needs_one_click(self):
        model = make_model()
        self.base_line(model)
        model.set_chain(True)
        click(model, (600.0, 200.0))
        assert model.view_state()['strokes'] == 2

    def test_chained_arc_needs_two_clicks(self):
        model = make_model()
        self.base_line(model)
        model.set_chain(True)
        model.select_tool('arc')
        click(model, (700.0, 300.0))
        assert model.view_state()['strokes'] == 1  # still collecting
        click(model, (600.0, 200.0))
        assert model.view_state()['strokes'] == 2

    def test_chained_sequence_submits_as_one_stroke(self):
        model = make_model()
        self.base_line(model)
        model.set_chain(True)
        model.select_tool('arc')
        click(model, (700.0, 300.0))
        click(model, (600.0, 200.0))
        model.select_tool('line')
        click(model, (200.0, 200.0))
        drawing = self.submitted(model)
        assert len(drawing) == 1
        assert drawing[0]['kind'] == 'polyline'
        # Continuous with no duplicate seam points (the merge drops them).
        points = drawing[0]['points']
        gaps = [math.dist(a, b) for a, b in zip(points, points[1:])]
        assert min(gaps) > 0.0005

    def test_toggle_off_restores_pen_lift(self):
        model = make_model()
        self.base_line(model)
        model.set_chain(True)
        click(model, (600.0, 200.0))
        model.set_chain(False)
        click(model, (200.0, 200.0))
        click(model, (200.0, 300.0))  # unchained line: two clicks again
        drawing = self.submitted(model)
        assert len(drawing) == 2
        assert [s['kind'] for s in drawing] == ['polyline', 'line']

    def test_first_stroke_with_chain_on_is_unchained(self):
        model = make_model()
        model.set_chain(True)
        self.base_line(model)
        drawing = self.submitted(model)
        assert len(drawing) == 1
        assert drawing[0]['kind'] == 'line'

    def test_freehand_chains_when_started_at_the_previous_end(self):
        model = make_model()
        self.base_line(model)
        end = model.ink_strokes()[0][-1]
        model.select_tool('freehand')
        model.set_chain(True)
        start = self.canvas_of(end)
        drag(model, [(start[0] + i, start[1] - i * 2.0) for i in range(40)])
        drawing = self.submitted(model)
        assert len(drawing) == 1
        # The seam is shared: the freehand start snapped onto the line's end.
        assert model.ink_strokes()[1][0] == pytest.approx(end)

    def test_detached_freehand_stays_unchained_with_notice(self):
        model = make_model()
        self.base_line(model)
        model.select_tool('freehand')
        model.set_chain(True)
        drag(model, [(100.0 + i * 4.0, 100.0) for i in range(40)])
        assert 'not chained' in model.view_state()['status']
        assert len(self.submitted(model)) == 2

    def test_chain_follows_undo(self):
        model = make_model()
        self.base_line(model)
        end = model.ink_strokes()[0][-1]
        model.set_chain(True)
        click(model, (600.0, 200.0))
        model.undo()
        click(model, (400.0, 200.0))  # chains to the base line again
        assert model.ink_strokes()[1][0] == pytest.approx(end)

    def test_toggle_drops_pending_clicks(self):
        model = make_model()
        model.select_tool('line')
        click(model, (200.0, 400.0))
        model.set_chain(True)
        assert model.view_state()['pending'] == []
        assert model.view_state()['chain'] is True


class TestLiveButtonState:
    def test_marking_needs_primary_and_arm(self):
        from rapidcode_draw_plane.surface_client import live_button_state
        assert live_button_state(True, False, True) == (True, True)
        # Any release cleared the arm: primary still held draws nothing.
        assert live_button_state(True, False, False) == (False, True)

    def test_secondary_follows_at_travel_height(self):
        from rapidcode_draw_plane.surface_client import live_button_state
        assert live_button_state(False, True, False) == (False, True)
        # Chord with the arm set: marking wins while armed.
        assert live_button_state(True, True, True) == (True, True)

    def test_no_buttons_disengages(self):
        from rapidcode_draw_plane.surface_client import live_button_state
        assert live_button_state(False, False, False) == (False, False)
        assert live_button_state(False, False, True) == (False, False)


class TestControls:
    def test_snapshot_sequences_and_drains_controls(self):
        model = SurfaceModel()
        model.request_stop()
        seq0, _, _, _, _, controls = model.snapshot()
        seq1, _, _, _, _, empty = model.snapshot()
        assert (seq0, seq1) == (0, 1)
        assert controls == [{'action': 'stop'}]
        assert empty == []

    def test_request_set_plane_converts_mm_to_metres(self):
        # The operator measures a board in millimetres; the wire is SI
        # Absolute, not a delta, so a re-queued control after a
        # transport stall cannot double the offset.
        model = make_model()
        model.request_set_plane(12.5, -4.0)
        _, _, _, _, _, controls = model.snapshot()
        assert controls == [{'action': 'set_plane',
                             'offset': [0.0125, -0.004], 'yaw': 0.0}]

    def test_request_set_plane_sends_the_rotation_in_degrees(self):
        # Degrees on both sides: it is what the operator reads off a
        # protractor and what the recipe format's own `rotate` already uses.
        # Only the offset is converted, because only the offset is SI.
        model = make_model()
        model.request_set_plane(12.5, -4.0, -37.5)
        _, _, _, _, _, controls = model.snapshot()
        assert controls == [{'action': 'set_plane',
                             'offset': [0.0125, -0.004], 'yaw': -37.5}]

    def test_plane_offset_mm_reads_back_what_the_bridge_reports(self):
        # What is in force, not what was typed: the bridge refuses an offset
        # that would take the working area out of reach.
        model = SurfaceModel()
        assert model.plane_offset_mm() is None
        model.apply_display(display(plane_offset=(0.0125, -0.004)))
        assert model.plane_offset_mm() == pytest.approx((12.5, -4.0))

    def test_plane_yaw_reads_back_what_the_bridge_reports(self):
        model = SurfaceModel()
        assert model.plane_yaw_deg() is None
        model.apply_display(display(plane_yaw_deg=-37.5))
        assert model.plane_yaw_deg() == pytest.approx(-37.5)

    def test_plane_yaw_is_none_from_a_bridge_without_it(self):
        # The client must not read a rotation into a bridge that predates
        # ; the panel shows nothing rather than a confident zero.
        model = SurfaceModel()
        stale = display()
        del stale['config']['plane_yaw_deg']
        model.apply_display(stale)
        assert model.plane_yaw_deg() is None

    def test_request_park_queues_the_control(self):
        model = make_model()
        model.request_park()
        _, _, _, _, _, controls = model.snapshot()
        assert controls == [{'action': 'park'}]

    def test_requeue_controls_restores_order(self):
        model = SurfaceModel()
        model.request_stop()
        model.request_reset()
        _, _, _, _, _, controls = model.snapshot()
        model.request_mode('batch')            # queued while disconnected
        model.requeue_controls(controls)       # failed send puts them back first
        _, _, _, _, _, resent = model.snapshot()
        assert [c['action'] for c in resent] == ['stop', 'reset', 'mode']

    def test_execute_carries_the_submitted_drawing(self):
        model = make_model()
        drag(model, [(400.0 + i * 4.0, 400.0) for i in range(40)])
        assert model.request_execute()
        _, _, _, _, _, controls = model.snapshot()
        assert controls[0]['action'] == 'execute'
        drawing = controls[0]['drawing']
        assert drawing[0]['kind'] == 'polyline'
        assert len(drawing[0]['points']) >= 2

    def test_empty_execute_and_save_refused_locally(self):
        model = make_model()
        assert not model.request_execute()
        assert 'nothing to execute' in model.view_state()['status']
        assert not model.request_save('demo')
        _, _, _, _, _, controls = model.snapshot()
        assert controls == []

    def test_save_carries_name_and_drawing(self):
        model = make_model()
        model.select_tool('line')
        click(model, (200.0, 400.0))
        click(model, (600.0, 400.0))
        assert model.request_save('demo')
        _, _, _, _, _, controls = model.snapshot()
        assert controls[0]['action'] == 'save'
        assert controls[0]['name'] == 'demo'
        assert controls[0]['drawing'][0]['kind'] == 'line'

    def test_recipe_requests_queue_their_controls(self):
        model = make_model()
        model.request_load_recipe('recipe_demo')
        model.request_load_recipe('recipe_demo', scale=0.75, fit=True)
        model.request_execute_recipe()
        model.request_clear_recipe()
        _, _, _, _, _, controls = model.snapshot()
        assert controls == [
            {'action': 'load_recipe', 'name': 'recipe_demo'},
            {'action': 'load_recipe', 'name': 'recipe_demo',
             'scale': 0.75, 'fit': True},
            {'action': 'execute_recipe'},
            {'action': 'clear_recipe'},
        ]

    def test_preview_rides_the_display_state(self):
        model = make_model()
        preview = {'name': 'recipe_demo', 'scale': 0.8,
                   'polylines': [[[0.0, 0.0], [0.01, 0.0]]]}
        model.apply_display(dict(display(), preview=preview))
        assert model.view_state()['display']['preview'] == preview

    def test_mode_request_clears_pending_clicks(self):
        model = make_model()
        model.select_tool('line')
        click(model, (200.0, 400.0))
        model.request_mode('live')
        assert model.view_state()['pending'] == []
        _, _, _, _, _, controls = model.snapshot()
        assert {'action': 'mode', 'mode': 'live'} in controls


class TestReadoutAndClose:
    def test_readout_maps_pointer_to_plane(self):
        model = SurfaceModel()
        assert model.readout() is None  # before config arrives
        model.apply_display(display())
        model.set_surface(800, 800)
        model.set_pointer(400, 400)
        assert model.readout() == pytest.approx((0.0, 0.0))
        model.set_pointer(800, 0)
        assert model.readout() == pytest.approx((0.1, 0.1))

    def test_close_controls_stop_only_while_executing(self):
        model = SurfaceModel()
        model.apply_display(display())
        assert model.close_controls() == []
        model.apply_display(display(executing=True))
        assert model.close_controls() == [{'action': 'stop'}]

    def test_drawing_survives_display_updates(self):
        model = make_model()
        drag(model, [(400.0 + i * 4.0, 400.0) for i in range(40)])
        model.apply_display(display(stopped=True))
        model.apply_display(display())
        assert model.view_state()['strokes'] == 1


class TestOneExecute:
    """One Execute button (stakeholder 2026-09-04): it runs what the canvas
    shows. Loading a recipe replaces the drawing; drawing dismisses the
    recipe; the button's target follows."""

    @staticmethod
    def preview(name='recipe_demo'):
        shown = display()
        shown['preview'] = {'name': name, 'scale': 1.0,
                            'polylines': [[[0.0, 0.0], [0.01, 0.0]]]}
        return shown

    def test_empty_canvas_has_no_target(self):
        model = make_model()
        assert model.view_state()['execute'] is None

    def test_drawing_is_the_target_and_executes(self):
        model = make_model()
        drag(model, [(400.0 + i * 4.0, 400.0) for i in range(40)])
        assert model.view_state()['execute'] == ('drawing', None)
        assert model.request_execute()
        _, _, _, _, _, controls = model.snapshot()
        assert controls[-1]['action'] == 'execute'

    def test_load_replaces_drawing_and_targets_recipe_at_once(self):
        model = make_model()
        drag(model, [(400.0 + i * 4.0, 400.0) for i in range(40)])
        model.request_load_recipe('recipe_demo')
        view = model.view_state()
        assert view['strokes'] == 0
        assert view['execute'] == ('recipe', 'recipe_demo')  # before preview
        assert model.request_execute()
        _, _, _, _, _, controls = model.snapshot()
        assert [c['action'] for c in controls] == ['load_recipe',
                                                  'execute_recipe']

    def test_pending_load_expires_without_a_preview(self, monkeypatch):
        import rapidcode_draw_plane.surface_client as sc
        model = make_model()
        model.request_load_recipe('missing')
        assert model.view_state()['execute'] == ('recipe', 'missing')
        now = sc.time.monotonic()
        monkeypatch.setattr(sc.time, 'monotonic',
                            lambda: now + sc.RECIPE_PENDING_TIMEOUT + 1.0)
        assert model.view_state()['execute'] is None
        assert not model.request_execute()

    def test_preview_from_bridge_keeps_the_recipe_target(self):
        model = make_model()
        model.request_load_recipe('recipe_demo')
        model.apply_display(self.preview())
        assert model.view_state()['execute'] == ('recipe', 'recipe_demo')

    def test_drawing_dismisses_the_loaded_recipe(self):
        model = make_model()
        model.apply_display(self.preview())
        drag(model, [(400.0 + i * 4.0, 400.0) for i in range(40)])
        _, _, _, _, _, controls = model.snapshot()
        assert controls[0] == {'action': 'clear_recipe'}
        # The stale preview still shows in the display until the bridge
        # answers; the target is the drawing regardless.
        assert model.view_state()['execute'] == ('drawing', None)
        model.apply_display(display())  # bridge dropped the preview
        assert model.view_state()['execute'] == ('drawing', None)

    def test_composing_without_a_recipe_sends_no_clear(self):
        model = make_model()
        drag(model, [(400.0 + i * 4.0, 400.0) for i in range(40)])
        drag(model, [(400.0, 400.0 + i * 4.0) for i in range(40)])
        _, _, _, _, _, controls = model.snapshot()
        assert controls == []

    def test_clear_canvas_drops_drawing_and_recipe(self):
        model = make_model()
        model.apply_display(self.preview())
        model.clear()
        _, _, _, _, _, controls = model.snapshot()
        assert controls == [{'action': 'clear_recipe'}]
        assert model.view_state()['execute'] is None
        assert not model.request_execute()
        assert 'nothing to execute' in model.view_state()['status']

    def test_reloading_after_dismissal_targets_the_recipe_again(self):
        model = make_model()
        model.apply_display(self.preview())
        model.clear()
        model.request_load_recipe('recipe_demo')
        assert model.view_state()['execute'] == ('recipe', 'recipe_demo')
        model.apply_display(self.preview())
        assert model.view_state()['execute'] == ('recipe', 'recipe_demo')
