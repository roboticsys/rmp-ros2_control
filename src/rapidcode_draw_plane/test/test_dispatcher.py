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

"""Unit tests for the dispatcher (pure logic; fake routes, no ROS).

The client owns the drawing: the dispatcher receives it only as
the submitted drawing on an execute or save control, so these tests submit
payloads rather than driving capture through input events.
"""

import math

import pytest

from rapidcode_draw_plane import file_manager
from rapidcode_draw_plane.dispatcher import (
    BATCH, DRAINING, DRAW, EXECUTING, FOLLOW, LIVE, Dispatcher,
    DispatcherConfig)
from rapidcode_draw_plane.recipe_loader import LoadedRecipe, RecipeLoadError

PLANE = file_manager.PlaneConfig(
    anchor_xyz=(-0.4, 0.15, 0.271), anchor_rpy=(math.pi, 0.0, math.pi),
    ready_joints=(0.0,) * 6, draw_z=-0.02, safe_z=0.0,
    velocity=0.2, acceleration=0.2, eef_step=0.004)

# The geometry here is synthetic: a 0.3 m working radius about a 0.427 m
# anchor is nothing the real arm has ever been configured with. The reach band
# is opened wide so it cannot interfere with what these tests are
# about; TestReachBound below sets realistic values on purpose.
CONFIG = DispatcherConfig(
    extent=((-0.1, -0.1), (0.1, 0.1)),
    working_centre=(0.0, 0.0), working_radius=0.3,
    duplicate_threshold=0.0005, resample_spacing=0.002, min_run=3,
    plane=PLANE, recipe_directory='/tmp', max_samples=20000,
    reach_inner=0.0, reach_outer=10.0)

SURFACE = (800.0, 800.0)


class FakeBatch:
    def __init__(self):
        self.executed = []
        self.parks = []
        self.stops = 0
        self.resets = 0
        self.reset_ok = True

    def execute(self, structure):
        self.executed.append(structure)

    def park(self, ready_joints, velocity, acceleration):
        self.parks.append((tuple(ready_joints), velocity, acceleration))

    def stop(self):
        self.stops += 1

    def reset(self):
        self.resets += 1
        return self.reset_ok

    def health(self):
        return True


class FakeLive:
    def __init__(self):
        self.planes = []
        self.intents = []
        self.stops = 0
        self.rests = 0
        self.releases = 0
        self.activations = 0
        self.activate_ok = True

    def set_plane(self, plane):
        self.planes.append(plane)

    def set_intent(self, target):
        self.intents.append(target)

    def rest(self):
        self.rests += 1

    def release(self):
        self.releases += 1

    def stop(self):
        self.stops += 1

    def activate(self):
        self.activations += 1
        return self.activate_ok

    def reset(self):
        return True

    def health(self):
        return True


class FakeStroke:
    def __init__(self):
        self.planes = []
        self.begins = []
        self.points = []
        self.ends = 0
        self.handbacks = 0
        self.stops = 0
        self.resets = 0
        self.active_flag = False
        self.reset_ok = True

    def set_plane(self, plane):
        self.planes.append(plane)

    def begin_stroke(self, plane):
        self.begins.append(plane)

    def add_point(self, plane):
        self.points.append(plane)

    def end_stroke(self):
        self.ends += 1

    def request_handback(self):
        self.handbacks += 1

    def stop(self):
        self.stops += 1

    def reset(self):
        self.resets += 1
        return self.reset_ok

    def health(self):
        return True

    @property
    def active(self):
        return self.active_flag


class FakeLoader:
    """Canned recipe loads; records call arguments."""

    def __init__(self):
        self.calls = []
        self.error = None
        self.result = LoadedRecipe(
            name='recipe_demo', doc={'segments': [{'type': 'lin'}]},
            polylines=[[(0.0, 0.0), (0.01, 0.0)]], scale=0.8, notes=[])

    def load(self, name, plane, extent, working_centre, working_radius,
             scale=None, fit=False):
        self.calls.append({'name': name, 'scale': scale, 'fit': fit})
        if self.error is not None:
            raise self.error
        return self.result._replace(name=name)

    def list_names(self):
        return ['recipe_demo', 'recipe_other']


@pytest.fixture
def rig():
    batch, live = FakeBatch(), FakeLive()
    displays = []
    dispatcher = Dispatcher(CONFIG, batch, live, displays.append)
    return dispatcher, batch, live, displays


@pytest.fixture
def stroke_rig():
    """A rig with the stroke route injected (pen-down -> sized goal chain)."""
    batch, live, stroke = FakeBatch(), FakeLive(), FakeStroke()
    displays = []
    dispatcher = Dispatcher(CONFIG, batch, live, displays.append,
                            stroke_route=stroke)
    dispatcher.on_control({'action': 'mode', 'mode': 'live'})
    return dispatcher, batch, live, stroke, displays


def sample(pointer, pen, controls=(), follow=True):
    return {'pointer': pointer, 'surface': SURFACE, 'pen': pen,
            'follow': follow, 'controls': list(controls), 'seq': 0}


def line_drawing(points=40, start=(0.0, 0.0), step=(0.001, 0.0)):
    """One conditioned polyline in plane metres, inside the extent."""
    return [{'kind': 'polyline',
             'points': [[start[0] + i * step[0], start[1] + i * step[1]]
                        for i in range(points)]}]


class TestSubmittedExecute:
    def test_execute_builds_structure_and_locks_mode(self, rig):
        dispatcher, batch, *_ = rig
        dispatcher.on_control({'action': 'execute', 'drawing': line_drawing()})
        assert dispatcher.mode == EXECUTING
        assert len(batch.executed) == 1
        structure = batch.executed[0]
        assert structure['segments'][0]['type'] == 'lin'
        assert any(seg['type'] == 'through' for seg in structure['segments'])
        # Locked: no second execute while executing.
        dispatcher.on_control({'action': 'execute', 'drawing': line_drawing()})
        assert len(batch.executed) == 1

    def test_park_moves_to_the_ready_pose(self, rig):
        # the park command is a joint PTP to the ready pose, and it
        # commands no recipe -- a recipe would approach the plane origin first.
        dispatcher, batch, *_ = rig
        dispatcher.on_control({'action': 'park'})
        assert dispatcher.mode == EXECUTING
        assert batch.executed == []
        assert batch.parks == [
            (tuple(PLANE.ready_joints), PLANE.velocity, PLANE.acceleration)]
        dispatcher.on_route_event({'kind': 'result', 'ok': True, 'message': ''})
        assert dispatcher.mode == BATCH

    def test_park_refused_outside_batch(self, rig):
        dispatcher, batch, live, displays = rig
        dispatcher.on_control({'action': 'mode', 'mode': 'live'})
        dispatcher.on_control({'action': 'park'})
        assert batch.parks == []
        assert any('park is a Batch command' in w
                   for w in displays[-1]['warnings'])

    def test_park_refused_while_stopped(self, rig):
        dispatcher, batch, *_ = rig
        dispatcher.on_control({'action': 'stop'})
        dispatcher.on_control({'action': 'park'})
        assert batch.parks == []

    def test_result_returns_to_composing(self, rig):
        dispatcher, batch, *_ = rig
        dispatcher.on_control({'action': 'execute', 'drawing': line_drawing()})
        dispatcher.on_route_event({'kind': 'result', 'ok': True, 'message': ''})
        assert dispatcher.mode == BATCH

    def test_empty_drawing_refused(self, rig):
        dispatcher, batch, *_ = rig
        dispatcher.on_control({'action': 'execute', 'drawing': []})
        assert dispatcher.mode == BATCH
        assert batch.executed == []

    def test_over_sample_bound_refused(self, rig):
        dispatcher, batch, _, displays = rig
        big = [{'kind': 'polyline',
                'points': [[0.0, 0.0]] * (CONFIG.max_samples + 1)}]
        dispatcher.on_control({'action': 'execute', 'drawing': big})
        assert dispatcher.mode == BATCH
        assert batch.executed == []
        assert any('sample' in w for w in displays[-1]['warnings'])

    def test_out_of_extent_points_are_clamped(self, rig):
        dispatcher, batch, *_ = rig
        drawing = [{'kind': 'polyline',
                    'points': [[0.5, 0.0], [0.5, 0.02], [0.5, 0.04],
                               [0.5, 0.06]]}]  # x beyond the extent
        dispatcher.on_control({'action': 'execute', 'drawing': drawing})
        assert len(batch.executed) == 1  # clamped to the boundary, not refused

    def test_out_of_working_area_runs_excluded_and_warned(self, rig):
        small = CONFIG._replace(extent=((-0.1, -0.1), (0.1, 0.1)),
                                working_radius=0.12)
        batch, live = FakeBatch(), FakeLive()
        displays = []
        dispatcher = Dispatcher(small, batch, live, displays.append)
        # A stroke along the extent's top edge: the corners lie outside the
        # 0.12 m disc, the middle inside.
        points = [[-0.1 + i * 0.004, 0.1] for i in range(51)]
        dispatcher.on_control({'action': 'execute',
                               'drawing': [{'kind': 'polyline',
                                            'points': points}]})
        assert len(batch.executed) == 1
        assert any('working area' in w for w in displays[-1]['warnings'])


class TestStopAndLoss:
    def test_stop_latches_and_reset_releases(self, rig):
        dispatcher, batch, *_ = rig
        dispatcher.on_control({'action': 'stop'})
        assert dispatcher.stopped
        assert batch.stops == 1
        dispatcher.on_control({'action': 'execute',
                               'drawing': line_drawing()})  # held by the latch
        assert batch.executed == []
        dispatcher.on_control({'action': 'reset'})
        assert not dispatcher.stopped
        assert batch.resets == 1

    def test_reset_refused_keeps_latch(self, rig):
        dispatcher, batch, *_ = rig
        batch.reset_ok = False
        dispatcher.on_control({'action': 'stop'})
        dispatcher.on_control({'action': 'reset'})
        assert dispatcher.stopped

    def test_input_loss_during_execution_stops(self, rig):
        dispatcher, batch, *_ = rig
        dispatcher.on_control({'action': 'execute', 'drawing': line_drawing()})
        dispatcher.on_input_loss()
        assert dispatcher.stopped
        assert batch.stops == 1

    def test_input_loss_while_composing_is_benign(self, rig):
        dispatcher, batch, *_ = rig
        dispatcher.on_input_loss()
        assert not dispatcher.stopped
        assert batch.stops == 0

    def test_health_loss_stops(self, rig):
        dispatcher, batch, *_ = rig
        dispatcher.on_route_event({'kind': 'health', 'ok': False, 'message': 'gone'})
        assert dispatcher.stopped

    def test_save_works_while_stopped(self, rig, tmp_path):
        config = CONFIG._replace(recipe_directory=str(tmp_path))
        batch, live = FakeBatch(), FakeLive()
        displays = []
        dispatcher = Dispatcher(config, batch, live, displays.append)
        dispatcher.on_control({'action': 'stop'})
        dispatcher.on_control({'action': 'save', 'name': 'held',
                               'drawing': line_drawing()})
        assert list(tmp_path.glob('recipe_held.yaml'))


class TestLiveMode:
    def test_mode_switch_and_intents(self, rig):
        dispatcher, _, live, _ = rig
        dispatcher.on_control({'action': 'mode', 'mode': 'live'})
        assert dispatcher.mode == LIVE
        dispatcher.on_input_event(sample((400.0, 400.0), True))
        dispatcher.on_input_event(sample((400.0, 400.0), False))
        assert live.intents[0][2] == PLANE.draw_z   # pen down -> draw depth
        assert live.intents[-1][2] == PLANE.safe_z  # pen up -> safe height

    def test_no_follow_publishes_no_intent(self, rig):
        # the arm tracks the pointer only while follow is engaged.
        dispatcher, _, live, _ = rig
        dispatcher.on_control({'action': 'mode', 'mode': 'live'})
        dispatcher.on_input_event(sample((100.0, 100.0), False, follow=False))
        dispatcher.on_input_event(sample((700.0, 700.0), True, follow=False))
        assert live.intents == []
        assert live.releases == 0   # never engaged: nothing to release

    def test_disengaging_follow_releases_once(self, rig):
        dispatcher, _, live, _ = rig
        dispatcher.on_control({'action': 'mode', 'mode': 'live'})
        dispatcher.on_input_event(sample((400.0, 400.0), True, follow=True))
        assert len(live.intents) == 1
        dispatcher.on_input_event(sample((500.0, 400.0), False, follow=False))
        dispatcher.on_input_event(sample((600.0, 400.0), False, follow=False))
        assert live.releases == 1           # one release per disengagement
        assert len(live.intents) == 1       # the pointer is no longer chased

    def test_leaving_live_rests_without_cycling_the_controller(self, rig):
        # Switching back to Batch rests by silence only: the hardware credits
        # only sized-goal frames toward completion, so no stop+reset workaround
        # is needed (and none may run -- a stop would abandon the open online
        # move and trip the out-of-frames watchdog).
        dispatcher, batch, live, _ = rig
        dispatcher.on_control({'action': 'mode', 'mode': 'live'})
        dispatcher.on_control({'action': 'mode', 'mode': 'batch'})
        assert live.rests == 1
        assert batch.stops == 0 and batch.resets == 0
        assert not dispatcher.stopped

    def test_activate_failure_stays_batch(self, rig):
        dispatcher, _, live, _ = rig
        live.activate_ok = False
        dispatcher.on_control({'action': 'mode', 'mode': 'live'})
        assert dispatcher.mode == BATCH

    def test_input_loss_in_live_rests(self, rig):
        dispatcher, _, live, _ = rig
        dispatcher.on_control({'action': 'mode', 'mode': 'live'})
        dispatcher.on_input_loss()
        assert live.rests == 1
        assert not dispatcher.stopped


class TestStrokeRouting:
    """Live sub-states with the stroke route: pen-down -> goal chain
    (stakeholder 2026-08-25), pen-up follow -> Servo."""

    def test_pen_down_begins_stroke_and_rests_servo(self, stroke_rig):
        dispatcher, _, live, stroke, _ = stroke_rig
        dispatcher.on_input_event(sample((400.0, 400.0), True))
        assert live.rests == 1              # instant Servo silence, no release
        assert live.releases == 0
        assert live.intents == []           # pen-down never reaches Servo
        assert stroke.begins == [(0.0, 0.0)]
        assert dispatcher.live_sub == DRAW

    def test_pen_down_samples_add_points(self, stroke_rig):
        dispatcher, _, _, stroke, _ = stroke_rig
        dispatcher.on_input_event(sample((400.0, 400.0), True))
        dispatcher.on_input_event(sample((500.0, 400.0), True))
        assert len(stroke.begins) == 1
        assert len(stroke.points) == 1

    def test_pen_up_finishes_stroke_and_drains(self, stroke_rig):
        dispatcher, _, live, stroke, _ = stroke_rig
        dispatcher.on_input_event(sample((400.0, 400.0), True))
        dispatcher.on_input_event(sample((500.0, 400.0), False))
        assert stroke.ends == 1 and stroke.handbacks == 1
        assert dispatcher.live_sub == DRAINING
        # Pointer samples while draining are dropped (no Servo intents).
        dispatcher.on_input_event(sample((600.0, 400.0), False))
        assert live.intents == []
        assert stroke.ends == 1

    def test_pen_up_follow_uses_servo_at_safe_height(self, stroke_rig):
        dispatcher, _, live, stroke, _ = stroke_rig
        dispatcher.on_input_event(sample((400.0, 400.0), False))
        assert live.intents == [(0.0, 0.0, PLANE.safe_z)]
        assert stroke.begins == []

    def test_chain_drained_hands_back_to_servo(self, stroke_rig):
        dispatcher, _, live, stroke, _ = stroke_rig
        dispatcher.on_input_event(sample((400.0, 400.0), True))
        dispatcher.on_input_event(sample((500.0, 400.0), False))
        activations = live.activations
        dispatcher.on_route_event({'kind': 'stroke', 'event': 'chain_drained'})
        assert dispatcher.live_sub == FOLLOW
        assert live.activations == activations + 1   # still following

    def test_chain_drained_without_follow_stays_quiet(self, stroke_rig):
        dispatcher, _, live, stroke, _ = stroke_rig
        dispatcher.on_input_event(sample((400.0, 400.0), True))
        dispatcher.on_input_event(sample((500.0, 400.0), False, follow=False))
        activations = live.activations
        dispatcher.on_route_event({'kind': 'stroke', 'event': 'chain_drained'})
        assert dispatcher.live_sub == FOLLOW
        assert live.activations == activations       # nobody is following

    def test_stale_chain_drained_after_repen_down_is_ignored(self, stroke_rig):
        dispatcher, _, live, stroke, _ = stroke_rig
        dispatcher.on_input_event(sample((400.0, 400.0), True))
        dispatcher.on_input_event(sample((500.0, 400.0), False))
        dispatcher.on_input_event(sample((600.0, 400.0), True))  # re-engaged
        dispatcher.on_route_event({'kind': 'stroke', 'event': 'chain_drained'})
        assert dispatcher.live_sub == DRAW    # the new stroke owns the route
        assert live.activations == 1          # only the mode-entry activate

    def test_repen_down_while_draining_keeps_chain(self, stroke_rig):
        dispatcher, _, _, stroke, _ = stroke_rig
        dispatcher.on_input_event(sample((400.0, 400.0), True))
        dispatcher.on_input_event(sample((500.0, 400.0), False))
        dispatcher.on_input_event(sample((600.0, 400.0), True))
        assert len(stroke.begins) == 2       # travel + plunge inside the chain
        assert dispatcher.live_sub == DRAW

    def test_stop_stops_both_live_routes(self, stroke_rig):
        dispatcher, _, live, stroke, _ = stroke_rig
        dispatcher.on_control({'action': 'stop'})
        assert live.stops == 1 and stroke.stops == 1
        assert dispatcher.stopped

    def test_reset_routes_through_stroke(self, stroke_rig):
        dispatcher, _, _, stroke, _ = stroke_rig
        dispatcher.on_control({'action': 'stop'})
        dispatcher.on_control({'action': 'reset'})
        assert stroke.resets == 1
        assert not dispatcher.stopped

    def test_mode_switch_refused_while_chain_active(self, stroke_rig):
        dispatcher, _, _, stroke, displays = stroke_rig
        stroke.active_flag = True
        dispatcher.on_control({'action': 'mode', 'mode': 'batch'})
        assert dispatcher.mode == LIVE
        assert any('locked while strokes execute' in w
                   for w in displays[-1]['warnings'])

    def test_input_loss_in_draw_finishes_strokes(self, stroke_rig):
        dispatcher, _, live, stroke, _ = stroke_rig
        dispatcher.on_input_event(sample((400.0, 400.0), True))
        dispatcher.on_input_loss()
        assert stroke.ends == 1 and stroke.handbacks == 1
        assert live.rests >= 1
        assert not dispatcher.stopped

    def test_follow_disengage_in_draw_finishes_strokes(self, stroke_rig):
        dispatcher, _, live, stroke, _ = stroke_rig
        dispatcher.on_input_event(sample((400.0, 400.0), True))
        dispatcher.on_input_event(sample((500.0, 400.0), False, follow=False))
        assert stroke.ends == 1 and stroke.handbacks == 1
        assert live.releases == 0            # the chain's lift replaces it
        assert dispatcher.live_sub == DRAINING

    def test_chain_failed_warns(self, stroke_rig):
        dispatcher, _, _, _, displays = stroke_rig
        dispatcher.on_route_event({'kind': 'stroke', 'event': 'chain_failed',
                                   'message': 'goal 0 failed to plan'})
        assert any('stroke chain failed' in w for w in displays[-1]['warnings'])


@pytest.fixture
def recipe_rig():
    """A rig with the recipe loader injected (batch recipe loading)."""
    batch, live, loader = FakeBatch(), FakeLive(), FakeLoader()
    displays = []
    dispatcher = Dispatcher(CONFIG, batch, live, displays.append,
                            recipe_loader=loader)
    return dispatcher, batch, live, loader, displays


class TestRecipeControls:
    def test_load_previews_and_execute_runs_the_doc(self, recipe_rig):
        dispatcher, batch, _live, loader, displays = recipe_rig
        dispatcher.on_control({'action': 'load_recipe',
                               'name': 'recipe_demo', 'scale': 0.5})
        assert loader.calls == [{'name': 'recipe_demo', 'scale': 0.5,
                                 'fit': False}]
        preview = displays[-1]['preview']
        assert preview['name'] == 'recipe_demo'
        assert preview['scale'] == 0.8
        assert preview['polylines'] == [[[0.0, 0.0], [0.01, 0.0]]]

        dispatcher.on_control({'action': 'execute_recipe'})
        assert dispatcher.mode == EXECUTING
        assert batch.executed[-1] is loader.result.doc
        # The result event returns to Batch; the loaded recipe survives for a
        # re-run.
        dispatcher.on_route_event({'kind': 'result', 'ok': True})
        assert dispatcher.mode == BATCH
        assert displays[-1]['preview'] is not None

    def test_fit_flag_reaches_the_loader(self, recipe_rig):
        dispatcher, _batch, _live, loader, _displays = recipe_rig
        dispatcher.on_control({'action': 'load_recipe',
                               'name': 'recipe_demo', 'fit': True})
        assert loader.calls == [{'name': 'recipe_demo', 'scale': None,
                                 'fit': True}]

    def test_execute_with_nothing_loaded_warns(self, recipe_rig):
        dispatcher, batch, _live, _loader, displays = recipe_rig
        dispatcher.on_control({'action': 'execute_recipe'})
        assert batch.executed == []
        assert dispatcher.mode == BATCH
        assert any('no recipe loaded' in warning
                   for warning in displays[-1]['warnings'])

    def test_load_is_allowed_while_stopped_but_execute_is_not(
            self, recipe_rig):
        dispatcher, batch, _live, _loader, displays = recipe_rig
        dispatcher.on_control({'action': 'stop'})
        dispatcher.on_control({'action': 'load_recipe',
                               'name': 'recipe_demo'})
        assert displays[-1]['preview'] is not None
        dispatcher.on_control({'action': 'execute_recipe'})
        assert batch.executed == []
        assert any('ignored: stopped' in warning
                   for warning in displays[-1]['warnings'])

    def test_load_in_live_mode_is_refused(self, recipe_rig):
        dispatcher, _batch, _live, loader, displays = recipe_rig
        dispatcher.on_control({'action': 'mode', 'mode': 'live'})
        dispatcher.on_control({'action': 'load_recipe',
                               'name': 'recipe_demo'})
        assert loader.calls == []
        assert any('Batch command' in warning
                   for warning in displays[-1]['warnings'])

    def test_failed_load_keeps_the_previous_recipe(self, recipe_rig):
        dispatcher, _batch, _live, loader, displays = recipe_rig
        dispatcher.on_control({'action': 'load_recipe',
                               'name': 'recipe_demo'})
        loader.error = RecipeLoadError('recipe_other: boom')
        dispatcher.on_control({'action': 'load_recipe',
                               'name': 'recipe_other'})
        assert displays[-1]['preview']['name'] == 'recipe_demo'
        assert any('recipe load failed' in warning
                   for warning in displays[-1]['warnings'])

    def test_loader_notes_surface_as_warnings(self, recipe_rig):
        dispatcher, _batch, _live, loader, displays = recipe_rig
        loader.result = loader.result._replace(
            notes=['recipe_demo: draws below draw_z'])
        dispatcher.on_control({'action': 'load_recipe',
                               'name': 'recipe_demo'})
        assert any('draws below draw_z' in warning
                   for warning in displays[-1]['warnings'])
        assert displays[-1]['preview'] is not None

    def test_clear_recipe_drops_the_preview(self, recipe_rig):
        dispatcher, _batch, _live, _loader, displays = recipe_rig
        dispatcher.on_control({'action': 'load_recipe',
                               'name': 'recipe_demo'})
        dispatcher.on_control({'action': 'clear_recipe'})
        assert displays[-1]['preview'] is None

    def test_recipe_names_ride_the_display_config(self, recipe_rig):
        dispatcher, _batch, _live, _loader, displays = recipe_rig
        dispatcher.push_display()
        assert displays[-1]['config']['recipes'] == ['recipe_demo',
                                                     'recipe_other']

    def test_without_a_loader_load_warns_unavailable(self, rig):
        dispatcher, batch, _live, displays = rig
        dispatcher.on_control({'action': 'load_recipe',
                               'name': 'recipe_demo'})
        assert batch.executed == []
        assert any('unavailable' in warning
                   for warning in displays[-1]['warnings'])
        assert displays[-1]['config']['recipes'] == []
        assert displays[-1]['preview'] is None


class TestStartupRefusal:
    def test_retract_below_safe_refused(self):
        bad = CONFIG._replace(plane=PLANE._replace(retract_z=-0.01))
        assert Dispatcher.check_startup(bad) is not None

    def test_oversized_extent_refused(self):
        # CONFIG's geometry is synthetic -- a 0.3 m working radius the real
        # arm has never had -- so it carries a band wide enough to stay out of
        # the way. The reach bound has its own tests below.
        bad = CONFIG._replace(working_radius=0.05)
        assert Dispatcher.check_startup(bad) is not None
        assert Dispatcher.check_startup(CONFIG) is None


class TestSetPlane:
    """The operator's plane calibration, applied at runtime.

    A calibration commands no motion, so it sits above the stopped pivot in
    the control chain -- but it re-bases every route, so it is admitted only
    in Batch, where neither route is streaming.
    """

    # Realistic geometry: the reach bound is what these tests are about, so
    # unlike CONFIG they use the numbers  actually record.
    REAL_PLANE = PLANE._replace(anchor_xyz=(-0.46, 0.0, 0.271))
    # working_radius 0.14 rather than the stock 0.16 leaves 20 mm of
    # calibration room at each edge of the band -- the trade the runbook
    # points an operator at. The extent shrinks to match, or the
    # extent-fits-disc check would refuse first and mask what is under test.
    REAL = CONFIG._replace(
        plane=REAL_PLANE, extent=((-0.09, -0.09), (0.09, 0.09)),
        working_radius=0.14, reach_inner=0.30, reach_outer=0.62)

    def real_rig(self):
        batch, live, stroke = FakeBatch(), FakeLive(), FakeStroke()
        adopted = []
        dispatcher = Dispatcher(self.REAL, batch, live, lambda state: None,
                                stroke_route=stroke,
                                on_plane_change=adopted.append)
        return dispatcher, live, stroke, adopted

    def test_offset_reaches_every_holder_of_the_configuration(self):
        # The dispatcher, both routes and the bridge must agree about where
        # the board is; a stale holder would draw somewhere else.
        dispatcher, live, stroke, adopted = self.real_rig()
        dispatcher.on_control({'action': 'set_plane', 'offset': [0.01, -0.005]})
        expected = (0.01, -0.005)
        assert dispatcher.plane.plane_offset == expected
        assert [p.plane_offset for p in live.planes] == [expected]
        assert [p.plane_offset for p in stroke.planes] == [expected]
        assert [p.plane_offset for p in adopted] == [expected]

    def test_the_offset_is_absolute_not_cumulative(self):
        # A re-queued control after a transport stall must not double.
        dispatcher, _live, _stroke, _adopted = self.real_rig()
        for _ in range(3):
            dispatcher.on_control(
                {'action': 'set_plane', 'offset': [0.01, 0.0]})
        assert dispatcher.plane.plane_offset == (0.01, 0.0)

    def test_the_display_carries_the_offset_and_the_band(self):
        batch, live = FakeBatch(), FakeLive()
        displays = []
        dispatcher = Dispatcher(self.REAL, batch, live, displays.append)
        dispatcher.on_control({'action': 'set_plane', 'offset': [0.01, 0.0]})
        config = displays[-1]['config']
        assert config['plane_offset'] == [0.01, 0.0]
        assert (config['reach_inner'], config['reach_outer']) == (0.30, 0.62)

    def test_an_outward_offset_past_the_band_is_refused(self):
        dispatcher, live, _stroke, adopted = self.real_rig()
        dispatcher.on_control({'action': 'set_plane', 'offset': [-0.05, 0.0]})
        assert dispatcher.plane.plane_offset == (0.0, 0.0)
        assert live.planes == [] and adopted == []
        assert 'refused' in dispatcher.display_state()['warnings'][-1]

    def test_an_inward_offset_past_the_band_is_refused(self):
        dispatcher, live, _stroke, _adopted = self.real_rig()
        dispatcher.on_control({'action': 'set_plane', 'offset': [0.05, 0.0]})
        assert dispatcher.plane.plane_offset == (0.0, 0.0)
        assert live.planes == []

    def test_refused_in_live(self):
        # The live intent is held in plane coordinates and remapped every
        # tick, so re-basing the frame under it would jump the arm.
        dispatcher, live, _stroke, _adopted = self.real_rig()
        dispatcher.on_control({'action': 'mode', 'mode': 'live'})
        dispatcher.on_control({'action': 'set_plane', 'offset': [0.01, 0.0]})
        assert dispatcher.plane.plane_offset == (0.0, 0.0)
        assert 'Batch command' in dispatcher.display_state()['warnings'][-1]

    def test_refused_while_executing(self):
        dispatcher, live, _stroke, _adopted = self.real_rig()
        dispatcher.on_control({'action': 'park'})
        assert dispatcher.mode == EXECUTING
        dispatcher.on_control({'action': 'set_plane', 'offset': [0.01, 0.0]})
        assert dispatcher.plane.plane_offset == (0.0, 0.0)

    def test_allowed_while_stopped_in_batch(self):
        # It commands no motion, and calibrating before a reset is reasonable.
        dispatcher, _live, _stroke, _adopted = self.real_rig()
        dispatcher.on_control({'action': 'stop'})
        dispatcher.on_control({'action': 'set_plane', 'offset': [0.01, 0.0]})
        assert dispatcher.plane.plane_offset == (0.01, 0.0)

    def test_a_stop_mid_execution_defers_it_to_the_reset(self):
        # A stop does not leave Executing, so the mode guard still refuses;
        # the reset is what returns the operator to Batch.
        dispatcher, _live, _stroke, _adopted = self.real_rig()
        dispatcher.on_control({'action': 'park'})
        dispatcher.on_control({'action': 'stop'})
        dispatcher.on_control({'action': 'set_plane', 'offset': [0.01, 0.0]})
        assert dispatcher.plane.plane_offset == (0.0, 0.0)
        dispatcher.on_control({'action': 'reset'})
        dispatcher.on_control({'action': 'set_plane', 'offset': [0.01, 0.0]})
        assert dispatcher.plane.plane_offset == (0.01, 0.0)

    def test_a_loaded_recipe_is_dropped(self):
        # Its doc was re-anchored and bound-checked against the old plane.
        batch, live = FakeBatch(), FakeLive()
        loader = FakeLoader()
        dispatcher = Dispatcher(self.REAL, batch, live, lambda state: None,
                                recipe_loader=loader)
        dispatcher.on_control({'action': 'load_recipe', 'name': 'shape'})
        assert dispatcher.display_state()['preview'] is not None
        dispatcher.on_control({'action': 'set_plane', 'offset': [0.01, 0.0]})
        assert dispatcher.display_state()['preview'] is None


class TestSetPlaneRotation:
    """The rest of the calibration -- how far the board is turned.

    It rides the same control, the same admission rule and the same fan-out as
    the offset, so those are not re-tested here; what is tested is that the
    rotation reaches every holder, that it is absolute, and that it interacts
    with the reach bound the way the geometry says it should.
    """

    real_rig = TestSetPlane.real_rig
    REAL = TestSetPlane.REAL
    REAL_PLANE = TestSetPlane.REAL_PLANE

    def test_the_rotation_reaches_every_holder_of_the_configuration(self):
        dispatcher, live, stroke, adopted = self.real_rig()
        dispatcher.on_control(
            {'action': 'set_plane', 'offset': [0.0, 0.0], 'yaw': 45.0})
        assert dispatcher.plane.plane_yaw_deg == 45.0
        assert [p.plane_yaw_deg for p in live.planes] == [45.0]
        assert [p.plane_yaw_deg for p in stroke.planes] == [45.0]
        assert [p.plane_yaw_deg for p in adopted] == [45.0]

    def test_the_rotation_is_absolute_not_cumulative(self):
        dispatcher, _live, _stroke, _adopted = self.real_rig()
        for _ in range(3):
            dispatcher.on_control(
                {'action': 'set_plane', 'offset': [0.0, 0.0], 'yaw': 45.0})
        assert dispatcher.plane.plane_yaw_deg == 45.0

    def test_the_offset_and_the_rotation_travel_together(self):
        dispatcher, _live, _stroke, _adopted = self.real_rig()
        dispatcher.on_control(
            {'action': 'set_plane', 'offset': [0.01, -0.005], 'yaw': -30.0})
        assert dispatcher.plane.plane_offset == (0.01, -0.005)
        assert dispatcher.plane.plane_yaw_deg == -30.0

    def test_an_omitted_rotation_leaves_the_one_in_force(self):
        # A client that predates  sends no "yaw". Zeroing the rotation
        # on its behalf would silently un-calibrate the board; it cannot know
        # the field exists to resend it.
        dispatcher, _live, _stroke, _adopted = self.real_rig()
        dispatcher.on_control(
            {'action': 'set_plane', 'offset': [0.0, 0.0], 'yaw': 45.0})
        dispatcher.on_control({'action': 'set_plane', 'offset': [0.01, 0.0]})
        assert dispatcher.plane.plane_yaw_deg == 45.0
        assert dispatcher.plane.plane_offset == (0.01, 0.0)

    def test_an_explicit_zero_clears_it(self):
        dispatcher, _live, _stroke, _adopted = self.real_rig()
        dispatcher.on_control(
            {'action': 'set_plane', 'offset': [0.0, 0.0], 'yaw': 45.0})
        dispatcher.on_control(
            {'action': 'set_plane', 'offset': [0.0, 0.0], 'yaw': 0.0})
        assert dispatcher.plane.plane_yaw_deg == 0.0

    def test_the_display_carries_the_rotation(self):
        batch, live = FakeBatch(), FakeLive()
        displays = []
        dispatcher = Dispatcher(self.REAL, batch, live, displays.append)
        dispatcher.on_control(
            {'action': 'set_plane', 'offset': [0.0, 0.0], 'yaw': -37.5})
        assert displays[-1]['config']['plane_yaw_deg'] == -37.5

    @pytest.mark.parametrize('degrees', [15.0, 45.0, 90.0, 180.0, -120.0])
    def test_a_rotation_costs_no_reach_budget(self, degrees):
        # With the working centre on the plane origin the disc turns about its
        # own centre, so it is the same disc: a rotation passes the band at
        # every angle, unlike an offset, which spends the budget.
        dispatcher, _live, _stroke, _adopted = self.real_rig()
        assert self.REAL.working_centre == (0.0, 0.0)
        dispatcher.on_control(
            {'action': 'set_plane', 'offset': [0.0, 0.0], 'yaw': degrees})
        assert dispatcher.plane.plane_yaw_deg == degrees

    # An off-origin working centre, chosen so the rotation ALONE decides the
    # verdict: at rest the disc sits 0.06 m crosswise of the anchor and its far
    # edge is 0.604 m out, inside the 0.62 m limit; turned 90 degrees the same
    # 0.06 m points straight away from the base and the far edge reaches 0.66.
    OFF_ORIGIN = TestSetPlane.REAL._replace(
        working_centre=(0.0, 0.06), extent=((-0.02, -0.02), (0.02, 0.02)))

    def off_origin_rig(self):
        batch, live = FakeBatch(), FakeLive()
        return Dispatcher(self.OFF_ORIGIN, batch, live, lambda state: None)

    def test_an_off_origin_working_centre_is_fine_unrotated(self):
        # The control half of the pair below: without the rotation this same
        # calibration is admitted, so the refusal there is the rotation's
        # doing and not the working centre's.
        dispatcher = self.off_origin_rig()
        dispatcher.on_control(
            {'action': 'set_plane', 'offset': [0.0, 0.0], 'yaw': 0.0})
        assert dispatcher.display_state()['warnings'] == []

    def test_a_rotation_swings_an_off_origin_working_centre_out_of_reach(self):
        # The no-op above holds only while the working centre sits on the
        # plane origin. Move it off, and the rotation carries the disc round
        # an arc -- which is exactly when omitting the rotation from the bound
        # would let an unreachable calibration through.
        dispatcher = self.off_origin_rig()
        dispatcher.on_control(
            {'action': 'set_plane', 'offset': [0.0, 0.0], 'yaw': 90.0})
        assert dispatcher.plane.plane_yaw_deg == 0.0
        assert 'refused' in dispatcher.display_state()['warnings'][-1]

    def test_refused_in_live(self):
        dispatcher, _live, _stroke, _adopted = self.real_rig()
        dispatcher.on_control({'action': 'mode', 'mode': 'live'})
        dispatcher.on_control(
            {'action': 'set_plane', 'offset': [0.0, 0.0], 'yaw': 45.0})
        assert dispatcher.plane.plane_yaw_deg == 0.0

    def test_a_loaded_recipe_is_dropped(self):
        # Its preview and its verdicts were computed against the old plane.
        batch, live = FakeBatch(), FakeLive()
        dispatcher = Dispatcher(self.REAL, batch, live, lambda state: None,
                                recipe_loader=FakeLoader())
        dispatcher.on_control({'action': 'load_recipe', 'name': 'shape'})
        dispatcher.on_control(
            {'action': 'set_plane', 'offset': [0.0, 0.0], 'yaw': 45.0})
        assert dispatcher.display_state()['preview'] is None


class TestStartupReachBound:
    """The same rule, applied to the launch parameters.

    Startup and the runtime control share one function, so a launch file
    cannot set a calibration the operator would be refused.
    """

    def test_the_stock_configuration_starts(self):
        assert Dispatcher.check_startup(TestSetPlane.REAL._replace(
            plane=TestSetPlane.REAL_PLANE, working_radius=0.16)) is None

    def test_a_launch_offset_past_the_band_refuses_startup(self):
        bad = TestSetPlane.REAL._replace(
            plane=TestSetPlane.REAL_PLANE._replace(plane_offset=(-0.05, 0.0)))
        assert 'reach band' in Dispatcher.check_startup(bad)

    def test_a_launch_rotation_starts_at_any_angle(self):
        # Same geometry as the runtime control: with the working centre on the
        # plane origin, a rotation cannot take the disc out of the band.
        for degrees in (15.0, 90.0, 180.0):
            good = TestSetPlane.REAL._replace(
                plane=TestSetPlane.REAL_PLANE._replace(plane_yaw_deg=degrees))
            assert Dispatcher.check_startup(good) is None

    def test_a_launch_rotation_swinging_an_off_origin_centre_refuses(self):
        # Unrotated the same configuration starts, so the rotation is what the
        # check catches.
        assert Dispatcher.check_startup(
            TestSetPlaneRotation.OFF_ORIGIN) is None
        bad = TestSetPlaneRotation.OFF_ORIGIN._replace(
            plane=TestSetPlane.REAL_PLANE._replace(plane_yaw_deg=90.0))
        assert 'reach band' in Dispatcher.check_startup(bad)
