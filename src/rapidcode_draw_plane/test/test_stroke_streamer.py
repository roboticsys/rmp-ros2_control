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

"""Unit tests for the stroke chain streamer (pure logic; fakes, no ROS).

The safety property under test is the finalize discipline: a goal is
submitted with finalize=false ONLY when its successor is already planned
(otherwise a starved open firmware move e-stops in ~32 ms), and every exit
from streaming either submits a finalize=true goal or rides ~/stop.

The chain loop is driven synchronously on the test thread (start_thread
False) with an injected ros dict; the ``poll`` fake consumes a script of
callables, so waits inside the loop advance the scenario deterministically.
"""

import math

import pytest

from rapidcode_draw_plane import file_manager
from rapidcode_draw_plane.dispatcher import DispatcherConfig
from rapidcode_draw_plane.live_streamer import plane_to_world
from rapidcode_draw_plane.stroke_streamer import StrokeChainStreamer

PLANE = file_manager.PlaneConfig(
    anchor_xyz=(-0.4, 0.15, 0.271), anchor_rpy=(math.pi, 0.0, math.pi),
    ready_joints=(0.0,) * 6, draw_z=-0.02, safe_z=0.0,
    velocity=0.2, acceleration=0.2, eef_step=0.004)

CONFIG = DispatcherConfig(
    extent=((-0.1, -0.1), (0.1, 0.1)),
    working_centre=(0.0, 0.0), working_radius=0.3,
    duplicate_threshold=0.0005, resample_spacing=0.002, min_run=3,
    plane=PLANE, recipe_directory='/tmp', max_samples=20000)

# Goal sizing in the tests: cartesian_speed = 0.25 * velocity = 0.05 m/s, so
# goal_seconds=2.0 cuts a window every 0.1 m of path.
GOAL_SECONDS = 2.0
WINDOW_METRES = 0.1


class FakeSender:
    JOINT_VELOCITY_LIMIT = 1.57
    JOINT_ACCEL_LIMIT = 3.0
    CORNER_DELTA_V = 0.05

    @staticmethod
    def retime_goal_window(positions, next_positions, limits,
                           entry_speed, entry_velocity):
        duration = 0.01 * len(positions)  # deterministic per-point time
        return [0.0, duration], ['velocities'], 0.1, 'seam-velocity'

    @staticmethod
    def build_goal_message(times, positions, velocities):
        return {'points': len(positions), 'duration': times[-1]}


class FakePlanned:
    def __init__(self, window, index):
        self.positions = list(window)
        self.end_joints = ('end', index)


class FakePlanner:
    def __init__(self, log, fail_at=()):
        self.log = log
        self.fail_at = set(fail_at)  # plan call indices that return None
        self.calls = 0

    def plan_goal(self, waypoints, start_joints):
        index = self.calls
        self.calls += 1
        self.log.append(('plan', index, len(waypoints)))
        if index in self.fail_at:
            return None
        return FakePlanned(waypoints, index)


class FakePipeline:
    def __init__(self, log, reject_at=()):
        self.log = log
        self.reject_at = set(reject_at)

    def submit(self, goal_index, message, timeout_sec=30.0):
        self.log.append(('submit', goal_index, message['duration']))
        return goal_index not in self.reject_at

    def drain(self, timeout_sec):
        self.log.append(('drain',))
        return True


class FakeFinalize:
    def __init__(self, log):
        self.log = log

    def set_finalize(self, value):
        self.log.append(('finalize', value))
        return True


class Rig:
    """One streamer with fakes and a scripted poll-driven clock."""

    def __init__(self, plan_fail_at=(), submit_reject_at=(), goal_seconds=GOAL_SECONDS):
        self.now = 0.0
        self.log = []
        self.events = []
        self.script = []           # callables, one consumed per poll
        self.planner = FakePlanner(self.log, plan_fail_at)
        self.pipeline = FakePipeline(self.log, submit_reject_at)
        self.streamer = StrokeChainStreamer(
            CONFIG, on_event=self.events.append, sender_module=FakeSender,
            goal_seconds=goal_seconds, chain_close_margin=0.75,
            clock=lambda: self.now, ros_factory=self._ros, start_thread=False)

    def _ros(self):
        return {'planner': self.planner, 'pipeline': self.pipeline,
                'finalize': FakeFinalize(self.log),
                'read_joints': lambda: [0.0] * 6,
                'settle': lambda timeout, hold: True,
                'poll': self._poll}

    def _poll(self, timeout_sec):
        if self.script:
            self.script.pop(0)()
        self.now += max(timeout_sec, 0.02)
        if self.now > 1000.0:
            raise RuntimeError('scenario never converged (runaway wait loop)')

    def draw_line(self, start_x, points, step=0.003):
        self.streamer.begin_stroke((start_x, 0.0))
        for index in range(1, points):
            self.streamer.add_point((start_x + index * step, 0.0))

    def finalize_values(self):
        return [entry[1] for entry in self.log if entry[0] == 'finalize']

    def submits(self):
        return [entry for entry in self.log if entry[0] == 'submit']

    def stroke_events(self):
        return [event['event'] for event in self.events
                if event.get('kind') == 'stroke']


@pytest.fixture
def rig():
    return Rig()


class TestIntakeSynthesis:
    def test_begin_add_end_polyline(self, rig):
        rig.streamer.begin_stroke((0.01, 0.02))
        rig.streamer.add_point((0.02, 0.02))
        rig.streamer.end_stroke()
        window = rig.streamer._intake.take_window(1e9, 1.0, flush=True)
        assert window == [
            (0.01, 0.02, PLANE.safe_z),   # travel at the safe height
            (0.01, 0.02, PLANE.draw_z),   # plunge
            (0.02, 0.02, PLANE.draw_z),   # the stroke
            (0.02, 0.02, PLANE.safe_z)]   # lift at the stroke's end

    def test_between_strokes_travel_and_plunge(self, rig):
        rig.streamer.begin_stroke((0.0, 0.0))
        rig.streamer.end_stroke()
        rig.streamer.begin_stroke((0.05, 0.0))
        window = rig.streamer._intake.take_window(1e9, 1.0, flush=True)
        assert window[-2:] == [(0.05, 0.0, PLANE.safe_z),
                               (0.05, 0.0, PLANE.draw_z)]

    def test_decimation_keeps_spacing(self, rig):
        rig.streamer.begin_stroke((0.0, 0.0))
        for index in range(100):
            rig.streamer.add_point((index * 0.0005, 0.0))  # below 0.002 spacing
        window = rig.streamer._intake.take_window(1e9, 1.0, flush=True)
        draws = [p for p in window if p[2] == PLANE.draw_z]
        gaps = [math.dist(a, b) for a, b in zip(draws, draws[1:])]
        assert all(gap >= CONFIG.resample_spacing - 1e-12 for gap in gaps)

    def test_add_point_without_stroke_reports_unopened(self, rig):
        assert rig.streamer._intake.add_point((0.0, 0.0)) is False


class TestFinalizeDiscipline:
    def test_interior_goals_only_with_planned_successor(self, rig):
        # Three windows' worth of stroke, then a hand-back.
        rig.draw_line(-0.09, points=100)   # ~0.3 m of path = 3+ windows
        rig.streamer.end_stroke()
        rig.streamer.request_handback()
        rig.streamer._run_session()
        finalize = rig.finalize_values()
        assert len(finalize) >= 2
        assert finalize[:-1] == [False] * (len(finalize) - 1)
        assert finalize[-1] is True

    def test_plan_before_submit_ordering(self, rig):
        rig.draw_line(-0.09, points=100)
        rig.streamer.end_stroke()
        rig.streamer.request_handback()
        rig.streamer._run_session()
        submits = [i for i, entry in enumerate(rig.log) if entry[0] == 'submit']
        plans = [i for i, entry in enumerate(rig.log) if entry[0] == 'plan']
        # Submit k (non-final) must come after plan k+1.
        for goal_index in range(len(submits) - 1):
            assert plans[goal_index + 1] < submits[goal_index]

    def test_single_window_stroke_is_final(self, rig):
        rig.draw_line(0.0, points=5)
        rig.streamer.end_stroke()
        rig.streamer.request_handback()
        rig.streamer._run_session()
        assert rig.finalize_values() == [True]
        assert len(rig.submits()) == 1
        assert 'chain_opened' in rig.stroke_events()
        assert 'chain_closed' in rig.stroke_events()

    def test_dot_stroke_still_executes(self, rig):
        rig.streamer.begin_stroke((0.0, 0.0))   # pen tap: travel + plunge only
        rig.streamer.end_stroke()
        rig.streamer.request_handback()
        rig.streamer._run_session()
        assert rig.finalize_values() == [True]


class TestCatchUpAndFailure:
    def test_catchup_deadline_closes_at_rest(self):
        rig = Rig()
        rig.draw_line(-0.09, points=80)  # two-plus windows, pen stays down
        # No end_stroke, no handback: the intake runs dry mid-stroke; the
        # scripted polls only let time pass until the deadline closes the
        # chain, then a hand-back ends the session.
        rig.script = [lambda: None] * 50 + [rig.streamer.end_stroke,
                                            rig.streamer.request_handback]
        rig.script += [lambda: None] * 200
        rig.streamer._run_session()
        finalize = rig.finalize_values()
        assert finalize[-1] is True          # closed at rest (the dwell)
        # Two chains: the dwell close mid-stroke, then the lift after pen-up.
        assert rig.stroke_events().count('chain_closed') >= 2
        assert 'chain_failed' not in rig.stroke_events()

    def test_plan_failure_closes_previous_goal(self):
        rig = Rig(plan_fail_at={2})          # third plan call fails
        rig.draw_line(-0.09, points=120)
        rig.streamer.end_stroke()
        rig.streamer.request_handback()
        rig.streamer._run_session()
        assert rig.finalize_values()[-1] is True   # chain still closed at rest
        warnings = [e for e in rig.events if e.get('kind') == 'warning']
        assert any('failed to plan' in w['message'] for w in warnings)

    def test_goal0_plan_failure_is_soft(self):
        rig = Rig(plan_fail_at={0})
        rig.draw_line(0.0, points=5)
        rig.streamer.end_stroke()
        rig.streamer.request_handback()
        rig.streamer._run_session()
        assert rig.submits() == []
        assert 'chain_failed' in rig.stroke_events()
        assert not rig.streamer._stop_flag.is_set()  # nothing was open

    def test_submit_failure_latches_stopped(self):
        rig = Rig(submit_reject_at={0})
        rig.draw_line(-0.09, points=100)
        rig.streamer.end_stroke()
        rig.streamer.request_handback()
        rig.streamer._run_session()
        assert rig.streamer._stop_flag.is_set()
        assert 'chain_failed' in rig.stroke_events()
        health = [e for e in rig.events if e.get('kind') == 'health']
        assert health and not health[0]['ok']

    def test_stop_mid_chain_never_submits_more(self):
        rig = Rig()
        rig.draw_line(-0.09, points=80)      # pen stays down (no handback)
        rig.script = [rig.streamer.stop] + [lambda: None] * 50
        rig.streamer._run_session()
        submits_after_stop = [entry for entry in rig.log
                              if entry[0] == 'submit']
        # The stop arrived while waiting for the successor window: at most the
        # already-planned goals went out, and nothing after the stop.
        assert rig.streamer._stop_flag.is_set()
        assert rig.streamer._intake.pending == 0

    def test_stopped_route_admits_no_new_strokes(self, rig):
        rig.streamer.stop()
        rig.streamer.begin_stroke((0.0, 0.0))
        rig.streamer.add_point((0.01, 0.0))
        assert rig.streamer._intake.pending == 0
        assert not rig.streamer.active


class TestPlaneOffset:
    """on the stroke route.

    The plane-to-world mapping has two call sites -- here and on the live
    route -- and nothing else asserts they agree. That is exactly the shape
    that drifts, so it is pinned here.
    """

    def test_world_conversion_matches_the_live_route(self):
        plane = PLANE._replace(plane_offset=(0.013, -0.007))
        streamer = StrokeChainStreamer(
            CONFIG._replace(plane=plane), sender_module=FakeSender,
            start_thread=False)
        window = [(0.0, 0.0, 0.0), (0.02, 0.01, -0.02), (0.04, -0.015, -0.02)]
        assert [position for position, _quat in streamer._world(window)] == \
            pytest.approx([plane_to_world(plane.effective_anchor(),
                                          plane.anchor_rpy, point)
                           for point in window])

    def test_set_plane_reaches_the_intake_as_well(self):
        # The intake holds its own plane reference: one component must never
        # carry two copies that can diverge.
        streamer = StrokeChainStreamer(CONFIG, sender_module=FakeSender,
                                       start_thread=False)
        moved = PLANE._replace(plane_offset=(0.01, 0.0))
        streamer.set_plane(moved)
        assert streamer._config.plane is moved
        assert streamer._intake._plane is moved
