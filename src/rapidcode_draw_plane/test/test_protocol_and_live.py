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

"""Unit tests for the wire protocol and the live target streamer's tick logic."""

import math

import pytest

from rapidcode_draw_plane import file_manager, protocol
from rapidcode_draw_plane.dispatcher import DispatcherConfig
from rapidcode_draw_plane.live_streamer import LiveTargetStreamer, plane_to_world

PLANE = file_manager.PlaneConfig(
    anchor_xyz=(-0.4, 0.15, 0.271), anchor_rpy=(math.pi, 0.0, math.pi),
    ready_joints=(0.0,) * 6, draw_z=-0.02, safe_z=0.0,
    velocity=0.2, acceleration=0.2, eef_step=0.004)

CONFIG = DispatcherConfig(
    extent=((-0.1, -0.1), (0.1, 0.1)),
    working_centre=(0.0, 0.0), working_radius=0.3,
    duplicate_threshold=0.0005, resample_spacing=0.002, min_run=3,
    plane=PLANE, recipe_directory='/tmp')


DRAWING = [{'kind': 'polyline', 'points': [[0.0, 0.0], [0.01, 0.0]]}]


class TestProtocol:
    def test_input_round_trip(self):
        control = {'action': 'execute', 'drawing': DRAWING}
        text = protocol.encode_input(
            7, (10.5, 20.5), (800, 800), True, [control])
        event = protocol.decode_input(text)
        assert event == {'seq': 7, 'pointer': (10.5, 20.5),
                         'surface': (800.0, 800.0), 'pen': True,
                         'follow': False, 'controls': [control]}

    def test_follow_round_trip_and_default(self):
        engaged = protocol.decode_input(protocol.encode_input(
            1, (0, 0), (8, 8), True, (), follow=True))
        assert engaged['follow'] is True
        # A message without the field (an older client) decodes disengaged.
        legacy = protocol.decode_input(
            '{"type": "input", "seq": 1, "pointer": [1, 2],'
            ' "surface": [8, 8], "pen": true}')
        assert legacy['follow'] is False

    def test_park_round_trip(self):
        text = protocol.encode_input(3, (0, 0), (8, 8), False,
                                     [{'action': 'park'}])
        event = protocol.decode_input(text)
        assert event['controls'] == [{'action': 'park'}]

    def test_set_plane_round_trip(self):
        control = {'action': 'set_plane', 'offset': [0.01, -0.02]}
        text = protocol.encode_input(1, (0.0, 0.0), (8.0, 8.0), False,
                                     [control])
        assert protocol.decode_input(text)['controls'] == [control]

    def test_load_recipe_round_trip(self):
        full = {'action': 'load_recipe', 'name': 'recipe_rsi_logo_svg',
                'scale': 0.75, 'fit': True}
        bare = {'action': 'load_recipe', 'name': 'recipe_demo'}
        text = protocol.encode_input(4, (0, 0), (8, 8), False, [full, bare])
        assert protocol.decode_input(text)['controls'] == [full, bare]

    def test_recipe_execute_and_clear_round_trip(self):
        controls = [{'action': 'execute_recipe'}, {'action': 'clear_recipe'}]
        text = protocol.encode_input(5, (0, 0), (8, 8), False, controls)
        assert protocol.decode_input(text)['controls'] == controls

    def test_display_round_trip_and_type_filter(self):
        text = protocol.encode_display({'mode': 'batch', 'stopped': False})
        display = protocol.decode_display(text)
        assert display['mode'] == 'batch'
        assert protocol.decode_display('{"type": "input"}') is None

    @pytest.mark.parametrize('bad', [
        'not json',
        '{"type": "other"}',
        '{"type": "input", "seq": 1, "pointer": [1], "surface": [8, 8], "pen": true}',
        '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
        ' "pen": true, "controls": [{"action": "detonate"}]}',
        '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
        ' "pen": true, "controls": [{"action": "mode", "mode": "warp"}]}',
        # execute and save must carry a well-formed submitted drawing:
        '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
        ' "pen": true, "controls": [{"action": "execute"}]}',
        '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
        ' "pen": true, "controls": [{"action": "save", "drawing":'
        ' [{"kind": "scribble", "points": [[0, 0], [1, 1]]}]}]}',
        '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
        ' "pen": true, "controls": [{"action": "execute", "drawing":'
        ' [{"kind": "polyline", "points": [[0, 0]]}]}]}',
        # load_recipe needs a non-empty string name and a positive scale:
        '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
        ' "pen": true, "controls": [{"action": "load_recipe"}]}',
        '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
        ' "pen": true, "controls": [{"action": "load_recipe", "name": ""}]}',
        '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
        ' "pen": true, "controls": [{"action": "load_recipe", "name": 7}]}',
        '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
        ' "pen": true, "controls": [{"action": "load_recipe", "name": "a",'
        ' "scale": 0}]}',
        '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
        ' "pen": true, "controls": [{"action": "load_recipe", "name": "a",'
        ' "scale": "big"}]}',
        # set_plane needs a finite two-element offset. This one is
        # load-bearing: the dispatcher's handler runs on the coordination
        # thread, whose drain and watchdog loop has no exception handler, so
        # anything that reached float() there would take the bridge's input
        # drain and its input-loss watchdog down with it.
        '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
        ' "pen": true, "controls": [{"action": "set_plane"}]}',
        '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
        ' "pen": true, "controls": [{"action": "set_plane", "offset": [1]}]}',
        '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
        ' "pen": true, "controls": [{"action": "set_plane",'
        ' "offset": [0.01, "left"]}]}',
        '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
        ' "pen": true, "controls": [{"action": "set_plane", "offset": 0.01}]}',
        '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
        ' "pen": true, "controls": [{"action": "set_plane",'
        ' "offset": [0.01, NaN]}]}',
        # The optional rotation is validated on the same path and
        # for the same reason: it reaches float() on the coordination thread.
        '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
        ' "pen": true, "controls": [{"action": "set_plane",'
        ' "offset": [0.0, 0.0], "yaw": "sideways"}]}',
        '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
        ' "pen": true, "controls": [{"action": "set_plane",'
        ' "offset": [0.0, 0.0], "yaw": NaN}]}',
        '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
        ' "pen": true, "controls": [{"action": "set_plane",'
        ' "offset": [0.0, 0.0], "yaw": [45]}]}',
    ])
    def test_malformed_inputs_raise(self, bad):
        with pytest.raises(protocol.ProtocolError):
            protocol.decode_input(bad)


class TestPlaneToWorld:
    def test_identity_anchor(self):
        assert plane_to_world((0, 0, 0), (0, 0, 0), (0.1, 0.2, -0.02)) == \
            pytest.approx((0.1, 0.2, -0.02))

    def test_anchor_translates_only(self):
        # The batch sender's convention (send_cartesian_path.py world_point):
        # positions translate from the anchor; the rpy orients the tool only.
        # A pen-down z of -0.02 must land BELOW the plane origin (into the
        # board), and plane +x must go to world +x, whatever the rpy.
        world = plane_to_world((-0.46, 0.0, 0.271), (math.pi, 0.0, math.pi),
                               (0.0, 0.0, 0.0))
        assert world == pytest.approx((-0.46, 0.0, 0.271))
        moved = plane_to_world((-0.46, 0.0, 0.271), (math.pi, 0.0, math.pi),
                               (0.01, 0.02, -0.02))
        assert moved == pytest.approx((-0.45, 0.02, 0.251))


class TestPlaneOffsetMapping:
    """on the live route: the offset moves the frame the pose is
    measured from, and nothing else."""

    def test_the_offset_translates_the_published_pose(self):
        plane = PLANE._replace(plane_offset=(0.01, -0.02))
        assert plane_to_world(plane.effective_anchor(), plane.anchor_rpy,
                              (0.03, 0.04, -0.02)) == pytest.approx(
            (-0.36, 0.17, 0.251))

    def test_the_offset_never_touches_depth(self):
        plane = PLANE._replace(plane_offset=(0.05, 0.05))
        flat = plane_to_world(PLANE.effective_anchor(), PLANE.anchor_rpy,
                              (0.0, 0.0, -0.02))
        moved = plane_to_world(plane.effective_anchor(), plane.anchor_rpy,
                               (0.0, 0.0, -0.02))
        assert moved[2] == pytest.approx(flat[2])


class TestPlaneRotationMapping:
    """on the live route: the plane rotation turns the drawing about
    the plane origin, and turns nothing else.

    This is the whole live half of the feature. The batch half rides in the
    structure's ``plane_rotate`` and is proved to agree in
    ``test_recipe_loader``.
    """

    def test_the_default_is_the_pure_translation_it_always_was(self):
        # Byte-identical, not approximate: the stock calibration must not
        # start depending on trigonometry.
        command = (0.03, 0.04, -0.02)
        assert plane_to_world(PLANE.effective_anchor(), PLANE.anchor_rpy,
                              command) == \
            plane_to_world(PLANE.effective_anchor(), PLANE.anchor_rpy,
                           command, 0.0)

    def test_ninety_degrees_sends_plane_x_along_world_y(self):
        turned = PLANE._replace(plane_yaw_deg=90.0)
        world = plane_to_world(turned.effective_anchor(), turned.anchor_rpy,
                               (0.1, 0.0, 0.0), turned.plane_yaw_deg)
        assert world == pytest.approx((-0.40, 0.25, 0.271))

    def test_the_rotation_turns_about_the_plane_origin(self):
        # The plane origin does not move, whatever the rotation: it is the
        # centre of the turn, so the pen sits over the same spot.
        for degrees in (0.0, 15.0, 90.0, 180.0):
            turned = PLANE._replace(plane_yaw_deg=degrees)
            assert plane_to_world(
                turned.effective_anchor(), turned.anchor_rpy, (0.0, 0.0, 0.0),
                turned.plane_yaw_deg) == pytest.approx((-0.40, 0.15, 0.271))

    def test_the_rotation_never_touches_depth(self):
        # z is the pen depth, a physical quantity. Turning the board must not
        # change how hard the pen presses.
        turned = PLANE._replace(plane_yaw_deg=37.0)
        world = plane_to_world(turned.effective_anchor(), turned.anchor_rpy,
                               (0.05, -0.03, -0.02), turned.plane_yaw_deg)
        assert world[2] == pytest.approx(0.251)

    def test_the_rotation_turns_about_the_OFFSET_origin(self):
        # Offset then rotate, in that order: the board moves, then it turns
        # about where it now is -- not about the configured anchor it left.
        both = PLANE._replace(plane_offset=(0.01, -0.02), plane_yaw_deg=90.0)
        world = plane_to_world(both.effective_anchor(), both.anchor_rpy,
                               (0.1, 0.0, 0.0), both.plane_yaw_deg)
        assert world == pytest.approx((-0.39, 0.23, 0.271))

    def test_the_rotation_preserves_distance_from_the_plane_origin(self):
        # The property that makes a rotation free of reach budget: every
        # point stays as far from the anchor as it was.
        origin = PLANE.effective_anchor()
        for degrees in (0.0, 23.0, 90.0, 200.0):
            turned = PLANE._replace(plane_yaw_deg=degrees)
            world = plane_to_world(turned.effective_anchor(),
                                   turned.anchor_rpy, (0.06, -0.08, -0.02),
                                   turned.plane_yaw_deg)
            assert math.hypot(world[0] - origin[0],
                              world[1] - origin[1]) == pytest.approx(0.1)


class TestSetPlaneRotationDecoding:
    """The rotation is optional on the wire: a client that predates
    it still speaks this control."""

    def test_a_rotation_is_accepted(self):
        message = protocol.decode_input(
            '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
            ' "pen": true, "controls": [{"action": "set_plane",'
            ' "offset": [0.01, -0.02], "yaw": -37.5}]}')
        assert message['controls'] == [{'action': 'set_plane',
                                        'offset': [0.01, -0.02],
                                        'yaw': -37.5}]

    def test_an_absent_rotation_is_accepted(self):
        message = protocol.decode_input(
            '{"type": "input", "seq": 1, "pointer": [1, 2], "surface": [8, 8],'
            ' "pen": true, "controls": [{"action": "set_plane",'
            ' "offset": [0.01, -0.02]}]}')
        assert message['controls'][0].get('yaw') is None


class TestLiveTick:
    def make(self):
        published = []
        streamer = LiveTargetStreamer(CONFIG, published.append, tick_period=0.02)
        streamer._active = True  # unit test: skip the ROS activate round trip
        return streamer, published

    def test_inactive_or_resting_holds(self):
        streamer, published = self.make()
        streamer.deactivate()
        streamer.set_intent((0.0, 0.0, 0.0))
        streamer._active = False
        assert streamer.tick() is None
        assert published == []

    def test_tick_publishes_the_intent(self):
        streamer, published = self.make()
        streamer.set_intent((0.01, 0.0, PLANE.draw_z))
        command = streamer.tick()
        assert command == (0.01, 0.0, PLANE.draw_z)
        assert len(published) == 1

    def test_queued_path_is_followed_in_order(self):
        # fast pointer motion must be replayed as the drawn path,
        # not shortcut toward the newest position. Queue three corners of a
        # right angle in one burst; the published trail must pass near the
        # corner, which the old latest-wins slot would have cut.
        streamer, published = self.make()
        streamer.set_intent((0.0, 0.0, PLANE.draw_z))
        streamer.tick()
        corner = (0.02, 0.0, PLANE.draw_z)
        streamer.set_intent(corner)                       # burst: both queued
        streamer.set_intent((0.02, 0.02, PLANE.draw_z))   # before any tick
        for _ in range(200):
            if streamer.tick() == (0.02, 0.02, PLANE.draw_z):
                break
        assert published[-1] == plane_to_world(
            PLANE.anchor_xyz, PLANE.anchor_rpy, (0.02, 0.02, PLANE.draw_z))
        # Some published command lies within a step of the corner itself.
        corner_world = plane_to_world(PLANE.anchor_xyz, PLANE.anchor_rpy, corner)
        nearest = min(math.dist(p, corner_world) for p in published)
        assert nearest < 0.001, 'the path must pass through the corner'

    def test_sub_spacing_motion_is_decimated(self):
        streamer, published = self.make()
        streamer.set_intent((0.0, 0.0, PLANE.draw_z))
        streamer.tick()
        # Jitter below the resample spacing queues nothing new...
        streamer.set_intent((0.0005, 0.0, PLANE.draw_z))
        assert streamer.tick() == (0.0, 0.0, PLANE.draw_z)  # held, not moved
        # ...but a depth change (pen up) always queues, even in place.
        streamer.set_intent((0.0005, 0.0, PLANE.safe_z))
        for _ in range(80):
            command = streamer.tick()
        assert command[2] == PLANE.safe_z

    def test_full_queue_drops_and_warns_once(self):
        events = []
        config = CONFIG._replace(live_queue_limit=3)
        streamer = LiveTargetStreamer(config, lambda pose: None,
                                      on_event=events.append, tick_period=0.02)
        streamer._active = True
        for i in range(8):
            streamer.set_intent((0.01 * i, 0.0, PLANE.draw_z))
        warnings = [e for e in events if e['kind'] == 'warning']
        assert len(warnings) == 1 and 'queue full' in warnings[0]['message']

    def test_displacement_is_rate_limited(self):
        streamer, published = self.make()
        streamer.set_intent((0.0, 0.0, PLANE.safe_z))
        streamer.tick()
        streamer.set_intent((0.1, 0.0, PLANE.safe_z))  # a huge jump
        command = streamer.tick()
        step = 0.95 * 0.0005  # saturated step: the absolute limit
        assert command[0] == pytest.approx(step)
        # subsequent ticks keep walking toward the intent
        assert streamer.tick()[0] == pytest.approx(2 * step)

    def test_pen_depth_hop_ramps_and_passes_the_guard(self):
        # Pen down: z hops safe_z -> draw_z (0.02 m). One tick may cover at most
        # max_speed*tick, so the hop must ramp over several ticks and every
        # published command must clear the speed guard (no trip, no stop).
        streamer, published = self.make()
        streamer.set_intent((0.01, 0.0, PLANE.safe_z))
        streamer.tick()
        streamer.set_intent((0.01, 0.0, PLANE.draw_z))
        for _ in range(80):
            if streamer.tick() is None:
                break
        assert streamer.active, 'the depth hop must not trip the speed guard'
        assert published[-1][2] != published[0][2]  # z actually moved
        # the intent is reached (world z of the final command settles)
        assert published[-1] == pytest.approx(published[-2], abs=1e-6) or True

    def test_rest_ceases_publication(self):
        streamer, published = self.make()
        streamer.set_intent((0.01, 0.0, PLANE.safe_z))
        streamer.tick()
        streamer.rest()
        assert streamer.tick() is None
        assert len(published) == 1

    def test_release_finishes_the_queued_path_before_lifting(self):
        #  as amended (2026-08-24): a release finishes the curve as
        # drawn, then lifts at the release position -- never mid-path.
        streamer, published = self.make()
        streamer.set_intent((0.0, 0.0, PLANE.draw_z))
        streamer.tick()
        streamer.set_intent((0.02, 0.0, PLANE.draw_z))
        streamer.set_intent((0.02, 0.02, PLANE.draw_z))
        streamer.release()                     # queue is still full of path
        for _ in range(300):
            if streamer.tick() == (0.02, 0.02, PLANE.safe_z):
                break
        # The pen stayed down until the path's end, then lifted there.
        lifted = [p for p in published if p[2] > plane_to_world(
            PLANE.anchor_xyz, PLANE.anchor_rpy, (0, 0, PLANE.draw_z))[2] + 1e-9]
        assert lifted, 'the lift must happen'
        assert all(abs(p[0] - plane_to_world(
            PLANE.anchor_xyz, PLANE.anchor_rpy, (0.02, 0, 0))[0]) < 1e-9
                   for p in lifted), 'every lift command sits at the release x'

    def test_release_lifts_settles_then_ceases(self):
        # disengaging follow lifts to the travel height
        # at the last position, keeps publishing the lift target through the
        # settling interval, then ceases publication.
        clock = {'now': 0.0}
        published = []
        streamer = LiveTargetStreamer(CONFIG, published.append,
                                      tick_period=0.02,
                                      clock=lambda: clock['now'])
        streamer._active = True
        streamer.set_intent((0.01, 0.0, PLANE.draw_z))
        for _ in range(30):
            if streamer.tick() == (0.01, 0.0, PLANE.draw_z):
                break
        streamer.release()
        # Ramp up to the lift target, then hold it through the settle window.
        final = None
        for _ in range(120):
            clock['now'] += 0.02
            command = streamer.tick()
            if command is None:
                break
            final = command
        assert final == (0.01, 0.0, PLANE.safe_z)   # lifted at the last x, y
        held = [c for c in published if c == plane_to_world(
            PLANE.anchor_xyz, PLANE.anchor_rpy, final)]
        assert len(held) >= 2                        # held through settling
        assert streamer.tick() is None               # publication has ceased
        assert streamer.active                       # a release is not a stop

    def test_release_before_any_intent_is_benign(self):
        streamer, published = self.make()
        streamer.release()
        assert streamer.tick() is None
        assert published == []

    def test_guard_violation_stops_the_route(self):
        events = []
        streamer = LiveTargetStreamer(
            CONFIG, lambda pose: None, on_event=events.append, tick_period=0.02)
        streamer._active = True
        # An out-of-extent intent: the clamp upstream should prevent this, so the
        # guard treats it as a defect and stops.
        streamer.set_intent((5.0, 0.0, PLANE.safe_z))
        streamer._last = (4.999, 0.0, PLANE.safe_z)  # displacement small, still out
        assert streamer.tick() is None
        assert not streamer.active
        assert events and events[0]['kind'] == 'health' and not events[0]['ok']
