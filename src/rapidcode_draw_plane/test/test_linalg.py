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

"""Unit tests for the linear algebra library (pure functions, no ROS)."""

import math

import pytest

from rapidcode_draw_plane import linalg

EXTENT = ((-0.1, -0.2), (0.3, 0.2))  # 0.4 x 0.4 m plane window


class TestReachBand:
    """The working area is plane-relative and travels with the plane
    origin, so this is the one bound judged in the planning frame."""

    def test_the_default_plane_sits_on_both_edges_and_still_fits(self):
        # The design derives the 0.16 m radius as the half-width of the
        # 0.30...0.62 m band, and puts the origin on that band's
        # midline at 0.46 m. So the stock configuration lands on BOTH limits
        # to the bit, and the bridge's ability to start must not hang on
        # floating-point rounding.
        assert linalg.disc_fits_reach_band((-0.46, 0.0), 0.16, 0.30, 0.62)

    def test_an_outward_offset_leaves_the_far_edge(self):
        assert not linalg.disc_fits_reach_band((-0.51, 0.0), 0.16, 0.30, 0.62)

    def test_an_inward_offset_crowds_the_base(self):
        assert not linalg.disc_fits_reach_band((-0.41, 0.0), 0.16, 0.30, 0.62)

    def test_the_bound_is_radial_not_per_axis(self):
        # Straight sideways is still a move outward: distance is what counts.
        assert not linalg.disc_fits_reach_band((-0.46, 0.08), 0.16, 0.30, 0.62)

    def test_a_smaller_working_disc_buys_room(self):
        # The lever an operator reaches for: trade drawing area for
        # calibration range.
        assert linalg.disc_fits_reach_band((-0.47, 0.0), 0.14, 0.30, 0.62)


class TestMapCanvasToPlane:
    def test_corners_map_with_y_flip_no_x_mirror(self):
        surface = (800.0, 800.0)
        # canvas top-left -> plane (x_min, y_max): y flips, x does not mirror
        assert linalg.map_canvas_to_plane((0.0, 0.0), surface, EXTENT) == (-0.1, 0.2)
        assert linalg.map_canvas_to_plane((800.0, 800.0), surface, EXTENT) == \
            pytest.approx((0.3, -0.2))

    def test_centre_maps_to_centre(self):
        assert linalg.map_canvas_to_plane((400.0, 400.0), (800.0, 800.0), EXTENT) == \
            pytest.approx((0.1, 0.0))

    def test_degenerate_surface_refused(self):
        with pytest.raises(ValueError):
            linalg.map_canvas_to_plane((0.0, 0.0), (0.0, 600.0), EXTENT)


class TestClamps:
    def test_clamp_to_extent(self):
        assert linalg.clamp_to_extent((1.0, -1.0), EXTENT) == (0.3, -0.2)
        assert linalg.clamp_to_extent((0.0, 0.0), EXTENT) == (0.0, 0.0)

    def test_clamp_to_working_area_projects_onto_disc(self):
        clamped = linalg.clamp_to_working_area((2.0, 0.0), (0.0, 0.0), 0.5)
        assert clamped == pytest.approx((0.5, 0.0))
        inside = (0.1, 0.2)
        assert linalg.clamp_to_working_area(inside, (0.0, 0.0), 0.5) == inside

    def test_limit_displacement_bounds_one_tick(self):
        stepped = linalg.limit_displacement((1.0, 0.0), (0.0, 0.0), 0.01)
        assert stepped == pytest.approx((0.01, 0.0))
        near = (0.005, 0.0)
        assert linalg.limit_displacement(near, (0.0, 0.0), 0.01) == near


class TestBoundVerdicts:
    LIMITS = linalg.Limits(
        extent=EXTENT, working_centre=(0.1, 0.0), working_radius=0.5,
        depth_min=-0.03, depth_max=0.05, max_speed=0.25)

    def test_command_inside_every_bound(self):
        verdicts = linalg.bound_verdicts(
            (0.1, 0.0, -0.02), (0.1, 0.001, -0.02), 0.02, self.LIMITS)
        assert verdicts == (True, True, True, True)
        assert verdicts.ok

    def test_each_bound_judged_independently(self):
        base = (0.1, 0.0, -0.02)
        out_extent = linalg.bound_verdicts((0.5, 0.0, -0.02), base, 0.02, self.LIMITS)
        assert not out_extent.extent_ok and out_extent.depth_ok
        too_deep = linalg.bound_verdicts((0.1, 0.0, -0.1), base, 0.02, self.LIMITS)
        assert not too_deep.depth_ok and too_deep.extent_ok
        # 10 mm in 20 ms = 0.5 m/s > 0.25 m/s
        overspeed = linalg.bound_verdicts((0.11, 0.0, -0.02), base, 0.02, self.LIMITS)
        assert not overspeed.speed_ok
        assert not overspeed.ok

    def test_nonpositive_period_fails_speed(self):
        verdicts = linalg.bound_verdicts(
            (0.1, 0.0, -0.02), (0.1, 0.0, -0.02), 0.0, self.LIMITS)
        assert not verdicts.speed_ok


class TestPrimitives:
    def test_sample_line_uniform_with_endpoints(self):
        stroke = linalg.sample_line((0.0, 0.0), (0.1, 0.0), 0.03)
        assert stroke[0] == (0.0, 0.0)
        assert stroke[-1] == pytest.approx((0.1, 0.0))
        gaps = [math.dist(a, b) for a, b in zip(stroke, stroke[1:])]
        assert all(gap <= 0.03 + 1e-12 for gap in gaps)
        assert max(gaps) == pytest.approx(min(gaps))

    def test_sample_arc_through_three_points(self):
        # Half circle of radius 0.05: (r,0) -> (0,r) -> (-r,0), centre origin.
        stroke = linalg.sample_arc((0.05, 0.0), (0.0, 0.05), (-0.05, 0.0), 0.005)
        assert stroke[0] == pytest.approx((0.05, 0.0))
        assert stroke[-1] == pytest.approx((-0.05, 0.0))
        for point in stroke:
            assert math.hypot(*point) == pytest.approx(0.05)
        # The middle click lies on the sampled sweep (max deviation < spacing).
        nearest = min(math.dist(point, (0.0, 0.05)) for point in stroke)
        assert nearest < 0.005

    def test_sample_arc_picks_the_sweep_through_the_middle_point(self):
        # Same endpoints, middle point below: the arc must sweep clockwise.
        stroke = linalg.sample_arc((0.05, 0.0), (0.0, -0.05), (-0.05, 0.0), 0.005)
        assert all(point[1] <= 1e-12 for point in stroke)

    def test_collinear_arc_degrades_to_line(self):
        stroke = linalg.sample_arc((0.0, 0.0), (0.05, 0.0), (0.1, 0.0), 0.03)
        assert stroke == linalg.sample_line((0.0, 0.0), (0.1, 0.0), 0.03)


class TestConditioning:
    def test_remove_duplicates_keeps_endpoints(self):
        stroke = [(0.0, 0.0), (0.0005, 0.0), (0.01, 0.0), (0.0101, 0.0)]
        kept = linalg.remove_duplicates(stroke, 0.001)
        assert kept[0] == (0.0, 0.0)
        assert kept[-1] == (0.0101, 0.0)
        assert (0.0005, 0.0) not in kept

    def test_resample_uniform_spacing_and_endpoints(self):
        stroke = [(0.0, 0.0), (0.03, 0.0), (0.1, 0.0)]
        out = linalg.resample_uniform(stroke, 0.025)
        assert out[0] == (0.0, 0.0)
        assert out[-1] == (0.1, 0.0)
        gaps = [math.dist(a, b) for a, b in zip(out, out[1:])]
        assert all(gap == pytest.approx(gaps[0]) for gap in gaps)
        assert gaps[0] <= 0.025 + 1e-12

    def test_resample_short_strokes_pass_through(self):
        assert linalg.resample_uniform([(0.0, 0.0)], 0.01) == [(0.0, 0.0)]
        assert linalg.resample_uniform([], 0.01) == []


class TestWorkingAreaSplit:
    def test_split_separates_runs_and_short_fragments(self):
        centre, radius = (0.0, 0.0), 0.1
        stroke = (
            [(-0.2, 0.0), (-0.15, 0.0)]              # outside -> excluded
            + [(-0.05, 0.0), (0.0, 0.0), (0.05, 0.0)]  # inside run of 3
            + [(0.2, 0.0)]                            # outside
            + [(0.05, 0.01)]                          # inside but < min_run
        )
        in_runs, excluded = linalg.split_at_working_area(stroke, centre, radius, 2)
        assert in_runs == [[(-0.05, 0.0), (0.0, 0.0), (0.05, 0.0)]]
        assert [(-0.2, 0.0), (-0.15, 0.0)] in excluded
        assert [(0.05, 0.01)] in excluded

    def test_fully_inside_stroke_is_one_run(self):
        stroke = [(0.0, 0.0), (0.01, 0.0)]
        in_runs, excluded = linalg.split_at_working_area(stroke, (0.0, 0.0), 1.0, 2)
        assert in_runs == [stroke]
        assert excluded == []


class TestExtentFits:
    def test_fit_and_refuse(self):
        assert linalg.extent_fits_working_area(
            ((-0.1, -0.1), (0.1, 0.1)), (0.0, 0.0), 0.15)
        assert not linalg.extent_fits_working_area(
            ((-0.1, -0.1), (0.1, 0.1)), (0.0, 0.0), 0.12)  # corner at ~0.141


class TestRotateInPlane:
    """The plane's own rotation, degrees CCW about the plane origin.

    The sender's ``build_waypoints`` implements the identical rotation for the
    batch route; ``test_recipe_loader`` is where the two are proved equal.
    """

    def test_zero_is_the_identity_exactly(self):
        # Not approximately: the live route calls this on every tick, and at
        # the stock calibration it must be the pure translation it always was.
        point = (0.037, -0.0125)
        assert linalg.rotate_in_plane(point, 0.0) == point

    def test_ninety_degrees_takes_plane_x_to_plane_y(self):
        assert linalg.rotate_in_plane((0.1, 0.0), 90.0) == \
            pytest.approx((0.0, 0.1))

    def test_the_sign_is_counter_clockwise(self):
        # Positive degrees turn +x toward +y, matching the recipe format's
        # `rotate` and the right-hand rule about the planning frame's +z.
        x, y = linalg.rotate_in_plane((0.1, 0.0), 30.0)
        assert x == pytest.approx(0.1 * math.cos(math.radians(30.0)))
        assert y == pytest.approx(0.1 * math.sin(math.radians(30.0)))
        assert y > 0.0

    def test_negative_degrees_turn_the_other_way(self):
        assert linalg.rotate_in_plane((0.1, 0.0), -90.0) == \
            pytest.approx((0.0, -0.1))

    def test_the_origin_is_the_fixed_point(self):
        # Which is why the rotation belongs on the point and not on the
        # anchor: the anchor IS the centre it turns about.
        assert linalg.rotate_in_plane((0.0, 0.0), 47.5) == \
            pytest.approx((0.0, 0.0))

    def test_rotation_preserves_the_radius(self):
        point = (0.06, -0.08)
        for degrees in (5.0, 30.0, 45.0, 90.0, 180.0, 275.0):
            turned = linalg.rotate_in_plane(point, degrees)
            assert math.hypot(*turned) == pytest.approx(math.hypot(*point))

    def test_angles_compose_by_addition(self):
        # The property the whole design leans on: an artwork rotated 30 on a
        # plane calibrated to 45 is the same as one rotation of 75, so the
        # sender can just add the two.
        point = (0.03, 0.02)
        twice = linalg.rotate_in_plane(
            linalg.rotate_in_plane(point, 30.0), 45.0)
        assert twice == pytest.approx(linalg.rotate_in_plane(point, 75.0))

    def test_a_full_turn_returns_the_point(self):
        assert linalg.rotate_in_plane((0.05, 0.01), 360.0) == \
            pytest.approx((0.05, 0.01))
