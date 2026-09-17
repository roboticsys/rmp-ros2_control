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

"""Unit tests for the drawing store."""

import pytest

from rapidcode_draw_plane.drawing_store import ARC, LINE, POLYLINE, DrawingStore, Stroke


def poly(n, y=0.0):
    return Stroke(POLYLINE, [(0.001 * i, y) for i in range(n)])


class TestStroke:
    def test_kinds_and_params(self):
        line = Stroke(LINE, [(0.0, 0.0), (0.1, 0.0)],
                      params={'start': (0.0, 0.0), 'end': (0.1, 0.0)})
        assert line.kind == LINE
        arc = Stroke(ARC, [(0.0, 0.0), (0.05, 0.05), (0.1, 0.0)],
                     params={'first': (0.0, 0.0), 'second': (0.05, 0.05),
                             'third': (0.1, 0.0)})
        assert arc.params['second'] == (0.05, 0.05)

    def test_invalid_strokes_refused(self):
        with pytest.raises(ValueError):
            Stroke('spline', [(0.0, 0.0)])
        with pytest.raises(ValueError):
            Stroke(POLYLINE, [])


class TestDrawingStore:
    def test_append_ordered_and_counted(self):
        store = DrawingStore(max_samples=100)
        assert store.append(poly(3))
        assert store.append(poly(4, y=0.1))
        assert len(store) == 2
        assert store.sample_count == 7
        assert [len(s.points) for s in store.strokes()] == [3, 4]

    def test_sample_bound_refuses_not_truncates(self):
        store = DrawingStore(max_samples=5)
        assert store.append(poly(3))
        assert not store.append(poly(3))  # 3 + 3 > 5 -> refused whole
        assert len(store) == 1
        assert store.sample_count == 3
        assert store.append(poly(2))  # exactly at the bound is retained

    def test_undo_restores_budget(self):
        store = DrawingStore(max_samples=5)
        store.append(poly(3))
        store.append(poly(2))
        assert store.undo_last()
        assert store.sample_count == 3
        assert store.append(poly(2))
        assert store.undo_last() and store.undo_last()
        assert not store.undo_last()  # empty drawing: nothing to undo

    def test_clear_empties_the_drawing(self):
        store = DrawingStore(max_samples=10)
        store.append(poly(3))
        store.clear()
        assert len(store) == 0
        assert store.sample_count == 0
        assert store.strokes() == []

    def test_positive_bound_required(self):
        with pytest.raises(ValueError):
            DrawingStore(max_samples=0)
