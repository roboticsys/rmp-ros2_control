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

"""Drawing store: the ordered strokes of the current drawing.

A stroke is tagged {polyline, line, arc} (stakeholder direction, 2026-08-21). A
line or arc carries its defining parameters AND its plane coordinates sampled at
the resample spacing, so every downstream consumer -- execution, saving,
publication -- reads sampled points and is unchanged by the primitive kinds.

The store holds conditioned plane-frame data only; it never sees canvas
coordinates and never talks to ROS. Owned and driven by the drawing surface
client; the bridge validates submitted drawings instead of
retaining them.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Tuple

Coordinate = Tuple[float, float]

POLYLINE = 'polyline'
LINE = 'line'
ARC = 'arc'
_KINDS = (POLYLINE, LINE, ARC)


@dataclass(frozen=True)
class Stroke:
    """One tagged, conditioned stroke in plane coordinates."""

    kind: str
    points: List[Coordinate]
    # The primitive's defining parameters: {} for a polyline, the operator's
    # clicked plane coordinates for a line ('start', 'end') or arc ('first',
    # 'second', 'third').
    params: Dict[str, Coordinate] = field(default_factory=dict)
    # Chained: this stroke continues the previous stroke with no
    # pen lift between them; its first point is the previous stroke's last.
    chained: bool = False

    def __post_init__(self):
        if self.kind not in _KINDS:
            raise ValueError(f'unknown stroke kind {self.kind!r}')
        if not self.points:
            raise ValueError('a stroke must hold at least one point')


class DrawingStore:
    """Ordered strokes with undo, clear, and the retention bound.

    ``max_samples`` bounds the TOTAL point count across the drawing (
    ): ``append`` refuses a stroke that would exceed it, so a runaway
    input stream can never grow the store without bound.
    """

    def __init__(self, max_samples: int):
        if max_samples <= 0:
            raise ValueError(f'max_samples must be positive, got {max_samples}')
        self._max_samples = max_samples
        self._strokes: List[Stroke] = []
        self._sample_count = 0

    def append(self, stroke: Stroke) -> bool:
        """Retain the stroke; False when the sample bound refuses it."""
        if self._sample_count + len(stroke.points) > self._max_samples:
            return False
        self._strokes.append(stroke)
        self._sample_count += len(stroke.points)
        return True

    def strokes(self) -> List[Stroke]:
        """The ordered drawing (a copy of the list; strokes are frozen)."""
        return list(self._strokes)

    def undo_last(self) -> bool:
        """Remove the newest stroke; False when the drawing is empty."""
        if not self._strokes:
            return False
        removed = self._strokes.pop()
        self._sample_count -= len(removed.points)
        return True

    def clear(self) -> None:
        """Empty the drawing."""
        self._strokes.clear()
        self._sample_count = 0

    @property
    def sample_count(self) -> int:
        return self._sample_count

    def __len__(self) -> int:
        return len(self._strokes)
