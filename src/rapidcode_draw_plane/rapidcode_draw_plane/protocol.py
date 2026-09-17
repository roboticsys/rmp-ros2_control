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

"""The client-to-bridge wire protocol: JSON text over one WebSocket.

Client to bridge, every send tick (<= 0.01 s), unconditional -- the cadence IS
the liveness signal::

    {"type": "input", "seq": 17, "pointer": [px, py], "surface": [w, h],
     "pen": true, "follow": false, "controls": [{"action": "stop"}, ...]}

``pen`` is the marking intent (primary button); ``follow`` is the Live-mode
engagement: the arm tracks the pointer only while it is true.

Control actions: execute (with "drawing") | save (with "drawing", optional
"name") | park | mode ("mode": "batch" | "live") | reset | stop |
load_recipe (with "name", optional "scale" > 0, optional "fit") |
execute_recipe | clear_recipe | set_plane (with "offset": [dx, dy] metres and
optional "yaw" in degrees CCW).
``park`` moves the tool to the ready pose for pen adjustment.
``set_plane`` carries the operator's plane calibration as an ABSOLUTE offset
and rotation, not deltas, so a duplicated or reordered control cannot
accumulate error. ``yaw`` is optional: a client that
predates the rotation omits it and the rotation in force is left alone.
The client owns the
drawing: undo, clear, the tool selection, and the stroke-chaining
toggle never cross the wire — consecutive chained strokes arrive already
merged as one submitted stroke.
``load_recipe`` names a bridge-side recipe file (from ``config.recipes``);
the bridge loads and previews it, and ``execute_recipe`` runs the loaded one.

The submitted drawing carried by execute and save is the client's retained
drawing, conditioned, in plane coordinates::

    "drawing": [{"kind": "polyline" | "line" | "arc",
                 "points": [[x, y], ...]}, ...]

Bridge to client, on state change plus each live tick::

    {"type": "display", "mode": ..., "stopped": bool, "cause": str,
     "echo": [x, y] | null, "warnings": [...], "executing": bool,
     "preview": null | {"name": str, "scale": float,
                        "polylines": [[[x, y], ...], ...]},
     "config": {..., "recipes": [name, ...]}}

``preview`` is the loaded recipe's pen-down path in plane coordinates
(decimated for display); ``config.recipes`` lists the names ``load_recipe``
accepts; ``config.plane_offset`` and ``config.plane_yaw_deg`` echo the
calibration in force, and ``config.reach_inner``/``reach_outer`` the band that
bounds it.

Both sides use these helpers so the schema lives in one place.
"""

import json
import math
from typing import Optional

INPUT = 'input'
DISPLAY = 'display'

ACTIONS = ('execute', 'save', 'park', 'mode', 'reset', 'stop',
           'load_recipe', 'execute_recipe', 'clear_recipe', 'set_plane')
MODES = ('batch', 'live')
STROKE_KINDS = ('polyline', 'line', 'arc')
CARRIES_DRAWING = ('execute', 'save')


class ProtocolError(ValueError):
    """The peer sent a message that does not fit the schema."""


def encode_input(seq: int, pointer, surface, pen: bool, controls=(),
                 follow: bool = False) -> str:
    return json.dumps({
        'type': INPUT,
        'seq': int(seq),
        'pointer': [float(pointer[0]), float(pointer[1])],
        'surface': [float(surface[0]), float(surface[1])],
        'pen': bool(pen),
        'follow': bool(follow),
        'controls': list(controls),
    })


def decode_input(text: str) -> dict:
    """Parse and validate one client input message."""
    try:
        message = json.loads(text)
    except json.JSONDecodeError as error:
        raise ProtocolError(f'not JSON: {error}') from error
    if not isinstance(message, dict) or message.get('type') != INPUT:
        raise ProtocolError(f'not an input message: {text[:80]!r}')
    try:
        pointer = (float(message['pointer'][0]), float(message['pointer'][1]))
        surface = (float(message['surface'][0]), float(message['surface'][1]))
        pen = bool(message['pen'])
        follow = bool(message.get('follow', False))
        seq = int(message['seq'])
    except (KeyError, IndexError, TypeError, ValueError) as error:
        raise ProtocolError(f'malformed input fields: {error}') from error
    controls = message.get('controls') or []
    if not isinstance(controls, list):
        raise ProtocolError('controls must be a list')
    for control in controls:
        if not isinstance(control, dict) or control.get('action') not in ACTIONS:
            raise ProtocolError(f'unknown control {control!r}')
        if control['action'] == 'mode' and control.get('mode') not in MODES:
            raise ProtocolError(f'unknown mode {control!r}')
        if control['action'] in CARRIES_DRAWING:
            _validate_drawing(control.get('drawing'))
        if control['action'] == 'load_recipe':
            _validate_load_recipe(control)
        if control['action'] == 'set_plane':
            _validate_set_plane(control)
    return {'seq': seq, 'pointer': pointer, 'surface': surface, 'pen': pen,
            'follow': follow, 'controls': controls}


def _validate_drawing(drawing) -> None:
    """Shape check for a submitted drawing; bounds are the dispatcher's job."""
    if not isinstance(drawing, list):
        raise ProtocolError('a submitted drawing must be a list of strokes')
    for stroke in drawing:
        if not isinstance(stroke, dict) or \
                stroke.get('kind') not in STROKE_KINDS:
            raise ProtocolError(f'unknown stroke {str(stroke)[:80]!r}')
        points = stroke.get('points')
        if not isinstance(points, list) or len(points) < 2:
            raise ProtocolError('a submitted stroke needs at least two points')
        for point in points:
            try:
                float(point[0]), float(point[1])
            except (IndexError, TypeError, ValueError) as error:
                raise ProtocolError(f'malformed stroke point: {error}') from error


def _validate_set_plane(control: dict) -> None:
    """Shape check for a set_plane control; the reach bound is the
    dispatcher's job.

    Strict on purpose. The dispatcher's handler runs on the coordination
    thread, whose drain and watchdog loop carries no exception handler, so a
    malformed payload that reached ``float()`` there would take the bridge's
    input drain and its input-loss watchdog down with it.
    """
    offset = control.get('offset')
    if not isinstance(offset, (list, tuple)) or len(offset) != 2:
        raise ProtocolError('set_plane needs "offset" as [dx, dy] in metres')
    for value in offset:
        try:
            value = float(value)
        except (TypeError, ValueError) as error:
            raise ProtocolError(f'malformed plane offset: {error}') from error
        if not math.isfinite(value):
            raise ProtocolError(f'plane offset must be finite, got {value}')
    if control.get('yaw') is None:
        return   # optional; the rotation in force stands
    try:
        yaw = float(control['yaw'])
    except (TypeError, ValueError) as error:
        raise ProtocolError(f'malformed plane rotation: {error}') from error
    if not math.isfinite(yaw):
        raise ProtocolError(f'plane rotation must be finite, got {yaw}')


def _validate_load_recipe(control: dict) -> None:
    """Shape check for a load_recipe control; existence is the loader's job."""
    name = control.get('name')
    if not isinstance(name, str) or not name:
        raise ProtocolError('load_recipe needs a non-empty "name"')
    if 'scale' in control:
        try:
            scale = float(control['scale'])
        except (TypeError, ValueError) as error:
            raise ProtocolError(f'malformed recipe scale: {error}') from error
        if not scale > 0.0:
            raise ProtocolError(f'recipe scale must be positive, got {scale}')


def encode_display(state: dict) -> str:
    return json.dumps({'type': DISPLAY, **state})


def decode_display(text: str) -> Optional[dict]:
    """Parse a bridge display message; None when it is some other type."""
    try:
        message = json.loads(text)
    except json.JSONDecodeError as error:
        raise ProtocolError(f'not JSON: {error}') from error
    if not isinstance(message, dict) or message.get('type') != DISPLAY:
        return None
    return message
