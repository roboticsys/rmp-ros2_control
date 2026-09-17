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

"""Scripted drawing-surface client stub for phantom verification.

Drives the real ``SurfaceModel`` and ``Transport`` (the drawing is
composed and retained client-side, then submitted with the execute control),
so the bridge cannot tell it from the Windows client. It draws a scripted
drawing -- a line primitive, then a freehand square -- and submits it,
printing every display-state change until the execution result arrives.

Usage:  client_stub [ws://host:8765] [--no-execute] [--hold SECONDS]
        client_stub --live [--reset]   # follow-pointer live drawing sequence
        client_stub --load-recipe NAME [--recipe-scale S] [--fit]
                    [--execute-recipe]   # bridge-side recipe load + preview
"""

import sys
import threading
import time

from .surface_client import SurfaceModel, Transport

SURFACE = (800.0, 800.0)


def wait_for(predicate, timeout: float, period: float = 0.05) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(period)
    return False


def start_display_printer(model: SurfaceModel, done: threading.Event):
    last = {}

    def run():
        nonlocal last
        while not done.is_set():
            view = model.view_state()
            display = view['display']
            if display and display != last:
                last = display
                print(f"display: mode={display.get('mode')} "
                      f"stopped={display.get('stopped')} "
                      f"executing={display.get('executing')} "
                      f"warnings={display.get('warnings')}", flush=True)
            time.sleep(0.1)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread


def click(model: SurfaceModel, pointer, settle: float = 0.05):
    model.set_pointer(*pointer)
    time.sleep(settle)
    model.set_pen(True)
    time.sleep(settle)
    model.set_pen(False)
    time.sleep(settle)


def freehand(model: SurfaceModel, path, step_seconds: float = 0.01):
    model.set_pointer(*path[0])
    time.sleep(0.05)
    model.set_pen(True)
    for point in path:
        model.set_pointer(*point)
        time.sleep(step_seconds)
    model.set_pen(False)
    time.sleep(0.05)


def compose(model: SurfaceModel) -> None:
    """The scripted drawing: a line primitive, then a freehand square."""
    # A line primitive: tool select, then two clicks.
    model.select_tool('line')
    click(model, (250.0, 500.0))
    click(model, (550.0, 500.0))

    # A freehand square above it (canvas y is DOWN; the mapping y-flips).
    model.select_tool('freehand')
    corners = [(300.0, 450.0), (500.0, 450.0), (500.0, 250.0),
               (300.0, 250.0), (300.0, 450.0)]
    path = []
    for start, end in zip(corners, corners[1:]):
        for i in range(20):
            path.append((start[0] + (end[0] - start[0]) * i / 20,
                         start[1] + (end[1] - start[1]) * i / 20))
    freehand(model, path)
    view = model.view_state()
    print(f"composed locally: strokes={view['strokes']} "
          f"samples={view['samples']}", flush=True)


def script(model: SurfaceModel, execute: bool, hold: float,
           reset: bool = False) -> None:
    if reset:
        print('sending reset (clears a standing stop latch)...', flush=True)
        model.request_reset()
        time.sleep(0.3)

    compose(model)

    if execute:
        print('submitting the drawing...', flush=True)
        started = time.monotonic()
        model.request_execute()
        # Keep the session (the liveness signal) up until the result.
        wait_for(lambda: (not model.view_state()['display'].get('executing'))
                 and time.monotonic() - started > 2.0, timeout=600.0,
                 period=0.1)
        display = model.view_state()['display']
        print(f'execution finished after {time.monotonic() - started:.1f} s '
              f'(stopped={display.get("stopped")})', flush=True)
    if hold > 0.0:
        print(f'holding the session for {hold:.0f} s...', flush=True)
        time.sleep(hold)


def live_script(model: SurfaceModel, reset: bool) -> None:
    """Follow-pointer live sequence: switch to Live, hover to a start point,
    draw a slow pen-down line, lift, rest, switch back to Batch."""
    if reset:
        print('sending reset (clears a standing stop latch)...', flush=True)
        model.request_reset()
        time.sleep(0.3)

    print('switching to Live mode...', flush=True)
    model.request_mode('live')
    if not wait_for(lambda: model.view_state()['display'].get('mode')
                    == 'live', timeout=15.0, period=0.1):
        print('live mode never engaged; aborting', flush=True)
        return

    # Hover to the start point: follow engaged pen-up (right-button
    # equivalent); the arm follows at the safe height.
    model.set_pointer(300.0, 400.0)
    model.set_follow(True)
    print('hovering to the start point (5 s)...', flush=True)
    time.sleep(5.0)

    # Pen down (left-button equivalent), draw a slow straight line, pen up.
    print('pen down; drawing live (8 s)...', flush=True)
    model.set_pen(True)
    started = time.monotonic()
    while time.monotonic() - started < 8.0:
        fraction = (time.monotonic() - started) / 8.0
        model.set_pointer(300.0 + 200.0 * fraction, 400.0)
        time.sleep(0.02)
    model.set_pen(False)
    print('pen still followed up; disengaging (lift + settle, 3 s)...',
          flush=True)
    time.sleep(1.0)
    model.set_follow(False)   # disengaged: lift at the last position, settle
    time.sleep(3.0)

    print('switching back to Batch...', flush=True)
    model.request_mode('batch')
    time.sleep(1.0)
    display = model.view_state()['display']
    print(f'live sequence done (stopped={display.get("stopped")})',
          flush=True)


def draw_live_stroke(model: SurfaceModel, start, end, seconds: float) -> None:
    """One pen-down stroke from ``start`` to ``end`` at drawing speed."""
    model.set_pointer(*start)
    time.sleep(0.1)
    model.set_pen(True)
    started = time.monotonic()
    while time.monotonic() - started < seconds:
        fraction = (time.monotonic() - started) / seconds
        model.set_pointer(start[0] + (end[0] - start[0]) * fraction,
                          start[1] + (end[1] - start[1]) * fraction)
        time.sleep(0.02)
    model.set_pen(False)


def live_strokes_script(model: SurfaceModel, reset: bool) -> None:
    """The pen-routing sequence (stakeholder 2026-08-25): pen-up follow on
    Servo, pen-down strokes as sized goal chains. Three strokes: 1 and 2 in
    quick succession (the chain must carry the travel between them -- no
    Servo hand-back), then a real pen-up follow (Servo resumes after the
    chain drains), then stroke 3, then disengage."""
    if reset:
        print('sending reset (clears a standing stop latch)...', flush=True)
        model.request_reset()
        time.sleep(0.3)

    # Park first: it brings the arm to the ready pose above the plane; a
    # cold-start pose is too far for the stroke chain's cartesian plan (and
    # for a short Servo hover). Park stopped approaching the anchor on
    # 2026-09-10, so the arm now ends at ready, not on the plane.
    print('parking at the ready pose first...', flush=True)
    model.request_park()
    started = time.monotonic()
    wait_for(lambda: (not model.view_state()['display'].get('executing'))
             and time.monotonic() - started > 2.0, timeout=300.0, period=0.1)
    print(f'parked after {time.monotonic() - started:.1f} s', flush=True)

    print('switching to Live mode...', flush=True)
    model.request_mode('live')
    if not wait_for(lambda: model.view_state()['display'].get('mode')
                    == 'live', timeout=15.0, period=0.1):
        print('live mode never engaged; aborting', flush=True)
        return

    model.set_pointer(250.0, 400.0)
    model.set_follow(True)
    print('hovering to the start point on Servo (8 s)...', flush=True)
    time.sleep(8.0)

    print('stroke 1 (4 s)...', flush=True)
    draw_live_stroke(model, (250.0, 400.0), (400.0, 400.0), 4.0)
    time.sleep(0.3)   # quick pen-up: the chain must hold (travel, no Servo)
    print('stroke 2 after 0.3 s (4 s)...', flush=True)
    draw_live_stroke(model, (400.0, 350.0), (250.0, 350.0), 4.0)

    print('pen-up follow (chain drains, then Servo resumes; 20 s)...',
          flush=True)
    started = time.monotonic()
    while time.monotonic() - started < 20.0:
        fraction = (time.monotonic() - started) / 20.0
        model.set_pointer(250.0 + 100.0 * fraction, 350.0 + 100.0 * fraction)
        time.sleep(0.05)

    print('stroke 3 (3 s)...', flush=True)
    draw_live_stroke(model, (350.0, 450.0), (450.0, 450.0), 3.0)
    print('disengaging; waiting for the chain to drain (25 s)...', flush=True)
    time.sleep(1.0)
    model.set_follow(False)
    time.sleep(25.0)

    print('switching back to Batch...', flush=True)
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:   # refused while the chain drains
        model.request_mode('batch')
        if wait_for(lambda: model.view_state()['display'].get('mode')
                    == 'batch', timeout=2.0, period=0.1):
            break
    display = model.view_state()['display']
    print(f"live-strokes sequence done (mode={display.get('mode')} "
          f"stopped={display.get('stopped')})", flush=True)


def recipe_script(model: SurfaceModel, name: str, scale, fit: bool,
                  execute: bool, reset: bool) -> bool:
    """Load a bridge-side recipe, report its preview, optionally execute."""
    if reset:
        print('sending reset (clears a standing stop latch)...', flush=True)
        model.request_reset()
        time.sleep(0.3)

    # A previous session's loaded recipe persists bridge-side; clear it so
    # the wait below can only match THIS load's outcome.
    model.request_clear_recipe()
    if not wait_for(lambda: model.view_state()['display'].get('preview')
                    is None, timeout=10.0):
        print('previous preview never cleared; aborting', flush=True)
        return False
    warnings_before = list(
        model.view_state()['display'].get('warnings') or [])

    def refused():
        warnings = model.view_state()['display'].get('warnings') or []
        return (warnings != warnings_before
                and any('recipe load failed' in w for w in warnings))

    print(f'loading recipe {name!r} (scale={scale}, fit={fit})...', flush=True)
    model.request_load_recipe(name, scale=scale, fit=fit)
    wait_for(lambda: refused() or (model.view_state()['display'].get(
        'preview') or {}).get('name') == name, timeout=15.0)
    display = model.view_state()['display']
    preview = display.get('preview')
    if not preview or preview.get('name') != name:
        print(f"recipe load failed: warnings={display.get('warnings')}",
              flush=True)
        return False
    points = sum(len(line) for line in preview.get('polylines', []))
    print(f"loaded: scale={preview.get('scale'):.4g} "
          f"polylines={len(preview.get('polylines', []))} points={points} "
          f"warnings={display.get('warnings')}", flush=True)

    if execute:
        print('executing the recipe...', flush=True)
        started = time.monotonic()
        model.request_execute_recipe()
        wait_for(lambda: (not model.view_state()['display'].get('executing'))
                 and time.monotonic() - started > 2.0, timeout=600.0,
                 period=0.1)
        display = model.view_state()['display']
        print(f'execution finished after {time.monotonic() - started:.1f} s '
              f'(stopped={display.get("stopped")})', flush=True)
    return True


def run(url: str, execute: bool, hold: float, reset: bool = False,
        live: bool = False, live_strokes: bool = False,
        recipe_name=None, recipe_scale=None, recipe_fit: bool = False,
        recipe_execute: bool = False) -> int:
    model = SurfaceModel()
    model.set_surface(*SURFACE)
    transport = Transport(url, model)
    transport.start()
    done = threading.Event()
    start_display_printer(model, done)

    if not wait_for(lambda: model.view_state()['connected'], timeout=10.0):
        print('never connected; aborting', flush=True)
        transport.stop()
        return 1
    # Composing needs the config block (extent, conditioning, sample bound).
    if not wait_for(lambda: model.view_state()['display'].get('config'),
                    timeout=10.0):
        print('no display config from the bridge; aborting', flush=True)
        transport.stop()
        return 1

    ok = True
    try:
        if recipe_name:
            ok = recipe_script(model, recipe_name, recipe_scale, recipe_fit,
                               recipe_execute, reset)
        elif live_strokes:
            live_strokes_script(model, reset)
        elif live:
            live_script(model, reset)
        else:
            script(model, execute, hold, reset)
    finally:
        done.set()
        transport.stop()
    failed = model.view_state()['display'].get('stopped', False)
    return 1 if failed or not ok else 0


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    url = 'ws://127.0.0.1:8765'
    execute, hold, reset, live = True, 0.0, False, False
    live_strokes = False
    recipe_name, recipe_scale = None, None
    recipe_fit, recipe_execute = False, False
    while argv:
        argument = argv.pop(0)
        if argument == '--no-execute':
            execute = False
        elif argument == '--reset':
            reset = True
        elif argument == '--live':
            live = True
        elif argument == '--live-strokes':
            live_strokes = True
        elif argument == '--hold' and argv:
            hold = float(argv.pop(0))
        elif argument == '--load-recipe' and argv:
            recipe_name = argv.pop(0)
        elif argument == '--recipe-scale' and argv:
            recipe_scale = float(argv.pop(0))
        elif argument == '--fit':
            recipe_fit = True
        elif argument == '--execute-recipe':
            recipe_execute = True
        elif argument.startswith('ws://'):
            url = argument
    return run(url, execute, hold, reset, live, live_strokes,
               recipe_name, recipe_scale, recipe_fit, recipe_execute)


if __name__ == '__main__':
    raise SystemExit(main())
