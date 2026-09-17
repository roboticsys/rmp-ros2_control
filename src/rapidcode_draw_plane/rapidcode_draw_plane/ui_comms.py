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

"""UI communications manager: the bridge end of the client WebSocket.

Owns the transport thread: one asyncio loop serving one WebSocket session at a
time (a second connection is refused -- one operator, one surface). Decoded
input events land in a thread-safe queue the coordination thread drains
(``drain``); display states cross back with ``send_display_state``
(loop.call_soon_threadsafe). Input loss is judged by message
cadence, not by the transport: ``input_lost`` compares the last-message time
against the timeout, and a closed session counts as loss while a drawing
executes or Live is active.
"""

import asyncio
import queue
import threading
import time
from typing import Callable, Optional

from . import protocol


class UiComms:
    def __init__(self, host: str, port: int, input_timeout: float,
                 log: Callable[[str], None] = lambda text: None):
        self._host = host
        self._port = port
        self._input_timeout = input_timeout
        self._log = log

        self._events: 'queue.Queue[dict]' = queue.Queue()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._session = None            # the one active websocket, loop thread only
        self._session_open = False      # mirror readable from any thread
        self._last_input = None         # time.monotonic of the newest input
        self._ready = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name='draw-plane-transport', daemon=True)

    # ---------------------------------------------------------------- lifecycle
    def start(self) -> None:
        self._thread.start()
        if not self._ready.wait(timeout=10.0):
            raise RuntimeError('transport thread failed to start')

    def shutdown(self) -> None:
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=5.0)

    def _run(self) -> None:
        import websockets

        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        async def handler(websocket):
            if self._session is not None:
                self._log(f'second session refused from '
                          f'{websocket.remote_address} (one already open)')
                await websocket.close(code=1013, reason='a session is already open')
                return
            self._session = websocket
            self._session_open = True
            self._last_input = time.monotonic()
            self._log(f'client connected from {websocket.remote_address}')
            try:
                async for text in websocket:
                    try:
                        event = protocol.decode_input(text)
                    except protocol.ProtocolError as error:
                        self._log(f'bad input message dropped: {error}')
                        continue
                    self._last_input = time.monotonic()
                    self._events.put(event)
            except websockets.exceptions.ConnectionClosed:
                pass  # an abrupt close IS input loss; the watchdog reports it
            finally:
                self._session = None
                self._session_open = False
                self._log('client disconnected')

        async def serve():
            async with websockets.serve(handler, self._host, self._port):
                self._ready.set()
                await asyncio.Future()  # run until loop.stop()

        try:
            self._loop.run_until_complete(serve())
        except RuntimeError:
            pass  # loop.stop() cancels the pending Future on shutdown
        finally:
            self._loop.close()

    # --------------------------------------------------------- coordination side
    def drain(self, limit: int = 64):
        """Pop up to ``limit`` decoded input events (coordination thread)."""
        events = []
        for _ in range(limit):
            try:
                events.append(self._events.get_nowait())
            except queue.Empty:
                break
        return events

    def send_display_state(self, state: dict) -> None:
        """Queue one display-state message to the client (any thread)."""
        if self._loop is None or not self._session_open:
            return
        text = protocol.encode_display(state)

        def _send():
            session = self._session
            if session is not None:
                asyncio.ensure_future(self._safe_send(session, text))

        self._loop.call_soon_threadsafe(_send)

    async def _safe_send(self, session, text: str) -> None:
        try:
            await session.send(text)
        except Exception:  # a dying session is loss, not a crash
            pass

    def input_lost(self) -> bool:
        """True when the session is silent past the timeout, or closed after
        having been open (watchdog_tick's verdict; the bridge node's timer is
        the tick)."""
        if self._last_input is None:
            return False  # no client yet: nothing to lose
        if not self._session_open:
            return True
        return (time.monotonic() - self._last_input) > self._input_timeout

    @property
    def session_open(self) -> bool:
        return self._session_open
