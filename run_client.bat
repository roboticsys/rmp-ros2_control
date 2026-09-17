@echo off
rem Copyright 2026 Robotic Systems Integration, Inc.
rem
rem Licensed under the Apache License, Version 2.0 (the "License");
rem you may not use this file except in compliance with the License.
rem You may obtain a copy of the License at
rem
rem     http://www.apache.org/licenses/LICENSE-2.0
rem
rem Unless required by applicable law or agreed to in writing, software
rem distributed under the License is distributed on an "AS IS" BASIS,
rem WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
rem See the License for the specific language governing permissions and
rem limitations under the License.
rem Drawing surface client for Windows. Installs what the client needs, then
rem starts it against the bridge running on the Linux host.
rem
rem Usage:  run_client.bat ws://<linux-host>:8765
rem
rem Needs Python 3.10+ from https://www.python.org/downloads/windows/ with
rem the "tcl/tk and IDLE" component (checked by default). Everything else is
rem installed here on first run. No ROS 2 is needed on this machine.
setlocal

set "PKG=%~dp0src\rapidcode_draw_plane"
if not exist "%PKG%\rapidcode_draw_plane\surface_client.py" (
    echo error: client source not found under "%PKG%".
    echo Run this script from the root of the repository checkout.
    exit /b 1
)

if "%~1"=="" (
    echo usage: run_client.bat ws://^<linux-host^>:8765
    echo The address is printed by ./run.sh up on the Linux host.
    exit /b 2
)

set "PY=py"
where py >nul 2>&1
if errorlevel 1 set "PY=python"
where %PY% >nul 2>&1
if errorlevel 1 (
    echo error: Python was not found.
    echo Install Python 3.10+ from https://www.python.org/downloads/windows/
    echo and keep the "tcl/tk and IDLE" component checked, then re-run.
    exit /b 1
)

%PY% -c "import tkinter" >nul 2>&1
if errorlevel 1 (
    echo error: this Python has no tkinter, so the window cannot open.
    echo Re-run the Python installer and check the "tcl/tk and IDLE" component.
    exit /b 1
)

%PY% -c "import websockets" >nul 2>&1
if errorlevel 1 (
    echo Installing the websockets package...
    %PY% -m pip install websockets
    if errorlevel 1 (
        echo error: pip install failed. Check the network or the proxy, then re-run.
        exit /b 1
    )
)

rem Optional: the Sun Valley theme. The client falls back to the native ttk
rem theme when it is missing, so a failed install is a warning, not an error.
%PY% -c "import sv_ttk" >nul 2>&1
if errorlevel 1 (
    echo Installing the optional sv-ttk theme...
    %PY% -m pip install sv-ttk
    if errorlevel 1 echo warning: sv-ttk did not install. The client will use the native theme.
)

set "PYTHONPATH=%PKG%;%PYTHONPATH%"
%PY% -m rapidcode_draw_plane.surface_client %*
