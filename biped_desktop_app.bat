@echo off
setlocal

set "ROOT=%~dp0"
set "PYW=%ROOT%\.venv\Scripts\pythonw.exe"
set "PY=%ROOT%\.venv\Scripts\python.exe"

if exist "%PYW%" (
    start "" "%PYW%" "%ROOT%biped_desktop_app.py"
    exit /b 0
)

if exist "%PY%" (
    start "" "%PY%" "%ROOT%biped_desktop_app.py"
    exit /b 0
)

start "" python "%ROOT%biped_desktop_app.py"
exit /b 0
