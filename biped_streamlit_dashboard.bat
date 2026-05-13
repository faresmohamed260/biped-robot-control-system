@echo off
setlocal

set "ROOT=%~dp0"
set "APP=%ROOT%streamlit_app.py"
set "VENV_PY=%ROOT%.venv\Scripts\python.exe"

if exist "%VENV_PY%" (
  start "" "%VENV_PY%" -m streamlit run "%APP%"
) else (
  start "" python -m streamlit run "%APP%"
)

endlocal
