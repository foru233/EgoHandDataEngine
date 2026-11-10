@echo off
cd /d "%~dp0"
if not exist logs mkdir logs
".venv\Scripts\python.exe" -m pytest -q --junitxml=logs\pytest.xml %*
pause
