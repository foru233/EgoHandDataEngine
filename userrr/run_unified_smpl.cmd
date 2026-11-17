@echo off
setlocal
cd /d "%~dp0.."
".venv\Scripts\python.exe" "userrr\run_client_gui.py" --open-results "data\session_184714_handfilter_20261116_v2_smpl_unified_v11" %*
if errorlevel 1 pause
endlocal
