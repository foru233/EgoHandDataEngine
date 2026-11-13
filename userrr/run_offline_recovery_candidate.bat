@echo off
cd /d "%~dp0.."
if "%~1"=="" (
  .venv\Scripts\python.exe userrr\run_client_gui.py --offline-recovery-candidate --recording data\session_20261112_184714_636910800
) else (
  .venv\Scripts\python.exe userrr\run_client_gui.py --offline-recovery-candidate --recording "%~1"
)
