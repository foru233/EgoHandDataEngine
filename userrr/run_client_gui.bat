@echo off
cd /d "%~dp0.."
.venv\Scripts\python.exe userrr\run_client_gui.py %*
