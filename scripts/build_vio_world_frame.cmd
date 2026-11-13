@echo off
setlocal
rem Builds the world-frame-revision bridge under its own DLL name.
rem It never touches egohand_vio_recovery_bridge_v9.dll, egohand_vio_stability_*.dll
rem or egohand_vio_atlas_bridge.dll, and it never edits configs/vio/**.
rem Usage: scripts\build_vio_world_frame.cmd [output_name]
set "WORLD_NAME=egohand_vio_world_frame"
if not "%~1"=="" set "WORLD_NAME=%~1"
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul 2>&1
if errorlevel 1 exit /b 10
if not defined VIO_CONDA set "VIO_CONDA=%USERPROFILE%\miniconda3\envs\egohand-vio"
set "PATH=%VIO_CONDA%;%VIO_CONDA%\Scripts;%VIO_CONDA%\Library\bin;%PATH%"
set "WORLD_CMAKE=%VIO_CONDA%\Lib\site-packages\cmake\data\bin\cmake.exe"
"%WORLD_CMAKE%" --build "%~dp0..\third_party\ORB_SLAM3\build_msvc_initial" --target ORB_SLAM3 --parallel 2
if not "%ERRORLEVEL%"=="0" exit /b 1
"%WORLD_CMAKE%" -G Ninja -S "%~dp0..\src\egohand\vio\native" -B "%~dp0..\build\vio_world_frame" -DCMAKE_BUILD_TYPE=Release -DEGOHAND_RECOVERY_BRIDGE=ON -DEGOHAND_RECOVERY_OUTPUT_NAME=%WORLD_NAME%
if not "%ERRORLEVEL%"=="0" exit /b 1
"%WORLD_CMAKE%" --build "%~dp0..\build\vio_world_frame" --parallel 2
if not "%ERRORLEVEL%"=="0" exit /b 1
"%~dp0..\.venv\Scripts\python.exe" "%~dp0write_vio_build_manifest.py" --library "%~dp0..\src\egohand\vio\native\bin\%WORLD_NAME%.dll" --build-dir "%~dp0..\build\vio_world_frame"
exit /b %ERRORLEVEL%
