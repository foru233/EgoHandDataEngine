@echo off
setlocal
rem Builds the per-map-point information-matrix-condition bridge under its own DLL name.
rem It only adds diagnostic output; it never touches egohand_vio_world_frame.dll,
rem egohand_vio_recovery_bridge_v9.dll, egohand_vio_stability_*.dll or
rem egohand_vio_atlas_bridge.dll, and it never edits configs/vio/**.
rem Usage: scripts\build_vio_point_cond.cmd [output_name]
set "POINT_COND_NAME=egohand_vio_point_cond"
if not "%~1"=="" set "POINT_COND_NAME=%~1"
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul 2>&1
if errorlevel 1 exit /b 10
if not defined VIO_CONDA set "VIO_CONDA=%USERPROFILE%\miniconda3\envs\egohand-vio"
set "PATH=%VIO_CONDA%;%VIO_CONDA%\Scripts;%VIO_CONDA%\Library\bin;%PATH%"
set "POINT_COND_CMAKE=%VIO_CONDA%\Lib\site-packages\cmake\data\bin\cmake.exe"
"%POINT_COND_CMAKE%" --build "%~dp0..\third_party\ORB_SLAM3\build_msvc_initial" --target ORB_SLAM3 --parallel 2
if errorlevel 1 exit /b %ERRORLEVEL%
"%POINT_COND_CMAKE%" -G Ninja -S "%~dp0..\src\egohand\vio\native" -B "%~dp0..\build\vio_point_cond" -DCMAKE_BUILD_TYPE=Release -DEGOHAND_RECOVERY_BRIDGE=ON -DEGOHAND_RECOVERY_OUTPUT_NAME=%POINT_COND_NAME%
if errorlevel 1 exit /b %ERRORLEVEL%
"%POINT_COND_CMAKE%" --build "%~dp0..\build\vio_point_cond" --parallel 2
if errorlevel 1 exit /b %ERRORLEVEL%
"%~dp0..\.venv\Scripts\python.exe" "%~dp0write_vio_build_manifest.py" --library "%~dp0..\src\egohand\vio\native\bin\%POINT_COND_NAME%.dll" --build-dir "%~dp0..\build\vio_point_cond"
exit /b %ERRORLEVEL%
