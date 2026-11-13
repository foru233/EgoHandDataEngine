@echo off
setlocal
rem Builds current preserved sources without applying or reversing patches.
rem Output is separate from every existing GUI/Atlas DLL.
set "STABILITY_NAME=egohand_vio_stability_diagnostic"
if not "%~1"=="" set "STABILITY_NAME=%~1"
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul 2>&1
if errorlevel 1 exit /b 10
if not defined VIO_CONDA set "VIO_CONDA=%USERPROFILE%\miniconda3\envs\egohand-vio"
set "PATH=%VIO_CONDA%;%VIO_CONDA%\Scripts;%VIO_CONDA%\Library\bin;%PATH%"
set "STABILITY_CMAKE=%VIO_CONDA%\Lib\site-packages\cmake\data\bin\cmake.exe"
"%STABILITY_CMAKE%" --build "%~dp0..\third_party\ORB_SLAM3\build_msvc_initial" --target ORB_SLAM3 --parallel 2
if not "%ERRORLEVEL%"=="0" exit /b 1
"%STABILITY_CMAKE%" -G Ninja -S "%~dp0..\src\egohand\vio\native" -B "%~dp0..\build\vio_stability" -DCMAKE_BUILD_TYPE=Release -DEGOHAND_RECOVERY_BRIDGE=ON -DEGOHAND_RECOVERY_OUTPUT_NAME=%STABILITY_NAME%
if not "%ERRORLEVEL%"=="0" exit /b 1
"%STABILITY_CMAKE%" --build "%~dp0..\build\vio_stability" --parallel 2
if not "%ERRORLEVEL%"=="0" exit /b 1
"%~dp0..\.venv\Scripts\python.exe" "%~dp0write_vio_build_manifest.py" --library "%~dp0..\src\egohand\vio\native\bin\%STABILITY_NAME%.dll" --build-dir "%~dp0..\build\vio_stability"
exit /b %ERRORLEVEL%
