@echo off
setlocal
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul 2>&1
if not "%ERRORLEVEL%"=="0" exit /b 10
if not defined VIO_CONDA set "VIO_CONDA=%USERPROFILE%\miniconda3\envs\egohand-vio"
set "PATH=%VIO_CONDA%;%VIO_CONDA%\Scripts;%VIO_CONDA%\Library\bin;%~dp0..\src\egohand\vio\native\bin;%PATH%"
set "CHEIRALITY_CMAKE=%VIO_CONDA%\Lib\site-packages\cmake\data\bin\cmake.exe"
"%CHEIRALITY_CMAKE%" --build "%~dp0..\third_party\ORB_SLAM3\build_msvc_initial" --target ORB_SLAM3 --parallel 2
if not "%ERRORLEVEL%"=="0" exit /b 1
"%CHEIRALITY_CMAKE%" -G Ninja -S "%~dp0..\src\egohand\vio\native" -B "%~dp0..\build\vio_cheirality" -DCMAKE_BUILD_TYPE=Release -DEGOHAND_RECOVERY_BRIDGE=ON -DEGOHAND_RECOVERY_OUTPUT_NAME=egohand_vio_cheirality_v3 -DEGOHAND_VIO_CHEIRALITY_UNIT=ON
if not "%ERRORLEVEL%"=="0" exit /b 1
"%CHEIRALITY_CMAKE%" --build "%~dp0..\build\vio_cheirality" --parallel 2
if not "%ERRORLEVEL%"=="0" exit /b 1
set "EGOHAND_VIO_CHEIRALITY_GUARD=1"
"%~dp0..\build\vio_cheirality\vio_cheirality_unit.exe"
if not "%ERRORLEVEL%"=="0" exit /b 1
"%~dp0..\.venv\Scripts\python.exe" "%~dp0write_vio_build_manifest.py" --library "%~dp0..\src\egohand\vio\native\bin\egohand_vio_cheirality_v3.dll" --build-dir "%~dp0..\build\vio_cheirality"
exit /b %ERRORLEVEL%
