@echo off
setlocal
set "ORB_ROOT=%~dp0..\third_party\ORB_SLAM3"
set "GIT=C:\Program Files\Git\cmd\git.exe"
rem Continue only when the source snapshot matches or applies cleanly.
"%GIT%" -C "%ORB_ROOT%" apply --reverse --check "%~dp0orbslam3_windows_source_snapshot.patch" >nul 2>&1
if errorlevel 1 (
    "%GIT%" -C "%ORB_ROOT%" apply --check "%~dp0orbslam3_windows_source_snapshot.patch"
    if errorlevel 1 exit /b 11
    "%GIT%" -C "%ORB_ROOT%" apply "%~dp0orbslam3_windows_source_snapshot.patch"
    if errorlevel 1 exit /b 11
)
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul 2>&1
if errorlevel 1 exit /b 10
if not defined VIO_CONDA set "VIO_CONDA=%USERPROFILE%\miniconda3\envs\egohand-vio"
set "PATH=%VIO_CONDA%;%VIO_CONDA%\Scripts;%VIO_CONDA%\Library\bin;%PATH%"
"%VIO_CONDA%\Lib\site-packages\cmake\data\bin\cmake.exe" --build "%~dp0..\third_party\ORB_SLAM3\build_msvc_initial" --target ORB_SLAM3 --parallel 2
if errorlevel 1 exit /b %ERRORLEVEL%
set "VIO_BUILD_DIR=%~dp0..\build\vio_recovery_bridge"
"%VIO_CONDA%\Lib\site-packages\cmake\data\bin\cmake.exe" -G Ninja -S "%~dp0..\src\egohand\vio\native" -B "%VIO_BUILD_DIR%" -DCMAKE_BUILD_TYPE=Release -DEGOHAND_RECOVERY_BRIDGE=ON -DEGOHAND_RECOVERY_OUTPUT_NAME=egohand_vio_recovery_bridge_v9
if errorlevel 1 exit /b %ERRORLEVEL%
"%VIO_CONDA%\Lib\site-packages\cmake\data\bin\cmake.exe" --build "%VIO_BUILD_DIR%" --parallel 2
exit /b %ERRORLEVEL%
