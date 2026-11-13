@echo off
setlocal
set "ORB_ROOT=%~dp0..\third_party\ORB_SLAM3"
set "GIT=C:\Program Files\Git\cmd\git.exe"
if /i "%~1"=="--atlas-only" goto atlas
set "RECOVERY_PATCH=%~dp0orbslam3_stereo_inertial_recovery.patch"
"%GIT%" -C "%ORB_ROOT%" apply --reverse --check "%RECOVERY_PATCH%" >nul 2>&1
if errorlevel 1 (
    "%GIT%" -C "%ORB_ROOT%" apply "%RECOVERY_PATCH%"
    if errorlevel 1 exit /b 11
)
:atlas
set "ATLAS_PATCH=%~dp0orbslam3_atlas_snapshot.patch"
"%GIT%" -C "%ORB_ROOT%" apply --reverse --check "%ATLAS_PATCH%" >nul 2>&1
if errorlevel 1 (
    "%GIT%" -C "%ORB_ROOT%" apply "%ATLAS_PATCH%"
    if errorlevel 1 exit /b 11
)
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul 2>&1
if errorlevel 1 exit /b 10
if not defined VIO_CONDA set "VIO_CONDA=%USERPROFILE%\miniconda3\envs\egohand-vio"
set "PATH=%VIO_CONDA%;%VIO_CONDA%\Scripts;%VIO_CONDA%\Library\bin;%PATH%"
"%VIO_CONDA%\Lib\site-packages\cmake\data\bin\cmake.exe" --build "%ORB_ROOT%\build_msvc_initial" --target ORB_SLAM3 --parallel 2
if errorlevel 1 exit /b %ERRORLEVEL%
call "%~dp0..\scripts\build_vio_native_bridge.cmd" %1
exit /b %ERRORLEVEL%
