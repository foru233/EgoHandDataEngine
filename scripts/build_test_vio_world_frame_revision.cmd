@echo off
setlocal
rem Builds and runs the standalone world-frame revision test.
rem Only VioWorldFrameRevision.h/.cc are used, so no ORB-SLAM3 runtime is linked.
set "ROOT=%~dp0.."
set "OUT=%ROOT%\build\vio_world_frame_test"
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul 2>&1
if errorlevel 1 exit /b 10
if not defined VIO_CONDA set "VIO_CONDA=%USERPROFILE%\miniconda3\envs\egohand-vio"
if not exist "%OUT%" mkdir "%OUT%"
pushd "%OUT%"
cl /nologo /std:c++17 /EHsc /O2 /MD ^
   /I"%ROOT%\third_party\ORB_SLAM3\include" ^
   /I"%VIO_CONDA%\Library\include\eigen3" ^
   "%ROOT%\scripts\test_vio_world_frame_revision.cpp" ^
   "%ROOT%\third_party\ORB_SLAM3\src\VioWorldFrameRevision.cc" ^
   /Fe:test_vio_world_frame_revision.exe
if errorlevel 1 (popd & exit /b %ERRORLEVEL%)
test_vio_world_frame_revision.exe
set "RC=%ERRORLEVEL%"
popd
exit /b %RC%
