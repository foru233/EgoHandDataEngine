@echo off
setlocal
rem Builds and runs the standalone VioSafeExp test (header-only, no ORB-SLAM3 runtime).
set "ROOT=%~dp0.."
set "OUT=%ROOT%\build\vio_safe_exp_test"
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul 2>&1
if errorlevel 1 exit /b 10
if not defined VIO_CONDA set "VIO_CONDA=%USERPROFILE%\miniconda3\envs\egohand-vio"
if not exist "%OUT%" mkdir "%OUT%"
pushd "%OUT%"
cl /nologo /std:c++17 /EHsc /O2 /MD ^
   /I"%ROOT%\third_party\ORB_SLAM3\include" ^
   /I"%VIO_CONDA%\Library\include\eigen3" ^
   "%ROOT%\scripts\test_vio_safe_exp.cpp" ^
   /Fe:test_vio_safe_exp.exe
if errorlevel 1 (popd & exit /b %ERRORLEVEL%)
test_vio_safe_exp.exe
set "RC=%ERRORLEVEL%"
popd
exit /b %RC%