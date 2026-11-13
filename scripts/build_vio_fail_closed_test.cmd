@echo off
setlocal
rem Requires the separately completed production/static-core build.
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul 2>&1
set "FAULT_CONDA=%USERPROFILE%\miniconda3\envs\egohand-vio"
set "PATH=%FAULT_CONDA%;%FAULT_CONDA%\Library\bin;%PATH%"
set "FAULT_CMAKE=%FAULT_CONDA%\Lib\site-packages\cmake\data\bin\cmake.exe"
"%FAULT_CMAKE%" -G Ninja -S "%~dp0..\src\egohand\vio\native" -B "%~dp0..\build\vio_fault_test" -DCMAKE_BUILD_TYPE=Release -DEGOHAND_RECOVERY_BRIDGE=ON -DEGOHAND_RECOVERY_OUTPUT_NAME=egohand_vio_fail_closed_test -DEGOHAND_VIO_TEST_HOOKS=ON
if not "%ERRORLEVEL%"=="0" exit /b 1
"%FAULT_CMAKE%" --build "%~dp0..\build\vio_fault_test" --parallel 2
if not "%ERRORLEVEL%"=="0" exit /b 1
set "PYTHONPATH=%~dp0..\src"
"%FAULT_CONDA%\python.exe" "%~dp0test_vio_fail_closed_bridge.py" --library "%~dp0..\src\egohand\vio\native\bin\egohand_vio_fail_closed_test.dll"
exit /b %ERRORLEVEL%
