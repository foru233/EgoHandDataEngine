@echo off
setlocal
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul 2>&1
if errorlevel 1 exit /b 10
if not defined VIO_CONDA set "VIO_CONDA=%USERPROFILE%\miniconda3\envs\egohand-vio"
set "PATH=%VIO_CONDA%;%VIO_CONDA%\Scripts;%VIO_CONDA%\Library\bin;%PATH%"
"%VIO_CONDA%\Lib\site-packages\cmake\data\bin\cmake.exe" -S "%~dp0..\src\egohand\vio\native" -B "%~dp0..\build\vio_bridge_msvc" -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 exit /b %ERRORLEVEL%
"%VIO_CONDA%\Lib\site-packages\cmake\data\bin\cmake.exe" --build "%~dp0..\build\vio_bridge_msvc" --parallel 2
exit /b %ERRORLEVEL%
