@echo off
setlocal
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul 2>&1
if errorlevel 1 exit /b 10
if not defined VIO_CONDA set "VIO_CONDA=%USERPROFILE%\miniconda3\envs\egohand-vio"
set "PATH=%VIO_CONDA%;%VIO_CONDA%\Scripts;%VIO_CONDA%\Library\bin;%PATH%"
set "VIO_BUILD_DIR=%~dp0..\build\vio_bridge_msvc"
set "VIO_ATLAS=OFF"
set "VIO_GENERATOR="
if /i "%~1"=="--atlas-only" (
    set "VIO_BUILD_DIR=%~dp0..\build\vio_atlas_bridge_ninja"
    set "VIO_ATLAS=ON"
    set "VIO_GENERATOR=-G Ninja"
)
"%VIO_CONDA%\Lib\site-packages\cmake\data\bin\cmake.exe" %VIO_GENERATOR% -S "%~dp0..\src\egohand\vio\native" -B "%VIO_BUILD_DIR%" -DCMAKE_BUILD_TYPE=Release -DEGOHAND_ATLAS_BRIDGE=%VIO_ATLAS%
if errorlevel 1 exit /b %ERRORLEVEL%
"%VIO_CONDA%\Lib\site-packages\cmake\data\bin\cmake.exe" --build "%VIO_BUILD_DIR%" --parallel 2
exit /b %ERRORLEVEL%
