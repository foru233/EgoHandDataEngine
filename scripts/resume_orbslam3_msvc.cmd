@echo off
setlocal
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul 2>&1
if errorlevel 1 (echo VSDEVCMD_FAILED & exit /b 10)
if not defined ENV set "ENV=%USERPROFILE%\miniconda3\envs\egohand-vio"
set "PATH=%ENV%;%ENV%\Scripts;%ENV%\Library\bin;%ENV%\Library\cmake\x64\vc16\bin;%PATH%"
set "CMAKE_PREFIX_PATH=%ENV%\Library"
set "CMAKE_EXE=%ENV%\Lib\site-packages\cmake\data\bin\cmake.exe"
echo CMAKE_BUILD_START
"%CMAKE_EXE%" --build "%~dp0..\third_party\ORB_SLAM3\build_msvc_initial" --target stereo_euroc --parallel 2 > "%~dp0..\logs\vio_resume_build_stereo_allstatic.log" 2>&1
set "RC=%ERRORLEVEL%"
echo CMAKE_BUILD_EXIT=%RC%
exit /b %RC%
