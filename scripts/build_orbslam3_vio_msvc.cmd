@echo off
setlocal
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul 2>&1
if errorlevel 1 (echo VSDEVCMD_FAILED & exit /b 10)
if not defined ENV set "ENV=%USERPROFILE%\miniconda3\envs\egohand-vio"
set "PATH=%ENV%;%ENV%\Scripts;%ENV%\Library\bin;%ENV%\Library\cmake\x64\vc16\bin;%PATH%"
set "CMAKE_PREFIX_PATH=%ENV%\Library"
set "CMAKE_EXE=%ENV%\Lib\site-packages\cmake\data\bin\cmake.exe"
echo BUILD_STEREO_INERTIAL_AND_TUMVI_START
"%CMAKE_EXE%" --build "%~dp0..\third_party\ORB_SLAM3\build_msvc_initial" --target stereo_euroc stereo_inertial_euroc stereo_tum_vi stereo_inertial_tum_vi --parallel 2 > "%~dp0..\logs\vio_msvc_build_inertial_tumvi.log" 2>&1
set "RC=%ERRORLEVEL%"
echo BUILD_STEREO_INERTIAL_AND_TUMVI_EXIT=%RC%
exit /b %RC%
