@echo off
setlocal
set "ROOT=%~dp0.."
if not defined ENV set "ENV=%USERPROFILE%\miniconda3\envs\egohand-vio"
set "ORB=%ROOT%\third_party\ORB_SLAM3"
set "DATA=%ROOT%\samples\date\VIO\converted\TUM-VI\room4"
set "PATH=%ENV%;%ENV%\Library\bin;%ENV%\Library\cmake\x64\vc16\bin;%ORB%\lib;%ORB%\Thirdparty\DBoW2\lib;%ORB%\Thirdparty\g2o\bin;%ORB%\build_msvc_initial\Thirdparty\g2o;%ORB%\build_msvc_initial;%PATH%"
if not exist "%ORB%\Examples\Stereo-Inertial\stereo_inertial_tum_vi.exe" (echo BUILD_OUTPUT_MISSING & exit /b 2)
if not exist "%DATA%\timestamps_inertial_smoke_220.txt" (echo TIME_LIST_MISSING & exit /b 3)
if not exist "%ROOT%\results\vio\tumvi_inertial_smoke" mkdir "%ROOT%\results\vio\tumvi_inertial_smoke"
cd /d "%ROOT%\results\vio\tumvi_inertial_smoke"
echo START_ORBSLAM_TUMVI_STEREO_INERTIAL_220
"%ORB%\Examples\Stereo-Inertial\stereo_inertial_tum_vi.exe" "%ORB%\Vocabulary\ORBvoc.txt" "%DATA%\calibration\TUM-VI.yaml" "%DATA%\mav0\cam0\data" "%DATA%\mav0\cam1\data" "%DATA%\timestamps_inertial_smoke_220.txt" "%DATA%\mav0\imu0\data.csv" > "%ROOT%\logs\vio_orbslam3_tumvi_inertial_smoke.log" 2>&1
set "RC=%ERRORLEVEL%"
echo TUMVI_INERTIAL_SMOKE_EXIT=%RC%
exit /b %RC%
