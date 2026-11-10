@echo off
setlocal
set "ROOT=%~dp0.."
if not defined ENV set "ENV=%USERPROFILE%\miniconda3\envs\egohand-vio"
set "ORB=%ROOT%\third_party\ORB_SLAM3"
set "DATA=%ROOT%\samples\date\VIO\converted\EuRoC\MH_01_easy"
set "PATH=%ENV%;%ENV%\Library\bin;%ENV%\Library\cmake\x64\vc16\bin;%ORB%\lib;%ORB%\Thirdparty\DBoW2\lib;%ORB%\Thirdparty\g2o\bin;%ORB%\build_msvc_initial\Thirdparty\g2o;%ORB%\build_msvc_initial;%PATH%"
if not exist "%ORB%\Examples\Stereo\stereo_euroc.exe" (echo BUILD_OUTPUT_MISSING & exit /b 2)
if not exist "%ORB%\Vocabulary\ORBvoc.txt" (echo VOCAB_MISSING & exit /b 3)
if not exist "%DATA%\timestamps_smoke_160.txt" (echo SMOKE_TIMESTAMPS_MISSING & exit /b 4)
if not exist "%ROOT%\results\vio\euroc_smoke" mkdir "%ROOT%\results\vio\euroc_smoke"
cd /d "%ROOT%\results\vio\euroc_smoke"
echo START_ORBSLAM_STEREO_EUROC_SMOKE_160
"%ORB%\Examples\Stereo\stereo_euroc.exe" "%ORB%\Vocabulary\ORBvoc.txt" "%DATA%\calibration\EuRoC.yaml" "%DATA%" "%DATA%\timestamps_smoke_160.txt" > "%ROOT%\logs\vio_orbslam3_stereo_smoke.log" 2>&1
set "RC=%ERRORLEVEL%"
echo ORB_STEREO_SMOKE_EXIT=%RC%
exit /b %RC%
