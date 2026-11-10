@echo off
setlocal
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul 2>&1
if errorlevel 1 exit /b 10
if not exist "%~dp0..\build\xdsdk_probe" mkdir "%~dp0..\build\xdsdk_probe"
tar -xf "%~dp0..\SDK\delivery\dist\xdsdk-0.2.0-windows-x64.tar.gz" -C "%~dp0..\build\xdsdk_probe" lib/xdsdk.lib
if errorlevel 1 exit /b %ERRORLEVEL%
cl /nologo /std:c++17 /EHsc /MD /O2 /utf-8 /DXDSDK_USE_SHARED /I"%~dp0..\SDK\delivery\include" "%~dp0..\scripts\probe_xdsdk_stream.cpp" /Fo"%~dp0..\build\xdsdk_probe\probe.obj" /Fe"%~dp0..\build\xdsdk_probe\probe_xdsdk_stream.exe" /link "%~dp0..\build\xdsdk_probe\lib\xdsdk.lib"
exit /b %ERRORLEVEL%
