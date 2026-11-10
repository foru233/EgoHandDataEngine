@echo off
setlocal
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul 2>&1
if errorlevel 1 exit /b 10
cl /nologo /std:c++17 /EHsc /MD /O2 /utf-8 /DXDSDK_USE_SHARED /I"%~dp0..\SDK\delivery\include" "%~dp0..\scripts\probe_xdsdk_live.cpp" /Fo"%~dp0..\build\xdsdk_probe\live.obj" /Fe"%~dp0..\build\xdsdk_probe\probe_live.exe" /link "%~dp0..\build\xdsdk_probe\lib\xdsdk.lib"
exit /b %ERRORLEVEL%
