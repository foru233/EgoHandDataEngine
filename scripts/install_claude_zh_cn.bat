@echo off
REM 在 Windows 上为 EgoHandDataEngine 的 VIO 工作站安装 Claude Desktop 中文界面补丁。
REM 依赖仓库根目录下 claude-desktop-zh-cn\scripts\install_windows.ps1。
REM
REM 用法：
REM   scripts\install_claude_zh_cn.bat          交互式安装（会弹 UAC）
REM   scripts\install_claude_zh_cn.bat install  非交互安装，简体中文
REM   scripts\install_claude_zh_cn.bat uninstall
REM
REM 安装前请先完全退出 Claude Desktop。

setlocal EnableExtensions
chcp 65001 >nul 2>&1

set "CLAUDE_ZH_ROOT=%~dp0..\claude-desktop-zh-cn"
if not exist "%CLAUDE_ZH_ROOT%\scripts\install_windows.ps1" (
    echo 找不到 %CLAUDE_ZH_ROOT%\scripts\install_windows.ps1
    echo 请确认 claude-desktop-zh-cn 目录完整。
    pause
    exit /b 1
)

if "%~1"=="" goto interactive

REM 非交互模式：调用方需自行以管理员身份运行。
powershell.exe -NoProfile -ExecutionPolicy Bypass -Command ^
  "& '%CLAUDE_ZH_ROOT%\scripts\install_windows.ps1' -Action '%~1' -Language '%~2'"
exit /b %ERRORLEVEL%

:interactive
powershell.exe -NoProfile -ExecutionPolicy Bypass -Command ^
  "& '%CLAUDE_ZH_ROOT%\scripts\install_windows.ps1' -Interactive"
exit /b %ERRORLEVEL%
