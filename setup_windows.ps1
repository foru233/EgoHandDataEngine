param([string]$PythonExe = "", [switch]$DownloadAssets)
$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
if (-not (Test-Path -LiteralPath '.venv\Scripts\python.exe')) {
    if ($PythonExe) {
        & $PythonExe -m venv .venv
    } elseif (Get-Command uv -ErrorAction SilentlyContinue) {
        uv venv --python 3.12 .venv
    } else {
        throw 'Supply -PythonExe pointing to Python 3.12, or install uv first.'
    }
    if ($LASTEXITCODE -ne 0) { throw 'Failed to create project environment.' }
}
$env:UV_HTTP_TIMEOUT = '300'
if (Get-Command uv -ErrorAction SilentlyContinue) {
    if (Test-Path -LiteralPath 'requirements-lock.txt') {
        uv pip install --python .venv\Scripts\python.exe -r requirements-lock.txt
        if ($LASTEXITCODE -ne 0) { throw 'Locked dependency installation failed.' }
        uv pip install --python .venv\Scripts\python.exe --no-deps -e .
    } else {
        uv pip install --python .venv\Scripts\python.exe -r requirements-dev.txt
    }
} else {
    if (Test-Path -LiteralPath 'requirements-lock.txt') {
        .venv\Scripts\python.exe -m pip install -r requirements-lock.txt
        if ($LASTEXITCODE -ne 0) { throw 'Locked dependency installation failed.' }
        .venv\Scripts\python.exe -m pip install --no-deps -e .
    } else {
        .venv\Scripts\python.exe -m pip install -r requirements-dev.txt
    }
}
if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed.' }
.venv\Scripts\python.exe scripts\verify_environment.py
if ($LASTEXITCODE -ne 0) { throw 'Environment verification failed.' }
if ($DownloadAssets) {
    .venv\Scripts\python.exe scripts\download_models.py
    if ($LASTEXITCODE -ne 0) { throw 'Official model download failed; see models/README.md.' }
    .venv\Scripts\python.exe scripts\download_sample.py
}
Write-Host 'Setup complete. Run run_gui.bat or run_tests.bat.'
