import importlib.metadata
import json
import platform
import shutil
import subprocess
import sys
from pathlib import Path
import psutil

def inspect():
    packages={}
    for name in ("numpy","scipy","opencv-contrib-python","mediapipe","PySide6","pyqtgraph","PyOpenGL","PyYAML","pytest","pytest-qt","psutil"):
        try:packages[name]=importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:packages[name]="MISSING"
    try:
        gpu=subprocess.run(["nvidia-smi","--query-gpu=name,memory.total,driver_version","--format=csv,noheader"],capture_output=True,text=True,timeout=10).stdout.strip()
    except (OSError,subprocess.TimeoutExpired):gpu="not available"
    return {"os":platform.platform(),"python":sys.version,"executable":sys.executable,"cpu":platform.processor(),"logical_cpus":psutil.cpu_count(),"physical_cpus":psutil.cpu_count(logical=False),"memory_gib":psutil.virtual_memory().total/1024**3,"disk_free_gib":shutil.disk_usage(Path.cwd()).free/1024**3,"gpu":gpu,"packages":packages}

if __name__=="__main__":
    data=inspect();Path("logs").mkdir(exist_ok=True)
    Path("logs/environment.json").write_text(json.dumps(data,indent=2),encoding="utf-8")
    print(json.dumps(data,indent=2))
    if "MISSING" in data["packages"].values():sys.exit(1)
