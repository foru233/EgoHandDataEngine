from pathlib import Path
import os,json,shutil,subprocess,platform
tools={k:shutil.which(k) for k in ['conda','git','cmake','cl','ninja','winget','vcpkg','wsl','g++','clang++']}
candidates=[r'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe',
r'C:\Program Files\Microsoft Visual Studio\Installer\vswhere.exe',
r'C:\Program Files\CMake\bin\cmake.exe',
r'C:\Program Files\Git\bin\git.exe']
found={p:Path(p).is_file() for p in candidates}
vs={}
for p in candidates:
 if p.endswith("vswhere.exe") and Path(p).is_file():
  cp=subprocess.run([p,"-all","-products","*","-format","json"],capture_output=True,text=True,timeout=20)
  vs={'exit':cp.returncode,'data':cp.stdout[:6000]}
out={'system':platform.platform(),'tools':tools,'found_candidates':found,'visual_studio':vs,'project_python':str(Path(__import__('pathlib').Path(__file__).resolve().parents[1] / '.venv/Scripts/python.exe')),'wsl_installed_exe':tools['wsl'] is not None}
dest=Path(__import__('pathlib').Path(__file__).resolve().parents[1] / 'logs/vio_build_environment.json')
dest.write_text(json.dumps(out,indent=2,ensure_ascii=False),encoding='utf-8')
print(json.dumps(out,indent=2,ensure_ascii=False),flush=True)
