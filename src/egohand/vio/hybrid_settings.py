"""Opt-in visual bootstrap using ORB-SLAM3's existing IMU.fastInit setting.

This only bypasses initial acceleration excitation. It does not mark the map
inertial-initialized or relax subsequent quality/numerical/recovery checks.
"""
from pathlib import Path
import hashlib
import re


def visual_bootstrap_settings(settings):
    settings=Path(settings).resolve()
    original=settings.read_bytes()
    text=original.decode('utf-8-sig').replace('\r\n','\n').replace('\r','\n')
    line='IMU.fastInit: 1'
    if re.search(r'^IMU\.fastInit\s*:',text,re.M):
        text=re.sub(r'^IMU\.fastInit\s*:.*$',line,text,flags=re.M)
    else:
        text=text.rstrip()+'\n'+line+'\n'
    root=Path(__file__).resolve().parents[3]/'userrr'/'runtime_configs'
    root.mkdir(parents=True,exist_ok=True)
    path=root/('hybrid_'+hashlib.sha256(original).hexdigest()[:20]+'.yaml')
    encoded=text.encode('utf8')
    if not path.exists() or path.read_bytes()!=encoded:
        # Stable bytes are required when phase verification binds effective settings.
        # Windows text-mode writes otherwise change LF and can duplicate existing CR.
        path.write_bytes(encoded)
    return path
