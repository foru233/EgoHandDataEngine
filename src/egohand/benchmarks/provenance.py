# 数据来源审计：保存源文件摘要和标定指纹，不将公开代码许可自动推定为数据许可。

import hashlib
import json
from pathlib import Path
from datetime import datetime,timezone
from urllib.request import Request,urlopen
import numpy as np
import cv2
from ..core.types import serializable

# 分块计算文件摘要，适合较大的 tar，不必一次读入整个数据集。
def sha256(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def calibration_fingerprint(c):
    if c is None:return None
    return hashlib.sha256(json.dumps(serializable(c),sort_keys=True,allow_nan=False).encode()).hexdigest()

# 有大小上限的公开下载；只校验和落盘，不处理登录、验证码或代替用户接受许可。
def download(url,path,expected_sha256=None,max_bytes=150_000_000):
    """Bounded public download; never authenticates or accepts a license form."""
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists():
        if expected_sha256 and sha256(path)!=expected_sha256:raise ValueError("已有文件SHA256与官方记录不符")
        return {"url":url,"sha256":sha256(path),"size_bytes":path.stat().st_size,"reused":True}
    part=path.with_suffix(path.suffix+'.part')
    with urlopen(Request(url,headers={'User-Agent':'EgoHandDataEngine/0.1 public benchmark'}),timeout=45) as response:
        size=int(response.headers.get('Content-Length') or 0)
        if size>max_bytes:raise ValueError(f"下载超过样例预算：{size} bytes")
        count=0
        with part.open('wb') as f:
            while True:
                block=response.read(1024*512)
                if not block:break
                count+=len(block)
                if count>max_bytes:raise ValueError("下载超过样例预算")
                f.write(block)
        final_url=response.url
    digest=sha256(part)
    if expected_sha256 and digest!=expected_sha256:raise ValueError("下载SHA256校验失败，保留part供排查")
    part.replace(path)
    return {"url":url,"final_url":final_url,"sha256":digest,"size_bytes":count,"downloaded_at":datetime.now(timezone.utc).isoformat()}

# 至少五组独立测量投影检查；由 GT 正投影再回算的闭环不算独立坐标证据。
def verify_projection_checks(c,evidence):
    checks=evidence.get('projection_checks',[])
    if not evidence.get('independent_observations') or len(checks)<5:
        return False,{"reason":"need >=5 independent measured left/right projections; an oracle roundtrip is not independent"}
    errors=[]
    for item in checks:
        xyz=np.asarray(item['xyz_left_m'],dtype=float).reshape(1,3)
        for side in ('left','right'):
            K,D=(c.K_left,c.D_left) if side=='left' else (c.K_right,c.D_right)
            R,t=(np.eye(3),np.zeros(3)) if side=='left' else (c.R_right_left,c.t_right_left)
            fn=cv2.fisheye.projectPoints if c.distortion_model=='fisheye' else cv2.projectPoints
            points=xyz.reshape(-1,1,3) if c.distortion_model=='fisheye' else xyz
            uv=fn(points,cv2.Rodrigues(R)[0],t,K,D)[0].reshape(2)
            errors.append(float(np.linalg.norm(uv-np.asarray(item[side+'_uv_px'],dtype=float))))
    maximum=max(errors)
    return bool(np.isfinite(errors).all() and maximum<=float(evidence.get('max_projection_error_px',3.0))),{"max_projection_error_px":maximum,"observations":len(checks),"source":evidence.get('source')}
