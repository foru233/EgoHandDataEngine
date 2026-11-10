"""Download the versioned official asset; record SHA256 and provenance."""
import argparse
import hashlib
import json
from datetime import datetime,timezone
from pathlib import Path
from urllib.request import Request,urlopen

MODEL_URL="https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
MODEL_ALTERNATE_URL="https://mediapipe-models.storage.googleapis.com/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"

def download_model(destination,expected_sha256=None):
    path=Path(destination)
    path.parent.mkdir(parents=True,exist_ok=True)
    used_url=MODEL_URL
    existing=path.with_suffix(".download")
    if existing.is_file() and existing.stat().st_size>1_000_000:
        data=existing.read_bytes()
        used_url=MODEL_ALTERNATE_URL
    else:
        try:
            with urlopen(Request(MODEL_URL,headers={"User-Agent":"EgoHandDataEngine/0.1"}),timeout=20) as response:
                data=response.read()
        except OSError:
            used_url=MODEL_ALTERNATE_URL
            with urlopen(Request(used_url,headers={"User-Agent":"EgoHandDataEngine/0.1"}),timeout=60) as response:
                data=response.read()
    digest=hashlib.sha256(data).hexdigest()
    if expected_sha256 and digest!=expected_sha256: raise ValueError("官方资产与预期 SHA256 不一致")
    if len(data)<1_000_000: raise ValueError("模型文件大小异常")
    import zipfile,io
    if not zipfile.is_zipfile(io.BytesIO(data)): raise ValueError("不是有效的 MediaPipe Tasks 模型包")
    temp=path.with_suffix(".download")
    temp.write_bytes(data)
    temp.replace(path)
    record={"url":used_url,"canonical_url":MODEL_URL,"model":"Hand Landmarker float16 version 1","sha256":digest,"size_bytes":len(data),"downloaded_at":datetime.now(timezone.utc).isoformat(),"license_reference":"https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker#models"}
    path.with_suffix(".json").write_text(json.dumps(record,indent=2),encoding="utf-8")
    return record

if __name__=="__main__":
    p=argparse.ArgumentParser(); p.add_argument("--output",default="models/hand_landmarker.task");p.add_argument("--sha256")
    a=p.parse_args()
    print(json.dumps(download_model(a.output,a.sha256),indent=2))
