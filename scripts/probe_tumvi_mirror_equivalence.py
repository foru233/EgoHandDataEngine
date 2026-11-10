from pathlib import Path
import urllib.request,hashlib
p=Path(__import__('pathlib').Path(__file__).resolve().parents[1] / 'samples/date/VIO/TUM-VI/room4/dataset-room4_512_16.bag.part')
url="https://huggingface.co/datasets/gladiator7737/tum-vi/resolve/main/room4.bag?download=true"
for offset in [0,335544320,1073741824,1572864000]:
 if p.stat().st_size < offset+1048576:continue
 req=urllib.request.Request(url,headers={"User-Agent":"Mozilla/5.0","Range":f"bytes={offset}-{offset+1048575}"})
 with urllib.request.urlopen(req,timeout=30) as res:
  data=res.read(1048576);status=res.status
 with p.open('rb') as f:f.seek(offset);original=f.read(1048576)
 print("OFFSET",offset,"STATUS",status,"MATCH",original==data,"HASH",hashlib.sha256(data).hexdigest(),flush=True)
 if status!=206 or original!=data:raise RuntimeError("MIRROR MISMATCH - STOP")
