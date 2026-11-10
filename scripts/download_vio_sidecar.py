from pathlib import Path
import urllib.request,hashlib,json
root=Path(__import__('pathlib').Path(__file__).resolve().parents[1] / 'samples/date/VIO/TUM-VI/room4');root.mkdir(parents=True,exist_ok=True)
url='https://huggingface.co/datasets/gladiator7737/tum-vi/resolve/main/room4.txt?download=true'
req=urllib.request.Request(url,headers={'User-Agent':'Mozilla/5.0'})
with urllib.request.urlopen(req,timeout=30) as resp:content=resp.read()
if len(content)<100 or b'<' in content[:5]:raise RuntimeError('Invalid GT sidecar')
path=root/'room4_gt_imu_frame.txt';path.write_bytes(content)
info={'source_url':url,'size_bytes':len(content),'sha256':hashlib.sha256(content).hexdigest(),'note':'Third-party mirror ground truth; verify frame/clock with original bag before ATE.'}
(root/'room4_gt_source.json').write_text(json.dumps(info,indent=2),encoding='utf-8')
print(info,flush=True)
