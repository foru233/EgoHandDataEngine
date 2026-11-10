"""Download ONLY selected public VIO benchmark sequences to this Windows project.
Resumable direct streaming, provenance and SHA256, no full-dataset clone."""
import argparse,hashlib,json,os,time,urllib.request,urllib.error,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]/"samples"/"date"/"VIO"
FILES={
 "euroc_MH_01_easy":("EuRoC/MH_01_easy/MH_01_easy.bag", "https://huggingface.co/datasets/kavehsgh/EuRoC_MAV_Dataset_Machine_Hall_Easy_01/resolve/main/MH_01_easy.bag?download=true",2673818914,"third-party mirror of EuRoC, source ETH Zurich"),
 "tumvi_room4":("TUM-VI/room4/dataset-room4_512_16.bag","https://vision.in.tum.de/tumvi/calibrated/512_16/dataset-room4_512_16.bag",2346890121,"TUM VI official 512_16 calibrated ROS1 bag"),
}
p=argparse.ArgumentParser();p.add_argument("--which",choices=list(FILES),required=True);args=p.parse_args()
sub,url,expected,note=FILES[args.which]
dest=ROOT/sub;dest.parent.mkdir(parents=True,exist_ok=True);partial=dest.with_suffix(dest.suffix+".part")
if shutil.disk_usage(dest.parent).free < expected*2:raise RuntimeError("Insufficient free space")
if dest.exists() and dest.stat().st_size==expected:print("ALREADY_COMPLETE",dest,flush=True)
else:
 for attempt in range(1,9):
  position=partial.stat().st_size if partial.exists() else 0
  if position>expected:raise RuntimeError("Partial larger than expected")
  headers={"User-Agent":"EgoHandDataEngine-VIO/1.0"}
  if position:headers["Range"]=f"bytes={position}-"
  try:
   response=urllib.request.urlopen(urllib.request.Request(url,headers=headers),timeout=45)
   if position and response.status!=206:
    print("WARNING: range unsupported; restarting",flush=True);response.close();partial.unlink();continue
   if position:
    cr=response.headers.get("Content-Range","")
    if not cr.startswith(f"bytes {position}-"):raise RuntimeError("Invalid Content-Range "+cr)
   received=position
   print("DOWNLOADING",args.which,"attempt",attempt,"resuming",position,"expected",expected,"status",response.status,flush=True)
   prev=time.monotonic(); prev_bytes=received
   with open(partial,"ab" if position else "wb") as f:
    while True:
     block=response.read(1024*1024)
     if not block:break
     f.write(block);received+=len(block)
     if received>expected:raise RuntimeError("Received more than expected")
     if time.monotonic()-prev>12:
      print("PROGRESS",args.which,received,expected,round(received/expected*100,2), "percent",round((received-prev_bytes)/(max(time.monotonic()-prev,.01)*1e6),2),"MB/s",flush=True)
      prev=time.monotonic();prev_bytes=received
    f.flush();os.fsync(f.fileno())
   response.close()
   if received!=expected:raise RuntimeError(f"Short read {received}/{expected}")
   os.replace(partial,dest)
   break
  except Exception as ex:
   print("RETRY",attempt,type(ex).__name__,str(ex)[:180],flush=True)
   if attempt==8:raise
   time.sleep(min(attempt*2,12))
with dest.open("rb") as f:
 head=f.read(16)
print("HEADER",repr(head),flush=True)
if not head.startswith(b"#ROSBAG V2.0"):raise RuntimeError("Invalid ROS1 bag magic")
h=hashlib.sha256()
with dest.open("rb") as f:
 while b:=f.read(16*1024*1024):h.update(b)
manifest={"sequence":args.which,"source_url":url,"source_note":note,"size_bytes":dest.stat().st_size,"expected_size":expected,"sha256":h.hexdigest(),"local_path":str(dest),"format":"rosbag-v2","valid_header":True,"download_complete":True}
manifest_file=dest.with_suffix(".download_manifest.json")
manifest_file.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding="utf8")
print("COMPLETE",json.dumps(manifest,ensure_ascii=False),flush=True)
