"""Lightweight transfer-integrity check; full ROS topic decoding is deferred to rosbags."""
from pathlib import Path
import json,hashlib,mmap
root=Path(__import__('pathlib').Path(__file__).resolve().parents[1] / 'samples/date/VIO')
paths=[
 root/"EuRoC/MH_01_easy/MH_01_easy.bag",
 root/"TUM-VI/room4/dataset-room4_512_16.bag"]
topics=[b"/cam0/image_raw",b"/cam1/image_raw",b"/imu0",b"state_groundtruth_estimate0",b"vrpn_client",b"sensor_msgs/Imu",b"sensor_msgs/Image"]
report={}
for p in paths:
 if not p.is_file():report[str(p)]={"status":"NOT_COMPLETE"};continue
 head=p.open('rb').read(20)
 if not head.startswith(b"#ROSBAG V2.0"):report[str(p)]={"status":"BAD_MAGIC"};continue
 size=p.stat().st_size
 with p.open('rb') as f:
  with mmap.mmap(f.fileno(),0,access=mmap.ACCESS_READ) as mm:
   found={q.decode():mm.find(q)>=0 for q in topics}
 report[str(p)]={"status":"HEADER_AND_TOPIC_SCAN_OK","size_bytes":size,"topic_strings_found":found}
 print(p.name,size,found,flush=True)
gt=root/"TUM-VI/room4/room4_gt_imu_frame.txt"
if gt.is_file():
 lines=gt.read_text(encoding='utf-8').splitlines()
 report["room4_gt"]={"nonempty_rows":sum(1 for x in lines if x.strip() and not x.lstrip().startswith('#')),"first_noncomment_line":next((x for x in lines if x.strip() and not x.lstrip().startswith('#')),"")}
 print("GT",report["room4_gt"],flush=True)
out=root/"download_transfer_validation.json"
out.write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
