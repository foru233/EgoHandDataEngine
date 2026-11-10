import sys,struct,json,time
from pathlib import Path
sys.path.insert(0,str(__import__('pathlib').Path(__file__).resolve().parents[1] / 'src'))
from egohand.sources.vendor_placeholder import VendorStereoSource,VendorBundleDecoder,MAGIC
original=VendorBundleDecoder.feed
seen=[]
def wrapped(self,payload):
    if payload.startswith(MAGIC) and len(payload)>=8 and len(seen)<16:
        kind=struct.unpack_from('<I',payload,4)[0]
        seen.append({"len":len(payload),"kind":kind,"hex":payload.hex(),
                     "u32":list(struct.unpack('<'+str(len(payload)//4)+'I',payload[:len(payload)//4*4])) if len(payload)%4==0 else None})
    return original(self,payload)
VendorBundleDecoder.feed=wrapped
source=VendorStereoSource()
rows=[]
try:
    source.open();source.start()
    for i in range(12):
        f=source.read()
        rows.append({"frame_id":f.frame_id,"timestamp_ns":f.timestamp_ns,"shape_left":list(f.left_image.shape),"shape_right":list(f.right_image.shape),"metadata":f.metadata})
finally:
    source.close()
out={"packets":seen,"frames":rows}
p=Path(__import__('pathlib').Path(__file__).resolve().parents[1] / 'results/camera/vs030_live_packet_probe.json')
p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
print("CAPTURED_FRAMES",len(rows))
print("FRAME_SAMPLE",json.dumps(rows[:3],ensure_ascii=False))
print("PACKET_HEADERS")
for x in seen[:12]: print(json.dumps(x,ensure_ascii=False))
