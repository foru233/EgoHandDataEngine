import sys,struct,json,time
from pathlib import Path
sys.path.insert(0,str(__import__('pathlib').Path(__file__).resolve().parents[1] / 'src'))
from egohand.sources.vendor_placeholder import VendorStereoSource,VendorBundleDecoder,MAGIC
orig=VendorBundleDecoder.feed; packets=[]
def feed(self,payload):
    if payload.startswith(MAGIC) and len(payload)>=8 and len(packets)<100:
        kind=struct.unpack_from('<I',payload,4)[0]
        if kind==1 and len(payload)==48:
            packets.append({"host_wall_ns":time.time_ns(),"host_perf_ns":time.perf_counter_ns(),"u32":list(struct.unpack('<12I',payload)),"u64_offset16":struct.unpack_from('<Q',payload,16)[0],"u64_offset24":struct.unpack_from('<Q',payload,24)[0],"u64_offset32":struct.unpack_from('<Q',payload,32)[0],"hex":payload.hex()})
    return orig(self,payload)
VendorBundleDecoder.feed=feed
s=VendorStereoSource();frames=[]
try:
 s.open();s.start()
 for i in range(24):
  f=s.read();frames.append({"frame_id":f.frame_id,"timestamp_ns":f.timestamp_ns,"host_wall_ns":time.time_ns(),"host_perf_ns":time.perf_counter_ns()})
finally:s.close()
p=Path(__import__('pathlib').Path(__file__).resolve().parents[1] / 'results/camera/vs030_header_timing_probe.json')
p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps({"packets":packets,"frames":frames},ensure_ascii=False,indent=2),encoding="utf8")
print("frames",len(frames),"headers",len(packets))
for i,x in enumerate(packets[:24]):print(i,x["u32"],"u64",x["u64_offset16"],x["u64_offset24"],x["u64_offset32"])
print("FRAME_TIMES",[(x["frame_id"],x["timestamp_ns"],x["host_perf_ns"]) for x in frames[:8]])
