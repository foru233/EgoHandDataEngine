"""Persistent ADB shell clock probe; avoids one adb startup per timestamp."""
import json, re, subprocess, time, statistics
from pathlib import Path
ADB=Path(__import__('pathlib').Path(__file__).resolve().parents[1] / 'tools/android/platform-tools/adb.exe')
SERIAL="<device-serial>"
p=subprocess.Popen([str(ADB),"-s",SERIAL,"exec-out","sh","-c","while true; do date '+%s %N'; done"],
                   stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
rows=[]
try:
    for i in range(80):
        t0=time.perf_counter_ns()
        line=p.stdout.readline()
        t1=time.perf_counter_ns()
        m=re.search(r"(\d+)\s+(\d+)",line)
        if not m: raise RuntimeError(f"bad device clock line: {line!r}")
        sec=int(m.group(1));frac=m.group(2)
        dev_ns=sec*1_000_000_000+int(frac.ljust(9,"0")[:9])
        rows.append({"i":i,"host_mid_perf_ns":(t0+t1)//2,"rtt_ns":t1-t0,
                     "device_realtime_ns":dev_ns,"offset_ns":(t0+t1)//2-dev_ns})
finally:
    p.terminate()
    try:p.wait(timeout=2)
    except subprocess.TimeoutExpired:p.kill();p.wait()
rtt=[x["rtt_ns"] for x in rows];off=[x["offset_ns"] for x in rows]
best=min(rows,key=lambda x:x["rtt_ns"])
out={"samples":len(rows),"rtt_ms":{"min":min(rtt)/1e6,"median":statistics.median(rtt)/1e6,
     "p95":sorted(rtt)[int(.95*(len(rtt)-1))]/1e6},
     "offset_ns":{"min":min(off),"median":int(statistics.median(off)),"max":max(off),
     "spread_ms":(max(off)-min(off))/1e6},"best":best,"first":rows[0],"last":rows[-1],
     "warning":"Clock mapping only; not camera exposure timestamp or image encode/transport latency.",
     "all_samples":rows}
path=Path(__import__('pathlib').Path(__file__).resolve().parents[1] / 'results/camera/vs030_device_clock_mapping_persistent.json')
path.write_text(json.dumps(out,indent=2),encoding="utf-8")
print(json.dumps({k:v for k,v in out.items() if k!="all_samples"},indent=2))
