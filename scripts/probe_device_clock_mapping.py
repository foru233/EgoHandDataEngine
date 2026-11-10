"""Measure Android IIO CLOCK_REALTIME -> Windows perf_counter_ns mapping via ADB RTT."""
import json, re, statistics, subprocess, time
from pathlib import Path

ADB=Path(__import__('pathlib').Path(__file__).resolve().parents[1] / 'tools/android/platform-tools/adb.exe')
SERIAL="<device-serial>"
rows=[]
for i in range(40):
    t0=time.perf_counter_ns()
    p=subprocess.run([str(ADB),"-s",SERIAL,"shell","date '+%s %N'"],capture_output=True,text=True,timeout=3)
    t1=time.perf_counter_ns()
    if p.returncode:
        raise RuntimeError(p.stderr)
    m=re.search(r"(\d+)\s+(\d+)",p.stdout)
    if not m: raise RuntimeError(f"unparseable device date: {p.stdout!r}")
    sec=int(m.group(1)); frac=m.group(2).strip()
    dev_ns=sec*1_000_000_000+int(frac.ljust(9,"0")[:9])
    rows.append({"i":i,"host_mid_perf_ns":(t0+t1)//2,"rtt_ns":t1-t0,
                 "device_realtime_ns":dev_ns,"offset_host_minus_device_ns":(t0+t1)//2-dev_ns})
    time.sleep(.05)
rtts=[r["rtt_ns"] for r in rows]
offsets=[r["offset_host_minus_device_ns"] for r in rows]
best=min(rows,key=lambda r:r["rtt_ns"])
out={
 "device_serial":SERIAL,"samples":len(rows),
 "rtt_ms":{"min":min(rtts)/1e6,"median":statistics.median(rtts)/1e6,"p95":sorted(rtts)[int(.95*(len(rtts)-1))]/1e6},
 "offset_host_perf_minus_device_realtime_ns":{"min":min(offsets),"median":int(statistics.median(offsets)),"max":max(offsets),"spread_ms":(max(offsets)-min(offsets))/1e6},
 "best_low_rtt_sample":best,
 "first":rows[0],"last":rows[-1],
 "warning":"ADB command midpoint is an estimate; this maps device wall clock to host monotonic only. It does not reveal camera exposure time or encode/network latency.",
 "all_samples":rows
}
path=Path(__import__('pathlib').Path(__file__).resolve().parents[1] / 'results/camera/vs030_device_clock_mapping.json')
path.write_text(json.dumps(out,indent=2),encoding="utf-8")
print(json.dumps({k:v for k,v in out.items() if k!="all_samples"},indent=2))
