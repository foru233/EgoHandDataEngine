from __future__ import annotations
import json, struct, subprocess, statistics
from pathlib import Path
ADB=Path(__import__('pathlib').Path(__file__).resolve().parents[1] / 'tools/android/platform-tools/adb.exe')
SERIAL="<device-serial>"
def pull(path):
    p=subprocess.run([str(ADB),"-s",SERIAL,"exec-out","cat",path],capture_output=True,check=True)
    return p.stdout
def parse(data):
    if len(data)%16: raise ValueError(f"partial record bytes: {len(data)%16}")
    rows=[struct.unpack_from("<hhh2xq",data,i) for i in range(0,len(data),16)]
    ts=[r[3] for r in rows]
    dt=[(b-a)/1e6 for a,b in zip(ts,ts[1:])]
    return {"n":len(rows),"first_ns":ts[0],"last_ns":ts[-1],"duration_s":(ts[-1]-ts[0])/1e9,
            "strictly_increasing":all(b>a for a,b in zip(ts,ts[1:])),
            "dt_ms_median":statistics.median(dt),"dt_ms_p95":sorted(dt)[int(.95*(len(dt)-1))],
            "first_samples":[list(r) for r in rows[:3]],"last_samples":[list(r) for r in rows[-3:]],"timestamps":ts}
g=parse(pull("/tmp/iio_gyro_live.bin"))
a=parse(pull("/tmp/iio_accel_live.bin"))
pairs=[abs(x-y)/1e6 for x,y in zip(g["timestamps"],a["timestamps"])]
out={"gyro":{k:v for k,v in g.items() if k!="timestamps"},"accel":{k:v for k,v in a.items() if k!="timestamps"},
     "same_index_timestamp_delta_ms":{"median":statistics.median(pairs),"p95":sorted(pairs)[int(.95*(len(pairs)-1))],"max":max(pairs)},
     "clock_start":pull("/tmp/iio_clock_start.txt").decode(errors="replace"),
     "clock_end":pull("/tmp/iio_clock_end.txt").decode(errors="replace"),
     "record_format":"little-endian int16 x,y,z + int64 timestamp; 16 bytes",
     "note":"IIO clock domain is device-side; same-index comparison is diagnostic only, not proof of synchronized sampling"}
path=Path(__import__('pathlib').Path(__file__).resolve().parents[1] / 'results/camera/vs030_iio_simultaneous_probe.json')
path.write_text(json.dumps(out,indent=2),encoding="utf-8")
print(json.dumps({k:v for k,v in out.items() if k not in ("gyro","accel")},indent=2))
print("GYRO",json.dumps(out["gyro"],indent=2))
print("ACCEL",json.dumps(out["accel"],indent=2))
