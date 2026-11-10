import subprocess,time,json
from pathlib import Path
import numpy as np
ADB=str(__import__('pathlib').Path(__file__).resolve().parents[1] / 'tools/android/platform-tools/adb.exe');SER="<device-serial>"
OUT=Path(__import__('pathlib').Path(__file__).resolve().parents[1] / 'results/camera/vs030_iio_sync_probe.json')
def run(args,timeout=10):
 return subprocess.run(args,capture_output=True,timeout=timeout)
def sh(cmd):
 p=run([ADB,"-s",SER,"shell",cmd])
 if p.returncode:raise RuntimeError(cmd+" "+p.stderr.decode("utf8","replace"))
 return p.stdout.decode("utf8","replace").strip()
def rd(p):return sh("cat "+p)
def wr(p,v):
 q=run([ADB,"-s",SER,"shell",f"echo {v} > {p}"])
 if q.returncode:raise RuntimeError(q.stderr.decode("utf8","replace"))
def device_clock_sample():
 h0=time.time_ns();p=run([ADB,"-s",SER,"shell","date +%s%N"]);h1=time.time_ns()
 return {"host_before_ns":h0,"host_after_ns":h1,"device_realtime_ns":int(p.stdout.decode().strip()),"rtt_ns":h1-h0}
clock=[]
for _ in range(20):clock.append(device_clock_sample())
sensors={"gyro":{"n":1,"prefix":"in_anglvel","scale":"in_anglvel_x_scale"},
         "accel":{"n":2,"prefix":"in_accel","scale":"in_accel_x_scale"}}
saved={};raw={};host_before=time.time_ns();perf_before=time.perf_counter_ns()
try:
 for name,c in sensors.items():
  base=f"/sys/bus/iio/devices/iio:device{c['n']}"
  keys=[f"{c['prefix']}_{a}_en" for a in "xyz"]+["in_timestamp_en"]
  saved[name]={"freq":rd(base+"/sampling_frequency"),"length":rd(base+"/buffer/length"),"enable":rd(base+"/buffer/enable"),
               "scan":{k:rd(base+"/scan_elements/"+k) for k in keys}}
  if saved[name]["enable"]=="1":wr(base+"/buffer/enable",0)
  wr(base+"/sampling_frequency",240)
  for k in keys:wr(base+"/scan_elements/"+k,1)
  wr(base+"/buffer/length",512)
 for c in sensors.values():wr(f"/sys/bus/iio/devices/iio:device{c['n']}/buffer/enable",1)
 time.sleep(.05)
 procs={name:subprocess.Popen([ADB,"-s",SER,"exec-out","dd",f"if=/dev/iio:device{c['n']}","bs=16","count=512","status=none"],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        for name,c in sensors.items()}
 for name,p in procs.items():
  try:b,e=p.communicate(timeout=8)
  except subprocess.TimeoutExpired:p.kill();b,e=p.communicate()
  raw[name]=b
  print("RAW",name,len(b),"exit",p.returncode,flush=True)
 host_after=time.time_ns();perf_after=time.perf_counter_ns()
finally:
 for name,c in sensors.items():
  base=f"/sys/bus/iio/devices/iio:device{c['n']}"
  try:wr(base+"/buffer/enable",0)
  except:pass
  if name in saved:
   for k,v in saved[name]["scan"].items():
    try:wr(base+"/scan_elements/"+k,v)
    except:pass
   for k,p in [("length","buffer/length"),("freq","sampling_frequency")]:
    try:wr(base+"/"+p,saved[name][k])
    except:pass
   if saved[name]["enable"]=="1":
    try:wr(base+"/buffer/enable",1)
    except:pass
best=min(clock,key=lambda x:x["rtt_ns"])
wall_perf_offset=perf_before-host_before
report={"device_clock_sync":{"samples":clock,"best_rtt_ns":best["rtt_ns"],"estimated_host_wall_minus_device_realtime_ns":int((best["host_before_ns"]+best["host_after_ns"])//2-best["device_realtime_ns"]),
 "median_offset_ns":int(np.median([(x["host_before_ns"]+x["host_after_ns"])//2-x["device_realtime_ns"] for x in clock])),
 "host_perf_minus_wall_ns":int(wall_perf_offset)},
 "sampling":{"host_start_ns":host_before,"host_end_ns":host_after,"duration_s":(host_after-host_before)/1e9,"saved_settings":saved},"sensors":{}}
for name,c in sensors.items():
 b=raw.get(name,b"");n=len(b)//16;arr=np.frombuffer(b[:n*16],dtype=np.dtype([("x","<i2"),("y","<i2"),("z","<i2"),("pad","<u2"),("t","<i8")]))
 scale=float(rd(f"/sys/bus/iio/devices/iio:device{c['n']}/{c['scale']}"))
 vals=np.column_stack([arr["x"],arr["y"],arr["z"]]).astype(float)*scale if n else np.empty((0,3))
 ts=arr["t"].astype(np.int64);dt=np.diff(ts)
 info={"bytes":len(b),"samples":n,"scale_si_per_lsb":scale}
 if n:
  info.update({"first_raw":arr[0][0:3].tolist(),"last_raw":arr[-1][0:3].tolist(),"mean_si":vals.mean(0).tolist(),"std_si":vals.std(0).tolist(),
   "mean_norm_si":float(np.linalg.norm(vals.mean(0))),"timestamp_first_ns":int(ts[0]),"timestamp_last_ns":int(ts[-1]),
   "timestamp_monotonic":bool(np.all(dt>0)),"interval_ms":{"median":float(np.median(dt)/1e6),"p95":float(np.percentile(dt,95)/1e6),"min":float(dt.min()/1e6),"max":float(dt.max()/1e6)},
   "measured_rate_hz":float(1e9/np.median(dt))})
 report["sensors"][name]=info
 print("SENSOR",name,json.dumps(info),flush=True)
OUT.parent.mkdir(parents=True,exist_ok=True);OUT.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf8")
print("REPORT",OUT,flush=True)
