import subprocess, time, json
from pathlib import Path
import numpy as np
ADB=str(__import__('pathlib').Path(__file__).resolve().parents[1] / 'tools/android/platform-tools/adb.exe')
SERIAL="<device-serial>"
OUT=Path(__import__('pathlib').Path(__file__).resolve().parents[1] / 'results/camera/vs030_iio_probe.json')
def sh(cmd,check=True):
    p=subprocess.run([ADB,"-s",SERIAL,"shell",cmd],capture_output=True,text=True,timeout=8)
    if check and p.returncode: raise RuntimeError(cmd+" failed: "+p.stderr+" "+p.stdout)
    return p.stdout.strip()
def read(path): return sh("cat "+path)
def write(path,value): sh("echo "+str(value)+" > "+path)
sensors={"gyro":{"n":1,"prefix":"in_anglvel","scale":"in_anglvel_x_scale"},
         "accel":{"n":2,"prefix":"in_accel","scale":"in_accel_x_scale"}}
saved={};raw={};host_before=time.time_ns()
try:
    for name,c in sensors.items():
        n=c["n"];base=f"/sys/bus/iio/devices/iio:device{n}"
        saved[name]={"freq":read(base+"/sampling_frequency"),"length":read(base+"/buffer/length"),
                     "enable":read(base+"/buffer/enable"),"scan":{}}
        for axis in "xyz":
            key=f"{c['prefix']}_{axis}_en";saved[name]["scan"][key]=read(base+"/scan_elements/"+key)
        saved[name]["scan"]["in_timestamp_en"]=read(base+"/scan_elements/in_timestamp_en")
        if saved[name]["enable"]=="1":write(base+"/buffer/enable",0)
        write(base+"/sampling_frequency",240)
        for key in saved[name]["scan"]:write(base+"/scan_elements/"+key,1)
        write(base+"/buffer/length",512)
    for c in sensors.values():write(f"/sys/bus/iio/devices/iio:device{c['n']}/buffer/enable",1)
    procs={name:subprocess.Popen([ADB,"-s",SERIAL,"exec-out","dd",f"if=/dev/iio:device{c['n']}","bs=16","count=512"],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
           for name,c in sensors.items()}
    for name,p in procs.items():
        try:out,err=p.communicate(timeout=6)
        except subprocess.TimeoutExpired:p.kill();out,err=p.communicate()
        raw[name]=out
        print("RAW",name,"bytes",len(out),"return",p.returncode,"stderr",err.decode("utf8","replace")[-300:],flush=True)
    host_after=time.time_ns()
finally:
    for name,c in sensors.items():
        base=f"/sys/bus/iio/devices/iio:device{c['n']}"
        try:write(base+"/buffer/enable",0)
        except Exception as e:print("disable warn",name,e,flush=True)
        if name in saved:
            for key,val in saved[name]["scan"].items():
                try:write(base+"/scan_elements/"+key,val)
                except Exception:pass
            for key in ("length","freq"):
                try:write(base+("/buffer/length" if key=="length" else "/sampling_frequency"),saved[name][key])
                except Exception:pass
            if saved[name]["enable"]=="1":
                try:write(base+"/buffer/enable",1)
                except Exception:pass
report={"host_time_before_ns":host_before,"host_time_after_ns":host_after,"device_date_after":sh("date +%s%N",False),"saved_settings":saved,"sensors":{}}
for name,c in sensors.items():
    b=raw.get(name,b"");usable=len(b)//16
    arr=np.frombuffer(b[:usable*16],dtype=np.dtype([("x","<i2"),("y","<i2"),("z","<i2"),("pad","<u2"),("t","<i8")]))
    scale=float(read(f"/sys/bus/iio/devices/iio:device{c['n']}/{c['scale']}"))
    if len(arr):
        vals=np.column_stack([arr["x"],arr["y"],arr["z"]]).astype(float)*scale
        ts=arr["t"].astype(np.int64);dt=np.diff(ts)
        info={"samples":int(len(arr)),"scale":scale,"mean_si":vals.mean(0).tolist(),"std_si":vals.std(0).tolist(),
              "mean_vector_norm_si":float(np.linalg.norm(vals.mean(0))),"timestamp_first_ns":int(ts[0]),"timestamp_last_ns":int(ts[-1]),
              "timestamp_monotonic":bool(np.all(dt>0)),
              "sample_interval_ms":{"median":float(np.median(dt)/1e6),"p95":float(np.percentile(dt,95)/1e6),"min":float(dt.min()/1e6),"max":float(dt.max()/1e6)} if len(dt) else None,
              "approx_rate_hz":float(1e9/np.median(dt)) if len(dt) else None}
    else:info={"samples":0,"scale":scale,"raw_bytes":len(b)}
    report["sensors"][name]=info;print("SENSOR",name,json.dumps(info),flush=True)
OUT.parent.mkdir(parents=True,exist_ok=True);OUT.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf8")
print("REPORT",OUT,flush=True)
