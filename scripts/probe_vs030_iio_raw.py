import subprocess,struct,json
from pathlib import Path
ADB=str(__import__('pathlib').Path(__file__).resolve().parents[1] / 'tools/android/platform-tools/adb.exe');SER="<device-serial>";n=1
def sh(c):
 p=subprocess.run([ADB,"-s",SER,"shell",c],capture_output=True,text=True,timeout=8)
 if p.returncode:raise RuntimeError(p.stderr+p.stdout)
 return p.stdout.strip()
def w(p,v):sh(f"echo {v} > {p}")
b="/sys/bus/iio/devices/iio:device1"
old={x:sh(f"cat {b}/{x}") for x in ["sampling_frequency","buffer/length","buffer/enable"]}
keys=["in_anglvel_x_en","in_anglvel_y_en","in_anglvel_z_en","in_timestamp_en"]
old["scan"]={k:sh(f"cat {b}/scan_elements/{k}") for k in keys}
raw=b""
try:
 if old["buffer/enable"]=="1":w(b+"/buffer/enable",0)
 w(b+"/sampling_frequency",240)
 for k in keys:w(b+"/scan_elements/"+k,1)
 w(b+"/buffer/length",64);w(b+"/buffer/enable",1)
 p=subprocess.run([ADB,"-s",SER,"exec-out","dd",f"if=/dev/iio:device{n}","bs=16","count=32","status=none"],capture_output=True,timeout=8)
 raw=p.stdout
 print("return",p.returncode,"len",len(raw),"stderr",p.stderr.decode("utf8","replace"))
finally:
 try:w(b+"/buffer/enable",0)
 except:pass
 for k,v in old["scan"].items():
  try:w(b+"/scan_elements/"+k,v)
  except:pass
 for k,pth in [("sampling_frequency","sampling_frequency"),("buffer/length","buffer/length")]:
  try:w(b+"/"+pth,old[k])
  except:pass
 if old["buffer/enable"]=="1":
  try:w(b+"/buffer/enable",1)
  except:pass
out=Path(__import__('pathlib').Path(__file__).resolve().parents[1] / 'results/camera/vs030_iio_raw_probe.bin');out.parent.mkdir(parents=True,exist_ok=True);out.write_bytes(raw)
print("HEX_FIRST_128",raw[:128].hex())
print("HEX_LAST_64",raw[-64:].hex())
for offset in (0,2,4,6,8):
 if len(raw)>=offset+8:
  print("S64_AT",offset,[struct.unpack_from("<q",raw,i+offset)[0] for i in range(0,min(len(raw)-offset-7,64),16)])
print("SAVED",out)
