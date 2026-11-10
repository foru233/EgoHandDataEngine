from pathlib import Path
import numpy as np,json
root=Path(__import__('pathlib').Path(__file__).resolve().parents[1])
folders={
 "euroc_stereo":root/"results/vio/euroc_smoke",
 "euroc_stereo_inertial":root/"results/vio/euroc_inertial_smoke",
 "tumvi_stereo_inertial":root/"results/vio/tumvi_inertial_smoke",
}
out={}
for name,folder in folders.items():
 out[name]={}
 for typ,file in [("frame","CameraTrajectory.txt"),("keyframe","KeyFrameTrajectory.txt")]:
  path=folder/file
  if not path.exists() or path.stat().st_size==0:
   out[name][typ]={"status":"missing_or_empty"};continue
  try:
   a=np.atleast_2d(np.loadtxt(path))
   if a.shape[1]!=8:raise ValueError("Expected 8 columns [t xyz xyzw], got "+str(a.shape))
   quat=np.linalg.norm(a[:,4:],axis=1)
   # SaveTrajectoryEuRoC 输出的是纳秒；先作差再换算，避免丢失过多精度。
   out[name][typ]={"count":int(len(a)),"timestamp_unit":"nanoseconds","time_span_s":float((a[-1,0]-a[0,0])*1e-9),"finite":bool(np.isfinite(a).all()),"max_abs_m":float(np.max(np.abs(a[:,1:4]))),"start_pos_m":a[0,1:4].round(4).tolist(),"end_pos_m":a[-1,1:4].round(4).tolist(),"quat_norm_min":float(quat.min()),"quat_norm_max":float(quat.max()),"timestamps_strictly_increasing":bool(np.all(np.diff(a[:,0])>0))}
  except Exception as ex:out[name][typ]={"status":"parse_failed","error":str(ex)}
  print(name,typ,out[name][typ],flush=True)
dest=root/"results/vio/runtime_validation.json"
dest.parent.mkdir(exist_ok=True,parents=True)
dest.write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf8')
print("REPORT",dest,flush=True)
