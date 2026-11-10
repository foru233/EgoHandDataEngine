"""Repeatable CPU measurements; synthetic geometry and actual model are separated."""
import hashlib
import json
import platform
import statistics
import time
from pathlib import Path
import cv2
import numpy as np
import psutil
from egohand.core.pipeline import EgoHandEngine
from egohand.sources.synthetic import SyntheticStereoSource
from egohand.detection.mediapipe_detector import MediaPipeHandDetector
from egohand.storage.recorder import Recorder
from egohand.storage.exporter import export_all,export_json
from verify_environment import inspect

def summarize(values):
    return {"mean":float(np.mean(values)),"p50":float(np.percentile(values,50)),"p95":float(np.percentile(values,95)),"max":float(np.max(values))}

def main():
    root=Path("logs");root.mkdir(exist_ok=True)
    tag=time.strftime("%Y%m%d_%H%M%S")
    report={"executed_at":time.strftime("%Y-%m-%d %H:%M:%S"),"environment":inspect(),"mode":"CPU; synthetic geometry + independent real-image detector"}
    source=SyntheticStereoSource("crossing",frames=180)
    e=EgoHandEngine();e.start(source)
    read=[];process=[];tri=[];errors=[];results=[];frames=[];rss=[]
    start=time.perf_counter()
    for _ in range(180):
        t=time.perf_counter();f=source.read();read.append((time.perf_counter()-t)*1000)
        r=e.process(f);results.append(r);frames.append(f)
        process.append(r.processing_time_ms);tri.append(r.diagnostics["matching_and_triangulation_ms"])
        rss.append(psutil.Process().memory_info().rss/1024**2)
        for h in r.hands:
            idx=0 if h.handedness=="Left" else 1
            for j in h.joints:
                if j.valid:errors.append(float(np.linalg.norm(j.xyz_m-f.metadata["ground_truth_xyz_m"][idx,j.joint_id])))
    duration=time.perf_counter()-start
    report["synthetic"]={"frames":len(results),"resolution":[640,480],"read_ms":summarize(read),"process_ms":summarize(process),"matching_triangulation_ms":summarize(tri),"total_wall_fps":len(results)/duration,"read_capacity_fps":1000/np.mean(read),"processing_capacity_fps":1000/np.mean(process),"xyz_error_m":summarize(errors),"valid_joint_observations":len(errors),"peak_rss_mib":max(rss),"note":"truth projection bypasses neural inference only; matching/triangulation/validation run normally"}
    target=Path("recordings")/f"benchmark_{tag}"
    w=Recorder(target,source.get_metadata(),source.get_calibration(),e.config)
    start=time.perf_counter()
    for f,r in zip(frames[:60],results[:60]):w.write_raw(f);w.write_result(r)
    w.close();write_duration=time.perf_counter()-start
    report["recording"]={"path":str(target),"frames":60,"seconds":write_duration,"frames_per_second":60/write_duration,"raw_uncompressed_MB_s":sum(f.left_image.nbytes+f.right_image.nbytes for f in frames[:60])/write_duration/1e6,"actual_file_MB_s":sum(p.stat().st_size for p in target.rglob("*") if p.is_file())/write_duration/1e6}
    export_all(results,Path("exports")/f"benchmark_{tag}")
    e.close()
    model=Path("models/hand_landmarker.task");image=Path("samples/public/hand.jpg")
    if model.exists() and image.exists():
        img=cv2.imread(str(image));small=cv2.resize(img,(640,round(img.shape[0]*640/img.shape[1])))
        a=MediaPipeHandDetector(model,"VIDEO");b=MediaPipeHandDetector(model,"VIDEO")
        left=[];right=[];count=[]
        try:
            for i in range(35):
                t=time.perf_counter();ds=a.detect(small,1_000_000_000+i*33_333_333);left.append((time.perf_counter()-t)*1000)
                t=time.perf_counter();b.detect(small,1_000_000_000+i*33_333_333);right.append((time.perf_counter()-t)*1000)
                count.append(len(ds))
            report["real_image"]={"model_sha256":a.sha256,"resolution":[small.shape[1],small.shape[0]],"left_detection_ms":summarize(left[5:]),"right_detection_ms":summarize(right[5:]),"sequential_two_detector_capacity_fps":1000/(np.mean(left[5:])+np.mean(right[5:])),"detected_hands":count,"note":"same public photo benchmarked independently in two detector instances; never passed as a stereo pair; no metric result generated"}
            export_json([results[0]],root/"synthetic_result_example.json")
        finally:a.close();b.close()
    else:report["real_image"]={"status":"BLOCKED_MISSING_ASSETS"}
    path=root/"performance.json";path.write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding="utf-8")
    print(json.dumps({"report":str(path),"max_xyz_error_m":max(errors),"synthetic_wall_fps":report["synthetic"]["total_wall_fps"],"recording_fps":report["recording"]["frames_per_second"]},indent=2))

if __name__=="__main__":main()
