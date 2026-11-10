import argparse
import json
import time
from pathlib import Path
from egohand.core.pipeline import EgoHandEngine
from egohand.sources.synthetic import SyntheticStereoSource,SCENARIOS
from egohand.sources.recorded import RecordedStereoSource
from egohand.detection.mediapipe_detector import MediaPipeHandDetector
from egohand.storage.recorder import Recorder
from egohand.storage.exporter import export_all

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--scenario",choices=SCENARIOS,default="two_static")
    p.add_argument("--frames",type=int,default=120)
    p.add_argument("--recorded")
    p.add_argument("--reuse-detections",action="store_true")
    p.add_argument("--output",default=None)
    p.add_argument("--record")
    a=p.parse_args()
    source=RecordedStereoSource(a.recorded,reuse_detections=a.reuse_detections) if a.recorded else SyntheticStereoSource(a.scenario,frames=a.frames)
    engine=EgoHandEngine();engine.start(source)
    writer=Recorder(a.record,source.get_metadata(),source.get_calibration(),engine.config) if a.record else None
    if a.recorded and not a.reuse_detections:
        engine.detectors=tuple(MediaPipeHandDetector("models/hand_landmarker.task","VIDEO") for _ in range(2))
    results=[]
    try:
        for _ in range(a.frames):
            frame=source.read()
            if frame is None:break
            frame.metadata["processing_mode"]="offline"
            if writer:writer.write_raw(frame)
            result=engine.process(frame);results.append(result)
            if writer:writer.write_result(result)
        output=a.output or "exports/cli_"+time.strftime("%Y%m%d_%H%M%S")
        export_all(results,output)
        print(json.dumps({"frames":len(results),"valid_joints":sum(h.valid_joint_count for r in results for h in r.hands),"output":output}))
    finally:
        if writer:writer.close()
        engine.close()

if __name__=="__main__":main()
