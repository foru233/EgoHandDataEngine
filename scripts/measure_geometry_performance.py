"""CPU geometry-only comparison, with optional isolated baseline source tree."""
import argparse
import json
from pathlib import Path
import sys
import time


def main():
    p=argparse.ArgumentParser();p.add_argument('--source-root',type=Path,default=Path(__file__).resolve().parents[1]);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();sys.path.insert(0,str(args.source_root/'src'))
    import numpy as np
    from egohand.core.pipeline import EgoHandEngine
    from egohand.sources.synthetic import SyntheticStereoSource
    times=[];errors=[];valid=0
    for repeat in range(3):
        engine=EgoHandEngine();source=SyntheticStereoSource('crossing',frames=180);engine.start(source)
        for i in range(180):
            frame=source.read();start=time.perf_counter();result=engine.process(frame);elapsed=time.perf_counter()-start
            if i>=10:times.append(elapsed)
            for h in result.hands:
                index=0 if h.handedness=='Left' else 1
                for j in h.joints:
                    if j.valid:valid+=1;errors.append(float(np.linalg.norm(j.xyz_m-frame.metadata['ground_truth_xyz_m'][index,j.joint_id])))
        engine.close()
    output={'source_root':str(args.source_root.resolve()),'scope':'Synthetic projected 2D; no neural inference; not real-image accuracy',
        'frames':540,'valid_joint_observations':valid,'max_error_m':max(errors),'mean_processing_ms':1000*float(np.mean(times)),
        'p95_processing_ms':1000*float(np.percentile(times,95)),'processing_fps':1/float(np.mean(times))}
    args.output.write_text(json.dumps(output,indent=2),encoding='utf8');print(json.dumps(output,indent=2))


if __name__=='__main__':main()
