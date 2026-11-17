"""Offline descriptive A/B of recorded 3D joints; no ground truth assumed."""
import json
from collections import Counter
from dataclasses import replace
from pathlib import Path
import numpy as np
from egohand.core.types import result_from_dict
from egohand.visualization.display_smoothing import DisplaySmoother
from egohand.vio.hand_identity_stabilizer import HandIdentityStabilizer


def evaluate(path):
    smoother=DisplaySmoother(); identities=HandIdentityStabilizer()
    previous={}; diffs=[]; raw_steps=[]; filtered_steps=[]; flips=Counter(); pairs=0; frames=0; valid=0
    for line in path.open(encoding='utf8'):
        result=result_from_dict(json.loads(line)); frames+=1
        stable,_=identities.update(result.hands,result.timestamp_ns)
        normalized=replace(result,hands=[replace(h.source,handedness=h.handedness) for h in stable])
        filtered=smoother.update(normalized)
        for raw, out in zip(result.hands,filtered.hands):
            mask=np.array([j.valid and np.isfinite(j.xyz_m).all() for j in raw.joints])
            xyz=np.array([j.xyz_m for j in raw.joints]); sm=np.array([j.xyz_m for j in out.joints]);valid+=int(mask.sum())
            diffs.extend(np.linalg.norm(sm[mask]-xyz[mask],axis=1))
            key=raw.track_id if raw.track_id>0 else ('side',out.handedness)
            old=previous.get(key)
            if old and 0<result.timestamp_ns-old[0]<=200_000_000:
                common=mask & old[3]
                raw_steps.extend(np.linalg.norm(xyz[common]-old[1][common],axis=1))
                filtered_steps.extend(np.linalg.norm(sm[common]-old[2][common],axis=1))
                flips['raw']+=old[4]!=raw.handedness; flips['stable']+=old[5]!=out.handedness; pairs+=1
            previous[key]=(result.timestamp_ns,xyz,sm,mask,raw.handedness,out.handedness)
    def stats(v):
        return {'count':len(v),'p50_mm':float(np.percentile(v,50)*1000),'p95_mm':float(np.percentile(v,95)*1000),'p99_mm':float(np.percentile(v,99)*1000)} if len(v) else None
    return {'recording':str(path),'frames':frames,'valid_joints':valid,'adjacent_hand_pairs':pairs,'label_changes':dict(flips),'raw_joint_step':stats(raw_steps),'filtered_joint_step':stats(filtered_steps),'filter_displacement_from_raw':stats(diffs)}


if __name__=='__main__':
    reports=[evaluate(p) for p in sorted(Path('data').rglob('results/frames.jsonl'))]
    print(json.dumps({'method':'descriptive; no ground truth or accuracy claim','recordings':reports},ensure_ascii=False,indent=2))
