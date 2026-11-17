"""Experimental offline-only transition for inferred arms before observation.

Input and output are separate JSONL files. Measured wrist frames are never
changed; no interpolation crosses a body reference or missing body frame.
"""
import argparse,json,copy
from pathlib import Path
import numpy as np
from egohand.body.two_bone_ik import solve_two_bone_ik

def apply(rows,window=12):
    out=copy.deepcopy(rows)
    for side in ('Left','Right'):
        wrist=side+'Wrist'; elbow=side+'Elbow'
        for i,row in enumerate(rows):
            b=row.get('body')
            if not b or wrist not in b.get('measured_joints',[]):continue
            previous=rows[i-1].get('body') if i else None
            if previous and wrist in previous.get('measured_joints',[]):continue
            ref=b.get('reference_key');start=i
            while start>0 and i-start<window:
                candidate=rows[start-1].get('body')
                if not candidate or candidate.get('reference_key')!=ref or wrist in candidate.get('measured_joints',[]):break
                if wrist not in candidate.get('joints',{}) or elbow not in candidate.get('joints',{}):break
                start-=1
            if start==i or wrist not in b['joints'] or elbow not in b['joints']:continue
            # Only modify inferred targets leading into the first measured frame.
            # Smoothstep preserves endpoints and avoids a sudden velocity onset.
            for n in range(start,i):
                alpha=(n-start+1)/(i-start+1);alpha=alpha*alpha*(3-2*alpha)
                old=rows[n]['body']['joints']; shoulder=np.asarray(old[side+'Shoulder'],float)
                target=(1-alpha)*np.asarray(old[wrist],float)+alpha*np.asarray(b['joints'][wrist],float)
                pole=np.asarray(old[elbow],float)-shoulder
                upper=np.linalg.norm(np.asarray(old[elbow],float)-shoulder)
                forearm=np.linalg.norm(np.asarray(old[wrist],float)-np.asarray(old[elbow],float))
                if upper<1e-6 or forearm<1e-6:continue
                new_elbow,new_wrist,_,_=solve_two_bone_ik(shoulder,target,upper,forearm,pole)
                out[n]['body']['joints'][wrist]=new_wrist.tolist()
                out[n]['body']['joints'][elbow]=new_elbow.tolist()
                out[n]['body'].setdefault('diagnostics',{})['offline_arm_anticipation']={'side':side,'target_frame':row['frame_id'],'estimated':True}
    return out

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path);ap.add_argument('output',type=Path);ap.add_argument('--window',type=int,default=12);args=ap.parse_args()
    if args.source.resolve()==args.output.resolve():raise ValueError('input and output must differ')
    with args.source.open(encoding='utf8') as f: rows=[json.loads(x) for x in f]
    out=apply(rows,args.window);args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('w',encoding='utf8') as f:
        for row in out:f.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
    print('frames',len(out),'adjusted',sum('offline_arm_anticipation' in (x.get('body') or {}).get('diagnostics',{}) for x in out))
