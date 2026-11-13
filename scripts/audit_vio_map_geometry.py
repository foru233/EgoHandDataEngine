"""Separate whole-map distortion from distant point contamination of the centroid.

Read-only common-ID comparisons. Pairwise ratios are invariant to world rigid motion.
The point sample is deterministic and bounded; it is not an accuracy ground truth.
"""
import argparse
import json
from pathlib import Path
import re
import sys
import numpy as np
from scipy.spatial.distance import pdist
sys.path.insert(0,str(Path(__file__).resolve().parent))
from summarize_vio_phase_confirmation import points
from compare_vio_optimization_snapshots import pose_alignment


def pairwise_geometry(before, after, *, maximum=2000):
    before=np.asarray(before);after=np.asarray(after)
    if len(before)!=len(after) or len(before)<3:
        return {'available':False}
    finite=np.isfinite(before).all(axis=1)&np.isfinite(after).all(axis=1)
    nonfinite=int(len(before)-finite.sum())
    before,after=before[finite],after[finite]
    if len(before)<3:return {'available':False,'nonfinite_points':nonfinite}
    selection=np.sort(np.random.default_rng(20261112).choice(len(before),min(maximum,len(before)),replace=False))
    x,y=before[selection],after[selection]
    a,b=pdist(x),pdist(y)
    valid=a>1e-6
    if not valid.any():return {'available':False,'reason':'coincident_points'}
    radius=lambda p:np.linalg.norm(p-p.mean(axis=0),axis=1)
    ca,cb=radius(before),radius(after)
    return {'available':True,'same_ids':len(before),'sampled_points':len(selection),
            'sampled_pairs':len(a),'nonfinite_points':nonfinite,
            'pair_distance_before_p50_p95_m':np.percentile(a,[50,95]).tolist(),
            'pair_distance_after_p50_p95_m':np.percentile(b,[50,95]).tolist(),
            'same_pair_ratio_p05_p50_p95':np.percentile(b[valid]/a[valid],[5,50,95]).tolist(),
            'pair_absolute_change_p50_p95_m':np.percentile(np.abs(b-a),[50,95]).tolist(),
            'centroid_radius_p50_growth':float(np.median(cb)/max(np.median(ca),1e-12)),
            'centroid_displacement_m':float(np.linalg.norm(after.mean(0)-before.mean(0)))}


def common_arrays(a,b):
    ids=sorted(a.keys()&b.keys())
    return np.array([a[k] for k in ids]),np.array([b[k] for k in ids])


def audit_run(folder):
    diag=folder/'native_diagnostics'
    result={'run':str(folder),'events':[]}
    inputs=folder/'inputs.json'
    if inputs.is_file():result['phase_ms']=json.loads(inputs.read_text())['timing']['lattice_phase_ms']
    for edge in sorted(diag.glob('map0_event*_fullba_before_edges.csv'),
                       key=lambda p:int(re.search(r'event(\d+)',p.name)[1])):
        text=edge.read_text()
        if 'EdgeGyroRW' not in text or 'EdgePriorAcc' in text:continue
        prefix=edge.name.removesuffix('_fullba_before_edges.csv')
        files={stage:{kind:diag/f'{prefix}_{stage}_{kind}.csv' for kind in ('points','kf')}
               for stage in ('fullba_before','fullba_solved')}
        if not all(p.is_file() for group in files.values() for p in group.values()):continue
        row={'event':prefix}
        for kind in ('points','kf'):
            a,b=(points(files[stage][kind]) for stage in ('fullba_before','fullba_solved'))
            row[kind]=pairwise_geometry(*common_arrays(a,b))
        alignment=pose_alignment(files['fullba_before']['kf'],files['fullba_solved']['kf'])
        row['keyframe_rigid_alignment']=alignment
        if alignment['status']=='ok':
            x,y=common_arrays(points(files['fullba_before']['points']),points(files['fullba_solved']['points']))
            predicted=x@np.asarray(alignment['rotation']).T+alignment['translation']
            residual=np.linalg.norm(y-predicted,axis=1)
            row['points_after_keyframe_rigid_alignment']={
                'residual_p50_p95_p99_max_m':np.percentile(residual,[50,95,99,100]).tolist(),
                'over_1m_fraction':float(np.mean(residual>1))}
        result['events'].append(row)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('roots',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();reports=[]
    for root in a.roots:
        folders=[root] if (root/'native_diagnostics').is_dir() else sorted(root.glob('round_*'))
        for folder in folders:
            if folder.is_dir():reports.append(audit_run(folder))
    result={'runs':reports,'interpretation':'Mean-centroid radius can be dominated by a few remote points; '
            'inspect same-pair distance ratios, keyframe deformation and aligned point residual tails together.',
            'limitations':['No absolute accuracy truth. Rigid alignment is for diagnosis only; no runtime pose is changed.',
                           'These are alternative diagnostics, not a retroactive replacement of a preregistered primary metric.',
                           'All runs, including failed/incomplete BA2, remain present; empty event lists are unavailable, not zero.']}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps({'runs':len(reports),'output':str(a.output)}))


if __name__=='__main__':main()
