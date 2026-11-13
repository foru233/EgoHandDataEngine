"""Audit paired controls and compare BA2 point spread without world-origin bias."""
import argparse
import csv
import itertools
import json
import math
from pathlib import Path
import re
import numpy as np


def points(path):
    with path.open() as stream:
        return {r['id']:np.array([float(r[k]) for k in ('x','y','z')]) for r in csv.DictReader(stream)}


def spread(xyz):
    radius=np.linalg.norm(xyz-xyz.mean(axis=0),axis=1)
    return {'p50':float(np.median(radius)), 'p95':float(np.percentile(radius,95))}


def analyze_run(folder):
    result=json.loads((folder/'experiment.json').read_text())
    inputs=json.loads((folder/'inputs.json').read_text())
    rows=[json.loads(l) for l in (folder/'poses.jsonl').read_text().splitlines()]
    out={'path':str(folder),'result':result,'inputs':inputs,'frames':len(rows),
         'raw_valid':sum(r['raw_pose_valid'] for r in rows),'gated_valid':sum(r['pose_valid'] for r in rows),
         'ba2_complete':any(r.get('inertial_scale_finalized') for r in rows),
         'timestamps':[r['timestamp_ns'] for r in rows],
         'image_means':[(r['image_diagnostics']['left_mean'],r['image_diagnostics']['right_mean']) for r in rows]}
    candidates=sorted((folder/'native_diagnostics').glob('map0_event*_fullba_before_edges.csv'),
                      key=lambda p:int(re.search(r'event(\d+)',p.name)[1]))
    for edge in candidates:
        text=edge.read_text()
        # Init/BA1 use a common bias and explicit bias priors; BA2 is the first
        # per-keyframe-bias full graph, with random-walk links and no common prior.
        if 'EdgeGyroRW' not in text or 'EdgePriorAcc' in text:
            continue
        before=edge.with_name(edge.name.replace('_edges.csv','_points.csv'))
        after=edge.with_name(edge.name.replace('fullba_before_edges.csv','fullba_solved_points.csv'))
        if not after.is_file():continue
        a,b=points(before),points(after);ids=sorted(a.keys() & b.keys())
        if len(ids)<3:continue
        x,y=spread(np.array([a[k] for k in ids])),spread(np.array([b[k] for k in ids]))
        out['ba2_geometry']={'event':edge.name.split('_fullba')[0],'common_point_ids':len(ids),
                            'before_count':len(a),'after_count':len(b),'before':x,'solved':y,
                            'p50_growth':y['p50']/max(x['p50'],1e-12),
                            'p95_growth':y['p95']/max(x['p95'],1e-12)}
        break
    return out


def pair_errors(a,b):
    errors=[]
    for key in ['files','python_sources','sensor','gauge_constraint','gauge_fix','recovery_enabled','image_range_ns']:
        if a['inputs'].get(key)!=b['inputs'].get(key):errors.append('input_'+key)
    ta,tb=dict(a['inputs']['timing']),dict(b['inputs']['timing'])
    ta.pop('lattice_phase_ms',None);tb.pop('lattice_phase_ms',None)
    if ta!=tb:errors.append('other_timing_fields')
    interrupted=any(r['result']['error'] or r['result']['worker_exit_code']!=0 for r in [a,b])
    common=min(len(a['timestamps']),len(b['timestamps']))
    # A failed process consumes only a prefix of its intended image range.
    # Preserve that failure as an outcome instead of calling truncation a new input.
    if interrupted:
        if a['timestamps'][:common]!=b['timestamps'][:common]:errors.append('image_timestamps')
        if a['image_means'][:common]!=b['image_means'][:common]:errors.append('decoded_image_means')
    else:
        if a['timestamps']!=b['timestamps']:errors.append('image_timestamps')
        if a['image_means']!=b['image_means']:errors.append('decoded_image_means')
    for r in [a,b]:
        if r['result']['error'] or r['result']['worker_exit_code']!=0:errors.append('failed_run')
        if not r['ba2_complete'] or 'ba2_geometry' not in r:errors.append('ba2_incomplete')
    return sorted(set(errors))


def main():
    p=argparse.ArgumentParser()
    p.add_argument('roots',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();pairs=[];controlled=[];valid=[]
    for root in a.roots:
        rounds=sorted({p.name.rsplit('_phase_',1)[0] for p in root.glob('round_*') if p.is_dir()})
        for name in rounds:
            paths=[root/f'{name}_phase_{phase}' for phase in [97,32]]
            if not all((p/'experiment.json').is_file() for p in paths):continue
            runs=[analyze_run(p) for p in paths];errors=pair_errors(*runs)
            item={'root':str(root),'round':name,'errors':errors,
                  'runs':[{k:v for k,v in r.items() if k not in ['inputs','timestamps','image_means']} for r in runs]}
            pairs.append(item)
            input_errors=[e for e in errors if e not in ('failed_run','ba2_incomplete')]
            if not input_errors:controlled.append(runs)
            if not errors:valid.append(runs)
    out={'complete_pairs':len(pairs),'valid_controlled_pairs':len(controlled),
         'successful_geometry_pairs':len(valid),'pairs':pairs,
         'primary':'First BA2 graph, common-ID point radius p50 about centroid: solved/before; lower growth preferable.',
         'limitations':['Does not establish absolute accuracy or physical exposure clock semantics.',
                        'Incomplete BA2/failed runs are retained as outcomes, not excluded from control counts.',
                        'Conditional geometry significance needs ten complete geometry pairs; source-mismatched pairs excluded.']}
    for phase_index,phase in enumerate([97,32]):
        selected=[r[phase_index] for r in controlled]
        out[f'phase_{phase}_outcomes']={'controlled_runs':len(selected),
            'execution_failures':sum(bool(r['result']['error']) or r['result']['worker_exit_code']!=0 for r in selected),
            'ba2_incomplete':sum(not r['ba2_complete'] for r in selected),
            'missing_original_map_ba2_geometry':sum('ba2_geometry' not in r for r in selected),
            'raw_valid_median':float(np.median([r['raw_valid'] for r in selected])) if selected else None,
            'gated_valid_median':float(np.median([r['gated_valid'] for r in selected])) if selected else None}
    if valid:
        for phase_index,phase in enumerate([97,32]):
            values=[r[phase_index]['ba2_geometry']['p50_growth'] for r in valid]
            out[f'phase_{phase}_growth']={'n':len(values),'median':float(np.median(values)),'min':min(values),'max':max(values)}
    if len(valid)>=10:
        d=np.array([math.log(r[1]['ba2_geometry']['p50_growth'])-math.log(r[0]['ba2_geometry']['p50_growth']) for r in valid])
        observed=abs(float(d.mean()));extreme=0;total=0
        for signs in itertools.product([-1,1],repeat=len(d)):
            extreme+=abs(float((d*np.array(signs)).mean()))>=observed-1e-12;total+=1
        out['paired_log_growth_randomization_p_two_sided']=extreme/total
        out['median_paired_growth_ratio_32_over_97']=float(np.median(np.exp(d)))
    a.output.write_text(json.dumps(out,indent=2),encoding='utf8')
    print(json.dumps({k:v for k,v in out.items() if k!='pairs'},indent=2))


if __name__=='__main__':main()
