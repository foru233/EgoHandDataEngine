"""Attribute recorded jump rejects to observed native stages; never infer GT."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from egohand.vio.vs030_geometry import prepare_vs030_vio_geometry
from run_public_recovery_acceptance import save, sha


def step(left, right):
    if left is None or right is None:
        return None
    a, b = (np.asarray(v, dtype=float).reshape(4,4) for v in (left, right))
    return dict(translation_m=float(np.linalg.norm(b[:3,3]-a[:3,3])),
                rotation_deg=float(np.rad2deg(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude())))


def audit(run, journal):
    posepath = run/'replay/poses.jsonl'
    planpath=run/'processing_inputs.json'
    plan=json.loads(planpath.read_text(encoding='utf8'))
    calibration=Path(plan['config']['paths']['calibration'])
    geometry=prepare_vs030_vio_geometry(calibration)
    orb_from_gui_inverse=np.linalg.inv(geometry.T_orb_left_from_gui_left)
    files = [posepath, journal, run/'validation.json', planpath, calibration]
    hashes = {str(p):sha(p) for p in files}
    poses = [json.loads(s) for s in posepath.open(encoding='utf8')]
    jumps = {r['frame_id']:i for i,r in enumerate(poses) if r['jump_detected']}
    stages = defaultdict(list)
    rebases = {}
    local_ba_pairs = defaultdict(dict)
    for line in journal.open(encoding='utf8'):
        row = json.loads(line)
        if row.get('type') == 'frame_stage' and row.get('frame_id') in jumps:
            stages[row['frame_id']].append(row)
        elif row.get('type') == 'last_frame_rebase' and row.get('frame_id') in jumps:
            rebases[row['frame_id']]=row
        elif row.get('type') == 'local_ba':
            local_ba_pairs[(row['map_id'],row['kind'],row['event'])][row['phase']]=row
    reports = []
    for frame_id,i in jumps.items():
        r = poses[i]
        before = next((s for s in stages[frame_id] if s['phase']=='local_before'), None)
        after = next((s for s in stages[frame_id] if s['phase'].startswith('local_') and s['phase'] not in ('local_before','local_success')), None)
        previous = poses[i-1] if i else None
        prediction = next((s for s in stages[frame_id] if s['phase']=='prediction_before'), None)
        archived_previous = previous.get('raw_matrix') if previous else None
        # Archive poses refer to raw GUI cam0, stage poses to rectified ORB cam1.
        # Compare only after applying the same calibrated camera transform.
        archived_previous_orb=(np.asarray(archived_previous) @ orb_from_gui_inverse
                               if archived_previous is not None else None)
        rebase=rebases.get(frame_id)
        lineage=None
        if rebase:
            for pair in local_ba_pairs.values():
                old,new=pair.get('before'),pair.get('after')
                if not old or not new or new['map_id']!=rebase['map_id'] or new['sequence']>=rebase['sequence']:
                    continue
                target=rebase['reference_kf_id']
                ks=[next((k for k in s['keyframes'] if k['id']==target),None) for s in (old,new)]
                if not all(ks):continue
                if lineage and lineage['after_sequence']>new['sequence']:continue
                a,b=[np.asarray(k['world_from_camera']).reshape(4,4) for k in ks]
                predicted=b@np.linalg.inv(a)@np.asarray(rebase['before']).reshape(4,4)
                residual=step(predicted,rebase['after'])
                lineage=dict(kind=new['kind'],event=new['event'],after_sequence=new['sequence'],
                    reference_kf_id=target,reference_kf_step=step(a,b),
                    rigid_transport_residual=residual,
                    after_map_change_index=new['map_change_index'])
        reports.append(dict(index=i, frame_id=frame_id, timestamp_ns=r['timestamp_ns'],
            tracking_state=r['tracking_state'], imu_initialized=r['imu_initialized'], ba2=r['ba2_complete'],
            map_id=r['map_id'], reset_id=r['reset_id'], map_revision=r['map_revision'],
            reference_generation=r['reference_generation'], world_revision=r['world_frame_revision'],
            recovery_observation=r['recovery_observation'], quality_gate=r['quality_gate'],
            previous_to_raw_step=step(archived_previous, r['raw_matrix']),
            archived_previous_to_native_previous=step(archived_previous_orb, prediction['previous']['world_from_camera']) if prediction else None,
            native_previous_to_search_seed=step(before['previous']['world_from_camera'], before['current']['world_from_camera']) if before else None,
            last_frame_rebase=rebase, latest_matching_local_ba=lineage,
            local_optimizer_phase=after['phase'] if after else None,
            map_updated=before['map_updated'] if before else None,
            local_optimizer_step=step(before['current']['world_from_camera'], after['current']['world_from_camera']) if before and after else None,
            stages=stages[frame_id]))
    assert all(sha(Path(p))==h for p,h in hashes.items())
    return dict(jump_frames=len(reports), phases=dict(Counter(r['local_optimizer_phase'] for r in reports)),
        reports=reports, hashes=hashes, source_sha256=sha(Path(__file__)), production_deployed=False,
        stage_camera_frame='rectified_orb_cam1_channel5', archive_camera_frame='raw_gui_cam0_channel4',
        T_orb_left_from_gui_left=geometry.T_orb_left_from_gui_left.tolist(),
        limits='Native observed attribution only; no external GT, counterfactual solver comparison or causal/safety claim.')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('journal',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    result=audit(args.run,args.journal)
    save(args.output,result)
    print(json.dumps(dict(jumps=result['jump_frames'],phases=result['phases'])))
