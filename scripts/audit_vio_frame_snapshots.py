"""Check frame-cache identity and separate local BA revisions from same-epoch jumps.

This audits internal algebraic consistency, not physical pose accuracy.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from audit_vio_stability import audit_rows, distribution, raw_matrix


def audit_snapshots(rows, body_from_camera=None, *, max_gap_s=.2, jump_m=.5):
    report, _, pairs = audit_rows(rows, max_gap_s=max_gap_s, jump_m=jump_m)
    identities, availability, violations = Counter(), Counter(), []
    closure = []
    previous_id = None
    for line, row in enumerate(rows, 1):
        consistent = row.get('snapshot_consistent')
        identities[str(consistent)] += 1
        availability[str(row.get('snapshot_available'))] += 1
        body = row.get('body_state_candidate')
        def issue(reason):
            violations.append({'line':line, 'reason':reason, 'timestamp_ns':row.get('timestamp_ns')})
        if consistent is False and row.get('raw_pose_valid',row.get('pose_valid')):
            issue('inconsistent_snapshot_reported_raw_valid')
        if consistent is False and body is not None:
            issue('inconsistent_snapshot_exposes_body')
        if consistent is True:
            if row.get('world_frame_id') != row.get('map_epoch'):
                issue('world_frame_id_differs_from_map_epoch')
            identity = (row.get('reset_id'),row.get('frame_id'))
            if previous_id is not None and identity[0] == previous_id[0] and identity[1] <= previous_id[1]:
                issue('frame_id_non_increasing_within_generation')
            previous_id = identity
            if not row.get('imu_initialized') and body is not None:
                issue('body_without_imu_initialization')
            if body is not None:
                for key in ('frame_id','timestamp_ns','world_frame_id'):
                    if body.get(key) != row.get(key):issue('body_'+key+'_mismatch')
                if body.get('snapshot_consistent') is not True:issue('body_not_marked_consistent')
                matrix = raw_matrix(row)
                if matrix is not None and body_from_camera is not None:
                    body_matrix = np.asarray(body['T_world_from_body'],dtype=float)
                    expected = body_matrix @ body_from_camera
                    closure.append(float(np.max(np.abs(matrix-expected))))
    categories = Counter()
    cases = []
    for pair in pairs:
        if not pair['same_epoch'] or not pair['short_gap'] or not pair['investigate_jump']:
            continue
        before,after = rows[pair['previous_line']-1], rows[pair['current_line']-1]
        a,b = before.get('map_change_index'),after.get('map_change_index')
        category = 'unknown_local_revision' if a is None or b is None else (
            'local_revision_unchanged' if a==b else 'local_revision_changed')
        categories[category] += 1
        cases.append({**pair,'local_revision_class':category,'previous_map_change_index':a,
            'current_map_change_index':b,'before_features':before.get('left_features'),
            'after_features':after.get('left_features'),'before_inliers':before.get('tracked_inliers'),
            'after_inliers':after.get('tracked_inliers')})
    return {'records':len(rows),'snapshot_consistency_counts':dict(identities),
            'snapshot_availability_counts':dict(availability),
            'inconsistent_snapshot_count':identities['False'],
            'unavailable_snapshot_count':availability['False'],
            'raw_valid_count':report['raw_valid_count'],'gated_valid_count':report['gated_valid_count'],
            'violations':violations,'violation_count':len(violations),
            'camera_body_extrinsic_closure_max_abs':distribution(closure),
            'same_epoch_short_gap_jump_revision_classes':dict(categories),'jump_cases':cases,
            'notes':['snapshot_consistent is an implementation claim checked here for internal contradictions, not proof of absence of all data races.',
                     'New native code refuses inconsistent/unavailable snapshots before raw_pose_valid; compare these counts alongside raw valid rates.',
                     'Camera/body closure checks a fixed extrinsic relation and is not independent motion or accuracy evidence.',
                     'A local revision change concurrent with a jump is correlation; it does not quantify BA versus tracking contributions.']}


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--poses',type=Path,required=True)
    p.add_argument('--calibration',type=Path)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args(argv)
    rows=[json.loads(line) for line in args.poses.read_text(encoding='utf8').splitlines() if line.strip()]
    transform=None
    if args.calibration:
        sys.path.insert(0,str(ROOT/'src'))
        from egohand.vio.vs030_geometry import prepare_vs030_vio_geometry
        geometry=prepare_vs030_vio_geometry(args.calibration)
        transform=geometry.T_imu_from_orb_left @ geometry.T_orb_left_from_gui_left
    report=audit_snapshots(rows,transform)
    report.update(input_path=str(args.poses.resolve()),calibration_path=str(args.calibration.resolve()) if args.calibration else None)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('jump_cases','violations','notes')},ensure_ascii=False))


if __name__=='__main__':main()
