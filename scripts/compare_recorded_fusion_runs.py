"""Compare sealed camera-space hand results and record separate VIO outcomes."""
import argparse
import json
from pathlib import Path

import numpy as np

from run_public_recovery_acceptance import save, sha


def compare(baseline, candidate):
    artifacts = [root / name for root in (baseline, candidate) for name in
        ('processing_inputs.json', 'validation.json', 'export/joints.npz',
         'runtime_samples.json', 'world_hand_audit.json')]
    hashes = {str(path): sha(path) for path in artifacts}
    plans = [json.loads((root / 'processing_inputs.json').read_text(encoding='utf-8'))
             for root in (baseline, candidate)]
    same_input = all(plans[0][key] == plans[1][key] for key in
        ('inputs', 'source_hashes', 'config', 'library_sha256'))
    fields = {}
    with np.load(baseline / 'export/joints.npz', allow_pickle=False) as left, \
            np.load(candidate / 'export/joints.npz', allow_pickle=False) as right:
        for key in sorted(set(left.files) | set(right.files)):
            fields[key] = key in left.files and key in right.files and bool(
                np.array_equal(left[key], right[key], equal_nan=True) if left[key].dtype.kind in 'fc'
                else np.array_equal(left[key], right[key]))
    outcomes = []
    for root in (baseline, candidate):
        validation = json.loads((root / 'validation.json').read_text(encoding='utf-8'))
        audit = json.loads((root / 'world_hand_audit.json').read_text(encoding='utf-8'))
        runtime = json.loads((root / 'runtime_samples.json').read_text(encoding='utf-8'))
        values = [call['ipc_track_ms'] for call in runtime['bridge_calls']]
        outcomes.append(dict(path=str(root),
            validation={key: validation.get(key) for key in ('result_frames', 'tracking', 'pose_admitted',
                'quarantined_frames', 'jump_detected_frames', 'segment_count', 'loss_episodes',
                'hand_counts', 'real_two_hand_fusion_observed', 'offline_processing_fps')},
            world_hand_errors=audit['errors'], world_joints_checked=audit['valid_world_joints_checked'],
            world_transform_max_error_m=audit['max_transform_error_m'],
            bridge_call_ms=dict(zip(('p50', 'p95', 'p99', 'max'),
                map(float, np.percentile(values, (50, 95, 99, 100)))))))
    assert all(sha(Path(path)) == value for path, value in hashes.items())
    return dict(same_input_configuration_library_root_sources=same_input,
        camera_hand_npz_field_equality=fields,
        camera_hand_numeric_and_identity_preserved=all(v for k, v in fields.items() if k != 'session_id'),
        session_id_semantics='Independent executions intentionally create separate session IDs.',
        outcomes=outcomes, hashes=hashes, script_sha256=sha(Path(__file__)),
        limits='Recorded contracts and algebra only; offline timings may overlap other runs. No absolute pose/hand accuracy, physical synchronization, realtime, or statistical noninferiority certification.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline', type=Path)
    parser.add_argument('candidate', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = compare(args.baseline, args.candidate)
    save(args.output, result)
    print(json.dumps({k: result[k] for k in ('same_input_configuration_library_root_sources',
                                           'camera_hand_numeric_and_identity_preserved')}))
