import importlib.util
from pathlib import Path

import numpy as np


spec = importlib.util.spec_from_file_location('world_hand_audit',
    Path(__file__).resolve().parents[1] / 'scripts/audit_recorded_world_hands.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def test_unavailable_allows_only_explicitly_invalid_coordinates():
    invalid = {'valid': [False], 'xyz_world_m': [[None, None, None]]}
    report = audit.audit([{'frame_id': 1, 'world_hands': {
        'status': 'no_valid_hand_joints', 'hands': [invalid]}}])
    assert report['errors'] == []
    invalid['valid'] = [True]
    invalid['xyz_world_m'] = [[1., 2., 3.]]
    report = audit.audit([{'frame_id': 1, 'world_hands': {
        'status': 'unavailable', 'hands': [invalid]}}])
    assert report['errors'][0]['reason'] == 'unavailable_contains_world_coordinates'


def test_available_requires_admission_and_checks_recorded_transform(monkeypatch):
    joints = [{'valid': True, 'xyz_m': [1., 2., 3.]} for _ in range(21)]
    pose = {'map_epoch': 0, 'world_frame_revision': 0, 'reference_generation': 0,
            'recovery_preaccept': {'session_id': 's', 'frame_sequence': 1}}
    row = {'frame_id': 7, 'timestamp_ns': 12, 'diagnostics': {'vio_pose': pose},
           'hands': [{'track_id': 1, 'handedness': 'Left', 'joints': joints}],
           'world_hands': {'status': 'available', 'frame_id': 7, 'timestamp_ns': 12,
               'pose_timestamp_ns': 12, 'world_reference': {'map_epoch': 0,
                   'world_frame_revision': 0, 'reference_generation': 0, 'cross_frame_merge_permitted': False},
               'join': {'recovery_session_id': 's', 'recovery_frame_sequence': 1, 'source_frame_index': 7},
               'hands': [{'track_id': 1, 'handedness': 'Left', 'valid': [True]*21,
                          'xyz_world_m': [[1., 2., 3.]]*21}]}}
    monkeypatch.setattr(audit, 'consumer_map_pose', lambda *a, **k: None)
    assert audit.audit([row])['errors'][0]['reason'] == 'available_without_admitted_pose'
    monkeypatch.setattr(audit, 'consumer_map_pose', lambda *a, **k: np.eye(4))
    assert audit.audit([row])['valid_world_joints_checked'] == 21
    row['world_hands']['hands'][0]['xyz_world_m'] = [[2., 2., 3.]]*21
    assert audit.audit([row])['errors'][0]['reason'] == 'world_transform_mismatch'
