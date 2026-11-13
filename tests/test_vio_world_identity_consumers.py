import numpy as np
from egohand.vio.map_identity import PresetMapIdentity
from egohand.vio.pose_quality import PoseQualityGate


def row(t, revision, x=0):
    matrix = np.eye(4); matrix[0, 3] = x
    return dict(timestamp_ns=t, map_epoch=8, world_frame_revision=revision,
                imu_initialized=True, tracking_state='OK', pose_valid=True,
                matrix=matrix.tolist())


def test_world_revision_restarts_quality_without_rewriting_raw_pose():
    gate = PoseQualityGate(stable_seconds=1)
    gate.check(row(1_000_000_000, 0))
    assert gate.check(row(2_000_000_000, 0))['pose_valid']
    result = gate.check(row(2_100_000_000, 1, 36))
    assert not result['pose_valid'] and not result['jump_detected']
    assert result['quality_status'] == '等待连续稳定跟踪'
    assert result['raw_matrix'][0][3] == 36
    assert result['display_segment'] == 1


def test_preset_map_rejects_world_reanchor_with_unchanged_epoch():
    identity = PresetMapIdentity(0)
    assert identity.rejection(row(1, 0)) is None
    assert '世界坐标版本' in identity.rejection(row(2, 1, 36))
