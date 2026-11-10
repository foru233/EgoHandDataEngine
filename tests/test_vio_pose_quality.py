import numpy as np
from egohand.vio.pose_quality import PoseQualityGate


def pose(t, x=0., *, initialized=True, state='OK', epoch=0):
    matrix=np.eye(4);matrix[0,3]=x
    return {'timestamp_ns':t,'tracking_state':state,'imu_initialized':initialized,
            'pose_valid':True,'matrix':matrix.tolist(),'map_epoch':epoch}


def test_static_stabilizes_jump_is_rejected_and_recovery_is_separate():
    gate=PoseQualityGate(stable_seconds=1.0)
    assert gate.check(pose(1_000_000_000, initialized=False))['quality_status']=='等待 IMU 初始化'
    assert not gate.check(pose(2_000_000_000))['pose_valid']
    assert gate.check(pose(3_000_000_000,.001))['pose_valid']
    jump=gate.check(pose(3_100_000_000,1.0))
    assert not jump['pose_valid'] and jump['raw_pose_valid'] and jump['raw_matrix'] is not None
    assert '突变' in jump['quality_status'] and jump['display_segment']==1
    assert not gate.check(pose(3_200_000_000,1.0))['pose_valid']
    stable=gate.check(pose(4_200_000_000,1.001))
    assert stable['pose_valid'] and stable['display_segment']==1
    lost=gate.check(pose(4_300_000_000,1.001,state='LOST'))
    assert not lost['pose_valid'] and lost['display_segment']==2


def test_map_change_and_time_reversal_invalidate_history():
    gate=PoseQualityGate(stable_seconds=0)
    assert gate.check(pose(1_000_000_000))['pose_valid']
    changed=gate.check(pose(2_000_000_000,10.,epoch=1))
    assert changed['pose_valid'] and changed['display_segment']==1
    reverse=gate.check(pose(1_500_000_000,10.,epoch=1))
    assert not reverse['pose_valid'] and '时间' in reverse['quality_status']
