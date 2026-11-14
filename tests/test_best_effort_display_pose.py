import numpy as np

from egohand.vio.display_pose import BestEffortPoseSelector


def _pose(stamp, *, epoch=1, valid=False, matrix=None, prediction=None, contract=None, raw=None):
    result = {'type':'pose','timestamp_ns':stamp,'frame_id':stamp,'map_epoch':epoch,
              'pose_valid':valid,'matrix':matrix,'tracking_state':'OK'}
    if prediction is not None:
        result['recovery_preaccept']={'loss_prediction':prediction}
    if contract is not None:
        result['display_pose_contract']=contract
    if raw is not None:
        result['display_visual_candidate']={'timestamp_ns':stamp,'frame_id':stamp,
                                            'reference_id':epoch,'matrix':raw}
    return result


def test_imu_prediction_converts_body_pose_with_full_extrinsic_rotation_and_offset():
    selector = BestEffortPoseSelector()
    trusted = np.eye(4).tolist()
    selector.update(_pose(1, valid=True, matrix=trusted))
    extrinsic = np.eye(4)
    extrinsic[:3,:3] = [[0,-1,0],[1,0,0],[0,0,1]]
    extrinsic[:3,3] = [.1,.2,.3]
    body = np.eye(4); body[:3,3] = [1,2,3]
    pose = selector.update(_pose(2, prediction={'status':'predicted','duration_s':.5,
        'T_world_from_body':body.tolist()}, contract={'T_body_from_camera':extrinsic.tolist()}))
    assert pose.source == 'imu_prediction'
    np.testing.assert_allclose(pose.matrix, body @ extrinsic)


def test_prediction_track_is_display_only_and_invalid_prediction_freezes():
    selector = BestEffortPoseSelector()
    base = np.eye(4); base[0,3] = 4
    assert selector.update(_pose(1, valid=True, matrix=base.tolist())).trusted
    pose = selector.update(_pose(2, prediction={'status':'predicted','duration_s':3,
        'T_world_from_body':base.tolist()}, contract={'T_body_from_camera':np.eye(4).tolist()}))
    assert pose.source == 'frozen' and not pose.trusted
    np.testing.assert_allclose(pose.matrix, base)


def test_visual_candidate_is_separate_and_map_change_does_not_inherit_pose():
    selector = BestEffortPoseSelector()
    candidate = np.eye(4); candidate[1,3] = 2
    pose = selector.update(_pose(10, raw=candidate.tolist()))
    assert pose.source == 'visual_candidate' and not pose.trusted
    changed = selector.update(_pose(11, epoch=2))
    assert changed.source == 'origin'
    np.testing.assert_allclose(changed.matrix, np.eye(4))


def test_invalid_candidate_and_out_of_order_prediction_are_frozen():
    selector = BestEffortPoseSelector()
    base = np.eye(4); base[2,3] = 1
    selector.update(_pose(20, valid=True, matrix=base.tolist()))
    bad = selector.update(_pose(19, raw=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,2]]))
    assert bad.source == 'frozen'


def test_loss_without_local_segment_keeps_same_map_prediction_and_json_identity():
    selector = BestEffortPoseSelector()
    base = np.eye(4)
    trusted = _pose(1, valid=True, matrix=base.tolist())
    trusted['world_frame_revision'] = 4
    trusted['local_motion'] = {'status':'estimated','segment_id':'session:0',
                               'T_segment_from_map':base.tolist()}
    selector.update(trusted)
    body = np.eye(4); body[1,3] = .4
    loss = _pose(2, prediction={'status':'predicted','duration_s':.2,
        'T_world_from_body':body.tolist()}, contract={'T_body_from_camera':base.tolist()})
    loss['world_frame_revision'] = 4
    loss['local_motion'] = {'status':'unavailable','segment_id':None}
    pose = selector.update(loss)
    assert pose.source == 'imu_prediction' and pose.coordinate_frame == 'segment'
    np.testing.assert_allclose(pose.matrix[:3,3], [0,.4,0])
    candidate = _pose(3, raw=base.tolist()); candidate['world_frame_revision'] = 4
    candidate['display_visual_candidate']['reference_id'] = [1,4]
    selector.reset()
    assert selector.update(candidate).source == 'visual_candidate'


def test_frozen_age_accumulates_from_last_real_estimate():
    selector = BestEffortPoseSelector()
    base = np.eye(4).tolist()
    selector.update(_pose(1_000_000_000, valid=True, matrix=base))
    first = selector.update(_pose(1_100_000_000))
    second = selector.update(_pose(1_600_000_000))
    assert first.source == second.source == 'frozen'
    assert first.age_ms == 100.0
    assert second.age_ms == 600.0

