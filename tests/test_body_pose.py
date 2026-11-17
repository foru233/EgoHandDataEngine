import types
import numpy as np

from egohand.body.body_pose_gate import evaluate_body_gate
from egohand.body.body_pose_estimator import BodyPoseEstimator
from egohand.body.two_bone_ik import solve_two_bone_ik


def _hand(side, offset):
    points = np.zeros((21, 3), dtype=float)
    points[:, 0] = offset[0]
    points[:, 1] = offset[1]
    points[:, 2] = offset[2]
    valid = np.zeros(21, dtype=bool)
    valid[[0, 5, 9, 13, 17]] = True
    return {"handedness": side, "track_id": 1, "xyz_world_m": points, "valid": valid}


def _input(*, left=True, right=True, pose=None, timestamp=100):
    hands = []
    if left:
        hands.append(_hand("Left", (-.35, .15, 1.0)))
    if right:
        hands.append(_hand("Right", (.35, .15, 1.0)))
    pose = pose or {"pose_valid": True, "matrix": np.eye(4).tolist(), "timestamp_ns": timestamp}
    result = types.SimpleNamespace(
        frame_id=3, timestamp_ns=timestamp, session_id="s", world_hands={
            "status": "available", "frame_id": 3, "timestamp_ns": timestamp,
            "session_id": "s", "hands": hands,
            "world_reference": {"map_epoch": None, "world_frame_revision": None,
                                "reference_generation": None},
        })
    return result, pose


def test_three_inputs_strict_valid():
    result, pose = _input()
    gate = evaluate_body_gate(result, pose)
    assert gate.status == "strict_valid"
    estimate, gate = BodyPoseEstimator().update(result, pose)
    assert estimate is not None and estimate.strict_valid
    assert {"LeftWrist", "RightWrist"} <= estimate.measured_joints


def test_default_neutral_model_height_includes_head_above_center():
    result, pose = _input(left=False, right=False)
    estimate, _ = BodyPoseEstimator().update(result, pose, strict_gate=False)
    head_to_floor = estimate.diagnostics['ground_y_m'] - estimate.joints['Head'][1]
    # The displayed head wireframe extends roughly 10.5 cm above its center.
    assert abs(head_to_floor + .105 - 1.75) < .025


def test_missing_hand_is_rejected_strict_but_preview_is_explicit():
    result, pose = _input(right=False)
    gate = evaluate_body_gate(result, pose, strict=True)
    assert not gate.strict_valid and gate.reason == "missing_right"
    estimate, preview = BodyPoseEstimator().update(result, pose, strict_gate=False)
    assert estimate is not None and preview.status == "estimated_preview"
    assert "RightWrist" not in estimate.measured_joints


def test_bad_head_prediction_never_becomes_strict():
    result, pose = _input(pose={"pose_valid": True, "matrix": np.eye(4).tolist(),
                               "timestamp_ns": 100, "pose_source": "imu_prediction"})
    gate = evaluate_body_gate(result, pose)
    assert gate.status == "waiting_head"
    assert gate.reason == "head_pose_not_visual_committed"


def test_timestamp_reference_and_stale_inputs_reject():
    result, pose = _input(timestamp=100)
    assert evaluate_body_gate(result, {**pose, "timestamp_ns": 101}).status == "waiting_head"
    assert evaluate_body_gate(result, pose, now_ns=400_000_101).status == "stale"
    result.world_hands["world_reference"]["map_epoch"] = 3
    pose["map_epoch"] = 4
    assert evaluate_body_gate(result, pose).reason == "reference_mismatch"


def test_two_bone_reachability_and_pole_stability():
    elbow, target, reachable, diagnostics = solve_two_bone_ik(
        [0, 0, 0], [0, 0, .4], .3, .26, [0, 1, 0])
    assert reachable
    assert np.isclose(np.linalg.norm(elbow), .3, atol=1e-6)
    assert np.isclose(np.linalg.norm(target-elbow), .26, atol=1e-6)
    elbow2, target2, reachable2, diagnostics2 = solve_two_bone_ik(
        [0, 0, 0], [0, 0, 2], .3, .26, [0, 1, 0])
    assert not reachable2 and diagnostics2["clamped"]
    assert np.isclose(np.linalg.norm(target2), .3+.26-1e-4, atol=1e-6)


def test_body_wrist_stays_on_recognized_world_hand_when_unreachable():
    result, pose = _input()
    # Deliberately place both recognized wrists beyond the nominal arm reach.
    for hand in result.world_hands["hands"]:
        hand["xyz_world_m"][0] = [2.0, 0.0, 2.0]
    estimate, gate = BodyPoseEstimator().update(result, pose)
    assert estimate is not None and estimate.strict_valid
    np.testing.assert_allclose(estimate.joints["LeftWrist"], [2.0, 0.0, 2.0])
    np.testing.assert_allclose(estimate.joints["RightWrist"], [2.0, 0.0, 2.0])
    assert estimate.diagnostics["ik"]["Left"]["clamped"]


def test_body_torso_stays_vertical_in_world_y():
    result, pose = _input()
    estimate, _ = BodyPoseEstimator().update(result, pose)
    assert estimate.joints["Pelvis"][1] > estimate.joints["Chest"][1] > estimate.joints["Neck"][1]
    assert np.isclose(estimate.joints["Pelvis"][0], estimate.joints["Chest"][0], atol=.25)


def test_body_wrist_matches_gui_camera_hand_transform():
    from egohand.core.types import Hand3D
    result, pose = _input()
    left = Hand3D("Left", 1.0, 1)
    left.joints[0].valid = True
    left.joints[0].xyz_m = np.array([0.2, 0.3, 1.0])
    right = Hand3D("Right", 1.0, 2)
    right.joints[0].valid = True
    right.joints[0].xyz_m = np.array([-0.2, 0.3, 1.0])
    result.hands = [left, right]
    # Deliberately disagreeing serialized world hand values must not win.
    estimate, _ = BodyPoseEstimator().update(result, pose)
    np.testing.assert_allclose(estimate.joints["LeftWrist"], [0.2, 0.3, 1.0])
    np.testing.assert_allclose(estimate.joints["RightWrist"], [-0.2, 0.3, 1.0])


def test_body_uses_stable_track_when_gate_side_and_world_point_disagree():
    from egohand.core.types import Hand3D
    estimator=BodyPoseEstimator()
    result,pose=_input(left=False,right=True,timestamp=100)
    hand=Hand3D('Right',1.,3);hand.joints[0].valid=True
    hand.joints[0].xyz_m=np.array([.1,.1,.5]);result.hands=[hand]
    result.world_hands['hands'][0]['track_id']=3
    estimator.update(result,pose,strict_gate=False)
    result.timestamp_ns=101;result.world_hands['timestamp_ns']=101
    pose={**pose,'timestamp_ns':101}
    hand.handedness='Left';hand.joints[0].xyz_m=np.array([.101,.1,.5])
    result.world_hands['hands'][0]['handedness']='Left'
    # Saved world point differs by far more than nearest-position threshold.
    result.world_hands['hands'][0]['xyz_world_m'][:]=[1.,1.,1.]
    estimate,_=estimator.update(result,pose,strict_gate=False)
    assert 'RightWrist' in estimate.measured_joints
    assert 'LeftWrist' not in estimate.measured_joints
    np.testing.assert_allclose(estimate.joints['RightWrist'],hand.joints[0].xyz_m)


def test_head_translation_and_rotation_drive_head_and_neck():
    result, pose = _input()
    first, _ = BodyPoseEstimator().update(result, pose)
    matrix = np.eye(4)
    matrix[:3,:3] = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]
    matrix[:3,3] = [1., 2., 3.]
    second, _ = BodyPoseEstimator().update(result, {**pose, 'matrix': matrix.tolist()})
    expected_head = matrix @ np.array([0., 0., 0., 1.])
    np.testing.assert_allclose(second.joints['Head'], expected_head[:3])
    np.testing.assert_allclose(second.joints['Neck'] - second.joints['Head'], matrix[:3,:3] @ np.array([0., .12, -.08]))
    np.testing.assert_allclose(second.diagnostics['head_rotation_world'], matrix[:3,:3])
    assert not np.allclose(first.joints['Head'], second.joints['Head'])


def test_missing_hand_holds_last_wrist_temporarily_without_claiming_measurement():
    result, pose = _input()
    estimator = BodyPoseEstimator()
    first, _ = estimator.update(result, pose, strict_gate=False)
    result.timestamp_ns += 100_000_000
    result.world_hands['timestamp_ns'] = result.timestamp_ns
    result.world_hands['hands'] = [h for h in result.world_hands['hands'] if h['handedness']=='Left']
    second, _ = estimator.update(result, {**pose, 'timestamp_ns': result.timestamp_ns}, strict_gate=False)
    np.testing.assert_allclose(second.joints['RightWrist'], first.joints['RightWrist'])
    assert 'Right' in second.diagnostics['held_hands']
    assert 'RightWrist' not in second.measured_joints


def test_missing_hand_remains_held_after_half_second():
    result,pose=_input(); estimator=BodyPoseEstimator()
    first,_=estimator.update(result,pose,strict_gate=False)
    result.timestamp_ns+=2_000_000_000
    result.world_hands['timestamp_ns']=result.timestamp_ns
    result.world_hands['hands']=[h for h in result.world_hands['hands'] if h['handedness']=='Left']
    second,_=estimator.update(result,{**pose,'timestamp_ns':result.timestamp_ns},strict_gate=False)
    np.testing.assert_allclose(second.joints['RightWrist'],first.joints['RightWrist'])
    assert 'Right' in second.diagnostics['held_hands']
    assert 'RightWrist' not in second.measured_joints


def test_ik_enforces_flexion_limit_for_near_target():
    elbow, target, _, info = solve_two_bone_ik([0,0,0], [0,0,0.01], .3, .26, [0,1,0], max_flexion_deg=150)
    upper = -elbow
    lower = target-elbow
    angle = np.degrees(np.arccos(np.clip(np.dot(upper,lower)/(np.linalg.norm(upper)*np.linalg.norm(lower)), -1,1)))
    assert angle >= 30 - 1e-6 and info['clamped']


def test_torso_does_not_twist_with_handheld_head_rotation_and_resets():
    result, pose = _input()
    estimator = BodyPoseEstimator()
    first, _ = estimator.update(result, pose)
    angle = np.deg2rad(35)
    matrix = np.eye(4)
    matrix[:3,:3] = [[np.cos(angle), -np.sin(angle), 0],
                      [np.sin(angle), np.cos(angle), 0], [0,0,1]]
    result.timestamp_ns += 33_000_000
    result.world_hands['timestamp_ns'] = result.timestamp_ns
    second, _ = estimator.update(result, {**pose, 'timestamp_ns':result.timestamp_ns, 'matrix':matrix.tolist()})
    torso = second.joints['Pelvis'] - second.joints['Chest']
    assert abs(torso[0]) < 1e-9
    assert abs(torso[2]) < 1e-9
    assert np.isfinite(list(second.joints.values())).all()
    estimator.reset()
    assert estimator._last_pose is None and estimator._torso_rotation is None


def test_neck_bends_with_head_and_is_bounded():
    result, pose = _input()
    angle = np.deg2rad(40)
    matrix = np.eye(4)
    matrix[:3,:3] = [[np.cos(angle), -np.sin(angle), 0],
                      [np.sin(angle), np.cos(angle), 0], [0,0,1]]
    estimate, _ = BodyPoseEstimator().update(result, {**pose, 'matrix':matrix.tolist()})
    direction = estimate.joints['CervicalBase'] - estimate.joints['Neck']
    assert abs(direction[0]) > .005
    assert np.isclose(np.linalg.norm(direction), .08)
    assert estimate.diagnostics['neck_source'] == 'bounded_head_torso_blend_prior'


def test_shoulders_are_centered_on_cervical_base():
    result, pose = _input()
    estimate, _ = BodyPoseEstimator().update(result, pose)
    joints = estimate.joints
    np.testing.assert_allclose((joints['LeftShoulder'] + joints['RightShoulder']) / 2,
                               joints['CervicalBase'])
    assert ('CervicalBase', 'LeftShoulder') in estimate.bones
    assert ('CervicalBase', 'RightShoulder') in estimate.bones
    assert ('Chest', 'LeftShoulder') not in estimate.bones


def test_initial_floor_is_based_on_hips_and_adjustment_moves_ankles():
    result, pose = _input()
    estimator = BodyPoseEstimator()
    first, _ = estimator.update(result, pose)
    hip = first.joints['LeftHip']
    ankle = first.joints['LeftAnkle']
    assert abs(np.linalg.norm(ankle-hip)-estimator.model.preset['leg_length_m']) < .01
    assert np.linalg.norm(first.joints['LeftKnee']-(hip+ankle)/2) < .04
    estimator.ground_offset_m = -.10
    second, _ = estimator.update(result, pose)
    assert np.isclose(second.joints['LeftAnkle'][1], ankle[1]-.10)
    assert np.linalg.norm(second.joints['LeftKnee']-(second.joints['LeftHip']+second.joints['LeftAnkle'])/2) > .04


def test_max_ground_raise_uses_leg_length_and_knee_limit():
    estimator = BodyPoseEstimator()
    assert np.isclose(estimator.max_ground_raise_m(), .895)


def test_ground_adjustment_sweeps_to_seated_without_breaking_leg_lengths():
    result, pose = _input()
    estimator = BodyPoseEstimator()
    positions=[]
    for cm in range(1, 91):
        estimator.ground_offset_m = -cm/100
        body, _ = estimator.update(result, pose)
        for side in ('Left','Right'):
            hip,knee,ankle = (body.joints[side+part] for part in ('Hip','Knee','Ankle'))
            assert np.isclose(np.linalg.norm(hip-knee), .45, atol=1e-5)
            assert np.isclose(np.linalg.norm(ankle-knee), .45, atol=1e-5)
            assert np.isclose(ankle[1], estimator._ground_y-.025)
            assert body.diagnostics['ik'][side+'Leg']['ground_reachable']
        positions.append(body.joints['LeftKnee'].copy())
    assert max(np.linalg.norm(b-a) for a,b in zip(positions,positions[1:])) < .12


def test_forward_translation_cannot_flip_knees_backward():
    result, pose = _input()
    estimator = BodyPoseEstimator()
    estimator.ground_offset_m = -.20
    estimator.update(result, pose)
    for frame, z in enumerate((.30, .60, -.20, .50)):
        matrix = np.eye(4);matrix[2,3] = z
        result.timestamp_ns = (frame+1)*33_333_333
        result.world_hands['timestamp_ns'] = result.timestamp_ns
        body, _ = estimator.update(result, {**pose,'timestamp_ns':result.timestamp_ns,'matrix':matrix.tolist()})
        forward = np.asarray(body.diagnostics['pelvis_forward_world'])
        for side in ('Left','Right'):
            hip,knee,ankle = (body.joints[side+part] for part in ('Hip','Knee','Ankle'))
            axis = ankle-hip
            bend = knee-hip-axis*np.dot(knee-hip,axis)/np.dot(axis,axis)
            assert np.dot(bend,forward) > .20


def test_pelvis_can_tilt_without_opposite_knees_or_invented_staggered_feet():
    from scipy.spatial.transform import Rotation
    result, pose = _input()
    estimator = BodyPoseEstimator()
    estimator.update(result,pose)
    tilted = False
    for frame in range(180):
        yaw = -150 + frame*2
        roll = 40*np.sin(frame/12)
        pitch = 25*np.sin(frame/17)
        matrix = np.eye(4)
        matrix[:3,:3] = Rotation.from_euler('yxz',[yaw,pitch,roll],degrees=True).as_matrix()
        matrix[:3,3] = [.1*np.sin(frame/8),0.,frame*.015]
        # Includes deeply bent and near-horizontal hip-to-ankle configurations.
        estimator.ground_offset_m = -.85 * (frame % 30)/29
        result.timestamp_ns = (frame+1)*33_333_333
        result.world_hands['timestamp_ns'] = result.timestamp_ns
        body,_ = estimator.update(result,{**pose,'timestamp_ns':result.timestamp_ns,'matrix':matrix.tolist()})
        joints = body.joints
        forward = np.asarray(body.diagnostics['pelvis_forward_world'])
        tilted |= abs(joints['LeftHip'][1]-joints['RightHip'][1]) > .01
        assert abs(np.dot(joints['LeftAnkle']-joints['RightAnkle'],forward)) < 1e-9
        if body.diagnostics['ground_contact']:
            assert np.isclose(joints['LeftAnkle'][1],joints['RightAnkle'][1])
        for side in ('Left','Right'):
            hip,knee,ankle = (joints[side+part] for part in ('Hip','Knee','Ankle'))
            assert np.isclose(np.linalg.norm(hip-knee),.45,atol=1e-6)
            assert np.isclose(np.linalg.norm(ankle-knee),.45,atol=1e-6)
            assert knee[1] <= ankle[1]+1e-8
            if body.diagnostics['ik'][side+'Leg']['ground_reachable']:
                assert abs(body.diagnostics['ik'][side+'Leg']['ground_error_m']) < 1e-6
                axis=ankle-hip
                bend=knee-hip-axis*np.dot(knee-hip,axis)/np.dot(axis,axis)
                assert np.dot(bend,forward) >= -1e-8
            assert side+'Knee' not in body.measured_joints
    assert not tilted  # Handheld camera yaw/roll must not twist the pelvis.


def test_impossible_floor_adjusts_inferred_pelvis_and_keeps_ankle_clearance():
    result, pose = _input()
    estimator=BodyPoseEstimator()
    estimator.update(result,pose)
    matrix=np.eye(4);matrix[1,3]=-1.0
    body,_=estimator.update(result,{**pose,'matrix':matrix.tolist()})
    assert body.diagnostics['ground_contact']
    assert body.diagnostics['torso_length_prior_relaxed']
    assert abs(body.diagnostics['pelvis_ground_adjustment_m'])>.9
    expected_head=matrix@np.array([0.,0.,0.,1.])
    np.testing.assert_allclose(body.joints['Head'],expected_head[:3])
    for side in ('Left','Right'):
        hip,knee,ankle=(body.joints[side+part] for part in ('Hip','Knee','Ankle'))
        assert np.isclose(np.linalg.norm(hip-knee),.45,atol=1e-6)
        assert np.isclose(np.linalg.norm(ankle-knee),.45,atol=1e-6)
        assert body.diagnostics['ik'][side+'Leg']['ground_reachable']
        assert abs(body.diagnostics['ik'][side+'Leg']['ground_error_m'])<1e-8
        assert np.isclose(body.diagnostics['ground_y_m']-ankle[1],.025)


def test_body_overlay_keeps_target_floor_and_ankle_clearance(qtbot):
    from PySide6.QtGui import QVector3D
    from egohand.gui.vio_panel import TrajectoryView
    from delivery_views import BodyPoseOverlay
    result,pose=_input()
    estimator=BodyPoseEstimator();estimator.update(result,pose)
    matrix=np.eye(4);matrix[1,3]=-1.0
    body,_=estimator.update(result,{**pose,'matrix':matrix.tolist()})
    view=TrajectoryView();qtbot.addWidget(view)
    overlay=BodyPoseOverlay(view);overlay.set_visible(True);overlay.update(body)
    floor_z=view.grid.transform().map(QVector3D()).z()
    assert np.isclose(floor_z,-body.diagnostics['ground_y_m'])
    assert body.diagnostics['ground_contact']
    assert abs(floor_z-(-body.joints['LeftAnkle'][1]-.025))<1e-6


def test_skeleton_is_batched_and_visibility_switches_are_independent(qtbot):
    from egohand.gui.vio_panel import TrajectoryView
    from delivery_views import BodyPoseOverlay
    result, pose = _input()
    body, _ = BodyPoseEstimator().update(result, pose)
    view = TrajectoryView(); qtbot.addWidget(view)
    overlay = BodyPoseOverlay(view)
    overlay.set_mesh_visible(False); overlay.set_visible(True); overlay.update(body)
    # Head guide, batched bones, and up to three point-color groups.
    skeleton_items = [x for x in overlay.items if not getattr(x, '_body_mesh', False)]
    assert 1 <= len(skeleton_items) <= 5
    overlay.set_skeleton_visible(False)
    overlay.set_visible(False); overlay.set_visible(True)
    assert not any(item.visible() for item in skeleton_items)


def test_body_mesh_invalid_frame_does_not_reappear_after_toggle(qtbot, tmp_path):
    from dataclasses import replace
    from egohand.gui.vio_panel import TrajectoryView
    from delivery_views import BodyPoseOverlay
    result, pose = _input()
    body, _ = BodyPoseEstimator().update(result, pose)
    vertices = np.array([[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]],
                         [[0.,0.,0.],[0.,0.,0.],[0.,0.,0.]]], dtype=np.float32)
    path=tmp_path/'body_vertices.npy'; np.save(path,vertices)
    np.save(tmp_path/'body_vertices_valid.npy',np.array([True,False]))
    mesh={'vertices':vertices[0].tolist(),'faces':[[0,1,2]]}
    view=TrajectoryView(); qtbot.addWidget(view)
    overlay=BodyPoseOverlay(view); overlay.set_visible(True)
    def frame(index):
        return replace(body,diagnostics={**body.diagnostics,'body_mesh':mesh,
            'body_mesh_index':index,'body_mesh_vertices_path':str(path)})
    overlay.update(frame(0)); assert overlay._mesh_frame_valid
    assert overlay._mesh_item.visible()
    overlay.set_mesh_visible(False);overlay.set_visible(False);overlay.set_visible(True)
    assert not overlay._mesh_item.visible()
    overlay.set_mesh_visible(True)
    assert overlay._mesh_item.visible()
    overlay.update(frame(1)); assert not overlay._mesh_frame_valid
    overlay.set_visible(True);overlay.set_mesh_visible(True)
    assert not overlay._mesh_item.visible()


def test_moving_floor_keeps_both_ankles_at_set_distance_with_tilt_and_head_motion():
    from scipy.spatial.transform import Rotation
    result,pose=_input()
    estimator=BodyPoseEstimator();estimator.update(result,pose)
    tilted=False
    for frame in range(181):
        matrix=np.eye(4)
        matrix[:3,:3]=Rotation.from_euler('zy',[35*np.sin(frame/9),frame*3],degrees=True).as_matrix()
        matrix[:3,3]=[0.,1.1*np.sin(frame/17),.02*frame]
        estimator.ground_offset_m=-estimator.max_ground_raise_m()*(1-np.cos(frame/12))/2
        result.timestamp_ns=(frame+1)*33_333_333
        result.world_hands['timestamp_ns']=result.timestamp_ns
        body,_=estimator.update(result,{**pose,'timestamp_ns':result.timestamp_ns,'matrix':matrix.tolist()})
        joints=body.joints
        tilted |= abs(joints['LeftHip'][1]-joints['RightHip'][1])>.01
        for side in ('Left','Right'):
            hip,knee,ankle=(joints[side+p] for p in ('Hip','Knee','Ankle'))
            assert np.isclose(body.diagnostics['ground_y_m']-ankle[1],.025,atol=1e-9)
            assert np.isclose(np.linalg.norm(hip-knee),.45,atol=1e-6)
            assert np.isclose(np.linalg.norm(ankle-knee),.45,atol=1e-6)
        assert body.diagnostics['ground_contact']
    assert not tilted
