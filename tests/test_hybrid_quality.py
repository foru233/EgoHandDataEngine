import numpy as np
from egohand.vio.hybrid_quality import HybridPoseQualityGate
from egohand.vio.modes import apply_mode_environment


def pose(t, initialized=False, x=0, **fields):
    matrix=np.eye(4);matrix[0,3]=x
    return dict(timestamp_ns=int(t*1e9),matrix=matrix.tolist(),pose_valid=True,
                tracking_state='OK',imu_initialized=initialized,map_epoch=1,**fields)


def test_hybrid_fallback_then_vio_requalifies_and_breaks_segment():
    gate=HybridPoseQualityGate(stable_seconds=1)
    assert not gate.check(pose(1))['pose_valid']
    visual=gate.check(pose(2))
    assert visual['pose_valid'] and visual['pose_source']=='visual_fallback'
    waiting=gate.check(pose(3,True))
    assert waiting['pose_source']=='visual_fallback' and not waiting['inertial_display_ready']
    assert 'BA2' in waiting['fallback_reason']
    inertial=gate.check(pose(4,True,ba2_complete=True,inertial_scale_finalized=True))
    assert inertial['pose_valid'] and inertial['pose_source']=='vio' and inertial['inertial_display_ready']
    assert inertial['display_segment']>visual['display_segment']
    fallback=gate.check(pose(5))
    assert fallback['pose_source']=='visual_fallback' and fallback['pose_valid']
    assert fallback['display_segment']>inertial['display_segment']


def test_fallback_never_bypasses_jump_lost_or_quarantine():
    gate=HybridPoseQualityGate(stable_seconds=0)
    assert gate.check(pose(1))['pose_valid']
    assert not gate.check(pose(1.01,x=10))['pose_valid']
    lost=pose(2);lost['tracking_state']='LOST'
    assert not gate.check(lost)['pose_valid']
    assert not gate.check(pose(3,recovery_preaccept={'await_visual_requalification':True}))['pose_valid']


def test_hybrid_uses_one_inertial_worker_environment():
    assert apply_mode_environment({},'hybrid')['EGOHAND_VIO_STEREO_ONLY']=='0'


def test_no_pose_is_never_replaced_by_a_second_tracker():
    gate=HybridPoseQualityGate(stable_seconds=0)
    inertial=pose(1)
    inertial.update(pose_valid=False,matrix=None,tracking_state='NOT_INITIALIZED')
    out=gate.check(inertial)
    assert not out['pose_valid'] and out['matrix'] is None
    assert out['fallback_kind']=='single_vio_visual_bootstrap'


def test_bootstrap_uses_existing_native_option_without_changing_original(tmp_path):
    from egohand.vio.hybrid_settings import visual_bootstrap_settings
    source=tmp_path/'settings.yaml'
    original='%YAML:1.0\nCamera.fx: 400.\nIMU.fastInit: 0\n'
    source.write_text(original)
    prepared=visual_bootstrap_settings(source)
    assert source.read_text()==original
    assert prepared.read_text().count('IMU.fastInit: 1')==1
    assert 'Camera.fx: 400.' in prepared.read_text()


def test_bootstrap_effective_settings_keep_phase_bound_bytes_on_reentry(tmp_path):
    from egohand.vio.hybrid_settings import visual_bootstrap_settings
    import hashlib
    source=tmp_path/'settings.yaml'
    expected=b'%YAML:1.0\nCamera.fx: 400.\nIMU.fastInit: 1\n'
    source.write_bytes(expected.replace(b'\n',b'\r\n'))
    original=source.read_bytes()
    first=visual_bootstrap_settings(source)
    assert source.read_bytes()==original
    assert first.read_bytes()==expected
    verified_hash=hashlib.sha256(first.read_bytes()).hexdigest()
    second=visual_bootstrap_settings(first)
    assert second.read_bytes()==expected
    assert hashlib.sha256(second.read_bytes()).hexdigest()==verified_hash
    assert visual_bootstrap_settings(second).read_bytes()==expected


def test_legacy_hybrid_recording_prefers_verified_phase_without_editing_snapshot():
    from dataclasses import replace
    from egohand.vio.config_snapshot import resolve_live_config
    from egohand.vio.replay_runtime import _verified_phase_policy_for_hybrid_recording

    original = resolve_live_config()
    original = replace(original, timing={**original.timing,
        'imu_alignment': 'lattice', 'lattice_reference': 'packet',
        'lattice_phase_mode': 'manual', 'lattice_phase_ms': 97.0})
    prepared, should_verify = _verified_phase_policy_for_hybrid_recording(
        original, mode='hybrid')
    assert should_verify
    assert original.timing['lattice_phase_mode'] == 'manual'
    assert original.timing['lattice_phase_ms'] == 97.0
    assert prepared.timing['lattice_phase_mode'] == 'session_auto'
    assert prepared.timing['lattice_phase_ms'] == 97.0  # Only a provisional starting value.
    for options in (
        {'mode': 'stereo'},
        {'mode': 'stereo_inertial'},
        {'mode': 'hybrid', 'explicit_timing': True},
        {'mode': 'hybrid', 'explicit_config': True},
    ):
        unchanged, promoted = _verified_phase_policy_for_hybrid_recording(original, **options)
        assert not promoted and unchanged is original
    verified = replace(original, timing={**original.timing, 'session_phase_verified': True})
    assert not _verified_phase_policy_for_hybrid_recording(verified, mode='hybrid')[1]
    wrong_reference = replace(original, timing={**original.timing, 'lattice_reference': 'count'})
    assert not _verified_phase_policy_for_hybrid_recording(wrong_reference, mode='hybrid')[1]


def test_hybrid_never_promotes_inertial_before_ba2_and_final_scale():
    gate=HybridPoseQualityGate(stable_seconds=0)
    early=gate.check(pose(1,True,ba2_complete=False,inertial_scale_finalized=False))
    assert early['pose_valid']
    assert early['pose_source']=='visual_fallback'
    assert not early['inertial_display_ready']
    incomplete=gate.check(pose(2,True,ba2_complete=True,inertial_scale_finalized=False))
    assert incomplete['pose_valid'] and incomplete['pose_source']=='visual_fallback'
    assert incomplete['display_segment']==early['display_segment']
    verified=gate.check(pose(3,True,ba2_complete=True,inertial_scale_finalized=True))
    assert verified['pose_valid'] and verified['pose_source']=='vio'
    assert verified['display_segment']>early['display_segment']
    no_imu=gate.check(pose(4,False,ba2_complete=True,inertial_scale_finalized=True))
    assert no_imu['pose_valid'] and no_imu['pose_source']=='visual_fallback'
