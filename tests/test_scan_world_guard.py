import importlib.util
from pathlib import Path

spec=importlib.util.spec_from_file_location('scan_world_guard',
    Path(__file__).resolve().parents[1]/'map_creat/scan_world_guard.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def pose():return {'map_epoch':0,'world_frame_revision':0}


def test_worker_restart_cannot_reuse_colliding_map_identity():
    guard=module.ScanWorldGuard()
    assert guard.accept(pose()) is None
    error=guard.worker_status({'type':'vio_reset'})
    assert error and guard.accept(pose())==error


def test_restart_before_any_geometry_can_continue_preparation():
    guard=module.ScanWorldGuard()
    assert guard.worker_status({'type':'vio_reset'}) is None
    assert guard.accept(pose()) is None


def test_temporary_loss_does_not_destroy_same_world():
    guard=module.ScanWorldGuard()
    assert guard.accept(pose()) is None
    assert guard.worker_status({'type':'status','message':'视觉跟踪未稳定'}) is None
    assert guard.accept(pose()) is None


def test_imu_initialization_alone_cannot_start_scanning():
    assert '等待 IMU 初始化' in module.scan_preparation_message({'pose_valid':True})
    assert '惯性优化' in module.scan_preparation_message({'pose_valid':True,'imu_initialized':True})
    ready={'pose_valid':True,'imu_initialized':True,'inertial_scale_finalized':True}
    assert module.scan_preparation_message(ready) is None
    assert module.scan_preparation_message({**ready,'pose_valid':False})


def test_missing_epoch_with_present_revision_does_not_pin_a_world():
    guard=module.ScanWorldGuard()
    assert guard.accept({'world_frame_revision':0})
    assert guard.identity is None
    assert guard.accept({**pose(),'snapshot_consistent':False})
    assert guard.identity is None
