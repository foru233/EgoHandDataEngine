import numpy as np
import pytest
import yaml
from egohand.vio.vs030_geometry import prepare_vs030_vio_geometry

def test_swapped_orb_input_has_positive_disparity_and_consistent_extrinsics():
    path='标定结果/1/data3-camchain-imucam.yaml'
    geom=prepare_vs030_vio_geometry(path)
    c=yaml.safe_load(open(path,encoding='utf8'))
    assert (geom.orb_left_channel,geom.orb_right_channel)==(5,4)
    assert geom.rectifier.P2[0,3]<0
    # 验证 GUI 左点经过 ORB 左和 IMU 变换，与直接从 cam0 到 IMU 一致。
    T_imu_from_gui_left=np.linalg.inv(np.asarray(c['cam0']['T_cam_imu']))
    np.testing.assert_allclose(geom.T_imu_from_orb_left@geom.T_orb_left_from_gui_left,
                               T_imu_from_gui_left,atol=2e-3)
    p=np.array([.02,.01,.6,1.]);p_orb=geom.T_orb_left_from_gui_left@p
    uv0=geom.rectifier.P1@p_orb;uv1=geom.rectifier.P2@p_orb
    assert uv0[0]/uv0[2]>uv1[0]/uv1[2]


def test_active_calibration2_centers_and_orb_settings():
    import cv2
    from pathlib import Path
    from egohand.sources.vendor_placeholder import CALIBRATION_PATH
    from egohand.vio.live_runtime import CALIBRATION, live_settings

    original=yaml.safe_load(Path('标定结果/2/data3-camchain-imucam(1).yaml').read_text(encoding='utf8'))
    active=yaml.safe_load(CALIBRATION_PATH.read_text(encoding='utf8'))
    assert CALIBRATION == CALIBRATION_PATH
    for camera,center in [('cam0',[.03022,.00375,.00258]),('cam1',[-.03485,.00375,.00258])]:
        old=np.asarray(original[camera]['T_cam_imu'])
        new=np.asarray(active[camera]['T_cam_imu'])
        np.testing.assert_allclose(new[:3,:3],old[:3,:3],atol=1e-12)
        np.testing.assert_allclose(np.linalg.inv(new)[:3,3],center,atol=1e-12)
        for key in active[camera]:
            if key != 'T_cam_imu': assert active[camera][key] == original[camera][key]
    geometry=prepare_vs030_vio_geometry(CALIBRATION_PATH)
    timing=yaml.safe_load(Path('configs/vio/live_timing.yaml').read_text(encoding='utf8'))
    fs=cv2.FileStorage(str(live_settings(timing)),cv2.FILE_STORAGE_READ)
    try:
        assert fs.isOpened()
        np.testing.assert_allclose(fs.getNode('IMU.T_b_c1').mat(),geometry.T_imu_from_orb_left,atol=1e-7)
        assert fs.getNode('Camera1.fx').real() == pytest.approx(geometry.rectifier.P1[0,0]/2)
        assert fs.getNode('Stereo.b').real() == pytest.approx(-geometry.rectifier.P2[0,3]/geometry.rectifier.P1[0,0])
    finally: fs.release()


def test_new_vs030_recording_snapshots_active_vio_configuration(tmp_path):
    from egohand.core.types import SourceMetadata
    from egohand.sources.vendor_placeholder import CALIBRATION_PATH
    from egohand.storage.recorder import Recorder
    from egohand.vio.live_runtime import live_settings
    from pathlib import Path

    root=tmp_path/'capture'
    recorder=Recorder(root,SourceMetadata('vendor_vs030',real_stereo=True,contains_imu=True))
    recorder.close()
    timing=yaml.safe_load(Path('configs/vio/live_timing.yaml').read_text(encoding='utf8'))
    assert (root/'kalibr_camchain.yaml').read_bytes() == CALIBRATION_PATH.read_bytes()
    assert (root/'vio_settings.yaml').read_bytes() == live_settings(timing).read_bytes()
