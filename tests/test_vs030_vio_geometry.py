import numpy as np
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
