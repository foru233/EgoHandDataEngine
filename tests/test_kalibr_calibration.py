import numpy as np
import pytest
import yaml
from egohand.calibration.loader import load_calibration
from egohand.calibration.rectifier import StereoRectifier
from egohand.calibration.validator import CalibrationError

def test_kalibr_stereo_mapping_and_rectified_epipolar_geometry(tmp_path):
    camera={'camera_model':'pinhole','distortion_model':'equidistant',
            'intrinsics':[500,501,320,240],'distortion_coeffs':[-.02,.001,0.,0.],
            'resolution':[640,480]}
    transform=np.eye(4);transform[:3,3]=[.065,.001,.002]
    data={'cam0':camera,'cam1':{**camera,'intrinsics':[510,511,315,238],
                                'T_cn_cnm1':transform.tolist()}}
    path=tmp_path/'kalibr.yaml';path.write_text(yaml.safe_dump(data),encoding='utf8')
    with pytest.raises(CalibrationError,match='明确指定'):
        load_calibration(path)
    c=load_calibration(path,kalibr_left_camera='cam0')
    assert c.distortion_model=='fisheye'
    np.testing.assert_allclose(c.t_right_left,transform[:3,3])
    inverse=load_calibration(path,kalibr_left_camera='cam1')
    np.testing.assert_allclose(inverse.t_right_left,np.linalg.inv(transform)[:3,3])
    np.testing.assert_allclose(inverse.K_left,c.K_right)
    r=StereoRectifier(c)
    # 使用模型投影检查非平凡双目外参和校正坐标；无硬件或同步假设。
    import cv2
    xyz=np.array([[[.03,.02,1.]], [[-.1,.1,.8]], [[.1,-.1,1.5]]])
    left=cv2.fisheye.projectPoints(xyz,np.zeros(3),np.zeros(3),c.K_left,c.D_left)[0].reshape(-1,2)
    right=cv2.fisheye.projectPoints(xyz,cv2.Rodrigues(c.R_right_left)[0],c.t_right_left,c.K_right,c.D_right)[0].reshape(-1,2)
    assert np.max(r.epipolar_errors(r.points(left,'left'),r.points(right,'right')))<1e-5

def test_kalibr_rejects_unknown_fisheye_model(tmp_path):
    path=tmp_path/'bad.yaml'
    path.write_text(yaml.safe_dump({'cam0':{'camera_model':'pinhole','distortion_model':'radtan'},'cam1':{}}))
    with pytest.raises(CalibrationError,match='equidistant'):
        load_calibration(path,kalibr_left_camera='cam0')
