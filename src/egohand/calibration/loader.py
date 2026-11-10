import json
from pathlib import Path
import yaml
import numpy as np
from .schema import StereoCalibration
from .validator import validate_calibration, CalibrationError
from ..core.types import serializable

def load_calibration(path, *, kalibr_left_camera=None):
    try:
        with Path(path).open(encoding="utf-8-sig") as f:
            data = json.load(f) if str(path).lower().endswith(".json") else yaml.safe_load(f)
        if isinstance(data,dict) and 'cam0' in data and 'cam1' in data:
            return load_kalibr_stereo(data, path, left_camera=kalibr_left_camera)
        return validate_calibration(StereoCalibration(**data))
    except (TypeError, KeyError, OSError, ValueError, yaml.YAMLError) as e:
        raise CalibrationError(f"标定加载失败：{e}") from e

def load_kalibr_stereo(data, path, *, left_camera):
    """Kalibr 的 T_cn_cnm1 是从 cam0 到 cam1，不能猜测通道顺序。"""
    if left_camera not in ('cam0','cam1'):
        raise CalibrationError('Kalibr 标定必须明确指定左侧视频对应 cam0 还是 cam1')
    a,b=data['cam0'],data['cam1']
    for camera in (a,b):
        if camera.get('camera_model')!='pinhole' or camera.get('distortion_model')!='equidistant':
            raise CalibrationError('目前仅支持 Kalibr pinhole + equidistant 鱼眼模型')
    if a['resolution']!=b['resolution']:
        raise CalibrationError('Kalibr 两路标定分辨率不同')
    def matrix(camera):
        fx,fy,cx,cy=map(float,camera['intrinsics'])
        return np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
    T=np.asarray(b['T_cn_cnm1'],dtype=float)
    if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1]):
        raise CalibrationError('Kalibr 双目变换不是有效的齐次矩阵')
    if left_camera=='cam1':
        # 先验证原旋转，避免用矩阵求逆掩盖错误标定。
        R=T[:3,:3]
        if not np.allclose(R.T@R,np.eye(3),atol=1e-6) or abs(np.linalg.det(R)-1)>1e-6:
            raise CalibrationError('Kalibr 双目旋转无效')
        T=np.linalg.inv(T);a,b=b,a
    width,height=map(int,a['resolution'])
    return validate_calibration(StereoCalibration(matrix(a),a['distortion_coeffs'],matrix(b),b['distortion_coeffs'],
        T[:3,:3],T[:3,3],width,height,'fisheye',f'kalibr-{Path(path).stem}-{left_camera}-left',
        f'Kalibr {Path(path).name}; 左图={left_camera}; equidistant', 'm',False))

def save_calibration(calibration, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(serializable(calibration), allow_unicode=True, sort_keys=False), encoding="utf-8")
