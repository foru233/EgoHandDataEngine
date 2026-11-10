"""VS030 到 ORB 校正双目坐标的几何适配；不提供传感器时钟。"""
from dataclasses import dataclass
from pathlib import Path
import numpy as np
import yaml
from ..calibration.loader import load_calibration
from ..calibration.rectifier import StereoRectifier
from .trajectory import rigid_transform

@dataclass(frozen=True)
class Vs030VioGeometry:
    rectifier: StereoRectifier
    T_orb_left_from_gui_left: np.ndarray
    T_imu_from_orb_left: np.ndarray
    orb_left_channel: int = 5
    orb_right_channel: int = 4

def prepare_vs030_vio_geometry(path):
    """交换 ORB 的左右图，使校正后的正深度对应正视差；输入仍是原始设备图。

    GUI 左为 cam0；ORB 左为 cam1。此函数只计算坐标变换，不能证明同步。
    """
    path=Path(path);data=yaml.safe_load(path.read_text(encoding='utf-8-sig'))
    T_cam1_from_cam0=rigid_transform(data['cam1']['T_cn_cnm1'])
    T_cam1_from_imu=rigid_transform(data['cam1']['T_cam_imu'])
    rectifier=StereoRectifier(load_calibration(path,kalibr_left_camera='cam1'))
    if rectifier.disparity_axis!=0 or rectifier.P2[0,3]>=0:
        raise ValueError('ORB-SLAM3 需要水平正视差；当前标定与通道顺序不满足')
    R=np.eye(4);R[:3,:3]=rectifier.R1
    T_orb_left_from_gui_left=rigid_transform(R@T_cam1_from_cam0)
    T_imu_from_orb_left=rigid_transform(np.linalg.inv(T_cam1_from_imu)@np.linalg.inv(R))
    return Vs030VioGeometry(rectifier,T_orb_left_from_gui_left,T_imu_from_orb_left)
