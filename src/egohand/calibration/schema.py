# 相机标定结构；K 为像素内参，D 的模型必须显式声明，平移单位必须为 m。

from dataclasses import dataclass
import numpy as np

# 外参从左相机到右相机：p_right = R_right_left @ p_left + t_right_left。
@dataclass
class StereoCalibration:
    K_left: np.ndarray
    D_left: np.ndarray
    K_right: np.ndarray
    D_right: np.ndarray
    R_right_left: np.ndarray
    t_right_left: np.ndarray
    image_width: int
    image_height: int
    distortion_model: str
    calibration_id: str
    calibration_source: str
    translation_unit: str
    synthetic: bool = False
    rectification_alpha: float = 0.0

    def __post_init__(self):
        for name in ("K_left", "D_left", "K_right", "D_right", "R_right_left", "t_right_left"):
            setattr(self, name, np.asarray(getattr(self, name), dtype=np.float64))

# 仅供合成源/几何测试使用；禁止把此示例标定当作真实相机米制测量依据。
def synthetic_calibration(width=640, height=480):
    K = np.array([[600.,0,width/2],[0,600.,height/2],[0,0,1.]])
    return StereoCalibration(K, np.zeros(5), K.copy(), np.zeros(5), np.eye(3), np.array([-.064,0,0]), width,height,"pinhole","synthetic-640-v1","synthetic", "m", True)
