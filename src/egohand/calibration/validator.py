# 标定输入门禁：宁可保留二维，也不接受不明单位、错误旋转或零基线。

import numpy as np

class CalibrationError(ValueError):
    pass

# 检查内外参形状、有限性、单位、来源与实际尺寸；返回通过验证的原对象。
def validate_calibration(c, image_size=None):
    def require(condition, message):
        if not condition:
            raise CalibrationError(message)
    # 米制必须显式给出，不能仅凭平移数值大小自动猜毫米/米。
    require(c.translation_unit == "m", "平移单位必须显式为 m")
    require(bool(c.calibration_id) and bool(c.calibration_source), "缺少标定 ID 或来源")
    require(c.distortion_model in ("pinhole", "fisheye"), "不支持的畸变模型")
    require(isinstance(c.image_width, int) and isinstance(c.image_height, int) and c.image_width > 0 and c.image_height > 0, "图像尺寸必须为正整数")
    for name in ("K_left", "K_right"):
        K = getattr(c,name)
        require(K.shape == (3,3) and np.isfinite(K).all(), f"{name} 维度或有限性错误")
        require(0 < K[0,0] < 1e7 and 0 < K[1,1] < 1e7, f"{name} 非法焦距")
        require(np.allclose(K[2], [0,0,1]) and abs(K[1,0]) < 1e-9 and abs(K[0,1]) < 1e-9, "K 必须为标准零偏斜相机矩阵")
    for name in ("D_left", "D_right"):
        D = getattr(c,name)
        require(D.ndim in (1,2) and (D.ndim == 1 or 1 in D.shape) and np.isfinite(D).all(), f"{name} 非法")
        require(D.size in ((4,) if c.distortion_model == "fisheye" else (4,5,8,12,14)), "畸变参数数量错误")
    # 旋转必须属于 SO(3)；非零平移是立体深度可观测的必要条件之一。
    R,t = c.R_right_left,c.t_right_left
    require(R.shape == (3,3) and np.isfinite(R).all(), "旋转维度或有限性错误")
    require(np.allclose(R.T@R,np.eye(3),atol=1e-6) and abs(np.linalg.det(R)-1)<1e-6, "旋转矩阵必须正交且 det=1")
    require(t.shape in ((3,), (3,1)) and np.isfinite(t).all() and np.linalg.norm(t)>1e-7, "基线为零或平移无效")
    require(c.synthetic == (c.calibration_source == "synthetic"), "合成标定来源与 synthetic 标识必须一致")
    if image_size is not None:
        require(tuple(image_size) == (c.image_width,c.image_height), "标定与实际图像尺寸不一致")
    return c
