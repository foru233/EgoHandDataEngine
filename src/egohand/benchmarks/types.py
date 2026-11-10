# 离线评测结构与 GUI 运行结果分开；标注坐标系、关节映射及来源不可省略。

from dataclasses import dataclass,field
from pathlib import Path
import numpy as np
from ..calibration.schema import StereoCalibration

class BenchmarkBlocked(RuntimeError):
    def __init__(self,reason,status="BLOCKED_BY_DATA"):
        super().__init__(reason);self.status=status

# 显式记录数据集关节与 MediaPipe 的语义可比性，不能仅按相同索引认为等价。
@dataclass(frozen=True)
class JointMapping:
    dataset_index: int
    dataset_name: str
    mediapipe_index: int | None
    mediapipe_name: str | None
    comparable: bool
    mapping_reason: str

# 图像、独立左右时间、标定、GT、映射与来源组成一个评测样本。
@dataclass
class BenchmarkSample:
    dataset_id: str
    sequence_id: str
    frame_id: str
    left_image: np.ndarray
    right_image: np.ndarray
    calibration: StereoCalibration | None
    ground_truth_xyz_m: np.ndarray | None
    ground_truth_frame: str | None
    joint_names: tuple
    joint_mapping: tuple[JointMapping,...]
    label_validity: np.ndarray | None
    provenance: dict
    left_timestamp_ns: int | None = None
    right_timestamp_ns: int | None = None
    image_geometry: str = "raw"
    native_annotations: dict = field(default_factory=dict)

    # 只把可比较且有效的关节放到 MediaPipe 索引槽，排除点仍保持 NaN。
    def mapped_ground_truth(self):
        """Comparable semantic subset only; never turn palm center into wrist."""
        xyz=np.full((21,3),np.nan);mask=np.zeros(21,dtype=bool)
        if self.ground_truth_xyz_m is None:return xyz,mask
        for m in self.joint_mapping:
            if m.comparable and m.mediapipe_index is not None:
                point=self.ground_truth_xyz_m[m.dataset_index]
                good=np.isfinite(point).all() and (self.label_validity is None or self.label_validity[m.dataset_index])
                if good:xyz[m.mediapipe_index]=point;mask[m.mediapipe_index]=True
        return xyz,mask
