# 运行时数据契约：像素用 px，三维坐标用 m，协方差用 m²。
# 不可用三维点用 NaN + valid/status 表示；对外严格 JSON 使用 null。

from dataclasses import dataclass, field, fields, is_dataclass
from enum import Enum
import math
import numpy as np
from ..detection.landmarks import JOINT_NAMES
from .status import JointStatus

SCHEMA_VERSION = "1.1.0"

# 统一序列化 dataclass、NumPy 和枚举；非有限浮点转为 null，禁止输出 JSON NaN。
def serializable(value):
    """Strict JSON representation: nonfinite numbers are null, never JSON NaN."""
    if is_dataclass(value):
        return {f.name: serializable(getattr(value, f.name)) for f in fields(value)}
    if isinstance(value, np.ndarray):
        return serializable(value.tolist())
    if isinstance(value, np.generic):
        return serializable(value.item())
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {str(k): serializable(v) for k,v in value.items()}
    if isinstance(value, (list, tuple)):
        return [serializable(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value

# 来源能力与证据声明；synchronized 或 metric_capable 声明不能替代实际验收。
@dataclass
class SourceMetadata:
    source_type: str
    real_stereo: bool = False
    synchronized: bool = False
    real_calibration: bool = False
    metric_capable: bool = False
    contains_imu: bool = False
    ground_truth: bool = False
    synthetic: bool = False
    synchronization_evidence: str | None = None
    timestamp_kind: str = "unknown"
    extra: dict = field(default_factory=dict)

# 单只手的二维模型观测：21×2 像素与有效掩码；world_landmarks 是模型局部坐标。
@dataclass
class HandDetection:
    handedness: str
    handedness_score: float
    normalized_landmarks: np.ndarray
    pixel_landmarks: np.ndarray
    world_landmarks: np.ndarray | None = None
    hand_track_id: int | None = None
    bounding_box: tuple | None = None
    timestamp_ns: int = 0
    valid: np.ndarray = field(default_factory=lambda: np.ones(21, dtype=bool))
    origin: str = "mediapipe"

    def __post_init__(self):
        self.pixel_landmarks = np.asarray(self.pixel_landmarks, dtype=np.float64)
        self.normalized_landmarks = np.asarray(self.normalized_landmarks, dtype=np.float64)
        self.valid = np.asarray(self.valid, dtype=bool)
        if self.pixel_landmarks.shape != (21,2) or self.valid.shape != (21,):
            raise ValueError("每只手必须为 21×2 像素点及 21 项有效掩码")
        if self.normalized_landmarks.shape not in ((21,2),(21,3)):
            raise ValueError("归一化关节必须为 21×2 或带模型 z 的 21×3")
        if self.world_landmarks is not None:
            self.world_landmarks=np.asarray(self.world_landmarks,dtype=np.float64)
            if self.world_landmarks.shape!=(21,3):raise ValueError("MediaPipe world landmarks 必须为 21×3")

# 一对图像及独立左右时间戳；timestamp_ns 是主处理时间，不应自动当作两路硬件时间。
@dataclass
class StereoFrame:
    frame_id: int
    timestamp_ns: int
    left_image: np.ndarray
    right_image: np.ndarray
    left_timestamp_ns: int | None = None
    right_timestamp_ns: int | None = None
    calibration_id: str | None = None
    source_type: str = "external"
    synchronized: bool = False
    image_geometry: str = "raw"
    metadata: dict = field(default_factory=dict)
    left_detections: list[HandDetection] | None = None
    right_detections: list[HandDetection] | None = None

# 单关节重建：xyz_m 与 valid/status 一起读；quality_score 为几何分数，不是概率。
@dataclass
class Joint3D:
    joint_id: int
    joint_name: str
    xyz_m: np.ndarray = field(default_factory=lambda: np.full(3, np.nan))
    valid: bool = False
    status: str = JointStatus.UNMATCHED
    left_uv_px: tuple | None = None
    right_uv_px: tuple | None = None
    epipolar_error_px: float | None = None
    reprojection_error_px: float | None = None
    quality_score: float | None = None
    # 以下可选字段兼容旧记录；深度 std 对应 Z，位置 std 是协方差最大主轴标准差。
    depth_std_m: float | None = None
    position_std_m: float | None = None
    position_covariance_m2: np.ndarray | None = None
    uncertainty_status: str | None = None

# 单只手固定 21 个关节；未匹配 track_id=-1，槽位顺序本身不代表跨帧身份。
@dataclass
class Hand3D:
    handedness: str
    handedness_score: float | None
    track_id: int
    joints: list[Joint3D] = field(default_factory=lambda: [Joint3D(i,n) for i,n in enumerate(JOINT_NAMES)])
    mediapipe_world_landmarks: np.ndarray | None = None
    valid_joint_count: int = 0
    tracking_status: str = "unmatched"
    right_mediapipe_world_landmarks: np.ndarray | None = None

# 单帧完整结果；坐标系/单位/来源/诊断必须随结果保存，不能只导出裸 XYZ。
@dataclass
class StereoHandResult:
    frame_id: int
    timestamp_ns: int
    calibration_id: str | None
    source_type: str
    hands: list[Hand3D] = field(default_factory=list)
    reconstruction_status: str = "2D Only"
    processing_time_ms: float = 0.0
    diagnostics: dict = field(default_factory=dict)
    schema_version: str = SCHEMA_VERSION
    coordinate_frame: str = "camera_left_optical"
    unit: str = "m"
    session_id: str = ""
    left_detections: list[HandDetection] = field(default_factory=list)
    right_detections: list[HandDetection] = field(default_factory=list)
    # 显式绑定 VIO 后才会附加，原始相机系关节保持不变。
    world_hands: dict | None = None
    # Additive local recording output. Legacy world-reference protection stays.
    local_motion: dict | None = None
    segment_hands: dict | None = None

def detection_from_dict(d):
    d = dict(d)
    for k in ("normalized_landmarks", "pixel_landmarks", "world_landmarks"):
        if d.get(k) is not None:
            d[k] = np.array(d[k], dtype=float)
    return HandDetection(**d)

# 兼容读取旧 JSON；缺失的新协方差字段保留默认 None，不伪造零不确定性。
def result_from_dict(d):
    d = dict(d)
    hands = []
    for h in d.pop("hands", []):
        h = dict(h)
        joints = []
        for j in h.pop("joints"):
            j = dict(j)
            j["xyz_m"] = np.array(j["xyz_m"], dtype=float)
            if j.get('position_covariance_m2') is not None:j['position_covariance_m2']=np.asarray(j['position_covariance_m2'],dtype=float)
            joints.append(Joint3D(**j))
        for key in ("mediapipe_world_landmarks", "right_mediapipe_world_landmarks"):
            if h.get(key) is not None:
                h[key] = np.asarray(h[key], dtype=float)
        hands.append(Hand3D(joints=joints, **h))
    for key in ("left_detections", "right_detections"):
        d[key] = [detection_from_dict(x) for x in d.get(key, [])]
    return StereoHandResult(hands=hands, **d)
