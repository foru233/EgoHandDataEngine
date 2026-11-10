"""读取 ORB-SLAM3 EuRoC 格式轨迹，明确区分相机和 IMU 坐标。"""
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from pathlib import Path
from bisect import bisect_left

import numpy as np


def rigid_transform(matrix):
    """拒绝非有限值、非齐次矩阵及反射矩阵，避免静默输出错误坐标。"""
    result = np.asarray(matrix, dtype=float)
    if result.shape != (4, 4) or not np.isfinite(result).all():
        raise ValueError('位姿必须是有限的 4×4 矩阵')
    rotation = result[:3, :3]
    if (not np.allclose(result[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5)):
        raise ValueError('位姿不是有效的刚体变换')
    return result.copy()


def quaternion_matrix(x, y, z, w):
    q = np.array([x, y, z, w], dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12 or abs(norm - 1) > 1e-3:
        raise ValueError('无效的 xyzw 四元数')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


@dataclass(frozen=True)
class OfflinePose:
    timestamp_ns: int
    T_world_from_camera_left: np.ndarray
    calibration_id: str
    pose_source: str = 'orb_slam3_offline_optimized'
    tracking_status: str = 'trajectory_available_tracking_state_unknown'


def load_euroc_trajectory(path, *, output_frame, calibration_id, T_body_from_camera=None, T_exported_camera_from_camera_left=None):
    """读取保存的 Twc 或 Twb；Twb 需明确提供真实的 T_body_from_camera。

    官方导出时间单位为纳秒，使用 Decimal 防止大时间戳先变成 float。
    输出只是离线优化轨迹，不声称逐帧实时跟踪状态或惯性初始化成功。
    """
    if output_frame not in ('camera', 'body') or not calibration_id:
        raise ValueError('必须明确输出坐标系和标定版本')
    if output_frame == 'body':
        if T_body_from_camera is None:
            raise ValueError('IMU 位姿必须提供 T_body_from_camera')
        extrinsic = rigid_transform(T_body_from_camera)
    else:
        if T_body_from_camera is not None:
            raise ValueError('相机轨迹不应重复应用 IMU 外参')
        if T_exported_camera_from_camera_left is None:
            raise ValueError('必须明确导出相机系到原始左相机系的外参，校正相机不可假定为原始相机')
        extrinsic = rigid_transform(T_exported_camera_from_camera_left)
    poses = []
    for number, line in enumerate(Path(path).read_text(encoding='utf-8').splitlines(), 1):
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        fields = line.split()
        if len(fields) != 8:
            raise ValueError(f'第 {number} 行应有 8 列')
        try:
            stamp = Decimal(fields[0])
            if not stamp.is_finite() or stamp != stamp.to_integral_value():
                raise ValueError('时间戳必须为整数纳秒')
            timestamp = int(stamp)
            values = np.array([float(v) for v in fields[1:]], dtype=float)
        except (InvalidOperation, ValueError, OverflowError) as exc:
            raise ValueError(f'第 {number} 行无效') from exc
        if not np.isfinite(values).all() or (poses and timestamp <= poses[-1].timestamp_ns):
            raise ValueError(f'第 {number} 行包含非有限值或非递增时间戳')
        transform = np.eye(4)
        transform[:3, :3] = quaternion_matrix(*values[3:])
        transform[:3, 3] = values[:3]
        transform = transform @ extrinsic
        poses.append(OfflinePose(timestamp, rigid_transform(transform), calibration_id))
    if not poses:
        raise ValueError('轨迹为空')
    return poses

def load_tum_body_ground_truth(path, *, calibration_id, T_body_from_camera):
    """读取 TUM 秒时间戳、xyzw 四元数的体坐标真值，转换为左相机位姿。"""
    extrinsic=rigid_transform(T_body_from_camera)
    poses=[]
    for number,line in enumerate(Path(path).read_text(encoding='utf-8-sig').splitlines(),1):
        if not line.strip() or line.lstrip().startswith('#'):continue
        fields=line.split()
        if len(fields)!=8:raise ValueError(f'真值第 {number} 行应为秒时间戳及 7 个位姿值')
        try:
            stamp=Decimal(fields[0])*Decimal(1_000_000_000)
            if not stamp.is_finite() or stamp!=stamp.to_integral_value():raise ValueError('时间戳精度超过纳秒')
            timestamp=int(stamp);values=np.array([float(v) for v in fields[1:]],float)
        except (InvalidOperation,ValueError,OverflowError) as exc:raise ValueError(f'真值第 {number} 行无效') from exc
        if not np.isfinite(values).all() or (poses and timestamp<=poses[-1].timestamp_ns):
            raise ValueError(f'真值第 {number} 行非有限或时间不递增')
        transform=np.eye(4);transform[:3,:3]=quaternion_matrix(*values[3:]);transform[:3,3]=values[:3]
        poses.append(OfflinePose(timestamp,rigid_transform(transform@extrinsic),calibration_id,
                                 pose_source='dataset_ground_truth',tracking_status='not_applicable'))
    if not poses:raise ValueError('真值轨迹为空')
    return poses

def align_positions_to_ground_truth(estimated,truth,max_delta_ns=5_000_000):
    """最近时间匹配后 SE(3) 对齐；不估计尺度、时间偏移或插值。"""
    if not estimated or not truth:return None
    timestamps=[p.timestamp_ns for p in truth];est=[];ref=[]
    for pose in estimated:
        i=bisect_left(timestamps,pose.timestamp_ns)
        options=[k for k in (i-1,i) if 0<=k<len(timestamps)]
        if not options:continue
        j=min(options,key=lambda k:abs(timestamps[k]-pose.timestamp_ns))
        if abs(timestamps[j]-pose.timestamp_ns)<=max_delta_ns:
            est.append(pose.T_world_from_camera_left[:3,3]);ref.append(truth[j].T_world_from_camera_left[:3,3])
    if len(est)<3:return None
    est=np.asarray(est);ref=np.asarray(ref);a=est-est.mean(0);b=ref-ref.mean(0)
    if np.linalg.matrix_rank(a,tol=1e-7)<2 or np.linalg.matrix_rank(b,tol=1e-7)<2:return None
    u,_,vt=np.linalg.svd(a.T@b);d=np.eye(3);d[-1,-1]=np.linalg.det(vt.T@u.T)
    rotation=vt.T@d@u.T;translation=ref.mean(0)-rotation@est.mean(0)
    return rotation,translation,len(est)


def transform_hand_points(points, T_world_from_camera_left):
    """将相机系米制点变换到局部世界系；非有限点保持不可用。"""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('手部点必须为 N×3')
    transform = rigid_transform(T_world_from_camera_left)
    return points @ transform[:3, :3].T + transform[:3, 3]


class OfflinePoseProvider:
    """有限轨迹的最近帧读取；不跨缺帧插值，不把旧位姿当作实时结果。"""
    def __init__(self, poses, *, max_time_error_ns=1000):
        self.poses = tuple(poses)
        self.timestamps = tuple(p.timestamp_ns for p in self.poses)
        if not self.poses or any(b <= a for a, b in zip(self.timestamps, self.timestamps[1:])):
            raise ValueError('轨迹必须非空且时间严格递增')
        if not isinstance(max_time_error_ns, int) or max_time_error_ns < 0:
            raise ValueError('时间容差必须是非负整数纳秒')
        self.max_time_error_ns = max_time_error_ns

    def pose_at(self, timestamp_ns):
        if not isinstance(timestamp_ns, (int, np.integer)):
            raise ValueError('图像时间戳必须是整数纳秒')
        idx = bisect_left(self.timestamps, timestamp_ns)
        candidates = [i for i in (idx - 1, idx) if 0 <= i < len(self.timestamps)]
        nearest = min(candidates, key=lambda i: abs(self.timestamps[i] - timestamp_ns))
        if abs(self.timestamps[nearest] - timestamp_ns) > self.max_time_error_ns:
            return None
        return self.poses[nearest]

    def camera_to_world(self, timestamp_ns):
        """兼容现有 CameraPoseProvider；无匹配时明确抛错。"""
        pose = self.pose_at(timestamp_ns)
        if pose is None:
            raise LookupError('没有时间同步的离线相机位姿')
        return pose.T_world_from_camera_left.copy()
