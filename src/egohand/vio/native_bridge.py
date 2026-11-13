"""ctypes interface to the native ORB-SLAM3 Stereo-Inertial bridge.

Contract:
- Images are rectified left/right grayscale uint8 arrays.
- Camera and IMU timestamps are int64 nanoseconds in ONE device clock domain.
- IMU samples passed with frame k are the samples in (t_camera[k-1], t_camera[k]].
- The first stereo frame is passed without IMU samples.
- Returned pose is T_world_from_rectified_left_camera, not raw-camera coordinates.
"""
from __future__ import annotations

import ctypes
import base64
import hashlib
import json
import math
import os
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np
from .library_manifest import select_library

_RecoveryCallback = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.POINTER(ctypes.c_float), ctypes.c_uint64, ctypes.c_void_p)


class _CImuSample(ctypes.Structure):
    _fields_ = [
        ("timestamp_ns", ctypes.c_int64),
        ("accel_x_mps2", ctypes.c_float),
        ("accel_y_mps2", ctypes.c_float),
        ("accel_z_mps2", ctypes.c_float),
        ("gyro_x_rps", ctypes.c_float),
        ("gyro_y_rps", ctypes.c_float),
        ("gyro_z_rps", ctypes.c_float),
    ]


class _CResult(ctypes.Structure):
    _fields_ = [
        ("tracking_state", ctypes.c_int32),
        ("pose_valid", ctypes.c_int32),
        ("is_lost", ctypes.c_int32),
        ("imu_initialized", ctypes.c_int32),
        ("inertial_scale_finalized", ctypes.c_int32),
        ("imu_init_elapsed_sec", ctypes.c_double),
        ("frame_timestamp_sec", ctypes.c_double),
        ("map_epoch", ctypes.c_uint64),
        ("T_world_from_rectified_left_camera", ctypes.c_float * 16),
    ]


class _CFrameSnapshot(ctypes.Structure):
    _fields_ = [
        ("schema_version", ctypes.c_uint32), ("struct_size", ctypes.c_uint32),
        *[(name, ctypes.c_uint64) for name in ("frame_id", "map_id", "map_revision", "reset_id", "world_frame_id", "map_change_index")],
        ("timestamp_ns", ctypes.c_int64),
        *[(name, ctypes.c_int32) for name in ("snapshot_consistent", "tracking_state", "pose_valid", "imu_initialized",
            "ba1_complete", "ba2_complete", "body_valid", "left_features", "right_features", "stereo_valid_depths", "tracked_inliers")],
        ("T_world_from_rectified_left_camera", ctypes.c_float * 16),
        ("T_world_from_body", ctypes.c_float * 16),
        ("velocity_world_mps", ctypes.c_float * 3), ("accel_gyro_bias", ctypes.c_float * 6),
    ]


class _CMapEvent(ctypes.Structure):
    _fields_ = [
        *[(name, ctypes.c_uint64) for name in ('sequence', 'map_id', 'source_keyframe_id',
            'revision_before', 'revision_after', 'change_before', 'change_after')],
        ('source_keyframe_time_sec', ctypes.c_double), ('scale', ctypes.c_float),
        ('rotation', ctypes.c_float * 9), ('translation', ctypes.c_float * 3),
        *[(name, ctypes.c_uint32) for name in ('type', 'map_update_lock_matches_caller', 'reset_generation_known', 'reserved')],
    ]


class _CMapAppearancePoint(ctypes.Structure):
    _fields_ = [('point_id', ctypes.c_uint64), ('position', ctypes.c_float * 3),
                ('descriptor', ctypes.c_uint8 * 32), ('observations', ctypes.c_uint32)]


class _CMapAppearanceHeader(ctypes.Structure):
    _fields_ = [*[(n, ctypes.c_uint32) for n in ('schema_version', 'struct_size', 'status', 'flags',
        'point_count', 'sampled_count', 'capacity', 'source_kind', 'image_width', 'image_height')],
        *[(n, ctypes.c_uint64) for n in ('frame_id', 'map_id', 'reset_id', 'atlas_generation',
            'map_revision', 'map_change_index', 'total_map_points', 'world_frame_revision')],
        ('timestamp_ns', ctypes.c_int64),
        *[(n, ctypes.c_float) for n in ('fx', 'fy', 'cx', 'cy', 'bf')], ('reserved', ctypes.c_uint32)]


class _CMapEventFrame(ctypes.Structure):
    _fields_ = [('schema_version', ctypes.c_uint32), ('struct_size', ctypes.c_uint32),
        *[(name, ctypes.c_uint64) for name in ('frame_id', 'map_id', 'reset_id')],
        ('frame_timestamp_ns', ctypes.c_int64),
        *[(name, ctypes.c_uint32) for name in ('snapshot_consistent', 'frame_bound', 'coverage', 'event_count')],
        ('overwritten_count', ctypes.c_uint64), ('events', _CMapEvent * 16)]


class _CReferenceFrame(ctypes.Structure):
    _fields_ = [('schema_version', ctypes.c_uint32), ('struct_size', ctypes.c_uint32),
        *[(name, ctypes.c_uint64) for name in ('frame_id', 'map_id', 'reset_id', 'atlas_generation',
            'generation', 'operation_sequence', 'geometry_sequence', 'map_revision', 'map_change_index')],
        ('timestamp_ns', ctypes.c_int64), ('snapshot_consistent', ctypes.c_uint32), ('frame_bound', ctypes.c_uint32)]


class _CWorldFrameInfo(ctypes.Structure):
    """世界系修订号（独立扩展，EgohandVioResult 布局不变）。"""
    _fields_ = [
        ("schema_version", ctypes.c_uint32), ("struct_size", ctypes.c_uint32),
        *[(name, ctypes.c_int32) for name in ("available", "verdict", "last_keyframes", "reserved")],
        *[(name, ctypes.c_uint64) for name in ("revision", "detections", "evaluations",
            "rejected_nonrigid", "map_id", "last_max_keyframe_id")],
        *[(name, ctypes.c_double) for name in ("last_timestamp_sec", "last_translation_m",
            "last_rotation_deg", "last_scale", "last_residual_p50_m", "last_residual_p95_m",
            "last_residual_p99_m", "last_residual_max_m", "last_residual_limit_m")],
    ]


@dataclass(frozen=True)
class ImuMeasurement:
    timestamp_ns: int
    acceleration_mps2: tuple[float, float, float]
    angular_velocity_rps: tuple[float, float, float]


@dataclass(frozen=True)
class VioFrameResult:
    timestamp_ns: int
    tracking_state: int
    tracking_state_name: str
    pose_valid: bool
    is_lost: bool
    imu_initialized: bool
    imu_init_elapsed_sec: float
    T_world_from_rectified_left_camera: np.ndarray | None
    map_epoch: int = 0
    pose_frame_id: str = "rectified_left_camera"
    inertial_scale_finalized: bool = False
    frame_snapshot: dict | None = None
    world_frame_revision: int = 0
    world_frame_info: dict | None = None
    map_event_frame: dict | None = None
    map_appearance: dict | None = None
    recovery_observation: dict | None = None


_STATE_NAMES = {
    -1: "SYSTEM_NOT_READY",
    0: "NO_IMAGES_YET",
    1: "NOT_INITIALIZED",
    2: "OK",
    3: "RECENTLY_LOST",
    4: "LOST",
    5: "OK_KLT",
}

# 与原生 VioWorldFrame::Verdict 一致；-1 表示还没有任何一次全 BA 拟合。只用于人读日志。
_WORLD_FRAME_VERDICTS = {
    -1: "not_evaluated",
    0: "reanchor",
    1: "translation_below_threshold",
    2: "non_rigid",
    3: "insufficient_samples",
}


class NativeVioBridge:
    def __init__(
        self,
        vocabulary_path: str | Path,
        settings_path: str | Path,
        library_path: str | Path | None = None,
        conda_prefix: str | Path | None = None,
        use_viewer: bool = False,
        map_mode: str | None = None,
        map_appearance_frames: Sequence[int] = (),
    ):
        self.vocabulary_path = Path(vocabulary_path).resolve()
        self.map_mode = map_mode
        self._recovery_callback_ref = None
        self._recovery_callback_error = None
        self.settings_path = Path(settings_path).resolve()
        self._appearance_frames = frozenset(map_appearance_frames)
        if any(isinstance(i, bool) or not isinstance(i, int) or i < 1 for i in self._appearance_frames):
            raise ValueError('map appearance frames must be positive input sequence numbers')
        self._input_sequence = 0
        self._bridge_instance_id = str(uuid.uuid4())
        if not self.vocabulary_path.is_file():
            raise FileNotFoundError(self.vocabulary_path)
        if not self.settings_path.is_file():
            raise FileNotFoundError(self.settings_path)

        native_dir = Path(__file__).resolve().parent / "native"
        if library_path is None:
            library_path = select_library(native_dir, map_mode=map_mode)
        self.library_path = Path(library_path).resolve()
        if not self.library_path.is_file():
            raise FileNotFoundError(
                f"Native bridge DLL not found: {self.library_path}. "
                "Build it with native/CMakeLists.txt first."
            )

        if conda_prefix is None:
            conda_prefix = os.environ.get(
                "CONDA_PREFIX",
                str(Path.home() / "miniconda3" / "envs" / "egohand-vio"),
            )
        conda_prefix = Path(conda_prefix)
        dll_dirs = [
            conda_prefix / "Library" / "bin",
            native_dir / "bin",
            self.library_path.parent,
        ]
        # DBoW2 is built as a DLL in the ORB-SLAM3 third-party folder.
        project_root = Path(__file__).resolve().parents[3]
        dll_dirs.append(
            project_root / "third_party" / "ORB_SLAM3" / "Thirdparty" / "DBoW2" / "lib"
        )
        self._dll_directory_handles = []
        for directory in dll_dirs:
            if directory.is_dir() and hasattr(os, "add_dll_directory"):
                self._dll_directory_handles.append(os.add_dll_directory(str(directory)))

        try:
            self._dll = ctypes.CDLL(str(self.library_path))
        except OSError as exc:
            # GUI Python 已加载的二进制依赖可能与独立 VIO Conda 环境冲突。
            # 保留原始 WinError，不在用户的项目环境里替换 DLL 或依赖。
            raise RuntimeError(
                '加载 ORB-SLAM3 桥接库失败；请在独立 egohand-vio 进程运行，'
                f'检查运行时 DLL 及依赖冲突：{exc}'
            ) from exc
        self._bind_api()
        if (hasattr(self._dll, 'egohand_vio_test_inject_numerical_fault')
                and os.environ.get('EGOHAND_VIO_ALLOW_TEST_HOOKS') != '1'):
            raise RuntimeError('拒绝加载含故障注入接口的测试库；测试库不能用于正式采集')
        self._check_abi()
        if self._appearance_frames:
            self._bind_map_appearance()
            self._appearance_settings_sha256 = hashlib.sha256(self.settings_path.read_bytes()).hexdigest()
        self._handle = ctypes.c_void_p()
        self._last_frame_timestamp_ns: int | None = None
        self._closed = False

        create = self._dll.egohand_vio_create
        mode = int(bool(use_viewer))
        if map_mode is not None:
            if map_mode not in ("build", "localize"):
                raise ValueError(f"Unknown map mode: {map_mode}")
            create = self._map_function("create_map", create.argtypes)
            mode = int(map_mode == "localize")
        rc = create(
            os.fsencode(self.vocabulary_path),
            os.fsencode(self.settings_path),
            mode,
            ctypes.byref(self._handle),
        )
        if rc != 0:
            raise RuntimeError(self._last_error())
        if not self._handle:
            raise RuntimeError("Native bridge returned a null ORB-SLAM3 handle")

    def _bind_api(self) -> None:
        dll = self._dll
        dll.egohand_vio_api_version.argtypes = []
        dll.egohand_vio_api_version.restype = ctypes.c_char_p
        dll.egohand_vio_last_error.argtypes = []
        dll.egohand_vio_last_error.restype = ctypes.c_char_p
        if not hasattr(dll, 'egohand_vio_result_size'):
            raise RuntimeError('桥接库缺少 ABI 版本校验接口，请重新编译与当前 Python 匹配的 DLL。')
        dll.egohand_vio_result_size.argtypes = []
        dll.egohand_vio_result_size.restype = ctypes.c_int32
        if hasattr(dll, 'egohand_vio_get_frame_snapshot'):
            if not hasattr(dll, 'egohand_vio_snapshot_size'):
                raise RuntimeError('原子快照接口缺少 ABI 大小校验')
            dll.egohand_vio_snapshot_size.argtypes = []
            dll.egohand_vio_snapshot_size.restype = ctypes.c_int32
            dll.egohand_vio_get_frame_snapshot.argtypes = [ctypes.c_void_p,
                ctypes.POINTER(_CFrameSnapshot), ctypes.c_int32]
            dll.egohand_vio_get_frame_snapshot.restype = ctypes.c_int
        if hasattr(dll, 'egohand_vio_get_map_event_frame'):
            if not hasattr(dll, 'egohand_vio_map_event_frame_size'):
                raise RuntimeError('地图事件接口缺少 ABI 大小校验')
            dll.egohand_vio_map_event_frame_size.argtypes = []
            dll.egohand_vio_map_event_frame_size.restype = ctypes.c_int32
            dll.egohand_vio_get_map_event_frame.argtypes = [ctypes.c_void_p,
                ctypes.POINTER(_CMapEventFrame), ctypes.c_int32]
            dll.egohand_vio_get_map_event_frame.restype = ctypes.c_int
        if hasattr(dll, 'egohand_vio_get_body_state'):
            dll.egohand_vio_get_body_state.argtypes = [ctypes.c_void_p,
                ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
                ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_int)]
            dll.egohand_vio_get_body_state.restype = ctypes.c_int
        if hasattr(dll, 'egohand_vio_set_recovery_callback'):
            dll.egohand_vio_set_recovery_callback.argtypes = [ctypes.c_void_p,_RecoveryCallback,ctypes.c_void_p]
            dll.egohand_vio_set_recovery_callback.restype = ctypes.c_int
        if hasattr(dll, 'egohand_vio_start_new_map'):
            dll.egohand_vio_start_new_map.argtypes = [ctypes.c_void_p]
            dll.egohand_vio_start_new_map.restype = ctypes.c_int
        if hasattr(dll, 'egohand_vio_get_world_frame_info'):
            if not hasattr(dll, 'egohand_vio_world_frame_info_size'):
                raise RuntimeError('世界系修订号接口缺少 ABI 大小校验')
            dll.egohand_vio_world_frame_info_size.argtypes = []
            dll.egohand_vio_world_frame_info_size.restype = ctypes.c_int32
            dll.egohand_vio_get_world_frame_info.argtypes = [ctypes.c_void_p,
                ctypes.POINTER(_CWorldFrameInfo), ctypes.c_int32]
            dll.egohand_vio_get_world_frame_info.restype = ctypes.c_int

        dll.egohand_vio_create.argtypes = [
            ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int,
            ctypes.POINTER(ctypes.c_void_p),
        ]
        dll.egohand_vio_create.restype = ctypes.c_int

        dll.egohand_vio_track_stereo.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_int, ctypes.c_int, ctypes.c_int,
            ctypes.c_int64,
            ctypes.POINTER(_CImuSample), ctypes.c_int,
            ctypes.POINTER(_CResult),
        ]
        dll.egohand_vio_track_stereo.restype = ctypes.c_int

        dll.egohand_vio_reset.argtypes = [ctypes.c_void_p]
        dll.egohand_vio_reset.restype = ctypes.c_int
        dll.egohand_vio_shutdown.argtypes = [ctypes.c_void_p]
        dll.egohand_vio_shutdown.restype = ctypes.c_int
        dll.egohand_vio_destroy.argtypes = [ctypes.c_void_p]
        dll.egohand_vio_destroy.restype = None

    @property
    def api_version(self) -> str:
        return self._dll.egohand_vio_api_version().decode("utf-8", "replace")

    def _check_abi(self) -> None:
        """结构体布局必须与 Python 侧一致，否则字段会被错位读取。

        EgohandVioResult 里 inertial_scale_finalized 的位置决定了后续 double 和
        数组的偏移；用错版本的 DLL 会把地图尺度状态读成垃圾值，进而误判
        “尺度已定标”。这里直接比对 sizeof，而不是依赖版本字符串。
        """
        if not hasattr(self._dll, 'egohand_vio_result_size'):
            raise RuntimeError('桥接库缺少 ABI 版本校验接口，不能安全解析位姿结构。')
        expected = ctypes.sizeof(_CResult)
        actual = int(self._dll.egohand_vio_result_size())
        if actual != expected:
            raise RuntimeError(
                f'桥接库 ABI 不匹配：Python 期望结构体大小 {expected} 字节，'
                f'DLL 实际为 {actual} 字节。请重新编译 native 桥接库并重启 GUI；'
                '旧的 egohand_vio_recovery_bridge_*.dll 与当前结构体布局不兼容。'
            )

    def _bind_map_appearance(self):
        """Opt-in extension; old deployed libraries need not implement it."""
        dll = self._dll
        capabilities = getattr(dll, 'egohand_vio_capabilities', None)
        if capabilities is None:
            raise RuntimeError('map appearance capability unavailable')
        capabilities.argtypes = []
        capabilities.restype = ctypes.c_char_p
        if 'map_appearance_v1' not in capabilities().decode('ascii').split(';'):
            raise RuntimeError('map appearance capability unavailable')
        for kind, structure, suffix in ((0, _CMapAppearanceHeader, 'header'), (1, _CMapAppearancePoint, 'point')):
            fn = getattr(dll, f'egohand_vio_map_appearance_{suffix}_size')
            fn.argtypes = []
            fn.restype = ctypes.c_int32
            if fn() != ctypes.sizeof(structure):
                raise RuntimeError('map appearance ABI size mismatch')
            offset = dll.egohand_vio_map_appearance_offset
            offset.argtypes = [ctypes.c_int32, ctypes.c_int32]
            offset.restype = ctypes.c_int32
            for index, (name, _) in enumerate(structure._fields_):
                if offset(kind, index) != getattr(structure, name).offset:
                    raise RuntimeError(f'map appearance ABI offset mismatch: {name}')
        fn = dll.egohand_vio_get_map_appearance
        fn.argtypes = [ctypes.c_void_p, ctypes.c_uint64, ctypes.c_int64,
            ctypes.POINTER(_CMapAppearanceHeader), ctypes.c_int32,
            ctypes.POINTER(_CMapAppearancePoint), ctypes.c_int32, ctypes.c_int32]
        fn.restype = ctypes.c_int

    def get_map_appearance(self, snapshot, frame_ns, *, capacity=1024):
        """Copy bounded map appearance, never Tracking correspondences or a release certificate.

        Calls on a bridge, including reset and close, must be serialized by its owner.
        Packed bytes use the validated native layout (no pointer fields).
        """
        if self._closed or not self._handle:
            raise RuntimeError('Native VIO bridge is closed')
        if isinstance(capacity, bool) or not isinstance(capacity, int) or not 1 <= capacity <= 4096:
            raise ValueError('map appearance capacity must be 1..4096')
        if (not isinstance(snapshot, dict) or snapshot.get('snapshot_consistent') is not True
                or snapshot.get('timestamp_ns') != frame_ns):
            return {'status': 'unavailable', 'reason': 'frame_snapshot_unavailable', 'coverage': 'partial'}
        reference = snapshot.get('reference_frame')
        has_reference = 'reference_generation' in snapshot or 'reference_frame' in snapshot
        if has_reference and (not isinstance(reference, dict) or reference.get('frame_bound') is not True
                or type(snapshot.get('reference_generation')) is not int
                or snapshot['reference_generation'] != reference.get('generation')):
            return {'status': 'unavailable', 'reason': 'reference_frame_unavailable', 'coverage': 'partial'}
        self._bind_map_appearance()
        header = _CMapAppearanceHeader()
        points = (_CMapAppearancePoint * capacity)()
        rc = self._dll.egohand_vio_get_map_appearance(self._handle, snapshot['frame_id'], frame_ns,
            ctypes.byref(header), ctypes.sizeof(header), points, ctypes.sizeof(_CMapAppearancePoint), capacity)
        if rc not in (0, 1):
            raise RuntimeError(self._last_error())
        if (header.schema_version != 1 or header.struct_size != ctypes.sizeof(header)
                or header.source_kind != 1 or header.capacity != capacity
                or header.flags & ~3 or not header.flags & 1 or header.reserved != 0
                or header.status not in (0, 1, 2, 3, 4)):
            raise RuntimeError('invalid map appearance header')
        if rc == 1:
            if header.point_count or header.status == 1:
                raise RuntimeError('invalid unavailable map appearance')
            return {'status': 'unavailable', 'native_status': int(header.status), 'coverage': 'partial'}
        if (header.status != 1 or not 0 <= header.point_count <= header.sampled_count <= capacity
                or header.sampled_count > header.total_map_points
                or bool(header.flags & 2) != (header.total_map_points > header.sampled_count)):
            raise RuntimeError('invalid map appearance counts')
        for key in ('frame_id', 'timestamp_ns', 'map_id', 'reset_id', 'map_revision', 'map_change_index'):
            if int(getattr(header, key)) != snapshot.get(key):
                raise RuntimeError(f'map appearance frame identity mismatch: {key}')
        reference_metadata = {}
        if has_reference:
            # The extension freshly revalidates the immutable frame and geometry
            # token after numeric copying. This is a narrow association check,
            # not certification of every MapPoint writer or a rigid map update.
            after = self.get_reference_frame(snapshot)
            fields = ('frame_id', 'timestamp_ns', 'map_id', 'reset_id', 'map_revision',
                      'map_change_index', 'atlas_generation', 'geometry_sequence',
                      'generation', 'operation_sequence')
            if (not isinstance(after, dict) or after.get('frame_bound') is not True
                    or any(type(reference.get(k)) is not int or type(after.get(k)) is not int
                           or reference[k] != after[k] for k in fields)
                    or int(header.atlas_generation) != after['atlas_generation']):
                return {'status': 'unavailable', 'reason': 'reference_changed_during_appearance_copy',
                        'coverage': 'partial'}
            reference_metadata = {'reference_generation': after['generation'], 'reference_frame': after}
        if (header.image_width <= 0 or header.image_height <= 0
                or not all(math.isfinite(getattr(header, k)) for k in ('fx', 'fy', 'cx', 'cy', 'bf'))
                or min(header.fx, header.fy, header.bf) <= 0):
            raise RuntimeError('invalid map appearance camera')
        ids = set()
        for p in points[:header.point_count]:
            if p.point_id in ids or p.observations < 2 or not all(map(math.isfinite, p.position)):
                raise RuntimeError('invalid or duplicate map appearance point')
            ids.add(p.point_id)
        metadata = {name: getattr(header, name) for name, _ in header._fields_}
        return {**metadata, **reference_metadata, 'status': 'copied', 'native_status': 1, 'coverage': 'partial',
            'advisory_only': True, 'production_release_authorized': False,
            'point_layout': 'u64_f32x3_u8x32_u32_le_v1', 'point_stride': ctypes.sizeof(_CMapAppearancePoint),
            'points_base64': base64.b64encode(bytes(points)[:header.point_count * ctypes.sizeof(_CMapAppearancePoint)]).decode('ascii')}

    def get_map_event_frame(self, snapshot, frame_ns):
        """Return diagnostic partial events bound to this frame, never continuity proof."""
        if self._closed or not self._handle:raise RuntimeError('Native VIO bridge is closed')
        fn = getattr(self._dll, 'egohand_vio_get_map_event_frame', None)
        if fn is None:return None
        expected = ctypes.sizeof(_CMapEventFrame)
        if int(self._dll.egohand_vio_map_event_frame_size()) != expected:
            raise RuntimeError('VIO 地图事件 ABI 不匹配')
        value = _CMapEventFrame()
        rc = fn(self._handle, ctypes.byref(value), expected)
        if rc == 1:return {'coverage':'unknown', 'frame_bound':False, 'events':[], 'reason':'snapshot_unavailable'}
        if rc != 0:raise RuntimeError(self._last_error())
        if value.schema_version != 1 or value.struct_size != expected or value.event_count > 16:
            raise RuntimeError('VIO 地图事件版本、大小或数量无效')
        bound = (bool(value.frame_bound) and bool(value.snapshot_consistent)
                 and snapshot is not None and snapshot.get('snapshot_consistent') is True
                 and snapshot.get('frame_id') == int(value.frame_id)
                 and snapshot.get('timestamp_ns') == int(frame_ns) == int(value.frame_timestamp_ns)
                 and snapshot.get('map_id') == int(value.map_id)
                 and snapshot.get('reset_id') == int(value.reset_id))
        if not bound:
            return {'coverage':'unknown', 'frame_bound':False, 'events':[], 'reason':'frame_mismatch_or_unverified'}
        events=[]
        for event in value.events[:value.event_count]:
            numbers=[float(event.scale), float(event.source_keyframe_time_sec),
                     *map(float,event.rotation), *map(float,event.translation)]
            if not all(math.isfinite(x) for x in numbers):
                raise RuntimeError('VIO 地图事件含非有限数值')
            events.append({'sequence':int(event.sequence), 'map_id':int(event.map_id),
                'source_keyframe_id':int(event.source_keyframe_id), 'source_keyframe_time_sec':float(event.source_keyframe_time_sec),
                'revision_before':int(event.revision_before), 'revision_after':int(event.revision_after),
                'change_before':int(event.change_before), 'change_after':int(event.change_after),
                'type':int(event.type), 'scale':float(event.scale),
                'rotation':list(map(float,event.rotation)), 'translation':list(map(float,event.translation)),
                'map_update_lock_matches_caller':bool(event.map_update_lock_matches_caller),
                'reset_generation_known':bool(event.reset_generation_known),
                'commit_timestamp_ns':None,
                'source_time_semantics':'keyframe_relative_seconds_not_commit_time',
                'association':'historical_ring_entry_not_a_frame_event'})
        return {'coverage':'partial' if value.coverage == 1 else 'unknown', 'frame_bound':True,
                'frame_bound_scope':'ring_copy_only_not_event_commit',
                'frame_id':int(value.frame_id), 'timestamp_ns':int(value.frame_timestamp_ns),
                'map_id':int(value.map_id), 'reset_id':int(value.reset_id),
                'reset_generation_verified':False,
                'overwritten_count':int(value.overwritten_count), 'events':events,
                'coordinate_continuity_verified':False}

    def get_reference_frame(self, snapshot):
        """Read a successful initializer's reference identity; never a recovery release."""
        if self._closed or not self._handle:raise RuntimeError('Native VIO bridge is closed')
        fn = getattr(self._dll, 'egohand_vio_get_reference_frame', None)
        if fn is None:return None
        size_fn = getattr(self._dll, 'egohand_vio_reference_frame_size', None)
        if size_fn is None:raise RuntimeError('VIO reference ABI size query missing')
        size_fn.argtypes = [];size_fn.restype = ctypes.c_int32
        fn.argtypes = [ctypes.c_void_p, ctypes.POINTER(_CReferenceFrame), ctypes.c_int32]
        fn.restype = ctypes.c_int
        expected = ctypes.sizeof(_CReferenceFrame)
        if int(size_fn()) != expected:raise RuntimeError('VIO reference ABI size mismatch')
        value = _CReferenceFrame();rc = fn(self._handle, ctypes.byref(value), expected)
        if rc == 1:return {'frame_bound':False, 'reason':'snapshot_unavailable'}
        if rc != 0:raise RuntimeError('VIO reference getter failed')
        if value.schema_version != 1 or value.struct_size != expected:
            raise RuntimeError('VIO reference ABI version mismatch')
        fields = ('frame_id', 'map_id', 'reset_id', 'map_revision', 'map_change_index', 'timestamp_ns')
        if (value.frame_bound != 1 or value.snapshot_consistent != 1
                or snapshot.get('snapshot_consistent') is not True
                or any(type(snapshot.get(k)) is not int or snapshot[k] != int(getattr(value,k)) for k in fields)):
            return {'frame_bound':False, 'reason':'frame_mismatch'}
        if (value.generation == 0) != (value.operation_sequence == 0):
            raise RuntimeError('VIO reference generation/operation mismatch')
        return {**{name:int(getattr(value,name)) for name,_ in value._fields_},
                'snapshot_consistent':True, 'frame_bound':True,
                'basis':'successful_initializer_commit', 'coordinate_continuity_verified':False}

    def get_frame_snapshot(self):
        """Read the immutable cache published under Track's map lock.

        None means legacy DLL capability unavailable, not proven consistency.
        No map/body getters are invoked to assemble this result.
        """
        if self._closed or not self._handle:raise RuntimeError('Native VIO bridge is closed')
        fn = getattr(self._dll, 'egohand_vio_get_frame_snapshot', None)
        if fn is None:return None
        expected = ctypes.sizeof(_CFrameSnapshot)
        if int(self._dll.egohand_vio_snapshot_size()) != expected:
            raise RuntimeError('VIO 原子快照 ABI 不匹配，请重新构建桥接库')
        value = _CFrameSnapshot()
        rc = fn(self._handle, ctypes.byref(value), expected)
        if rc == 1:return {'snapshot_available':False, 'snapshot_consistent':False}
        if rc != 0:raise RuntimeError(self._last_error())
        if value.schema_version != 1 or value.struct_size != expected:
            raise RuntimeError('VIO 原子快照版本或结构体大小不匹配')
        out = {key:int(getattr(value,key)) for key in ('frame_id','map_id','map_revision','reset_id',
            'world_frame_id','map_change_index','timestamp_ns','left_features','right_features','stereo_valid_depths','tracked_inliers')}
        out.update(snapshot_available=True, snapshot_consistent=bool(value.snapshot_consistent),
                   ba1_complete=bool(value.ba1_complete), ba2_complete=bool(value.ba2_complete))
        out['body_state'] = None
        if value.snapshot_consistent and value.body_valid:
            transform = np.asarray(value.T_world_from_body, dtype=float).reshape(4,4)
            velocity = np.asarray(value.velocity_world_mps, dtype=float)
            bias = np.asarray(value.accel_gyro_bias, dtype=float)
            if not all(np.isfinite(a).all() for a in (transform,velocity,bias)):
                raise RuntimeError('原子快照惯性体状态包含非有限值')
            out['body_state'] = {'T_world_from_body':transform,'velocity_world_mps':velocity,
                'accel_bias_mps2':bias[:3], 'gyro_bias_rps':bias[3:], 'ba2_complete':bool(value.ba2_complete),
                'frame_id':int(value.frame_id),'timestamp_ns':int(value.timestamp_ns),
                'world_frame_id':int(value.world_frame_id),'snapshot_consistent':True}
        reference = self.get_reference_frame(out)
        if reference is not None:
            out['reference_frame'] = reference
            out['reference_generation'] = reference.get('generation') if reference.get('frame_bound') else None
        return out


    def get_optimization_counters(self):
        """读取可选的原生计数器（远点门限拒绝数、止血丢弃数）。

        这两个符号是增量导出：旧 DLL 没有时返回 None，而不是 0，
        以免把"库不支持"误读成"没有拒绝任何点"。
        """
        if self._closed or not self._handle:raise RuntimeError('Native VIO bridge is closed')
        result={}
        for key,name in (('depth_gate_rejected','egohand_vio_depth_gate_rejected'),
                         ('point_clamp_rejected','egohand_vio_point_clamp_count'),
                         ('entry_pruned','egohand_vio_entry_pruned'),
                         ('invalid_depth_checks','egohand_vio_invalid_depth_checks'),
                         ('invalid_depth_removed','egohand_vio_invalid_depth_removed'),
                         ('invalid_depth_points','egohand_vio_invalid_depth_points')):
            fn=getattr(self._dll,name,None)
            if fn is None:
                result[key]=None
                continue
            fn.restype=ctypes.c_uint64
            result[key]=int(fn())
        return result
    def get_world_frame_info(self):
        """读取世界系修订号：整图重锚次数与最近一次 SE(3) 拟合结果。

        `None` 只表示这个 DLL 没有该扩展（旧库），不代表“没有发生重锚”。
        判据在原生侧：同 ID 关键帧优化前后的整体拟合残差小但位移大 ⇒ 重锚；
        因此 revision 递增与 `map_epoch` 是否变化无关。
        """
        if self._closed or not self._handle:raise RuntimeError('Native VIO bridge is closed')
        fn=getattr(self._dll,'egohand_vio_get_world_frame_info',None)
        if fn is None:return None
        expected=ctypes.sizeof(_CWorldFrameInfo)
        if int(self._dll.egohand_vio_world_frame_info_size())!=expected:
            raise RuntimeError('世界系修订号 ABI 不匹配，请重新构建桥接库')
        value=_CWorldFrameInfo()
        if fn(self._handle,ctypes.byref(value),expected)!=0:raise RuntimeError(self._last_error())
        if value.schema_version!=1 or value.struct_size!=expected:
            raise RuntimeError('世界系修订号结构版本或大小不匹配')
        return {'revision':int(value.revision),'detections':int(value.detections),
                'evaluations':int(value.evaluations),'rejected_nonrigid':int(value.rejected_nonrigid),
                'verdict':_WORLD_FRAME_VERDICTS.get(int(value.verdict),'unknown'),
                'last_keyframes':int(value.last_keyframes),'map_id':int(value.map_id),
                'last_max_keyframe_id':int(value.last_max_keyframe_id),
                'last_timestamp_sec':float(value.last_timestamp_sec),
                'last_translation_m':float(value.last_translation_m),
                'last_rotation_deg':float(value.last_rotation_deg),
                'last_scale':float(value.last_scale),
                'last_residual_p50_m':float(value.last_residual_p50_m),
                'last_residual_p95_m':float(value.last_residual_p95_m),
                'last_residual_p99_m':float(value.last_residual_p99_m),
                'last_residual_max_m':float(value.last_residual_max_m),
                'last_residual_limit_m':float(value.last_residual_limit_m)}

    def get_candidate_body_state(self):
        """读取 ORB 最近的惯性体状态；候选不等同最后可信锚点。"""
        if self._closed or not self._handle:raise RuntimeError('Native VIO bridge is closed')
        fn=getattr(self._dll,'egohand_vio_get_body_state',None)
        if fn is None:
            return None  # Older bridges still provide camera pose and tracking status.
        T=(ctypes.c_float*16)();velocity=(ctypes.c_float*3)();bias=(ctypes.c_float*6)();ba2=ctypes.c_int()
        rc=fn(self._handle,T,velocity,bias,ctypes.byref(ba2))
        if rc==1:return None
        if rc!=0:raise RuntimeError(self._last_error())
        return {'T_world_from_body':np.array(T,dtype=float).reshape(4,4),
                'velocity_world_mps':np.array(velocity,dtype=float),
                'accel_bias_mps2':np.array(bias[:3],dtype=float),
                'gyro_bias_rps':np.array(bias[3:],dtype=float),'ba2_complete':bool(ba2.value)}

    def set_recovery_callback(self, callback):
        """注册同步候选检查：1 一致、0 拒绝、-1 无法判断。"""
        if self._closed or not self._handle:raise RuntimeError('Native VIO bridge is closed')
        fn=getattr(self._dll,'egohand_vio_set_recovery_callback',None)
        if fn is None:raise RuntimeError('VIO 原生桥缺少重定位候选接口，请重新构建')
        if callback is None:wrapped=_RecoveryCallback()
        else:
            @_RecoveryCallback
            def wrapped(matrix,epoch,context):
                try:
                    value=np.ctypeslib.as_array(matrix,shape=(16,)).astype(float).reshape(4,4)
                    decision=int(callback(value,int(epoch)))
                    return decision if decision in (-1,0,1) else -1
                except Exception as exc:
                    # 异常不得穿过 C ABI；无法判断时不冒充已通过检查。
                    self._recovery_callback_error=f'{type(exc).__name__}: {exc}'
                    return -1
        rc=fn(self._handle,wrapped,None)
        if rc!=0:raise RuntimeError(self._last_error())
        self._recovery_callback_ref=wrapped if callback is not None else None

    def get_short_recovery_report(self,native_pose=None):
        """Read this thread's candidate/returned-frame geometry from an opt-in DLL."""
        fn=getattr(self._dll,'egohand_vio_short_recovery_report',None)
        if fn is None:return None
        fn.argtypes=[];fn.restype=ctypes.c_char_p
        raw=fn()
        if not raw or len(raw)>8192:return None
        value=json.loads(raw.decode('utf8'))
        if value.get('enabled') is not True:return None
        stamp=getattr(self,'_tracking_call_timestamp_ns',None)
        origin=getattr(self,'_tracking_origin_ns',None)
        relative=value.get('timestamp_sec')
        if (type(stamp) is not int or type(origin) is not int or
            type(relative) not in (int,float) or not math.isfinite(relative) or
            abs(relative-(stamp-origin)/1e9)>2e-6):return None
        bound=False
        if native_pose is not None:
            try:
                observed=np.asarray(value['T_world_from_camera'],dtype=float).reshape(4,4)
                bound=bool(np.isfinite(observed).all() and np.allclose(observed,native_pose,atol=2e-5,rtol=1e-5))
            except (KeyError,TypeError,ValueError):pass
        return {**value,'timestamp_ns':stamp,'bridge_instance_id':self._bridge_instance_id,
                'pose_binding_verified':bound}

    def _last_error(self) -> str:
        value = self._dll.egohand_vio_last_error()
        return value.decode("utf-8", "replace") if value else "unknown native error"

    @staticmethod
    def _prepare_image(image: np.ndarray, name: str) -> np.ndarray:
        if not isinstance(image, np.ndarray):
            raise TypeError(f"{name} must be a numpy.ndarray")
        if image.dtype != np.uint8 or image.ndim != 2:
            raise ValueError(f"{name} must be a 2-D uint8 grayscale image")
        if image.size == 0:
            raise ValueError(f"{name} is empty")
        return np.ascontiguousarray(image)

    @staticmethod
    def _prepare_imu(samples: Sequence[ImuMeasurement]):
        if not samples:
            return None, 0
        array_type = _CImuSample * len(samples)
        c_array = array_type()
        previous = None
        for i, sample in enumerate(samples):
            stamp = int(sample.timestamp_ns)
            if previous is not None and stamp <= previous:
                raise ValueError("IMU timestamps must be strictly increasing")
            values = tuple(map(float, (*sample.acceleration_mps2, *sample.angular_velocity_rps)))
            if len(values) != 6 or not all(map(math.isfinite, values)):
                raise ValueError("IMU sample must contain six finite SI values")
            c_array[i] = _CImuSample(stamp, *values)
            previous = stamp
        return c_array, len(samples)

    def track_stereo(
        self,
        left_gray: np.ndarray,
        right_gray: np.ndarray,
        frame_timestamp_ns: int,
        imu_samples: Sequence[ImuMeasurement] = (),
    ) -> VioFrameResult:
        if self._closed or not self._handle:
            raise RuntimeError("Native VIO bridge is closed")
        left = self._prepare_image(left_gray, "left_gray")
        right = self._prepare_image(right_gray, "right_gray")
        if left.shape != right.shape:
            raise ValueError("left and right image shapes must match")
        frame_ns = int(frame_timestamp_ns)
        if frame_ns <= 0:
            raise ValueError("frame_timestamp_ns must be a positive int64 nanosecond timestamp")
        if self._last_frame_timestamp_ns is not None:
            if frame_ns <= self._last_frame_timestamp_ns:
                raise ValueError("camera timestamps must be strictly increasing")
            for sample in imu_samples:
                if not (self._last_frame_timestamp_ns < int(sample.timestamp_ns) <= frame_ns):
                    raise ValueError(
                        "IMU samples for frame k must be in "
                        "(previous_camera_timestamp, current_camera_timestamp]"
                    )
        elif imu_samples:
            raise ValueError("first stereo frame must be submitted without IMU samples")

        c_imu, count = self._prepare_imu(imu_samples)
        imu_ptr = (
            ctypes.cast(c_imu, ctypes.POINTER(_CImuSample))
            if c_imu is not None else ctypes.POINTER(_CImuSample)()
        )
        left_ptr = left.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
        right_ptr = right.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
        result = _CResult()
        if getattr(self,'_tracking_origin_ns',None) is None:self._tracking_origin_ns=frame_ns
        self._tracking_call_timestamp_ns=frame_ns
        rc = self._dll.egohand_vio_track_stereo(
            self._handle, left_ptr, right_ptr,
            int(left.shape[1]), int(left.shape[0]), int(left.strides[0]),
            ctypes.c_int64(frame_ns), imu_ptr, count, ctypes.byref(result),
        )
        if rc != 0:
            raise RuntimeError(self._last_error())
        self._last_frame_timestamp_ns = frame_ns

        snapshot = self.get_frame_snapshot()
        if snapshot is not None and snapshot.get('snapshot_available'):
            if snapshot['timestamp_ns'] != frame_ns or snapshot['world_frame_id'] != int(result.map_epoch):
                raise RuntimeError('原子快照与当前位姿的帧或地图版本不一致')
        map_event_frame = self.get_map_event_frame(snapshot, frame_ns)

        pose = None
        if result.pose_valid:
            pose = np.asarray(
                list(result.T_world_from_rectified_left_camera),
                dtype=np.float32,
            ).reshape(4, 4).copy()
        # 世界系修订号：整图重锚计数，与 map_epoch 独立（epoch 在这次重锚里不变）。
        world_frame = self.get_world_frame_info()
        self._input_sequence = getattr(self, '_input_sequence', 0) + 1
        appearance = None
        if self._input_sequence in getattr(self, '_appearance_frames', ()):
            appearance = self.get_map_appearance(snapshot, frame_ns)
            appearance.update(bridge_instance_id=self._bridge_instance_id,
                settings_sha256=self._appearance_settings_sha256, input_sequence=self._input_sequence,
                input_timestamp_ns=frame_ns, input_image_size=[int(left.shape[1]), int(left.shape[0])],
                left_sha256=hashlib.sha256(left.tobytes()).hexdigest(),
                right_sha256=hashlib.sha256(right.tobytes()).hexdigest(),
                native_T_map_from_camera=pose.tolist() if pose is not None else None)
            if appearance['status'] == 'copied' and [appearance['image_width'], appearance['image_height']] != appearance['input_image_size']:
                raise RuntimeError('map appearance input dimensions mismatch')
        return VioFrameResult(
            timestamp_ns=frame_ns,
            tracking_state=int(result.tracking_state),
            tracking_state_name=_STATE_NAMES.get(
                int(result.tracking_state), "UNKNOWN"
            ),
            pose_valid=bool(result.pose_valid),
            is_lost=bool(result.is_lost),
            imu_initialized=bool(result.imu_initialized),
            imu_init_elapsed_sec=float(result.imu_init_elapsed_sec),
            T_world_from_rectified_left_camera=pose,
            map_epoch=int(result.map_epoch),
            inertial_scale_finalized=bool(result.inertial_scale_finalized),
            frame_snapshot=snapshot,
            world_frame_revision=int(world_frame['revision']) if world_frame else 0,
            world_frame_info=world_frame,
            map_event_frame=map_event_frame,
            map_appearance=appearance,
            recovery_observation=self.get_short_recovery_report(pose),
        )

    def _map_function(self, name, argtypes):
        try:
            function = getattr(self._dll, f"egohand_vio_{name}")
        except AttributeError as exc:
            raise RuntimeError("请先运行 scripts/build_vio_core_patch.cmd 构建地图功能") from exc
        function.argtypes = argtypes
        function.restype = ctypes.c_int
        return function

    def export_map(self, atlas_path: Path, pose_path: Path) -> None:
        export = self._map_function(
            "export_map", [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p],
        )
        if export(self._handle, os.fsencode(atlas_path), os.fsencode(pose_path)) != 0:
            raise RuntimeError(self._last_error())

    def start_new_map(self) -> None:
        if self._closed or not self._handle:raise RuntimeError('Native VIO bridge is closed')
        fn=getattr(self._dll,'egohand_vio_start_new_map',None)
        if fn is None:raise RuntimeError('当前 VIO 桥接库不支持自动新建地图')
        if fn(self._handle)!=0:raise RuntimeError(self._last_error())

    def reset(self) -> None:
        if self._closed or not self._handle:
            raise RuntimeError("Native VIO bridge is closed")
        if self._dll.egohand_vio_reset(self._handle) != 0:
            raise RuntimeError(self._last_error())
        self._last_frame_timestamp_ns = None
        self._tracking_origin_ns = None

    def close(self) -> None:
        if self._closed:
            return
        if self._handle:
            rc = self._dll.egohand_vio_shutdown(self._handle)
            if rc != 0:
                error = self._last_error()
                self._dll.egohand_vio_destroy(self._handle)
                self._handle = ctypes.c_void_p()
                self._closed = True
                raise RuntimeError(error)
            self._dll.egohand_vio_destroy(self._handle)
            self._handle = ctypes.c_void_p()
        self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self.close()
        except Exception as cleanup_error:
            if exc is None:
                raise
            exc.add_note(f"VIO cleanup also failed: {cleanup_error}")
