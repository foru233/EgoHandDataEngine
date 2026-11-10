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
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np


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
        ("imu_init_elapsed_sec", ctypes.c_double),
        ("frame_timestamp_sec", ctypes.c_double),
        ("map_epoch", ctypes.c_uint64),
        ("T_world_from_rectified_left_camera", ctypes.c_float * 16),
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


_STATE_NAMES = {
    -1: "SYSTEM_NOT_READY",
    0: "NO_IMAGES_YET",
    1: "NOT_INITIALIZED",
    2: "OK",
    3: "RECENTLY_LOST",
    4: "LOST",
    5: "OK_KLT",
}


class NativeVioBridge:
    def __init__(
        self,
        vocabulary_path: str | Path,
        settings_path: str | Path,
        library_path: str | Path | None = None,
        conda_prefix: str | Path | None = None,
        use_viewer: bool = False,
    ):
        self.vocabulary_path = Path(vocabulary_path).resolve()
        self.settings_path = Path(settings_path).resolve()
        if not self.vocabulary_path.is_file():
            raise FileNotFoundError(self.vocabulary_path)
        if not self.settings_path.is_file():
            raise FileNotFoundError(self.settings_path)

        native_dir = Path(__file__).resolve().parent / "native"
        if library_path is None:
            library_path = native_dir / "bin" / "egohand_vio_bridge.dll"
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
        self._handle = ctypes.c_void_p()
        self._last_frame_timestamp_ns: int | None = None
        self._closed = False

        rc = self._dll.egohand_vio_create(
            os.fsencode(self.vocabulary_path),
            os.fsencode(self.settings_path),
            int(bool(use_viewer)),
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
        rc = self._dll.egohand_vio_track_stereo(
            self._handle, left_ptr, right_ptr,
            int(left.shape[1]), int(left.shape[0]), int(left.strides[0]),
            ctypes.c_int64(frame_ns), imu_ptr, count, ctypes.byref(result),
        )
        if rc != 0:
            raise RuntimeError(self._last_error())
        self._last_frame_timestamp_ns = frame_ns

        pose = None
        if result.pose_valid:
            pose = np.asarray(
                list(result.T_world_from_rectified_left_camera),
                dtype=np.float32,
            ).reshape(4, 4).copy()
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
        )

    def reset(self) -> None:
        if self._closed or not self._handle:
            raise RuntimeError("Native VIO bridge is closed")
        if self._dll.egohand_vio_reset(self._handle) != 0:
            raise RuntimeError(self._last_error())
        self._last_frame_timestamp_ns = None

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
        self.close()
