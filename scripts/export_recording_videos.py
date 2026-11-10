"""把 vendor_vs030 录制导出为左右 MP4 预览、IMU CSV、帧表与 export.json。

用法：
    python scripts/export_recording_videos.py --recording <dir> [--output <dir>]
        [--scale 0.5] [--fps 30] [--limit N]

实现要点：
- 解码/配对完全复用 egohand.storage.sdk_recording.iter_paired_frames（与离线回放一致）；
- 编码器优先 OpenCV mp4v，失败时回退 PyAV libx264，最终用哪个写进 export.json；
- 默认输出 <recording>/export/，目录已存在则拒绝覆盖；
- 不依赖 GUI，不做 VIO。
"""
import argparse
import csv
import json
import math
from fractions import Fraction
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from egohand.storage.sdk_recording import (  # noqa: E402
    iter_imu_rows,
    iter_paired_frames,
)

IMU_CSV_FIELDS = ("time_a_ns", "time_b_ns", "ax", "ay", "az", "gx", "gy", "gz")
FRAME_CSV_FIELDS = (
    "index", "sequence", "device_ts",
    "capture_ts_left_ns", "capture_ts_right_ns",
    "arrival_ts_left_ns", "arrival_ts_right_ns",
)


class _OpenCvVideoWriter:
    encoder = "opencv-mp4v"

    def __init__(self, path, fps, size):
        import cv2

        self._cv2 = cv2
        self._writer = cv2.VideoWriter(
            str(path), cv2.VideoWriter_fourcc(*"mp4v"), float(fps),
            (int(size[0]), int(size[1])),
        )
        if not self._writer.isOpened():
            self._writer.release()
            raise RuntimeError("OpenCV mp4v 编码器不可用")

    def write(self, frame):
        self._writer.write(frame)

    def close(self):
        self._writer.release()


class _PyAvVideoWriter:
    encoder = "pyav-libx264"

    def __init__(self, path, fps, size):
        import av

        self._av = av
        self._fps = Fraction(str(fps)).limit_denominator(1_000_000)
        self._time_base = Fraction(1, 1) / self._fps
        self._container = av.open(str(path), "w")
        self._stream = self._container.add_stream("libx264", rate=self._fps)
        self._stream.width = int(size[0])
        self._stream.height = int(size[1])
        self._stream.pix_fmt = "yuv420p"
        self._count = 0

    def write(self, frame):
        video_frame = self._av.VideoFrame.from_ndarray(frame, format="bgr24")
        video_frame.pts = self._count
        video_frame.time_base = self._time_base
        self._count += 1
        for packet in self._stream.encode(video_frame):
            self._container.mux(packet)

    def close(self):
        for packet in self._stream.encode(None):
            self._container.mux(packet)
        self._container.close()


def _open_video_writer(path, fps, size):
    for factory in (_OpenCvVideoWriter, _PyAvVideoWriter):
        try:
            return factory(path, fps, size)
        except Exception:  # noqa: BLE001 - 换一种可用编码器继续
            continue
    raise RuntimeError("没有可用的 MP4 编码器（OpenCV mp4v / PyAV libx264 均失败）")


def _target_size(width, height, scale):
    scale = float(scale)
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError(f"--scale 必须为正数：{scale}")
    target_width = max(2, int(round(int(width) * scale)) // 2 * 2)
    target_height = max(2, int(round(int(height) * scale)) // 2 * 2)
    return target_width, target_height


def _resize(frame, size):
    import cv2

    if frame.shape[1] == size[0] and frame.shape[0] == size[1]:
        return frame
    return cv2.resize(frame, (int(size[0]), int(size[1])), interpolation=cv2.INTER_AREA)


def _write_imu_csv(path, recording):
    """写 imu.csv；返回 (行数, 警告列表)。IMU 解析失败不阻止视频导出。"""
    warnings = []
    rows = 0
    with path.open("w", encoding="utf-8", newline="") as file_obj:
        writer = csv.writer(file_obj)
        writer.writerow(IMU_CSV_FIELDS)
        try:
            for time_a, time_b, values in iter_imu_rows(recording):
                writer.writerow((time_a, time_b, *values))
                rows += 1
        except (ValueError, OSError) as exc:
            warnings.append(f"IMU 导出失败：{exc}")
    return rows, warnings


def _file_size(path):
    try:
        return path.stat().st_size
    except OSError:
        return None


def export_recording(recording, output=None, scale=1.0, fps=30.0, limit=None):
    """导出录制目录，返回 export.json 内容（同时写入磁盘）。输出目录已存在时拒绝覆盖。"""
    recording = Path(recording).resolve()
    if not recording.is_dir():
        raise FileNotFoundError(f"录制目录不存在：{recording}")
    fps = float(fps)
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError(f"--fps 必须为正数：{fps}")
    if limit is not None:
        limit = int(limit)
        if limit <= 0:
            raise ValueError(f"--limit 必须为正数：{limit}")
    out = Path(output).resolve() if output else recording / "export"
    if out.exists():
        raise FileExistsError(f"输出目录已存在，拒绝覆盖：{out}")
    out.mkdir(parents=True)

    warnings = []
    errors = []
    imu_rows, imu_warnings = _write_imu_csv(out / "imu.csv", recording)
    warnings.extend(imu_warnings)

    frame_count = 0
    encoder = None
    left_writer = right_writer = None
    with (out / "frames.csv").open("w", encoding="utf-8", newline="") as file_obj:
        csv_writer = csv.writer(file_obj)
        csv_writer.writerow(FRAME_CSV_FIELDS)
        try:
            for frame in iter_paired_frames(recording, limit=limit):
                if frame_count == 0:
                    size = _target_size(frame.left_bgr.shape[1], frame.left_bgr.shape[0], scale)
                    left_writer = _open_video_writer(out / "left.mp4", fps, size)
                    right_writer = _open_video_writer(out / "right.mp4", fps, size)
                    encoder = left_writer.encoder
                left = _resize(frame.left_bgr, size)
                right = _resize(frame.right_bgr, size)
                left_writer.write(left)
                right_writer.write(right)
                csv_writer.writerow((
                    frame.index, frame.sequence, frame.device_ts,
                    frame.capture_ts_left_ns, frame.capture_ts_right_ns,
                    frame.arrival_ts_left_ns, frame.arrival_ts_right_ns,
                ))
                frame_count += 1
        except (FileNotFoundError, OSError, RuntimeError, ValueError) as exc:
            errors.append(f"视频导出失败：{exc}")
        finally:
            for writer in (left_writer, right_writer):
                if writer is not None:
                    try:
                        writer.close()
                    except Exception as exc:  # noqa: BLE001 - 收尾失败只记录
                        errors.append(f"编码器收尾失败：{exc}")

    if frame_count == 0 and not errors:
        errors.append("没有解码出可用的双目帧，请先用回放校验确认录制可回放")
    report = {
        "status": "completed" if frame_count and not errors else "failed",
        "recording": str(recording),
        "output": str(out),
        "encoder": encoder,
        "fps": fps,
        "scale": float(scale),
        "limit": limit,
        "frames": frame_count,
        "duration_s": round(frame_count / fps, 6),
        "imu_rows": imu_rows,
        "files": {
            "left_mp4": "left.mp4",
            "right_mp4": "right.mp4",
            "imu_csv": "imu.csv",
            "frames_csv": "frames.csv",
            "left_mp4_bytes": _file_size(out / "left.mp4"),
            "right_mp4_bytes": _file_size(out / "right.mp4"),
            "imu_csv_bytes": _file_size(out / "imu.csv"),
            "frames_csv_bytes": _file_size(out / "frames.csv"),
        },
        "warnings": warnings,
        "errors": errors,
        "generated_time_ns": time.time_ns(),
        "note": "MP4 仅用于人工预览；离线 VIO 回放直接读原始 H.265/IMU 记录，不依赖本导出。",
    }
    (out / "export.json").write_text(
        json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2),
        encoding="utf-8",
    )
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="导出 VS030 录制为左右 MP4 + IMU CSV")
    parser.add_argument("--recording", required=True, help="录制目录（含 manifest.json）")
    parser.add_argument("--output", default=None, help="输出目录，默认 <recording>/export/")
    parser.add_argument("--scale", type=float, default=1.0, help="预览缩放比例，如 0.5")
    parser.add_argument("--fps", type=float, default=30.0, help="MP4 播放帧率（名义值）")
    parser.add_argument("--limit", type=int, default=None, help="只导出前 N 对帧")
    args = parser.parse_args(argv)
    try:
        report = export_recording(
            args.recording, output=args.output, scale=args.scale,
            fps=args.fps, limit=args.limit,
        )
    except FileExistsError as exc:
        print(f"失败：{exc}", file=sys.stderr)
        return 2
    except (FileNotFoundError, ValueError) as exc:
        print(f"失败：{exc}", file=sys.stderr)
        return 2
    if report["status"] != "completed":
        print(f"导出未完成：{'; '.join(report['errors'])}", file=sys.stderr)
    for warning in report["warnings"]:
        print(f"警告：{warning}", file=sys.stderr)
    print(
        f"{report['status']}：{report['frames']} 对帧，IMU {report['imu_rows']} 行，"
        f"编码器 {report['encoder']}，输出 {report['output']}"
    )
    print(f"报告：{Path(report['output']) / 'export.json'}")
    return 0 if report["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
