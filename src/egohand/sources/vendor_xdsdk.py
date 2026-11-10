"""新版 SDK 双目采集：原始 IPC 接收/录制与预览解码分线程，预览过载不丢原始数据。"""
import logging
import os
from pathlib import Path
import queue
import re
import subprocess
import struct
import threading
import time

from .vendor_placeholder import VendorStereoSource, ROOT, CALIBRATION_PATH, KALIBR_LEFT_CAMERA
from ..calibration.loader import load_calibration
from ..core.types import StereoFrame, SourceMetadata

WORKER = ROOT / "build/xdsdk_probe/xdsdk_stream_worker.exe"
DISCOVERY = ROOT / "build/xdsdk_probe/probe_discovery.exe"
SDK_DIR = ROOT / "SDK/delivery/xdclient"
RECORD = struct.Struct("<IHHIIQQQQ")
RECORD_VERSION = 2
MAGIC = 0x58444652
MAX_PAYLOAD = 8 * 1024 * 1024
PREVIEW_PACKET_QUEUE_SIZE = 128
DECODER_METADATA_LIMIT = 32
FIRST_PREVIEW_TIMEOUT_S = 6.0


def read_exact(stream, size):
    data = bytearray()
    while len(data) < size:
        chunk = stream.read(size - len(data))
        if not chunk:
            raise EOFError("SDK 数据流意外结束")
        data.extend(chunk)
    return bytes(data)


class XdSdkStereoSource(VendorStereoSource):
    """原始 SDK 包优先接收；GUI 解码仅消费独立的有界预览队列。"""

    def __init__(self):
        super().__init__()
        self.process = None
        self.imu_vio_queue = None
        self.imu_vio_error = None
        self.sdk_diagnostics = ""
        self.sdk_record_callback = None

        self.reader_thread = None
        self.decoder_thread = None
        self.reader_done = threading.Event()
        self.decoder_done = threading.Event()
        self.read_interrupt = threading.Event()
        self.preview_ready = threading.Event()
        self.decode_queue = queue.Queue(maxsize=PREVIEW_PACKET_QUEUE_SIZE)
        self._decode_lock = threading.Lock()
        self._decode_generation = 0
        self._imu_lock = threading.Lock()
        self._imu_packets_pending = []
        self._stats_lock = threading.Lock()
        self.capture_stats = {
            "sdk_packets_received": 0,
            "sdk_packets_by_sink": {},
            "sdk_payload_bytes": 0,
            "preview_decode_packets_dropped": 0,
            "preview_decode_queue_resets": 0,
            "preview_decoded_frames": {"4": 0, "5": 0},
            "preview_output_frames_dropped": 0,
            "preview_decode_errors": 0,
            "preview_pending_pair_drops": 0,
            "imu_preview_metadata_drops": 0,
            "imu_vio_queue_drops": 0,
        }
        self.preview_error = None
        self.callback_error = None

    def enable_imu_capture(self, max_packets=8192, *, allow_running=False):
        """启用独立有界 IMU 队列。队列过载只停止实时 VIO，不停止原始 SDK 录制。"""
        if self.running and not allow_running:
            raise RuntimeError("须在开始采集前启用 IMU 通道")
        if max_packets < 1:
            raise ValueError("IMU 队列容量必须为正数")
        self.imu_vio_error = None
        self.imu_vio_queue = queue.Queue(maxsize=max_packets)

    def open(self):
        import av  # noqa: F401

        for path in (WORKER, DISCOVERY, SDK_DIR / "xdsdk.dll"):
            if not path.is_file():
                raise RuntimeError(f"新版相机 SDK 运行文件不存在：{path}")
        self.calibration = (
            load_calibration(CALIBRATION_PATH, kalibr_left_camera=KALIBR_LEFT_CAMERA)
            if CALIBRATION_PATH.is_file() else None
        )
        env = os.environ.copy()
        env["PATH"] = str(SDK_DIR) + os.pathsep + env.get("PATH", "")
        try:
            result = subprocess.run(
                [str(DISCOVERY)], capture_output=True, text=True, timeout=10, env=env,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            )
        except subprocess.TimeoutExpired as exc:
            raise TimeoutError("VS030 设备发现超时") from exc
        addresses = list(dict.fromkeys(re.findall(r"address=(\d+\.\d+\.\d+\.\d+)", result.stdout)))
        usb_addresses = [address for address in addresses if address.startswith("169.254.")]
        if len(usb_addresses) == 1:
            addresses = usb_addresses
        if not addresses:
            raise TimeoutError("新版 SDK 未发现 VS030，请检查设备连接或关闭其他客户端")
        if result.returncode or len(addresses) != 1:
            raise RuntimeError(
                f"新版 SDK 需发现一台相机，当前发现 {len(addresses)} 台；请检查连接或关闭其他客户端"
            )

        self.camera_ip = addresses[0]
        self.index = 0
        self.error = None
        self.preview_error = None
        self.callback_error = None
        self.imu_vio_error = None
        self.sdk_diagnostics = ""
        self.stop_event.clear()
        self.read_interrupt.clear()
        self.preview_ready.clear()
        self.reader_done.clear()
        self.decoder_done.clear()
        self.running = False
        self.capture_stats = {
            "sdk_packets_received": 0,
            "sdk_packets_by_sink": {},
            "sdk_payload_bytes": 0,
            "preview_decode_packets_dropped": 0,
            "preview_decode_queue_resets": 0,
            "preview_decoded_frames": {"4": 0, "5": 0},
            "preview_output_frames_dropped": 0,
            "preview_decode_errors": 0,
            "preview_pending_pair_drops": 0,
            "imu_preview_metadata_drops": 0,
            "imu_vio_queue_drops": 0,
        }
        self._decode_generation = 0
        self._imu_packets_pending.clear()
        while True:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                break
        while True:
            try:
                self.decode_queue.get_nowait()
            except queue.Empty:
                break
        if self.imu_vio_queue is not None:
            while True:
                try:
                    self.imu_vio_queue.get_nowait()
                except queue.Empty:
                    break

    def start(self):
        env = os.environ.copy()
        env["PATH"] = str(SDK_DIR) + os.pathsep + env.get("PATH", "")
        self.process = subprocess.Popen(
            [str(WORKER), self.camera_ip],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            env=env, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
        self.running = True
        # 先启动预览解码消费者，再启动原始 IPC 接收者。
        self.decoder_thread = threading.Thread(target=self._decode_preview, name="XdSdkPreviewDecode", daemon=True)
        self.reader_thread = threading.Thread(target=self._receive_raw, name="XdSdkRawReceive", daemon=True)
        self.thread = self.reader_thread
        self.decoder_thread.start()
        self.reader_thread.start()

    def wait_for_preview(self, cancelled, timeout=FIRST_PREVIEW_TIMEOUT_S):
        """Wait for the first stereo pair without consuming it from the capture queue."""
        deadline = time.monotonic() + timeout
        while not cancelled.is_set():
            if self.preview_ready.is_set():
                return True
            if self.reader_done.is_set() or self.preview_error:
                detail = self.error or self.preview_error or "SDK 数据流已结束"
                raise RuntimeError(f"VS030 启动失败：{detail}")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                stats = self.get_capture_stats()
                if not stats["sdk_packets_received"]:
                    raise TimeoutError("VS030 未收到相机数据，请检查设备连接或其他相机客户端")
                decoded = stats["preview_decoded_frames"]
                raise TimeoutError(f"VS030 未获得首对双目画面（左路解码 {decoded['4']} 帧，右路 {decoded['5']} 帧）")
            self.preview_ready.wait(min(remaining, 0.05))
        return False

    def _stat(self, key, amount=1):
        with self._stats_lock:
            if key in ("preview_decoded_frames",):
                return
            self.capture_stats[key] = int(self.capture_stats.get(key, 0)) + int(amount)

    def _enqueue_preview(self, packet):
        """Drop only stale preview work on overload; never block the raw reader or writer callback."""
        with self._decode_lock:
            generation = self._decode_generation
            try:
                self.decode_queue.put_nowait((generation, packet))
                return
            except queue.Full:
                dropped = 0
                while True:
                    try:
                        self.decode_queue.get_nowait()
                        dropped += 1
                    except queue.Empty:
                        break
                self._decode_generation += 1
                generation = self._decode_generation
                self._stat("preview_decode_packets_dropped", dropped)
                self._stat("preview_decode_queue_resets")
                # Decoder contexts are reset by the consumer on the new generation.
                self.decode_queue.put_nowait((generation, packet))

    def _receive_raw(self):
        """Read every IPC record, archive it immediately, then dispatch preview/VIO copies."""
        try:
            while True:
                (
                    magic, version, header_size, sink, size, sequence, device_ts,
                    capture_ts_ns, arrival_ts_ns,
                ) = RECORD.unpack(read_exact(self.process.stdout, RECORD.size))
                if (
                    magic != MAGIC or version != RECORD_VERSION or header_size != RECORD.size
                    or size > MAX_PAYLOAD
                ):
                    raise RuntimeError("新版 SDK 数据帧格式错误或 worker IPC 版本不匹配")
                payload = read_exact(self.process.stdout, size)

                # Stop 时仍排空 stdout，避免 SDK 回调阻塞；但不再向已结束会话追加数据。
                if self.stop_event.is_set():
                    continue

                host_monotonic_ns = time.perf_counter_ns()
                host_unix_ns = time.time_ns()
                self._stat("sdk_packets_received")
                self._stat("sdk_payload_bytes", size)
                with self._stats_lock:
                    by_sink = self.capture_stats["sdk_packets_by_sink"]
                    by_sink[str(sink)] = int(by_sink.get(str(sink), 0)) + 1

                callback = self.sdk_record_callback
                if callback is not None:
                    try:
                        callback((
                            sink, sequence, device_ts, capture_ts_ns, arrival_ts_ns,
                            host_monotonic_ns, host_unix_ns, payload,
                        ))
                    except Exception as exc:
                        # 录制回调故障不应阻塞 SDK pipe；由会话状态明确报告。
                        self.callback_error = str(exc)

                if sink == 7:
                    vio_queue = self.imu_vio_queue
                    if vio_queue is not None:
                        try:
                            vio_queue.put_nowait((device_ts, capture_ts_ns, arrival_ts_ns, payload))
                        except queue.Full:
                            self._stat("imu_vio_queue_drops")
                            self.imu_vio_error = "实时 VIO IMU 队列溢出；原始 IMU 仍继续录制"
                            self.imu_vio_queue = None
                    with self._imu_lock:
                        self._imu_packets_pending.append((device_ts, capture_ts_ns, arrival_ts_ns, payload))
                        if len(self._imu_packets_pending) > 120:
                            self._imu_packets_pending.pop(0)
                            self._stat("imu_preview_metadata_drops")
                    continue

                if sink in (4, 5):
                    packet = (
                        sink, sequence, device_ts, capture_ts_ns, arrival_ts_ns,
                        host_monotonic_ns, host_unix_ns, payload,
                    )
                    self._enqueue_preview(packet)
        except (EOFError, OSError, RuntimeError, struct.error) as exc:
            if not self.stop_event.is_set():
                detail = ""
                if self.process and self.process.poll() is not None:
                    try:
                        detail = self.process.stderr.read().decode("utf-8", "replace").strip()
                    except OSError:
                        pass
                self.error = f"{exc} {detail}".strip()
        finally:
            self.reader_done.set()

    def _decode_preview(self):
        import av

        hw_name = os.environ.get("EGOHAND_DECODE_HW", "cuda").strip().lower()
        if hw_name in ("", "0", "off", "none", "software"):
            hw_name = ""

        def new_decoder():
            if hw_name:
                try:
                    from av.codec.hwaccel import HWAccel
                    return av.CodecContext.create("hevc", "r", hwaccel=HWAccel(hw_name))
                except Exception:
                    pass
            return av.CodecContext.create("hevc", "r")

        pending = {}
        active_generation = -1
        last_pair_ts = None
        try:
            while not self.stop_event.is_set() and (not self.reader_done.is_set() or not self.decode_queue.empty()):
                try:
                    generation, packet = self.decode_queue.get(timeout=0.1)
                except queue.Empty:
                    continue

                if generation != active_generation:
                    decoders = {channel: new_decoder() for channel in (4, 5)}
                    metadata = {channel: {} for channel in decoders}
                    pts = 0
                    pending.clear()
                    active_generation = generation

                sink, sequence, device_ts, capture_ts, arrival_ts, host_mono, host_unix, payload = packet
                # HEVC may output an older frame. Match against its packet PTS, never the latest input packet.
                pts += 1
                channel_metadata = metadata[sink]
                channel_metadata[pts] = (sequence, device_ts, capture_ts, arrival_ts, host_mono, host_unix)
                if len(channel_metadata) > DECODER_METADATA_LIMIT:
                    del channel_metadata[next(iter(channel_metadata))]
                encoded = av.Packet(payload)
                encoded.pts = pts
                try:
                    frames = decoders[sink].decode(encoded)
                except av.error.FFmpegError:
                    if getattr(decoders[sink], "is_hwaccel", False):
                        # 硬件解码失败：该通道回退软件解码并重试当前包。
                        decoders[sink] = av.CodecContext.create("hevc", "r")
                        try:
                            frames = decoders[sink].decode(encoded)
                        except av.error.FFmpegError:
                            channel_metadata.pop(pts, None)
                            self._stat("preview_decode_errors")
                            continue
                    else:
                        channel_metadata.pop(pts, None)
                        self._stat("preview_decode_errors")
                        continue

                for frame in frames:
                    with self._stats_lock:
                        self.capture_stats["preview_decoded_frames"][str(sink)] += 1
                    origin = channel_metadata.pop(frame.pts, None)
                    if origin is None:
                        self._stat("preview_pending_pair_drops")
                        continue
                    sequence, *times = origin
                    entry = pending.setdefault(sequence, {})
                    entry[sink] = (frame, *times)
                    if 4 in entry and 5 in entry:
                        del pending[sequence]
                        left_frame, left_ts, left_capture, left_arrival, left_host_mono, left_host_unix = entry[4]
                        right_frame, right_ts, right_capture, right_arrival, right_host_mono, right_host_unix = entry[5]
                        if left_ts != right_ts or (last_pair_ts is not None and left_ts <= last_pair_ts):
                            self._stat("preview_pending_pair_drops")
                            continue
                        last_pair_ts = left_ts
                        left = left_frame.to_ndarray(format="bgr24")
                        right = right_frame.to_ndarray(format="bgr24")
                        with self._imu_lock:
                            imu_packets = tuple(self._imu_packets_pending)
                            self._imu_packets_pending.clear()
                        item = (
                            left, right, time.perf_counter_ns(), left_ts, sequence, imu_packets,
                            left_capture, right_capture, left_arrival, right_arrival,
                            left_host_mono, right_host_mono, left_host_unix, right_host_unix,
                        )
                        try:
                            self.queue.put_nowait(item)
                        except queue.Full:
                            try:
                                self.queue.get_nowait()
                                self._stat("preview_output_frames_dropped")
                            except queue.Empty:
                                pass
                            self.queue.put_nowait(item)
                        self.preview_ready.set()

                while len(pending) > 8:
                    del pending[next(iter(pending))]
                    self._stat("preview_pending_pair_drops")
        except Exception as exc:
            # Preview decoder failures are isolated from the raw receiver/recorder.
            self.preview_error = str(exc)
            logging.exception("VS030 preview decoder failed")
        finally:
            self.decoder_done.set()
            self.running = not (self.stop_event.is_set() or self.reader_done.is_set())

    def read(self):
        deadline = time.monotonic() + 10
        while not self.read_interrupt.is_set():
            try:
                (
                    left, right, stamp, device_ts, seq, imu_packets,
                    left_capture_ns, right_capture_ns, left_arrival_ns, right_arrival_ns,
                    left_host_mono_ns, right_host_mono_ns, left_host_unix_ns, right_host_unix_ns,
                ) = self.queue.get(timeout=0.2)
                break
            except queue.Empty:
                if self.reader_done.is_set() and self.decoder_done.is_set():
                    self.running = False
                    if self.error:
                        raise RuntimeError("新版 SDK 相机采集失败：" + self.error)
                    return None
                if time.monotonic() >= deadline:
                    # A preview stall must not terminate raw SDK/IMU recording.
                    return None
        else:
            return None

        if left.shape != right.shape or left.ndim != 3:
            raise RuntimeError("新版相机双目图像尺寸不一致")
        index = self.index
        self.index += 1
        imu_timing = tuple((int(t), int(c), int(a)) for t, c, a, _ in imu_packets)
        return StereoFrame(
            index, stamp, left, right, None, None,
            self.calibration.calibration_id if self.calibration else None,
            "vendor_vs030", False, "raw",
            {
                "real_stereo": True,
                "synthetic": False,
                "sync_quality": "UNVERIFIED",
                "timestamp_kind": "host_bundle_decode_completed_monotonic_ns",
                "sdk_device_timestamp_raw": device_ts,
                "sdk_sequence": seq,
                "sdk_capture_ts_ns_channel_4": left_capture_ns,
                "sdk_capture_ts_ns_channel_5": right_capture_ns,
                "sdk_arrival_ts_ns_channel_4": left_arrival_ns,
                "sdk_arrival_ts_ns_channel_5": right_arrival_ns,
                "sdk_capture_pair_delta_ns": abs(left_capture_ns - right_capture_ns),
                "sdk_arrival_pair_delta_ns": abs(left_arrival_ns - right_arrival_ns),
                "sdk_capture_to_arrival_ns_channel_4": left_arrival_ns - left_capture_ns,
                "sdk_capture_to_arrival_ns_channel_5": right_arrival_ns - right_capture_ns,
                "host_receive_monotonic_ns_channel_4": left_host_mono_ns,
                "host_receive_monotonic_ns_channel_5": right_host_mono_ns,
                "host_receive_unix_ns_channel_4": left_host_unix_ns,
                "host_receive_unix_ns_channel_5": right_host_unix_ns,
                "host_receive_pair_delta_ns": abs(left_host_mono_ns - right_host_mono_ns),
                "sdk_timestamp_clock_domain": "sdk_internal_monotonic_ns",
                "sdk_timestamp_semantics": "diagnostic_only_not_verified_device_exposure_time",
                "sdk_imu_packet_count": len(imu_packets),
                "sdk_imu_packets_raw": imu_packets,
                "sdk_imu_frame_timing": imu_timing,
                "sdk_imu_delivery": "raw_recorder_priority; preview_metadata_bounded; live_vio_queue_independent",
                "sdk_device_timestamp_semantics": "unverified",
                "channel_ids": [4, 5],
                "transport_pairing": "sdk_sequence_and_equal_device_timestamp",
                "left_right_mapping_evidence": "user_confirmed_matches_visionviewer",
                "hardware_sync_precision_verified": False,
            },
        )

    def interrupt_read(self):
        # Release the pipeline capture loop without stopping the raw SDK reader.
        self.read_interrupt.set()

    def get_capture_stats(self):
        with self._stats_lock:
            result = dict(self.capture_stats)
            result["sdk_packets_by_sink"] = dict(self.capture_stats["sdk_packets_by_sink"])
            result["preview_decoded_frames"] = dict(self.capture_stats["preview_decoded_frames"])
        result.update({
            "scope": "source_lifetime_since_open; manifest sdk_count_by_sink is the exact recording-session count",
            "reader_done": self.reader_done.is_set(),
            "decoder_done": self.decoder_done.is_set(),
            "reader_error": self.error,
            "preview_error": self.preview_error,
            "record_callback_error": self.callback_error,
            "imu_vio_error": self.imu_vio_error,
            "sdk_worker_diagnostics": self.sdk_diagnostics,
        })
        return result

    def stop(self):
        self.stop_event.set()
        self.read_interrupt.set()
        self.running = False
        proc = self.process
        if proc:
            try:
                proc.stdin.close()
            except OSError:
                pass
            try:
                proc.wait(timeout=6)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=3)

            if self.reader_thread and self.reader_thread is not threading.current_thread():
                self.reader_thread.join(timeout=5)
            if self.decoder_thread and self.decoder_thread is not threading.current_thread():
                self.decoder_thread.join(timeout=5)
            try:
                self.sdk_diagnostics = proc.stderr.read().decode("utf-8", "replace").strip()
            except (OSError, AttributeError):
                self.sdk_diagnostics = ""
            if self.sdk_diagnostics:
                logging.info("xdsdk: %s", self.sdk_diagnostics)
            for stream in (proc.stdout, proc.stderr):
                if stream:
                    stream.close()
        self.process = None
        self.running = False

    def close(self):
        self.stop()

    def get_metadata(self):
        return SourceMetadata(
            "vendor_vs030", real_stereo=True, synchronized=False,
            real_calibration=self.calibration is not None, metric_capable=False,
            contains_imu=True,
            timestamp_kind="host_bundle_decode_completed_monotonic_ns",
            extra={
                "device": "VS030 / updated firmware",
                "transport": "xdsdk USB network UDP / H.265",
                "channels": [4, 5, 7],
                "camera_channels": {"4": "left H265", "5": "right H265"},
                "imu_channel": 7,
                "raw_imu_payload": True,
                "device_timestamp_semantics": "unverified",
                "sdk_ipc_record_version": RECORD_VERSION,
                "preview_and_raw_recording_decoupled": True,
            },
        )
