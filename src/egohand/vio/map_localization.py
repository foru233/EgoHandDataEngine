"""Read-only real-time localization against one imported VS030 Atlas."""
import queue
import socket
import threading
from concurrent.futures import ThreadPoolExecutor
from contextlib import closing
from uuid import uuid4

import cv2
import numpy as np

from .atlas_worker import ROOT, atlas_worker, reply
from .live_runtime import CALIBRATION
from .live_subprocess import HEADER
from .map_package import digest
from .map_comparison import MapComparison
from .pose_quality import PoseQualityGate
from .vs030_geometry import prepare_vs030_vio_geometry
from .map_identity import PresetMapIdentity


class LiveMapLocalizer:
    def __init__(self, directory, manifest, updates, comparison_enabled, on_status):
        self.directory = directory
        self.manifest = manifest
        self.on_status = on_status
        self.updates = updates
        self.comparison_enabled = comparison_enabled
        self.pending = queue.Queue(maxsize=1)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.connection = None

    def start(self):
        self.thread.start()

    def submit(self, frame):
        try:
            self.pending.get_nowait()
        except queue.Empty:
            pass
        self.pending.put_nowait(frame)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=3)
        if self.thread.is_alive() and self.connection is not None:
            try:
                self.connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self.thread.join(timeout=3)

    def run(self):
        try:
            self.localize()
        except Exception as exc:
            if not self.stop.is_set():
                self.on_status({"type": "error", "message": f"预设地图定位失败 · {exc}"})

    def localize(self):
        if digest(CALIBRATION) != self.manifest["sha256"]["calibration.yaml"]:
            raise ValueError("地图双目标定与当前 VS030 配置不匹配")
        geometry = prepare_vs030_vio_geometry(CALIBRATION)
        settings = self.manifest["processing"]["timing"]
        scale = float(settings["image_scale"])
        if not .25 <= scale <= 1.:
            raise ValueError("地图图像缩放参数无效")
        workspace = ROOT / "logs" / f"map_localize_{uuid4().hex}"
        workspace.mkdir(parents=True)
        self.on_status({"type": "status", "message": "正在寻找地图位置 · 加载定位地图"})
        identity = PresetMapIdentity(self.manifest["atlas_map_id"])
        gate = PoseQualityGate(stable_seconds=.25, resume_same_map=True, require_imu=False)
        previous_stamp = 0
        accepted = None
        comparison = MapComparison(self.directory, self.manifest, geometry.rectifier, scale)
        candidates = None
        job = None
        last_sample = 0
        generation = 0
        continuous = False
        comparison_failed = False
        with (closing(comparison),
              atlas_worker(workspace, self.directory / "orb.yaml", self.directory / "atlas.osa",
                           cancelled=self.stop) as connection,
              ThreadPoolExecutor(max_workers=1, thread_name_prefix="MapComparison") as background):
            self.connection = connection
            while not self.stop.is_set():
                try:
                    frame = self.pending.get(timeout=.2)
                except queue.Empty:
                    continue
                stamp = int(frame.metadata.get("sdk_device_timestamp_raw", 0)) * 1000
                if stamp <= previous_stamp:
                    continue
                previous_stamp = stamp
                left, right = geometry.rectifier.remap_pair(frame.right_image, frame.left_image)
                left, right = [
                    cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), None, fx=scale, fy=scale,
                               interpolation=cv2.INTER_AREA) for image in (left, right)
                ]
                height, width = left.shape
                try:
                    connection.sendall(HEADER.pack(stamp, width, height, 0))
                    connection.sendall(left.tobytes() + right.tobytes())
                    result = reply(connection, "pose")
                except (OSError, EOFError):
                    if self.stop.is_set():
                        break
                    raise
                identity_error = identity.rejection(result)
                checked = gate.check(result)
                if identity_error is not None:
                    checked.update(pose_valid=False, matrix=None, quality_status=identity_error)
                if checked["pose_valid"]:
                    position = np.asarray(checked["matrix"])[:3, 3]
                    if accepted is not None and np.linalg.norm(position - accepted) > .75:
                        checked.update(pose_valid=False, matrix=None, quality_status="位姿突变，地图叠加已暂停")
                    else:
                        accepted = position.copy()
                checked["preset_map_id"] = self.manifest["map_id"]
                self.on_status(checked)
                comparing = (checked["pose_valid"] and self.comparison_enabled.is_set()
                             and not comparison_failed)
                if not comparing:
                    if continuous:
                        generation += 1
                    continuous = False
                if job is not None and job.done():
                    try:
                        layers, confirmed = job.result()
                    except Exception as exc:
                        comparison_failed = True
                        comparing = False
                        self.on_status({"type": "status", "message": f"地图差异暂不可用 · {exc}"})
                    else:
                        if comparing and job_generation == generation:
                            self.on_status({"type": "map_difference", "layers": layers,
                                            "preset_map_id": self.manifest["map_id"]})
                            if self.updates.is_set() and job_writable:
                                candidates = confirmed
                    job = None
                if comparing and job is None and stamp - last_sample >= 500_000_000:
                    job_generation = generation
                    job_writable = self.updates.is_set()
                    job = background.submit(comparison.evaluate, left, right,
                                            np.asarray(checked["matrix"]), continuous)
                    continuous = True
                    last_sample = stamp
        if candidates and any(candidates):
            self.on_status({"type": "map_updates_ready", "comparison": comparison,
                            "candidates": candidates, "preset_map_id": self.manifest["map_id"]})

