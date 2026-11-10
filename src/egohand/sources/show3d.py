"""SHOW3D v3: read-only, verified-scene stereo source + separate GT/prediction processor.

Dataset loader from the pinned official SHOW3D checkout. World coordinates and
camera translations are millimetres; EgoHand's triangulation uses metres.
"""
from dataclasses import replace
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

from .base import StereoDataSource
from ..calibration.schema import StereoCalibration
from ..calibration.rectifier import StereoRectifier
from ..core.types import SourceMetadata, StereoFrame, StereoHandResult, Hand3D, HandDetection
from ..benchmarks.hot3d import CANONICAL_MP

ROOT = Path(__file__).resolve().parents[3]
# 原工具目录损坏时使用独立恢复的官方代码；保留原目录供检查，不覆盖用户文件。
SHOW3D_TOOLS = ROOT / "tools" / ("show3d_recovered" if (ROOT / "tools" / "show3d_recovered" / "show3d" / "dataset.py").is_file() else "show3d")
OFFICIAL_URL = "https://huggingface.co/datasets/facebook/show3d-dataset"
REVISION = "81403d3e4254cef959027118bc668bb2f26c86f6"
_LOADER_ERROR = None
if SHOW3D_TOOLS.is_dir() and str(SHOW3D_TOOLS) not in sys.path:
    sys.path.insert(0, str(SHOW3D_TOOLS))
try:
    from show3d.dataset import Show3DDataset, Show3DFrameRef, seek_frame
    from show3d.camera import project_to_image, world_to_camera
except (ImportError, SyntaxError) as exc:
    # Do not break the other GUI sources when the optional toolkit is absent.
    _LOADER_ERROR = exc


def _verified_scenes(root, selected=None):
    """Only scenes with both a completed local validation and file-verification report."""
    candidates = []
    for validation in sorted((root / "verification").glob("*/*/validation.json")):
        subject, scene = validation.parent.parent.name, validation.parent.name
        if selected and (subject, scene) != selected:
            continue
        proof = validation.parent / "file_verification.json"
        if not proof.is_file():
            continue
        try:
            v = json.loads(validation.read_text(encoding="utf-8"))
            p = json.loads(proof.read_text(encoding="utf-8"))
            if (v.get("annotation_version") != "v3" or
                v.get("camera_model") != "PinholePlane" or
                p.get("revision") != REVISION or
                v.get("revision") != REVISION):
                continue
            required = p.get("files", [])
            if len(required) < 8 or any(
                not (root / info["path"]).is_file() or
                (root / info["path"]).stat().st_size != info["size"]
                for info in required
            ):
                continue
            video = v["videos"]
            count = int(video["headset0"]["frame_count"])
            if count <= 0 or int(video["headset1"]["frame_count"]) != count:
                continue
            candidates.append((subject, scene, count))
        except (OSError, KeyError, ValueError, TypeError):
            continue
    return candidates


class SHOW3DSource(StereoDataSource):
    """Open verified scenes from dataset root, or one verified scene directory."""

    def __init__(self, path=None, loop=False, view_mode="ground_truth"):
        self.path = Path(path) if path else ROOT / "samples/date/SHOW3D"
        self.loop = bool(loop)
        self.view_mode = view_mode
        self.fps = 60.
        self.running = False
        self.index = 0
        self.rows = []
        self.scenes = []
        self._current_scene = None
        self._captures = []
        self._dataset = None
        self._times = {}
        self.metadata = None

    def open(self):
        if _LOADER_ERROR is not None:
            raise RuntimeError("SHOW3D optional official loader unavailable; restore tools/show3d: " + str(_LOADER_ERROR))
        self.close()
        path = self.path.resolve()
        if not path.is_dir():
            raise ValueError("SHOW3D 请选择数据集根目录或已验收的场景文件夹")
        selected = None
        root = path
        if (path / "headset0.mp4").exists() or (path / "headset1.mp4").exists():
            selected = (path.parent.name, path.name)
            root = path.parents[2]
        if not (root / "scenes").is_dir() or not (root / "verification").is_dir():
            raise ValueError("SHOW3D 根目录须包含 scenes/ 和 verification/；不能选择未验收的半成品目录")
        self.root = root
        self.scenes = _verified_scenes(root, selected)
        if not self.scenes:
            raise ValueError("尚无已通过双目、v3、哈希与视频验收的 SHOW3D 场景")
        self.rows = [(s, scene, i) for s, scene, n in self.scenes for i in range(n)]
        self.index = 0
        self.metadata = SourceMetadata(
            "show3d", real_stereo=True, synchronized=False,
            real_calibration=True, metric_capable=True, ground_truth=True,
            synchronization_evidence=OFFICIAL_URL + "@" + REVISION,
            timestamp_kind="dataset_frame_grid",
            extra={"dataset_id": "show3d", "annotation_version": "v3",
                   "camera_model": "PinholePlane", "frame_count": len(self.rows),
                   "scenes": len(self.scenes), "revision": REVISION,
                   "source_path": str(path), "streams": ["headset0", "headset1"],
                   "pairing_quality": "DATASET_DOCUMENTED",
                   "hardware_sync_precision_verified": False},
        )

    def _open_scene(self, subject, scene):
        self._close_scene()
        ref = Show3DFrameRef(subject, scene, 0)
        self._dataset = Show3DDataset(
            self.root, [ref], hand_pose_version="v3",
            views=("headset0", "headset1"), decode_images=False)
        paths = self._dataset.paths
        self._captures = [cv2.VideoCapture(str(paths.video_path(ref, side)))
                          for side in ("headset0", "headset1")]
        if not all(cap.isOpened() for cap in self._captures):
            self._close_scene()
            raise ValueError("SHOW3D 双目 MP4 无法解码: " + subject + "/" + scene)
        frame_info = json.loads(paths.frame_info_path(ref).read_text(encoding="utf-8"))
        self._times = {int(x["index"]): float(x["timestamp"]) for x in frame_info}
        self._current_scene = (subject, scene)

    def _close_scene(self):
        for cap in self._captures:
            cap.release()
        self._captures = []
        self._dataset = None
        self._times = {}
        self._current_scene = None

    def get_calibration(self):
        # Per-frame extrinsics cannot be represented by a static calibration.
        return None

    def get_metadata(self):
        return self.metadata

    def start(self):
        self.running = True

    def pause(self):
        self.running = False

    def resume(self):
        self.running = True

    def stop(self):
        self.running = False

    def close(self):
        self.running = False
        self._close_scene()

    def seek(self, index):
        if not 0 <= index < len(self.rows):
            raise IndexError(index)
        self.index = index

    def read(self):
        if not self.running:
            return None
        if self.index >= len(self.rows):
            if not self.loop:
                return None
            self.index = 0
        ordinal = self.index
        subject, scene, local_frame = self.rows[ordinal]
        self.index += 1
        if self._current_scene != (subject, scene):
            self._open_scene(subject, scene)
        ref = Show3DFrameRef(subject, scene, local_frame)
        entry = self._dataset.frame_data_for_ref(ref)
        views = [entry.views[name] for name in ("headset0", "headset1")]
        images = []
        for cap, name in zip(self._captures, ("headset0", "headset1")):
            if int(cap.get(cv2.CAP_PROP_POS_FRAMES)) != local_frame:
                if not seek_frame(cap, local_frame):
                    raise ValueError(f"SHOW3D seek failed: {subject}/{scene} {name} #{local_frame}")
            ok, image = cap.read()
            if not ok or image is None or image.ndim != 3:
                raise ValueError(f"SHOW3D decode failed: {subject}/{scene} {name} #{local_frame}")
            images.append(image)
        gt = {}
        for side, hand in (("left", entry.left_hand), ("right", entry.right_hand)):
            if (hand is not None and hand.confidence > 0.1 and
                hand.landmarks_world_mm is not None):
                points = np.asarray(hand.landmarks_world_mm, dtype=float)
                if points.shape == (21, 3) and np.isfinite(points).all():
                    gt[side] = {"confidence": float(hand.confidence),
                                "world_mm": points}
        source_time = self._times.get(local_frame)
        # Dataset provides a shared recording timestamp, not separate hardware clock
        # timestamps. Use monotonic playback ticks exclusively for the detector API.
        tick_ns = int((ordinal + 1) * (1_000_000_000 / self.fps))
        show = {"subject_id": subject, "scene_id": scene, "frame_index": local_frame,
                "sequence_id": subject + "/" + scene, "source_timestamp_s": source_time,
                "calibration": [v.calibration for v in views],
                "pose_valid": all(v.calibration is not None and
                                  v.calibration.is_pose_valid is True and
                                  v.calibration.t_world_from_camera is not None and
                                  not v.missing for v in views),
                "missing_cameras": [v.name for v in views if v.missing],
                "ground_truth": gt}
        # 仅在同一场景确有官方 MANO 姿态和受试者手形时，登记真值网格来源。
        mano_scene=self.root/'hand_pose'/'v3'/'scenes'/subject/scene/'hand_pose_mano.json'
        mano_profile=self.root/'hand_pose'/'hand_profiles'/subject/'profile_mano.json'
        mano_truth=[]
        if show['pose_valid'] and mano_scene.is_file() and mano_profile.is_file():
            for slot in (0,1):
                mano_truth.append({'kind':'show3d','root':str(self.root),'subject':subject,'scene':scene,
                    'frame_index':local_frame,'slot':slot,'camera_world_from':views[0].calibration.t_world_from_camera.tolist()})
        return StereoFrame(
            ordinal, tick_ns, *images, None, None, None, "show3d",
            False, "raw",
            {"real_stereo": True, "synthetic": False,
             "timestamp_kind": "playback_ticks_from_dataset_index",
             "original_dataset_timestamp_s": source_time,
             "sync_quality": "DATASET_DOCUMENTED",
             "synchronization_evidence": OFFICIAL_URL + "@" + REVISION,
             "ground_truth_mano":mano_truth,
             "show3d": show})


def _native_calibration(show):
    left, right = show["calibration"]
    if not show["pose_valid"]:
        return None
    if left.image_width != right.image_width or left.image_height != right.image_height:
        raise ValueError("SHOW3D headset image sizes differ")
    t_l = left.t_world_from_camera
    t_r = right.t_world_from_camera
    r = t_r[:3, :3].T @ t_l[:3, :3]
    t = t_r[:3, :3].T @ (t_l[:3, 3] - t_r[:3, 3]) / 1000.
    if np.linalg.norm(t) < 0.001:
        raise ValueError("SHOW3D stereo baseline too small")
    def k(c):
        return np.array([[c.fx, 0, c.cx], [0, c.fy, c.cy], [0, 0, 1]], dtype=float)
    return StereoCalibration(
        k(left), np.zeros(5), k(right), np.zeros(5), r, t,
        left.image_width, left.image_height, "pinhole",
        f"show3d-v3/{show['sequence_id']}/{show['frame_index']}",
        "SHOW3D official per-frame PinholePlane", "m", False)


class SHOW3DFrameProcessor:
    """GT is never sent to the detection/triangulation engine in prediction mode."""

    def __init__(self):
        self._rectifier = None
        self._last_sequence = None

    def prepare(self, frame):
        show = frame.metadata["show3d"]
        calibration = _native_calibration(show)
        if calibration is None:
            # Pose invalid: display native images but never invent metric geometry.
            raw = replace(frame, metadata={**frame.metadata, "show3d_pose_invalid": True})
            raw.debug_native_images = (frame.left_image, frame.right_image)
            raw.debug_ground_truth_uv = [{}, {}]
            return None, raw
        rectifier = StereoRectifier(calibration)
        rectified = rectifier.rectify(replace(frame, calibration_id=calibration.calibration_id))
        rectified.debug_native_images = (frame.left_image, frame.right_image)
        projections = [{}, {}]
        for side, entry in show["ground_truth"].items():
            world = entry["world_mm"]
            for i, cam in enumerate(show["calibration"]):
                uv, valid = project_to_image(world, cam)
                corrected = rectifier.points(uv, ("left", "right")[i])
                corrected[~valid] = np.nan
                mp = np.full((21, 2), np.nan, dtype=float)
                mask = np.zeros(21, bool)
                for canonical, target in enumerate(CANONICAL_MP):
                    mp[target] = corrected[canonical]
                    mask[target] = bool(valid[canonical] and
                                        np.isfinite(corrected[canonical]).all())
                projections[i][side] = (mp, mask)
        rectified.debug_ground_truth_uv = projections
        self._rectifier = rectifier
        return rectifier, rectified

    def process(self, engine, frame, mode):
        if mode not in ("ground_truth", "prediction"):
            raise ValueError("Unknown SHOW3D mode")
        start = time.perf_counter()
        show = frame.metadata["show3d"]
        rectifier, prepared = self.prepare(frame)
        if show["sequence_id"] != self._last_sequence:
            engine.reset_tracking()
            self._last_sequence = show["sequence_id"]
        if rectifier is not None and (
            engine.calibration is None or
            engine.calibration.calibration_id != rectifier.c.calibration_id
        ):
            engine.set_calibration(rectifier.c)

        if mode == "ground_truth":
            result = StereoHandResult(frame.frame_id, frame.timestamp_ns,
                                      rectifier.c.calibration_id if rectifier else None,
                                      "show3d", session_id=engine.session_id)
            detections = [[], []]
            if rectifier is not None:
                for side, entry in show["ground_truth"].items():
                    native = world_to_camera(entry["world_mm"],
                                             show["calibration"][0].t_world_from_camera) / 1000.
                    hand = Hand3D(side.title(), entry["confidence"],
                                  1 if side == "left" else 2,
                                  tracking_status="dataset_ground_truth")
                    for i, projection in enumerate(prepared.debug_ground_truth_uv):
                        pixels, mask = projection[side]
                        detections[i].append(HandDetection(
                            side.title(), entry["confidence"],
                            pixels / [rectifier.c.image_width, rectifier.c.image_height],
                            pixels, valid=mask, origin="show3d_v3_gt_projection"))
                    for canonical, target in enumerate(CANONICAL_MP):
                        joint = hand.joints[target]
                        joint.xyz_m = native[canonical].copy()
                        joint.valid = True
                        joint.status = "DATASET_GROUND_TRUTH"
                        joint.uncertainty_status = "NOT_APPLICABLE_GROUND_TRUTH"
                        for i, key in enumerate(("left_uv_px", "right_uv_px")):
                            pixels, mask = prepared.debug_ground_truth_uv[i][side]
                            if mask[target]:
                                setattr(joint, key, tuple(pixels[target]))
                    hand.valid_joint_count = 20
                    result.hands.append(hand)
            result.left_detections, result.right_detections = detections
            available = bool(result.hands)
            result.reconstruction_status = ("Official Ground Truth" if available else
                                            "Ground Truth Unavailable")
            result.diagnostics = {
                "result_kind": "official_ground_truth" if available else "ground_truth_unavailable",
                "left_detection_origin": "official_ground_truth",
                "right_detection_origin": "official_ground_truth",
                "metric_valid": False, "synthetic": False,
                "reason": None if available else "No valid headset poses or v3 world annotations",
                "mean_reprojection_error_px": None,
                "output_geometry": prepared.image_geometry,
            }
            engine.last_frame = prepared
        else:
            # 官方 PinholePlane 已去畸变；引擎先极线校正，再在校正图检测。
            payload = replace(frame, source_type="benchmark:show3d",
                              calibration_id=rectifier.c.calibration_id if rectifier else None,
                              metadata={k: v for k,v in frame.metadata.items()
                                        if k != "show3d"})
            if rectifier is not None:
                result = engine.process(payload, dataset_pairing={
                    "dataset_id": "show3d", "sync_quality": "DATASET_DOCUMENTED",
                    "pair_id": show["sequence_id"] + "/" + str(show["frame_index"]),
                    "source": OFFICIAL_URL + "@" + REVISION})
            else:
                engine.calibration = None
                engine.rectifier = None
                result = engine.process(payload)
            result.source_type = "show3d"
            result.diagnostics.update(
                result_kind="algorithm_prediction", detector_input="rectified_monochrome_pinhole" if rectifier else "unavailable_rectification_required",
                metric_scope="public_dataset_only_not_local_device",
            )
            engine.last_frame = replace(engine.last_frame, source_type="show3d")
            engine.last_frame.debug_native_images=(frame.left_image,frame.right_image)
            engine.last_frame.debug_ground_truth_uv=prepared.debug_ground_truth_uv
            # Post-hoc accuracy only: official GT is never an input to the model,
            # stereo association, or triangulation. No alignment/scaling applied.
            errors_mm = []
            counts = {}
            if rectifier is not None and not result.diagnostics.get("identity_ambiguous", False):
                for side, entry in show["ground_truth"].items():
                    predictions = [h for h in result.hands
                                   if h.handedness.lower() == side and h.valid_joint_count > 0]
                    if len(predictions) != 1:
                        continue
                    ground_truth_m = world_to_camera(
                        entry["world_mm"], show["calibration"][0].t_world_from_camera) / 1000.
                    hand = predictions[0]
                    per_hand = []
                    for canonical, mapped in enumerate(CANONICAL_MP):
                        joint = hand.joints[mapped]
                        if joint.valid and np.isfinite(joint.xyz_m).all():
                            per_hand.append(float(np.linalg.norm(
                                joint.xyz_m - ground_truth_m[canonical]) * 1000.))
                    if per_hand:
                        errors_mm.extend(per_hand)
                        counts[side] = len(per_hand)
            result.diagnostics["show3d_gt_comparison"] = {
                "MPJPE_mm": float(np.mean(errors_mm)) if errors_mm else None,
                "compared_joints": len(errors_mm),
                "by_hand_joint_counts": counts,
                "matching_rule": "single unambiguous prediction per handedness",
                "coordinate_frame": "original_headset0_optical",
                "alignment": "none",
                "warning": "Handedness matching is not a verified identity association; N/A if no matched valid 3D joints.",
            }
        result.diagnostics.update(
            show3d_sequence_id=show["sequence_id"],
            show3d_frame_index=show["frame_index"],
            show3d_source_timestamp_s=show["source_timestamp_s"],
            ground_truth_available=bool(show["ground_truth"]) and rectifier is not None,
            native_camera_model="PinholePlane", revision=REVISION,
            joint_mapping_version="show3d-v3-umetrack-canonical20-to-mediapipe-v1",
            input_geometry="raw_pinhole", pixel_coordinate_frame=prepared.image_geometry,
            coordinate_frame_note="XYZ original headset0 optical (m), UV rectified if pose valid",
            hardware_sync_precision_verified=False,
            dataset_pairing_quality="DATASET_DOCUMENTED",
            real_stereo=True,
        )
        result.processing_time_ms = (time.perf_counter() - start) * 1000
        overlay=[]
        if rectifier is not None:
            for entry in show['ground_truth'].values():
                native=world_to_camera(entry['world_mm'],show['calibration'][0].t_world_from_camera)/1000.
                overlay.append({'xyz_m':native.tolist(),'valid':np.isfinite(native).all(axis=1).tolist(),'mediapipe_indices':list(CANONICAL_MP)})
        engine.last_frame.metadata={**engine.last_frame.metadata,'ground_truth_left_camera':overlay}
        engine.last_frame.debug_native_images = (frame.left_image, frame.right_image)
        engine.last_frame.debug_ground_truth_uv = prepared.debug_ground_truth_uv
        return result
