"""Sampled geometry and transient foreground detection for a localized map view."""
from pathlib import Path

import cv2
import numpy as np


class ForegroundDetector:
    def __init__(self):
        self.people = cv2.HOGDescriptor()
        self.people.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self.previous = None
        self.hands = None

    def reset(self):
        self.previous = None

    def close(self):
        if self.hands is not None:
            self.hands.close()
            self.hands = None

    def mask(self, gray, depth, pose, camera):
        mask = np.zeros(gray.shape, dtype=np.uint8)
        boxes, _ = self.people.detectMultiScale(gray, winStride=(8, 8), scale=1.15)
        for x, y, w, h in boxes:
            mask[max(0, y-8):y+h+8, max(0, x-8):x+w+8] = 1
        if self.hands is None:
            from ..detection.mediapipe_detector import MediaPipeHandDetector

            model = Path(__file__).resolve().parents[3] / "models/hand_landmarker.task"
            self.hands = MediaPipeHandDetector(model, rescue_config={"enabled": False})
        for hand in self.hands.detect(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)):
            points = hand.pixel_landmarks[hand.valid & np.isfinite(hand.pixel_landmarks).all(axis=1)]
            if len(points):
                x, y, w, h = cv2.boundingRect(points.astype(np.float32))
                mask[max(0, y-15):max(0, y+h+15), max(0, x-15):max(0, x+w+15)] = 1
        if self.previous is not None:
            old_gray, old_depth, old_pose = self.previous
            self.motion_mask(mask, old_gray, old_depth, old_pose, gray, depth, pose, camera)
        self.previous = gray, depth, pose
        return mask.astype(bool)

    @staticmethod
    def motion_mask(mask, old_gray, old_depth, old_pose, gray, depth, pose, camera):
        features = cv2.goodFeaturesToTrack(old_gray, maxCorners=600, qualityLevel=.015, minDistance=8)
        if features is None:
            return
        current, ok, _ = cv2.calcOpticalFlowPyrLK(old_gray, gray, features, None)
        if current is None:
            return
        back, back_ok, _ = cv2.calcOpticalFlowPyrLK(gray, old_gray, current, None)
        if back is None:
            return
        valid = (ok.ravel() > 0) & (back_ok.ravel() > 0)
        valid &= np.linalg.norm(features[:, 0] - back[:, 0], axis=1) < 1
        old = np.rint(features[:, 0][valid]).astype(int)
        new = np.rint(current[:, 0][valid]).astype(int)
        h, w = gray.shape
        inside = (new[:, 0] >= 0) & (new[:, 0] < w) & (new[:, 1] >= 0) & (new[:, 1] < h)
        old, new = old[inside], new[inside]
        oz, nz = old_depth[old[:, 1], old[:, 0]], depth[new[:, 1], new[:, 0]]
        observed = (oz > 0) & (nz > 0)
        if not observed.any():
            return
        old, new, oz, nz = old[observed], new[observed], oz[observed], nz[observed]
        a = camera.unproject(*old.T, oz) @ old_pose[:3, :3].T + old_pose[:3, 3]
        b = camera.unproject(*new.T, nz) @ pose[:3, :3].T + pose[:3, 3]
        delta = b - a
        moving = np.linalg.norm(delta, axis=1) > .12 + .035 * nz**2
        xy, motion = new[moving], delta[moving]
        if len(xy) < 3:
            return
        image_distance = np.linalg.norm(xy[:, None] - xy[None, :], axis=-1)
        motion_distance = np.linalg.norm(motion[:, None] - motion[None, :], axis=-1)
        coherent = ((image_distance < 50) & (motion_distance < .20)).sum(axis=1) >= 3
        for x, y in xy[coherent]:
            cv2.circle(mask, (int(x), int(y)), 24, 1, -1)

