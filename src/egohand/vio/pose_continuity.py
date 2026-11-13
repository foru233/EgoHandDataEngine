"""重定位后的刚体重锚：把同图内的位姿跳变修正为连续轨迹。

原理：显示位姿 D = A · R（R 为算法原始位姿）。检测到不可物理的跳变时，
令 A = D_prev · R_new⁻¹，使新位姿正好落在上一显示位姿处；之后 A 保持不变，
后续相对运动完全保留。A 是一个刚体变换（不改变尺度与形状）。

这不是门控：位姿仍然输出、轨迹仍然连续；代价是世界坐标系在跳变处被重新定义。
"""
import math

import numpy as np


class PoseContinuityCorrector:
    def __init__(self, *, enabled=True, max_speed_mps=3.0, min_jump_m=0.25,
                 max_rotation_deg=60.0, max_gap_s=2.0):
        self.enabled = bool(enabled)
        self.max_speed = float(max_speed_mps)
        self.min_jump = float(min_jump_m)
        self.max_rotation = float(max_rotation_deg)
        self.max_gap = float(max_gap_s)
        self.correction = np.eye(4)
        self.last = None
        self.reanchors = 0

    def process(self, result):
        out = dict(result)
        out['continuity_applied'] = False
        out['display_map_coordinates_preserved'] = not self.enabled or self.reanchors == 0
        matrix = result.get('matrix')
        if not self.enabled or not result.get('pose_valid') or matrix is None:
            return out
        raw = np.asarray(matrix, dtype=float)
        if raw.shape != (4, 4) or not np.isfinite(raw).all():
            return out
        stamp = int(result['timestamp_ns'])
        display = self.correction @ raw
        if self.last is not None:
            last_stamp, last_matrix = self.last
            dt = max(0.0, (stamp - last_stamp) / 1e9)
            jump = float(np.linalg.norm(display[:3, 3] - last_matrix[:3, 3]))
            rel = last_matrix[:3, :3].T @ display[:3, :3]
            angle = math.degrees(math.acos(float(np.clip((np.trace(rel) - 1) / 2, -1, 1))))
            allowed = max(self.min_jump, self.max_speed * dt + 0.05)
            if jump > allowed or (dt <= self.max_gap and angle > self.max_rotation):
                self.correction = last_matrix @ np.linalg.inv(raw)
                display = self.correction @ raw
                self.reanchors += 1
                out['continuity_applied'] = True
                out['display_map_coordinates_preserved'] = False
        out['matrix'] = display.tolist()
        out['continuity_reanchors'] = self.reanchors
        self.last = (stamp, display)
        return out
