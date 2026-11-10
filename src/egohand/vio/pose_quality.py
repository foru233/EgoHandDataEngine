"""实时位姿质量门控；不修饰原始数据，也不把暂时丢失伪装成平滑轨迹。"""
import math
import numpy as np


class PoseQualityGate:
    def __init__(self, *, stable_seconds=1.5, max_translation_speed_mps=1.5,
                 translation_noise_allowance_m=.02, max_rotation_speed_deg_s=360.,
                 rotation_noise_allowance_deg=3.):
        self.stable_ns=int(stable_seconds*1e9)
        self.translation_speed=float(max_translation_speed_mps)
        self.translation_allowance=float(translation_noise_allowance_m)
        self.rotation_speed=float(max_rotation_speed_deg_s)
        self.rotation_allowance=float(rotation_noise_allowance_deg)
        if min(self.stable_ns,self.translation_speed,self.translation_allowance,self.rotation_speed,self.rotation_allowance)<0:
            raise ValueError('位姿门控参数不得为负数')
        self.previous=None;self.stable_start=None;self.map_epoch=None;self.segment=0

    def _reset(self):
        if self.previous is not None:self.segment+=1
        self.previous=None;self.stable_start=None

    def check(self, result):
        out=dict(result);out['raw_pose_valid']=bool(result.get('pose_valid'))
        out['raw_matrix']=result.get('matrix');out['quality_status']='waiting'
        epoch=result.get('map_epoch')
        if self.map_epoch is not None and epoch!=self.map_epoch:self._reset()
        self.map_epoch=epoch
        if not out['raw_pose_valid'] or result.get('tracking_state')!='OK':
            self._reset();reason='视觉跟踪未稳定'
        elif not result.get('imu_initialized'):
            self._reset();reason='等待 IMU 初始化'
        else:
            matrix=np.asarray(result.get('matrix'),dtype=float)
            stamp=int(result['timestamp_ns'])
            if matrix.shape!=(4,4) or not np.isfinite(matrix).all():
                self._reset();reason='位姿数据无效'
            else:
                reason=None
                if self.previous is not None:
                    previous_stamp,previous_matrix=self.previous
                    dt=(stamp-previous_stamp)/1e9
                    if dt<=0:
                        self._reset();reason='位姿时间非递增'
                    else:
                        distance=float(np.linalg.norm(matrix[:3,3]-previous_matrix[:3,3]))
                        rel=previous_matrix[:3,:3].T@matrix[:3,:3]
                        angle=math.degrees(math.acos(float(np.clip((np.trace(rel)-1)/2,-1,1))))
                        out['translation_step_m']=distance;out['rotation_step_deg']=angle
                        if distance>self.translation_allowance+self.translation_speed*dt or angle>self.rotation_allowance+self.rotation_speed*dt:
                            self._reset();reason=f'位姿突变已拒绝：平移 {distance:.3f} m，旋转 {angle:.1f}°'
                if reason is None:
                    if self.stable_start is None:self.stable_start=stamp
                    self.previous=(stamp,matrix.copy())
                    if stamp-self.stable_start<self.stable_ns:reason='等待连续稳定跟踪'
        if reason is not None:
            out['pose_valid']=False;out['matrix']=None;out['quality_status']=reason
        else:out['quality_status']='可信度待验证'
        out['display_segment']=self.segment
        return out
