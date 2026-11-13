"""实时位姿质量门控；不修饰原始数据，也不把暂时丢失伪装成平滑轨迹。"""
import math
import numpy as np
from .map_identity import (decode_map_epoch, world_identity, invalid_world_revision_transition, invalid_reference_frame,
                           snapshot_identity_matches_epoch)
from .trajectory import rigid_transform
from .recovery_contract import PROTOCOL, RecoveryProtocolError, validate_report


def _same_map_revision(before, after):
    """A changed epoch is only a revision when map and reset identities agree."""
    try:
        old, new = decode_map_epoch(before), decode_map_epoch(after)
    except (TypeError, ValueError, OverflowError):
        return False
    return (old['map_id'] == new['map_id'] and old['reset_id'] == new['reset_id']
            and old['map_revision'] != new['map_revision'])


class PoseQualityGate:
    def __init__(self, *, stable_seconds=1.5, max_translation_speed_mps=1.5,
                 translation_noise_allowance_m=.02, max_rotation_speed_deg_s=360.,
                 rotation_noise_allowance_deg=3., resume_same_map=False, require_imu=True,
                 allow_verified_stereo_bootstrap=False):
        self.stable_ns=int(stable_seconds*1e9)
        self.translation_speed=float(max_translation_speed_mps)
        self.translation_allowance=float(translation_noise_allowance_m)
        self.rotation_speed=float(max_rotation_speed_deg_s)
        self.rotation_allowance=float(rotation_noise_allowance_deg)
        self.resume_same_map=resume_same_map
        self.require_imu=require_imu
        self.allow_verified_stereo_bootstrap=bool(allow_verified_stereo_bootstrap)
        self._map_accepted=False
        if min(self.stable_ns,self.translation_speed,self.translation_allowance,self.rotation_speed,self.rotation_allowance)<0:
            raise ValueError('位姿门控参数不得为负数')
        self.previous=None;self.stable_start=None;self.map_epoch=None;self.segment=0
        self._last_epoch=None;self._last_world_revision=None
        self._last_reference_generation=None;self._reference_epoch=None

    def _reset(self):
        if self.previous is not None:self.segment+=1
        self.previous=None;self.stable_start=None

    def check(self, result):
        out=dict(result);out['raw_pose_valid']=bool(result.get('pose_valid'))
        out['raw_matrix']=result.get('matrix');out['quality_status']='waiting'
        out['jump_detected']=False
        # An uninitialized IMU cannot supply a motion anchor. A real stereo
        # measurement may nevertheless supply a separate local visual segment,
        # but only with frame-bound geometry and the sole coordinator's report.
        bootstrap=False
        if self.allow_verified_stereo_bootstrap and result.get('imu_initialized') is False:
            from .short_recovery import geometry_matches_frame, visual_reference
            report=result.get('recovery_preaccept') or {}
            bootstrap=(report.get('schema_version')==PROTOCOL and
                report.get('owner')=='RecoveryCoordinator' and
                not recovery_quarantine_required(result) and
                geometry_matches_frame(result,result.get('recovery_observation'),visual_reference(result)))
        if self.allow_verified_stereo_bootstrap:
            out['pose_source']='stereo_bootstrap' if bootstrap else 'stereo_inertial'
            out['inertial_display_ready']=bool(result.get('imu_initialized') is True and
                result.get('ba2_complete') is True and result.get('inertial_scale_finalized') is True)
            out['stereo_bootstrap_geometry_verified']=bool(bootstrap)
        epoch=world_identity(result)
        raw_epoch=result.get('map_epoch')
        world_revision=result.get('world_frame_revision')
        reference_bad=invalid_reference_frame(self._last_reference_generation,result,self._reference_epoch)
        reference_changed=result.get('reference_generation')!=self._last_reference_generation
        revision_change=(epoch != self.map_epoch and self.previous is not None
                         and _same_map_revision(self._last_epoch, raw_epoch)
                         and world_revision == self._last_world_revision and not reference_changed)
        if not reference_bad:
            self._last_reference_generation=result.get('reference_generation');self._reference_epoch=raw_epoch
        if epoch!=self.map_epoch and not revision_change:
            self._reset()
            self._map_accepted=False
        self.map_epoch=epoch
        self._last_epoch=raw_epoch
        self._last_world_revision=world_revision
        if not out['raw_pose_valid'] or result.get('tracking_state')!='OK':
            self._reset();reason='视觉跟踪未稳定'
            gate_id='VIO.TRACKING';reason_code='tracking_unavailable'
            if revision_change:self._map_accepted=False
        elif result.get('snapshot_consistent') is False:
            # An explicitly inconsistent native snapshot cannot authorize pose
            # output, even if its pose matrix and tracking status look valid.
            self._reset();reason='原生快照不一致'
            gate_id='VIO.SNAPSHOT';reason_code='snapshot_inconsistent'
            if revision_change:self._map_accepted=False
        elif result.get('snapshot_available') is True and not snapshot_identity_matches_epoch(result):
            self._reset();reason='原生快照地图身份不一致'
            gate_id='VIO.IDENTITY';reason_code='snapshot_identity_mismatch'
            if revision_change:self._map_accepted=False
        elif invalid_world_revision_transition(None, world_revision):
            self._reset();reason='世界坐标版本无效'
            gate_id='VIO.IDENTITY';reason_code='world_revision_invalid'
            if revision_change:self._map_accepted=False
        elif type(result.get('timestamp_ns')) is not int or result['timestamp_ns'] < 0:
            self._reset();reason='位姿时间戳无效'
            gate_id='VIO.TIMESTAMP';reason_code='timestamp_invalid'
            if revision_change:self._map_accepted=False
        elif reference_bad:
            self._reset();reason='初始化参考系证据无效或缺失'
            gate_id='VIO.IDENTITY';reason_code='reference_frame_unverified'
            self._map_accepted=False
        elif self.require_imu and not result.get('imu_initialized') and not bootstrap:
            self._reset();reason='等待 IMU 初始化'
            gate_id='VIO.INERTIAL_READINESS';reason_code='imu_initialization_pending'
            if revision_change:self._map_accepted=False
        else:
            try:
                matrix=rigid_transform(result.get('matrix'))
            except (TypeError, ValueError, OverflowError):
                matrix=None
            stamp=int(result['timestamp_ns'])
            if matrix is None:
                self._reset();reason='位姿数据无效'
                gate_id='VIO.POSE_GEOMETRY';reason_code='invalid_rigid_transform'
                if revision_change:self._map_accepted=False
            else:
                reason=None
                gate_id='VIO.POSE_QUALITY';reason_code='accepted'
                has_previous=self.previous is not None
                if has_previous:
                    previous_stamp,previous_matrix=self.previous
                    dt=(stamp-previous_stamp)/1e9
                    if dt<=0:
                        self._reset();reason='位姿时间非递增'
                        gate_id='VIO.TIMESTAMP';reason_code='timestamp_nonmonotonic'
                        if revision_change:self._map_accepted=False
                    else:
                        distance=float(np.linalg.norm(matrix[:3,3]-previous_matrix[:3,3]))
                        rel=previous_matrix[:3,:3].T@matrix[:3,:3]
                        angle=math.degrees(math.acos(float(np.clip((np.trace(rel)-1)/2,-1,1))))
                        out['translation_step_m']=distance;out['rotation_step_deg']=angle
                        if distance>self.translation_allowance+self.translation_speed*dt or angle>self.rotation_allowance+self.rotation_speed*dt:
                            out['jump_detected']=True
                            self._reset();reason=f'位姿突变已拒绝：平移 {distance:.3f} m，旋转 {angle:.1f}°'
                            gate_id='VIO.POSE_STEP';reason_code='pose_jump'
                            if revision_change:self._map_accepted=False
                if reason is None:
                    if revision_change:
                        # Compare against the last pose before resetting: otherwise
                        # an epoch change can conceal a large discontinuity. A small
                        # step is not proof of an unchanged world reference, and no
                        # complete native geometry verifier is available yet.
                        self._reset()
                        self._map_accepted=False
                        has_previous=False
                    if self.stable_start is None:self.stable_start=stamp
                    self.previous=(stamp,matrix.copy())
                    resumed=self.resume_same_map and self._map_accepted and has_previous
                    if stamp-self.stable_start<self.stable_ns and not resumed:
                        reason='等待连续稳定跟踪'
                        gate_id='VIO.STABILITY';reason_code='stable_window_pending'
                    else:
                        self._map_accepted=epoch is not None
        if reason is not None:
            out['pose_valid']=False;out['matrix']=None;out['quality_status']=reason
        else:out['quality_status']='可信度待验证'
        out['display_segment']=self.segment
        out['quality_gate']={'schema_version':1,'gate_id':gate_id,
            'passed':reason is None,'reason_code':reason_code,'owner':'PoseQualityGate',
            'effect':'allow_pose' if reason is None else 'suppress_pose',
            'triggers_recovery':bool(out['jump_detected']),
            'evidence_level':'runtime_checks_not_accuracy_verification'}
        return out


def recovery_quarantine_required(result):
    """Shared admission predicate; neither source can overrule a quarantine."""
    preaccept=result.get('recovery_preaccept')
    preaccept=preaccept if isinstance(preaccept,dict) else {}
    if 'schema_version' in preaccept:
        if type(preaccept['schema_version']) is not int or preaccept['schema_version']!=PROTOCOL:
            return True
        # Native-stage reports are also consumed by the shared quality gate
        # before the commit. They cannot be presented as committed to a consumer.
        stage=preaccept.get('stage')
        if stage not in ('native','committed'):return True
        try:validate_report(preaccept,result,stage=stage)
        except RecoveryProtocolError:return True
    return (bool(preaccept.get('await_visual_requalification'))
            or result.get('recovery_monitor_awaiting_visual_requalification') is True)


def apply_recovery_quarantine(result):
    """不让已隔离的地图位姿因短暂稳定而重新作为可信轨迹输出。"""
    if recovery_quarantine_required(result):
        result=dict(result)
        result['pose_valid']=False
        result['matrix']=None
        result['quality_status']='位姿异常或恢复未验证：等待地图复核或新地图'
        result['recovery_gate']={'schema_version':1,'gate_id':'VIO.RECOVERY_QUARANTINE',
            'passed':False,'reason_code':'independent_visual_requalification_required',
            'owner':'recovery_admission','effect':'suppress_pose'}
    return result
