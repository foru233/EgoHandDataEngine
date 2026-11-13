"""跨帧管理恢复预测证据；仅生成诊断，不能替代 ORB 候选内部拒绝。"""
import numpy as np
from .imu_loss_prediction import BodyState, LossRecoveryPredictor
from .native_bridge import ImuMeasurement
from .map_identity import (decode_map_epoch, snapshot_identity_matches_epoch,
                           invalid_world_revision_transition, snapshot_body_matches_frame, invalid_reference_frame)


class RecoveryMonitor:
    def __init__(self, *, max_boundary_hold_ns=5_000_000, **predictor_options):
        self.predictor=LossRecoveryPredictor(**predictor_options)
        self.max_hold=int(max_boundary_hold_ns)
        if self.max_hold<0:raise ValueError('边界保持上限不能为负数')
        self.last_sample=None;self.loss=False;self.invalid_reason=None;self.start_hold_ns=None
        self.await_visual_requalification=False
        self._last_epoch=None;self._last_world_revision=None;self._has_identity=False
        self._reference_generation=None

    def requalify_after_independent_visual_check(self):
        """Fail closed until an actual independent verifier is integrated.

        A caller's claim that a visual check occurred is not geometric evidence.
        This diagnostic monitor must not convert that claim into a trusted anchor.
        """
        return False

    def reset_imu_alignment(self):
        """Discard IMU history while retaining world identity and quarantine.

        Keeping the last reference lets the next authenticated new map open a
        separate segment and lets a missing known world revision fail closed.
        A sensor-buffer reset cannot qualify a recovery during an existing loss.
        """
        self.await_visual_requalification = bool(self.await_visual_requalification or self.loss)
        self.predictor.anchor = None
        self.predictor.samples = []
        self.last_sample = None
        self.loss = False
        self.invalid_reason = None
        self.start_hold_ns = None

    def observe(self, result, measurements, *, trusted_body=None):
        """记录帧间样本，恢复首帧返回诊断；trusted_body 必须由外层通过质量检查。"""
        epoch=result.get('map_epoch')
        world_revision=result.get('world_frame_revision')
        epoch_changed=self._has_identity and epoch!=self._last_epoch
        missing_world=invalid_world_revision_transition(self._last_world_revision, world_revision)
        reference_bad=invalid_reference_frame(self._reference_generation,result,self._last_epoch)
        if (epoch_changed and not snapshot_identity_matches_epoch(result)) or missing_world or reference_bad:
            self.predictor.anchor=None;self.predictor.samples=[];self.loss=False
            self.invalid_reason=('reference_frame_unverified' if reference_bad else
                                 'world_revision_unavailable' if missing_world else 'identity_snapshot_unverified')
            self.start_hold_ns=None
            self.await_visual_requalification=True
            return {'status':'unavailable','reason':self.invalid_reason,
                    'mode':'advisory_after_orb_acceptance'}
        world_changed=self._has_identity and (world_revision!=self._last_world_revision or
                                              result.get('reference_generation')!=self._reference_generation)
        self._reference_generation=result.get('reference_generation')
        same_generation=False
        if epoch_changed:
            try:
                old=decode_map_epoch(self._last_epoch);new=decode_map_epoch(epoch)
                same_generation=(old['map_id']==new['map_id'] and old['reset_id']==new['reset_id'])
            except (TypeError,ValueError,OverflowError):
                # Unknown identity must never count as a fresh, independent map.
                same_generation=True
        reference_changed=epoch_changed or world_changed
        new_map=epoch_changed and not same_generation
        self._last_epoch=epoch;self._last_world_revision=world_revision;self._has_identity=True
        loss_across_reference_change = bool(reference_changed and not new_map and self.loss)
        if reference_changed:
            self.predictor.anchor=None;self.predictor.samples=[];self.loss=False
            self.invalid_reason=None;self.start_hold_ns=None
            # A map/reset generation opens a separate segment.  An internal
            # revision or world revision cannot erase an existing quarantine.
            if new_map:self.await_visual_requalification=False
            elif loss_across_reference_change:self.await_visual_requalification=True
        for sample in measurements:
            if self.last_sample is not None and sample.timestamp_ns<=self.last_sample.timestamp_ns:
                self.invalid_reason='imu_non_monotonic'
            self.last_sample=sample
            if self.predictor.anchor is not None and sample.timestamp_ns>self.predictor.anchor.timestamp_ns:
                try:self.predictor.push(sample)
                except (ValueError,OverflowError) as exc:self.invalid_reason=str(exc)
        ok=bool(result.get('pose_valid')) and result.get('tracking_state')=='OK'
        if not ok:
            if self.predictor.anchor is not None:self.loss=True
            return None
        decision=({'status':'unavailable',
                   'reason':('map_changed' if new_map else
                             'reference_changed_during_loss' if loss_across_reference_change else
                             'reference_revision_unverified'),
                   'mode':'advisory_after_orb_acceptance'} if reference_changed else None)
        if self.loss:
            body=result.get('body_state_candidate')
            if self.invalid_reason:decision={'status':'unavailable','reason':self.invalid_reason}
            elif not snapshot_body_matches_frame(result,body):
                decision={'status':'unavailable','reason':'body_snapshot_unavailable_or_mismatched'}
            else:
                decision=self.predictor.evaluate(body['T_world_from_body'],int(result['timestamp_ns']),
                    str(result.get('map_epoch')),max_endpoint_hold_ns=self.max_hold)
                decision['start_hold_ns']=self.start_hold_ns
            decision['mode']='advisory_after_orb_acceptance'
            self.loss=False
            if decision['status']!='consistent':self.await_visual_requalification=True
        # 丢失恢复未经检查或不一致时，不能把该候选当成新的可信起点。
        may_anchor=not self.await_visual_requalification and (decision is None or decision.get('status')=='consistent' or reference_changed)
        if trusted_body is not None and may_anchor:
            if not isinstance(trusted_body,BodyState):raise TypeError('可信状态类型错误')
            if trusted_body.timestamp_ns!=int(result['timestamp_ns']) or trusted_body.map_id!=str(epoch):
                raise ValueError('可信体状态与当前帧或地图不一致')
            age=trusted_body.timestamp_ns-self.last_sample.timestamp_ns if self.last_sample is not None else -1
            if 0<=age<=self.max_hold:
                self.predictor.set_trusted_state(trusted_body);self.invalid_reason=None;self.start_hold_ns=age
                last=self.last_sample
                self.predictor.push(ImuMeasurement(trusted_body.timestamp_ns,last.acceleration_mps2,last.angular_velocity_rps))
        return decision


def observe_recovery(monitor, result, measurements, *, allow_anchor=True):
    """适配 ORB 体状态及已对齐 IMU；该诊断在 ORB 接受视觉候选之后运行。"""
    samples=[ImuMeasurement(int(row[0]),tuple(map(float,row[1:4])),tuple(map(float,row[4:7]))) for row in measurements]
    body=result.get('body_state_candidate');trusted=None
    if (allow_anchor and snapshot_body_matches_frame(result,body) and body.get('ba2_complete') is True
            and result.get('inertial_scale_finalized') is True
            and result.get('pose_valid') and result.get('tracking_state')=='OK'):
        matrix=np.asarray(body['T_world_from_body'],dtype=float)
        if matrix.shape==(4,4) and np.isfinite(matrix).all():
            trusted=BodyState(int(result['timestamp_ns']),matrix[:3,:3],matrix[:3,3],
                np.asarray(body['velocity_world_mps'],dtype=float),np.asarray(body['accel_bias_mps2'],dtype=float),
                np.asarray(body['gyro_bias_rps'],dtype=float),str(result.get('map_epoch')))
    # 恢复证据必须在 ORB 首次恢复时计算，而不是等待显示门控的稳定窗口；
    # 锚点仍只取通过门控的体状态。
    tracking_result=dict(result)
    tracking_result['pose_valid']=bool(result.get('raw_pose_valid',result.get('pose_valid')))
    return monitor.observe(tracking_result,samples,trusted_body=trusted)
