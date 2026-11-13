"""重定位候选的预接受检查状态机；不把缺失 IMU 证据解释为一致。"""
import numpy as np
import uuid
from .imu_loss_prediction import BodyState, LossRecoveryPredictor
from .native_bridge import ImuMeasurement
from .pose_quality import PoseQualityGate
from .map_identity import (decode_map_epoch, snapshot_identity_matches_epoch,
                           invalid_world_revision_transition, snapshot_body_matches_frame, invalid_reference_frame)
from .recovery_contract import (PROTOCOL, RecoveryProtocolError,
                                validate_feedback, identity_fields)
from .short_recovery import geometry_candidate,geometry_matches_frame,visual_reference


class RecoveryCoordinator:
    """按帧输入 IMU；只在已初始化且通过稳定窗口时更新候选锚点。

    阈值是实验性经验包络，结果不是已标定的统计置信度或真值。
    """
    def __init__(self, *, max_boundary_hold_ns=5_000_000, stable_seconds=1.5,
                 quarantine_timeout_s=3.0, timeout_action='report_only',
                 application_admission=False, short_recovery=False,
                 allow_initialized_anchor=False, **predictor_options):
        self.predictor=LossRecoveryPredictor(**predictor_options)
        self.application_admission=bool(application_admission)
        self.short_recovery=bool(short_recovery)
        self.allow_initialized_anchor=bool(allow_initialized_anchor)
        self.trusted_visual_identity=None
        self.quality=(None if self.application_admission else
                      PoseQualityGate(stable_seconds=stable_seconds,resume_same_map=False))
        self.session_id=uuid.uuid4().hex
        self.frame_sequence=0
        self._pending_application=None
        self.max_hold=int(max_boundary_hold_ns)
        self.last_sample=None;self.invalid_reason=None;self.map_epoch=None
        self.callback_results=[];self.quarantined=False;self.anchor_hold_ns=None
        self.tracking_was_lost=False
        # Unlike initialization, LOST after a tracked frame requires explicit
        # recovery evidence even if a legacy DLL cannot produce a trusted anchor.
        self.ever_tracking_ok=False;self.recovery_evidence_required=False
        self.quarantine_timeout_ns=int(float(quarantine_timeout_s)*1e9)
        if self.quarantine_timeout_ns<=0:raise ValueError('隔离超时必须为正数')
        if timeout_action not in ('report_only','legacy_new_map','independent_new_map'):
            raise ValueError('未知的隔离超时策略')
        self.timeout_action=timeout_action
        if self.application_admission and timeout_action not in ('report_only','independent_new_map'):
            raise ValueError('单所有者恢复协议只允许报告隔离超时')
        self.quarantine_since_ns=None;self.new_map_requested=False
        self.quarantine_cause=None;self.evidence_status=None
        self.reference_generation=None

    def reset_for_new_map(self):
        """Start an independent map after an explicit recovery timeout.

        The old map's quarantine must not leak into the new map.  The session
        and transaction sequence stay intact, while all coordinate-bound
        evidence and prediction state are deliberately discarded.
        """
        self.trusted_visual_identity=None
        self.predictor.anchor=None
        self.predictor.samples=[]
        self.invalid_reason=None
        self.map_epoch=None
        self.world_frame_revision=None
        self.reference_generation=None
        self.tracking_was_lost=False
        self.ever_tracking_ok=False
        self.recovery_evidence_required=False
        self.quarantined=False
        self.anchor_hold_ns=None
        self.quarantine_since_ns=None
        self.new_map_requested=False
        self.quarantine_cause=None
        self.evidence_status=None
        self.callback_results=[]

    def _timeout_request(self, stamp, *, application_commit=False):
        """Return timeout fields for both normal and identity-failure paths."""
        if self.quarantined and self.quarantine_since_ns is None:
            self.quarantine_since_ns=int(stamp)
        elapsed=max(0, int(stamp)-self.quarantine_since_ns) if self.quarantine_since_ns is not None else 0
        timed_out=bool(self.quarantined and elapsed>=self.quarantine_timeout_ns)
        allowed_actions=('independent_new_map','legacy_new_map') if (not self.application_admission or application_commit) else ('legacy_new_map',)
        request=bool(timed_out and self.timeout_action in allowed_actions
                     and not self.new_map_requested)
        if request:self.new_map_requested=True
        return elapsed, timed_out, request

    def _quarantine(self, cause, evidence_status):
        # Preserve the original trigger: subsequent unavailable callbacks while
        # already quarantined are not independent recovery evidence.
        if not self.quarantined:
            self.quarantine_cause=cause
            self.evidence_status=evidence_status
        elif self.short_recovery and cause=='pose_jump':
            # A later discontinuity escalates an existing loss isolation. It
            # must survive subsequent geometry checks in that same map; those
            # checks cannot establish continuity across the rejected jump.
            self.quarantine_cause='pose_jump'
            self.evidence_status=evidence_status
        self.quarantined=True

    def begin_frame(self,samples):
        if self._pending_application is not None:
            raise RecoveryProtocolError('previous application admission is still pending')
        self.callback_results=[]
        for s in samples:
            if self.last_sample is not None and s.timestamp_ns<=self.last_sample.timestamp_ns:
                self.invalid_reason='imu_non_monotonic'
            self.last_sample=s
            if self.predictor.anchor is not None and s.timestamp_ns>self.predictor.anchor.timestamp_ns:
                try:self.predictor.push(s)
                except (ValueError,OverflowError) as exc:
                    self.invalid_reason=str(exc)
                    # 清空污染样本；锚点保留用于后续一致性检查，但预测不可用直到重建锚点。
                    self.predictor.samples=[]

    def check_candidate(self,T_world_from_body,timestamp_ns,map_epoch,geometry=None):
        if self.quarantined:result={'status':'unavailable','reason':'awaiting_visual_requalification'}
        elif self.invalid_reason:result={'status':'unavailable','reason':self.invalid_reason}
        else:result=self.predictor.evaluate(T_world_from_body,timestamp_ns,str(map_epoch),max_endpoint_hold_ns=self.max_hold)
        result['start_hold_ns']=self.anchor_hold_ns
        result['mode']='pre_accept_experimental'
        # Callback is a relocalization candidate even when native loses and
        # re-localizes within one TrackStereo call before Python sees LOST.
        # Preserve the IMU decision as diagnostics; native acceptance needs an
        # independent visual verifier, which this implementation does not have.
        result['visual_requalification_required']=bool(self.ever_tracking_ok or self.quarantined)
        result['independent_visual_verification']='unavailable'
        # Provisional acceptance by Tracking still needs final snapshot binding.
        if (self.short_recovery and not self.invalid_reason and self.quarantine_cause!='pose_jump' and
            map_epoch==self.map_epoch and self.trusted_visual_identity is not None and
            geometry_candidate(geometry,timestamp_ns=timestamp_ns,map_epoch=map_epoch,
                reference_generation=self.reference_generation,candidate_body=T_world_from_body)):
            motion=(self.predictor.evaluate(T_world_from_body,timestamp_ns,str(map_epoch),
                max_endpoint_hold_ns=self.max_hold) if self.predictor.anchor is not None and
                not self.invalid_reason else {'status':'unavailable','reason':self.invalid_reason or 'no_trusted_anchor'})
            if motion['status']!='inconsistent':
                result={**result,'status':'consistent','reason':'native_stereo_geometry_candidate',
                    'motion_check':motion,'independent_visual_verification':'provisional_native_geometry',
                    'evidence_kind':'stereo_map_geometry_not_imu_truth'}
        self.callback_results.append(result)
        return result

    def reset_imu_alignment(self):
        """Discard sensor history, retaining identity and unresolved recovery."""
        if self._pending_application is not None:
            raise RecoveryProtocolError('cannot reset IMU during a pending admission')
        if self.tracking_was_lost and self.ever_tracking_ok:
            self._quarantine('imu_alignment_reset_during_loss','unavailable')
        self.predictor.anchor=None;self.predictor.samples=[]
        self.last_sample=None;self.invalid_reason=None;self.anchor_hold_ns=None

    def finish_frame(self,result,body):
        if self._pending_application is not None:
            raise RecoveryProtocolError('previous application admission is still pending')
        report=self._finish_native_frame(result,body)
        if not self.application_admission:return report
        self.frame_sequence+=1
        report={**report, 'schema_version':PROTOCOL,'owner':'RecoveryCoordinator',
                'session_id':self.session_id,'frame_sequence':self.frame_sequence,'stage':'native',
                **{key:result.get(key) for key in identity_fields(result)}}
        self._pending_application=(dict(result),body,report)
        return report

    def finish_application_frame(self,feedback):
        """Commit host quality evidence before the next native frame can start.

        This is the same coordinator used by native relocalization callbacks;
        no second predictor or quarantine is maintained in the host process.
        """
        if not self.application_admission or self._pending_application is None:
            raise RecoveryProtocolError('no pending application admission')
        result,body,native=self._pending_application
        feedback=validate_feedback(feedback,native)
        if feedback['pose_admitted'] and not result.get('pose_valid'):
            raise RecoveryProtocolError('application admitted a missing native pose')
        # Before this independent map has ever supplied a trusted pose there
        # is no admitted reference to protect. A rejected startup step must
        # still reset the host's stability window, but repeatedly abandoning
        # that unqualified map prevents the estimator from settling. Never
        # apply this exception to an existing quarantine or a trusted map.
        startup_step=(self.short_recovery and not self.quarantined and
            self.trusted_visual_identity is None and self.predictor.anchor is None)
        if feedback['jump_detected'] and not startup_step:
            self._quarantine('pose_jump','unavailable')
        if feedback['inertial_anchor_admitted'] and not self.quarantined:
            self._update_anchor(result,body)
        if feedback['pose_admitted'] and not self.quarantined:
            self.trusted_visual_identity=visual_reference(result)
        stamp=int(result['timestamp_ns'])
        # Abandon the unverified old reference; never release its pose on time.
        elapsed,timed_out,request_new_map=self._timeout_request(stamp,application_commit=True)
        report={**native,'stage':'committed','anchor_available':self.predictor.anchor is not None,
                'await_visual_requalification':self.quarantined,
                'quarantine_cause':self.quarantine_cause,'evidence_status':self.evidence_status,
                'quarantine_elapsed_s':elapsed*1e-9,
                'quarantine_timed_out':timed_out,
                'request_new_map':request_new_map,'application_quality_passed':feedback['quality_passed'],
                'application_pose_admitted':feedback['pose_admitted'] and not self.quarantined}
        self._pending_application=None
        return report

    def _update_anchor(self,result,body):
        initialized_anchor=(self.allow_initialized_anchor and result.get('imu_initialized') is True)
        if (not snapshot_body_matches_frame(result,body) or
            not (initialized_anchor or (body.get('ba2_complete') is True and
                result.get('inertial_scale_finalized') is True))):return
        T=np.asarray(body['T_world_from_body'],dtype=float)
        t=int(result['timestamp_ns']);last=self.last_sample
        age=t-int(last.timestamp_ns) if last is not None else -1
        if T.shape==(4,4) and np.isfinite(T).all() and 0<=age<=self.max_hold:
            state=BodyState(t,T[:3,:3],T[:3,3],np.asarray(body['velocity_world_mps'],dtype=float),
                np.asarray(body['accel_bias_mps2'],dtype=float),np.asarray(body['gyro_bias_rps'],dtype=float),str(result.get('map_epoch')))
            self.predictor.set_trusted_state(state);self.invalid_reason=None;self.anchor_hold_ns=age
            self.predictor.push(ImuMeasurement(t,last.acceleration_mps2,last.angular_velocity_rps))

    def _finish_native_frame(self,result,body):
        incoming_epoch = result.get('map_epoch')
        prior_world = getattr(self, 'world_frame_revision', None)
        missing_world = invalid_world_revision_transition(prior_world, result.get('world_frame_revision'))
        reference_bad=invalid_reference_frame(self.reference_generation,result,self.map_epoch)
        if ((self.map_epoch is not None and incoming_epoch != self.map_epoch and not snapshot_identity_matches_epoch(result))
                or missing_world or reference_bad):
            # A failed native snapshot can contain default map fields.  Do not
            # advance the trusted identity or clear quarantine on that frame.
            failure = ('reference_frame_unverified' if reference_bad else
                       'world_revision_unavailable' if missing_world else 'identity_snapshot_unverified')
            self.predictor.anchor=None;self.predictor.samples=[];self.invalid_reason=failure
            if self.ever_tracking_ok or self.quarantined:
                self._quarantine(failure,'unavailable')
            stamp=int(result['timestamp_ns'])
            elapsed,timed_out,request_new_map=self._timeout_request(stamp)
            return {'candidates':list(self.callback_results),'direct_recovery':None,'loss_prediction':None,
                    'anchor_available':False,'await_visual_requalification':self.quarantined,
                    'quarantine_elapsed_s':elapsed*1e-9,
                    'quarantine_timed_out':timed_out,
                    'timeout_action':self.timeout_action,'quarantine_cause':self.quarantine_cause,
                    'evidence_status':self.evidence_status,'body_snapshot_consistent':False,
                    'request_new_map':request_new_map,'mode':'experimental',
                    'identity_transition_status':failure}
        body_consistent=snapshot_body_matches_frame(result,body)
        revision=result.get('world_frame_revision')
        prior_revision=getattr(self,'world_frame_revision',None)
        had_anchor=self.predictor.anchor is not None
        loss_requires_evidence=bool(self.tracking_was_lost and (had_anchor or self.recovery_evidence_required))
        revision_jump_context=False
        world_changed=(prior_revision is not None and revision != prior_revision) or (
            self.reference_generation is not None and result.get('reference_generation')!=self.reference_generation)
        self.reference_generation=result.get('reference_generation')
        if world_changed:
            self.predictor.anchor=None;self.predictor.samples=[];self.invalid_reason=None
            self.tracking_was_lost=False
        self.world_frame_revision=revision
        epoch=result.get('map_epoch')
        same_map_revision=False
        known_new_map=False
        if self.map_epoch is not None and epoch!=self.map_epoch:
            # 地图整体修正可能改变坐标系，不能直接比较修正前后的坐标。
            # 低 20 位是重大修正计数；地图或重置变化则允许重新建立独立坐标系。
            try:
                old=decode_map_epoch(self.map_epoch); new=decode_map_epoch(epoch)
                same_map_revision=(old['map_id']==new['map_id'] and old['reset_id']==new['reset_id'])
                known_new_map=not same_map_revision
            except (TypeError,ValueError,OverflowError):
                # An unknown identity is not proof of a fresh, independent map.
                same_map_revision=True
            # Retain only the fact that an anchor existed, not its coordinates.
            # A same-map revision jump is suspicious even though the old anchor
            # must be discarded before any prediction in the revised map.
            revision_jump_context=bool(had_anchor and same_map_revision and revision==prior_revision)
            self.predictor.anchor=None;self.predictor.samples=[];self.invalid_reason=None
            self.quarantined=bool(same_map_revision and self.quarantined)
            if not self.quarantined:
                self.quarantine_since_ns=None;self.new_map_requested=False
                self.quarantine_cause=None;self.evidence_status=None
            self.tracking_was_lost=False
            if known_new_map:
                self.ever_tracking_ok=False;self.recovery_evidence_required=False
                self.trusted_visual_identity=None
        if loss_requires_evidence and not known_new_map and (world_changed or same_map_revision):
            self._quarantine('reference_changed_during_loss','unavailable')
        self.map_epoch=epoch
        checked=result if self.application_admission else self.quality.check(result)
        # 已有锚点后发生异常跳变时，即使 ORB 仍声称 OK 也必须隔离；
        # 仅凭后续几秒稳定不能证明新位置属于正确的地图坐标。
        if (self.predictor.anchor is not None or revision_jump_context or self.ever_tracking_ok) and checked.get('jump_detected'):
            self._quarantine('pose_jump','unavailable')
        candidate=self.callback_results[-1] if self.callback_results else None
        # ORB 可在短暂丢失后通过惯性预测和局部地图直接恢复，不经过重定位回调。
        # 此时只能在接受之后检查；证据不足或不一致时禁止把该状态作为可信锚点。
        direct_recovery=None
        if self.tracking_was_lost and result.get('pose_valid') and candidate is None:
            if self.predictor.anchor is None:
                direct_recovery={'status':'unavailable','reason':'no_trusted_anchor'}
            elif not body_consistent:
                direct_recovery={'status':'unavailable','reason':'body_snapshot_unavailable_or_mismatched'}
            elif self.invalid_reason:
                direct_recovery={'status':'unavailable','reason':self.invalid_reason}
            else:
                direct_recovery=self.predictor.evaluate(body['T_world_from_body'],result['timestamp_ns'],str(epoch),max_endpoint_hold_ns=self.max_hold)
            direct_recovery['mode']='post_accept_direct_recovery'
            if (self.predictor.anchor is not None or self.recovery_evidence_required) and direct_recovery['status']!='consistent':
                status=direct_recovery['status']
                self._quarantine('direct_recovery_'+status,status)
        if result.get('tracking_state') in ('LOST','RECENTLY_LOST') or not result.get('pose_valid'):
            if self.ever_tracking_ok and result.get('tracking_state') in ('LOST','RECENTLY_LOST'):
                self.recovery_evidence_required=True
            self.tracking_was_lost=True
        elif result.get('pose_valid'):
            self.tracking_was_lost=False
            if result.get('tracking_state')=='OK':self.ever_tracking_ok=True
        # 被 ORB 拒绝的候选不改变当前锚点；只有未验证候选实际被接受时才隔离。
        if (self.predictor.anchor is not None and candidate is not None and candidate['status']!='consistent' and result.get('pose_valid')
                and candidate.get('reason') not in ('no_trusted_anchor','map_changed')):
            status=candidate['status']
            self._quarantine('candidate_'+status,status)
        if (self.recovery_evidence_required and candidate is not None and result.get('pose_valid')
                and candidate.get('status')!='consistent'):
            status=candidate.get('status','unavailable')
            self._quarantine('candidate_'+status,status)
        # A heuristic IMU residual can support an investigation but it is not
        # an independent visual-geometric requalification. No production
        # verifier is available yet, so even an IMU-consistent recovery remains
        # quarantined until an actual verifier or a distinct map is available.
        if (self.recovery_evidence_required and result.get('pose_valid')
                and result.get('tracking_state')=='OK' and not self.quarantined):
            self._quarantine('independent_visual_verification_unavailable','unavailable')
        if result.get('pose_valid') and result.get('tracking_state')=='OK':
            self.recovery_evidence_required=False
        visual_release=None
        if (self.short_recovery and not self.invalid_reason and self.quarantined and self.quarantine_cause!='pose_jump' and
            geometry_matches_frame(result,result.get('recovery_observation'),self.trusted_visual_identity)):
            motion=({'status':'unavailable','reason':self.invalid_reason or 'no_trusted_anchor'}
                if self.invalid_reason or self.predictor.anchor is None or not body_consistent else
                self.predictor.evaluate(body['T_world_from_body'],result['timestamp_ns'],str(epoch),
                    max_endpoint_hold_ns=self.max_hold))
            if motion['status']!='inconsistent':
                visual_release={'status':'qualified','kind':'same_reference_stereo_map_geometry',
                    'timestamp_ns':result['timestamp_ns'],'motion_check':motion,
                    'geometry':result['recovery_observation']}
                self.quarantined=False;self.quarantine_since_ns=None;self.new_map_requested=False
                self.quarantine_cause=None;self.evidence_status='stereo_map_geometry_qualified'
                self.recovery_evidence_required=False;self.tracking_was_lost=False
        prediction=None
        if self.tracking_was_lost:
            if self.quarantined and not self.short_recovery:prediction={'status':'unavailable','reason':'awaiting_visual_requalification'}
            elif self.invalid_reason:prediction={'status':'unavailable','reason':self.invalid_reason}
            else:
                try:
                    state,hold=self.predictor.predict(result['timestamp_ns'],str(epoch),max_endpoint_hold_ns=self.max_hold)
                    T=np.eye(4);T[:3,:3]=state.rotation_world_from_body;T[:3,3]=state.position_world_m
                    prediction={'status':'predicted','T_world_from_body':T.tolist(),
                        'velocity_world_mps':state.velocity_world_mps.tolist(),
                        'duration_s':(int(result['timestamp_ns'])-self.predictor.anchor.timestamp_ns)*1e-9,
                        'start_hold_ns':self.anchor_hold_ns,'endpoint_hold_ns':hold}
                except ValueError as exc:prediction={'status':'unavailable','reason':str(exc)}
            prediction['mode']='imu_only_extrapolation_not_visual_pose'
            prediction['reference_quarantined']=bool(self.quarantined)
        # BA2 完成是既有保守准入条件，不代表准确性或置信度已获证明。
        # 双目本身由基线提供米制尺度；这里不把 BA2 等同“从非米制变为米制”。
        scale_finalized=result.get('inertial_scale_finalized')
        if (not self.application_admission and body_consistent and body.get('ba2_complete') is True and checked.get('pose_valid')
                and not self.quarantined and scale_finalized is True):
            self._update_anchor(result,body)
        stamp=int(result['timestamp_ns'])
        if not self.quarantined:self.quarantine_since_ns=None;self.new_map_requested=False
        elapsed,timed_out,request_new_map=self._timeout_request(stamp)
        return {'candidates':list(self.callback_results),'direct_recovery':direct_recovery,
                'visual_release':visual_release,'loss_prediction':prediction,'anchor_available':self.predictor.anchor is not None,
                'await_visual_requalification':self.quarantined,'quarantine_elapsed_s':elapsed*1e-9,
                'quarantine_timed_out':timed_out,'timeout_action':self.timeout_action,
                'quarantine_cause':self.quarantine_cause,'evidence_status':self.evidence_status,
                'body_snapshot_consistent':body_consistent,
                'request_new_map':request_new_map,'mode':'experimental'}
