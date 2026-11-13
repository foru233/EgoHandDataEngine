"""Qualify visual bootstrap and initialized VIO from one IMU_STEREO instance.

Never bypass numerical faults, world identity or each tracker's quarantine.
"""
from .pose_quality import PoseQualityGate, apply_recovery_quarantine
from .map_identity import world_identity


class HybridPoseQualityGate:
    def __init__(self, **options):
        options.pop('require_imu', None)
        self.visual = PoseQualityGate(require_imu=False, **options)
        self.inertial = PoseQualityGate(require_imu=True, **options)
        self._identity = None
        self._segment = 0

    def check(self, result):
        result = dict(result)
        visual = apply_recovery_quarantine(self.visual.check(result))
        inertial = apply_recovery_quarantine(self.inertial.check(result))
        # Native IMU initialization alone is not an accepted inertial solution:
        # BA1/BA2 can still revise map scale and origin. Never advertise a VIO
        # takeover until the native snapshot confirms BA2 and finalized scale.
        # The visual bootstrap is still the SAME native IMU_STEREO tracker,
        # not a separately computed pure-stereo trajectory.
        inertial_ready = (result.get('imu_initialized') is True
                          and result.get('ba2_complete') is True
                          and result.get('inertial_scale_finalized') is True)
        if inertial_ready and inertial.get('pose_valid'):
            out = inertial
            source = 'vio'
        else:
            out = visual
            source = 'visual_fallback'
        out['pose_source'] = source
        out['inertial_display_ready'] = bool(inertial_ready and inertial.get('pose_valid'))
        out['fallback_reason'] = (None if source == 'vio' else
            '等待 BA2 和惯性尺度最终确认' if not inertial_ready else inertial.get('quality_status'))
        out['fallback_kind'] = 'single_vio_visual_bootstrap'
        if out.get('pose_valid'):
            identity = (world_identity(out), source, out.get('display_segment'))
            if self._identity is not None and identity != self._identity:
                self._segment += 1
            self._identity = identity
        out['display_segment'] = self._segment
        return out


def make_quality_gate(mode, *, recovery_config=None, **options):
    # The offline candidate profile is an explicit opt-in shared with the
    # worker. The normal live/recorded quality configuration is unchanged.
    if mode == 'stereo_inertial':
        import os
        from pathlib import Path
        import yaml
        path=recovery_config or os.environ.get('EGOHAND_VIO_RECOVERY_CONFIG')
        if path:
            profile=yaml.safe_load(Path(path).read_text(encoding='utf8')) or {}
            if not isinstance(profile,dict):raise ValueError('VIO 恢复参数必须是字典')
            enabled=profile.get('allow_verified_stereo_bootstrap',False)
            if type(enabled) is not bool:raise ValueError('双目几何回退开关必须为布尔值')
            if enabled and profile.get('short_recovery') is not True:
                raise ValueError('双目几何回退需要短时恢复几何证据协议')
            options['allow_verified_stereo_bootstrap']=enabled
    if mode == 'hybrid':
        return HybridPoseQualityGate(**options)
    return PoseQualityGate(require_imu=mode != 'stereo', **options)
