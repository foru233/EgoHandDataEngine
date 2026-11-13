"""Independent, fail-closed per-session stereo/gyro phase verification.

The finite-grid width is an objective diagnostic, not physical clock accuracy.
No IMU samples or active SLAM history are modified by this estimator.
"""
import cv2
import numpy as np


def _phase_gray_pair(left, right, scale):
    """Convert rectified BGR inputs to gray and resize only when required."""
    images = [cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) for image in (left, right)]
    if scale != 1.0:
        images = [cv2.resize(image, None, fx=scale, fy=scale,
                             interpolation=cv2.INTER_AREA) for image in images]
    return images


def rotation_errors(samples, times, gyro, camera_from_body, phase_ms):
    """Midpoint SO(3) integration, vectorized over visual frame pairs."""
    anchor = int(times[0])
    clock = (np.asarray(times, dtype=np.int64) - anchor) / 1e9 + phase_ms / 1000
    starts = np.array([(int(s[0])-anchor)/1e9 for s in samples])
    ends = np.array([(int(s[1])-anchor)/1e9 for s in samples])
    if np.any(starts < clock[0]) or np.any(ends > clock[-1]):
        raise ValueError('Phase candidates must share complete IMU coverage')
    counts = np.maximum(1, np.rint((ends-starts)*480).astype(int))
    dt = (ends-starts)/counts
    predicted = np.tile(np.eye(3), (len(samples), 1, 1))
    for index in range(int(counts.max())):
        active = np.flatnonzero(index < counts)
        nodes = starts[active] + (index+.5)*dt[active]
        w = np.stack([np.interp(nodes, clock, gyro[:, k]) for k in range(3)], axis=1)
        v = w*dt[active, None]
        angle = np.linalg.norm(v, axis=1)
        skew = np.zeros((len(v), 3, 3))
        skew[:, 0, 1], skew[:, 0, 2] = -v[:, 2], v[:, 1]
        skew[:, 1, 0], skew[:, 1, 2] = v[:, 2], -v[:, 0]
        skew[:, 2, 0], skew[:, 2, 1] = -v[:, 1], v[:, 0]
        a = np.sinc(angle/np.pi)
        b = .5*np.sinc(angle/(2*np.pi))**2
        delta = np.eye(3) + a[:, None, None]*skew + b[:, None, None]*(skew@skew)
        predicted[active] = predicted[active]@delta
    c = np.asarray(camera_from_body)
    visual = np.array([c.T@np.asarray(s[2]).T@np.asarray(s[3])@c for s in samples])
    residual = np.swapaxes(predicted, 1, 2)@visual
    return np.degrees(np.arccos(np.clip((np.trace(residual, axis1=1, axis2=2)-1)/2, -1, 1)))


def verify_session_phase(samples, times, gyro, camera_from_body):
    """Fit first half, verify a separate chronological half; unavailable stays false."""
    times = np.asarray(times, dtype=np.int64)
    gyro = np.asarray(gyro, dtype=float)
    if (len(times)<2 or gyro.shape!=(len(times),3) or not np.isfinite(gyro).all()
            or np.any(np.diff(times)<=0)):
        raise ValueError('Invalid gyro reference')
    samples = [s for s in samples if s[0]>=times[0]+200_000_000
               and s[1]<=times[-1]-150_000_000]
    waiting = {'verified':False, 'reason':'insufficient_visual_motion', 'pairs':len(samples)}
    if len(samples)<60 or samples[-1][1]-samples[0][0]<3_000_000_000:
        return waiting
    split = len(samples)//2
    boundary = samples[split-1][1]
    train = samples[:split]
    holdout = [s for s in samples[split:] if s[0]>=boundary]
    if len(holdout)<25:
        return waiting
    angular_speed = np.array([np.linalg.norm(cv2.Rodrigues(np.asarray(s[2]).T@np.asarray(s[3]))[0])
                              /((s[1]-s[0])/1e9) for s in samples])
    variation = float(np.std(angular_speed))
    if variation<.10:
        return {**waiting, 'reason':'insufficient_changing_rotation', 'speed_std_rps':variation}
    def sweep(group, phases):
        return [{'phase_ms':int(p), 'rms_deg':float(np.sqrt(np.mean(
                    rotation_errors(group,times,gyro,camera_from_body,p)**2)))} for p in phases]
    coarse = sweep(train,range(-150,201,5))
    best = min(coarse,key=lambda r:r['rms_deg'])['phase_ms']
    fine = sweep(train,range(max(-150,best-5),min(200,best+5)+1))
    chosen = min(fine,key=lambda r:r['rms_deg'])
    holdout_grid = sweep(holdout, range(-150,201,5))
    holdout_best = min(holdout_grid,key=lambda r:r['rms_deg'])['phase_ms']
    errors = rotation_errors(holdout,times,gyro,camera_from_body,chosen['phase_ms'])
    near = [r['phase_ms'] for r in coarse+fine if r['rms_deg']<=chosen['rms_deg']*1.05]
    width = max(near)-min(near) if near else 0
    reasons = []
    if not -140<chosen['phase_ms']<190:reasons.append('phase_at_search_boundary')
    if abs(holdout_best-chosen['phase_ms'])>12:reasons.append('train_holdout_disagree')
    if width>20:reasons.append('phase_objective_too_flat')
    if float(np.sqrt(np.mean(errors**2)))>.30 or float(np.percentile(errors,95))>.6:
        reasons.append('visual_gyro_disagreement')
    return {'verified':not reasons, 'reason':','.join(reasons) or 'verified_independent_rotation',
            'pairs':len(samples), 'train_pairs':len(train), 'holdout_pairs':len(holdout),
            'phase_ms':chosen['phase_ms'], 'train_rms_deg':chosen['rms_deg'],
            'holdout_rms_deg':float(np.sqrt(np.mean(errors**2))),
            'holdout_p95_deg':float(np.percentile(errors,95)),
            'holdout_best_phase_ms':holdout_best, 'objective_5percent_width_ms':width,
            'speed_std_rps':variation, 'coarse_train':coarse, 'fine_train':fine,
            'coarse_holdout':holdout_grid,
            'warning':'Image/gyro consistency check; not physical exposure timestamp certification.'}


class SessionPhasePreparation:
    """Bounded startup collector; expensive fit runs outside the live frame thread."""
    def __init__(self, camera, baseline, camera_from_body):
        from collections import deque
        from concurrent.futures import ThreadPoolExecutor
        from .visual_rotation import StereoRotationReference
        self.reference = StereoRotationReference(camera, baseline)
        self.camera_from_body = camera_from_body
        self.samples = deque(maxlen=150)
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='VioPhase')
        self.job = None
        self.next_attempt_ns = 0
        self.frames = 0
        self.message = '正在核验本次时序 · 对准有纹理的静态场景，缓慢左右转动并改变速度'

    def process(self, stamp, left, right, buffer):
        if self.job is not None and self.job.done():
            result = self.job.result()
            self.job = None
            self.next_attempt_ns = stamp+2_000_000_000
            if result['verified']:
                return result
            self.message = '本次时序尚未确认 · 请增加有纹理场景下的缓慢变速转动'
        self.frames += 1
        if self.frames%2:
            return None
        visual = self.reference.process(stamp, left, right)
        if visual['valid']:
            self.samples.append((visual['start_ns'],visual['end_ns'],np.eye(3),
                                 visual['previous_from_current_rotation']))
        # Before any SLAM submission, retain only a bounded calibration window.
        if buffer.consumed:
            raise RuntimeError('Cannot verify timing after inertial history has started')
        while buffer.rows and buffer.rows[0][0]<stamp-15_000_000_000:
            buffer.rows.popleft()
        if self.job is None and stamp>=self.next_attempt_ns and len(self.samples)>=60:
            rows = list(buffer.rows)
            times = np.array([r[0]-buffer.phase_ns for r in rows], dtype=np.int64)
            gyro = np.array([r[4:7] for r in rows])
            self.job = self.executor.submit(verify_session_phase, list(self.samples),
                                            times,gyro,self.camera_from_body)
        return None

    def close(self):
        self.executor.shutdown(wait=False, cancel_futures=True)


def prepare_recording_phase(recording, config, workspace, *, cancelled=None, on_progress=None):
    """Verify an unprepared auto-mode raw recording into a new working snapshot.

    Source recordings are read-only. Manual historical snapshots remain faithful.
    """
    if (config.timing.get('lattice_phase_mode')!='session_auto'
            or config.timing.get('session_phase_verified')):
        return config
    if cancelled is not None and cancelled.is_set():raise InterruptedError('时序核验已取消')
    from collections import deque
    from dataclasses import replace
    import json
    from pathlib import Path
    from ..storage.sdk_recording import iter_paired_frames, iter_imu_packets
    from .imu_lattice import PacketLatticeImuBuffer
    from .visual_rotation import StereoRotationReference
    geometry=config.validate_geometry();scale=config.image_scale
    camera=geometry.rectifier.P1[:3,:3].copy();camera[:2]*=scale
    reference=StereoRotationReference(camera,-geometry.rectifier.P2[0,3]/geometry.rectifier.P2[0,0])
    buffer=PacketLatticeImuBuffer(sample_rate_hz=float(config.timing.get('imu_rate_hz',480)),
        camera_to_imu_drift_ppm=float(config.timing.get('camera_to_imu_drift_ppm',0)),phase_ns=0)
    packets=iter(iter_imu_packets(recording));packet=next(packets,None)
    samples=deque(maxlen=150);verification=None
    for frame in iter_paired_frames(recording, limit=900, stride=2):
        index=frame.index
        if cancelled is not None and cancelled.is_set():raise InterruptedError('时序核验已取消')
        stamp=int(frame.device_ts)*1000
        while packet is not None and packet.device_ts*1000<=stamp+250_000_000:
            buffer.add(packet.payload,packet.device_ts);packet=next(packets,None)
        while buffer.rows and buffer.rows[0][0]<stamp-15_000_000_000:buffer.rows.popleft()
        left,right=geometry.rectifier.remap_pair(frame.right_bgr,frame.left_bgr)
        left,right=_phase_gray_pair(left,right,scale)
        visual=reference.process(stamp,left,right)
        if visual['valid']:
            samples.append((visual['start_ns'],visual['end_ns'],np.eye(3),
                            visual['previous_from_current_rotation']))
        if index%90==0:
            if on_progress:on_progress('正在独立核验录制的双目/IMU时序')
            if len(samples)<60 or not buffer.rows:continue
            rows=list(buffer.rows)
            verification=verify_session_phase(list(samples),np.array([r[0] for r in rows],dtype=np.int64),
                        np.array([r[4:7] for r in rows]),geometry.T_imu_from_orb_left[:3,:3].T)
            if verification['verified']:
                model={'schema_version':1,'type':'session_phase','config_hashes':config.describe()['hashes'],
                       'verified_at_device_ns':stamp,'verification':verification,
                       'source':'offline independent stereo/gyro preparation; original recording unchanged'}
                workspace=Path(workspace);workspace.mkdir(parents=True,exist_ok=True)
                (workspace/'session_phase_preparation.json').write_text(json.dumps(model,indent=2),encoding='utf8')
                prepared=replace(config,timing={**config.timing,'lattice_phase_ms':verification['phase_ms'],
                                                'session_phase_verified':True})
                prepared.write(workspace/'verified_config')
                return prepared
    reason=verification['reason'] if verification else 'insufficient_visual_motion'
    raise ValueError(f'录制时序无法确认（{reason}）：需要有纹理静态场景下的变速转动片段；原始录制未改写')
