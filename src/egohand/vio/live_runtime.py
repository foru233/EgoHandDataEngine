from .stereo_transport import SHARED_STEREO, negotiate as negotiate_stereo
"""VS030 实验性实时 VIO：独立进程、显式时序与可观察的失败状态。"""
from collections import deque
from dataclasses import replace
import json
import hashlib
import os
from pathlib import Path
import queue
import socket
import struct
import subprocess
import threading
import time
import cv2
import numpy as np
import yaml
import uuid

from .vs030_imu_candidate import parse_candidate_payload
from .vs030_geometry import prepare_vs030_vio_geometry
from .hybrid_quality import make_quality_gate
from .pose_continuity import PoseContinuityCorrector
from .localization_state import LocalizationPipeline
from .map_identity import world_identity
from .modes import DEFAULT_MODE, STEREO, validate_mode, apply_mode_environment
from .imu_lattice import LatticeImuBuffer, PacketLatticeImuBuffer, packet_lattice_phase_ns
from .recovery_contract import (PROTOCOL, FRAME_CONTEXT, RESET_IMU_ALIGNMENT,
    RecoveryProtocolError, application_feedback, send_message, negotiate_recovery)

ROOT=Path(__file__).resolve().parents[3]
_VIO_CONDA_PYTHON=Path.home()/'miniconda3/envs/egohand-vio/python.exe'
# 优先使用现有独立 VIO 环境；可通过环境变量指定其他解释器，无需安装或替换 GUI 环境。
PYTHON=Path(os.environ.get('EGOHAND_VIO_PYTHON') or (str(_VIO_CONDA_PYTHON) if _VIO_CONDA_PYTHON.is_file() else 'python.exe'))
from ..sources.vendor_placeholder import CALIBRATION_PATH as CALIBRATION
HEADER=struct.Struct('<qIII'); IMU=struct.Struct('<q6f')
# Diagnostic raw packet archive: magic, version, header_size, device_ts, SDK capture/arrival ns, payload_size.
IMU_RAW_HEADER=struct.Struct('<4sHHQQQI')
MAX_IMU_GAP_NS=30_000_000
IMU_HISTORY_KEEP_NS=1_000_000_000
MAX_WORKER_RESTARTS=2  # 避免原生故障时无限重启。

def vio_subprocess_environment(python=PYTHON, *, role='live', recovery_protocol=1):
    """隔离 VIO 运行库搜索路径，不改动 GUI 进程或系统环境。"""
    env=os.environ.copy()
    registry=ROOT/'src/egohand/vio/native/bin/active_bridges.json'
    if registry.is_file() and not env.get('EGOHAND_VIO_LIBRARY'):
        deployment=json.loads(registry.read_text(encoding='utf8'))
        if role not in ('live','atlas'):
            raise ValueError('未知 VIO 部署角色')
        flags=deployment.get('roles',{}).get(role,{}).get('environment',{})
        allowed={'EGOHAND_VIO_GAUGE_CONSTRAINT','EGOHAND_VIO_WORLD_FRAME_DISABLE',
                 'EGOHAND_VIO_DEPTH_GATE','EGOHAND_VIO_POINT_CLAMP',
                 'EGOHAND_VIO_ENTRY_PRUNE','EGOHAND_VIO_GAUGE_FIX','EGOHAND_VIO_CHEIRALITY_GUARD'}
        if any(key not in allowed or value not in ('0','1') for key,value in flags.items()):
            raise ValueError('VIO 部署清单包含无效功能开关')
        env.update(flags)
    env['PYTHONPATH']=str(ROOT/'src')+os.pathsep+env.get('PYTHONPATH','')
    env.setdefault('EGOHAND_VIO_RECOVERY_PROTOCOL',str(recovery_protocol))
    executable=Path(python)
    prefix=executable.parent
    if executable.is_absolute() and (prefix/'conda-meta').is_dir():
        env['CONDA_PREFIX']=str(prefix)
        dirs=(prefix,prefix/'Scripts',prefix/'Library'/'bin')
        env['PATH']=os.pathsep.join(str(p) for p in dirs)+os.pathsep+env.get('PATH','')
    return env

def _recv(sock,n):
    result=bytearray()
    while len(result)<n:
        chunk=sock.recv(n-len(result))
        if not chunk:raise EOFError('VIO 子进程连接已关闭')
        result.extend(chunk)
    return bytes(result)

def _send_tracking_frame(sock, stamp, left, right, measurements, *, recovery_protocol=1,
                         reset_imu_alignment=False, shared_transport=None):
    packet = b''.join(IMU.pack(*row) for row in measurements)
    if shared_transport is not None:
        if recovery_protocol != PROTOCOL:
            raise ValueError('shared stereo requires negotiated frame context')
        descriptor=shared_transport.publish(left,right)
        sock.sendall(HEADER.pack(int(stamp),left.shape[1],left.shape[0],len(measurements)))
        sock.sendall(FRAME_CONTEXT.pack((RESET_IMU_ALIGNMENT if reset_imu_alignment else 0)|SHARED_STEREO))
        sock.sendall(struct.pack('<I',len(descriptor))+descriptor)
        if packet:sock.sendall(packet)
        return
    sock.sendall(HEADER.pack(int(stamp), left.shape[1], left.shape[0], len(measurements)))
    if recovery_protocol==PROTOCOL:
        sock.sendall(FRAME_CONTEXT.pack(RESET_IMU_ALIGNMENT if reset_imu_alignment else 0))
    sock.sendall(memoryview(left).cast('B'))
    sock.sendall(memoryview(right).cast('B'))
    if packet:
        sock.sendall(packet)

class CandidateImuBuffer:
    """实测候选：两个 ns 时间字段、加速度 xyz、角速度 xyz；语义未获厂商确认。"""
    def __init__(self,*,camera_to_imu_lag_ns=0):
        self.rows=deque();self.last_a=0;self.last_b=0
        self.camera_to_imu_lag_ns=int(camera_to_imu_lag_ns)
    def add(self,payload):
        added=[]
        for row in parse_candidate_payload(payload):
            # t_imu=t_camera+lag，转回相机时间为 t_camera=t_imu-lag。
            row=replace(row,time_a_raw=row.time_a_raw-self.camera_to_imu_lag_ns,
                            time_b_raw=row.time_b_raw-self.camera_to_imu_lag_ns)
            if row.time_a_raw<=self.last_a or row.time_b_raw<=self.last_b:raise ValueError('IMU 时间非递增')
            self.rows.append(row);added.append(row);self.last_a=row.time_a_raw;self.last_b=row.time_b_raw
        if len(self.rows)>60000:raise OverflowError('IMU 缓冲超限')
        return added
    def reset(self):
        self.rows.clear();self.last_a=0;self.last_b=0
    def interval(self,start,end):
        rows=list(self.rows); out=[]
        bt=np.array([r.time_b_raw for r in rows],dtype=np.int64)
        for row in rows:
            t=row.time_a_raw
            if t<=start or t>end:continue
            index=int(np.searchsorted(bt,t))
            if index==0 or index==len(rows):continue
            before,after=rows[index-1],rows[index]
            if after.time_b_raw-before.time_b_raw>MAX_IMU_GAP_NS:raise ValueError(f'陀螺仪时间缺口过大：{(after.time_b_raw-before.time_b_raw)/1e6:.2f} ms')
            alpha=(t-before.time_b_raw)/(after.time_b_raw-before.time_b_raw)
            gyro=(1-alpha)*np.array(before.values[3:])+alpha*np.array(after.values[3:])
            out.append((t,*row.values[:3],*gyro))
        if not out or max(b-a for a,b in zip([start]+[r[0] for r in out],[r[0] for r in out]+[end]))>MAX_IMU_GAP_NS:
            raise ValueError(f'IMU 帧间样本不足或存在缺口：样本 {len(out)}，帧间 {(end-start)/1e6:.2f} ms')
        while len(self.rows)>2 and self.rows[1].time_a_raw<=end-IMU_HISTORY_KEEP_NS:self.rows.popleft()
        return out

def live_settings(timing):
    config=timing.get('orb_config')
    if config:
        return (ROOT/str(config)).resolve()
    return Path(os.environ.get('EGOHAND_VIO_CONFIG') or
                ROOT/'configs/vio/vs030_rectified_preliminary.yaml').resolve()


def _environment_identity(environment):
    """Return a stable identity for every VIO policy sent to the child.

    The persistent worker is safe to reuse only when the effective child
    environment is unchanged.  Include file content hashes for path-valued
    policy inputs so editing a recovery YAML in place cannot silently reuse an
    old coordinator.
    """
    identity = []
    for key, value in sorted(environment.items()):
        if not key.startswith('EGOHAND_VIO_'):
            continue
        item = [key, value]
        if key.endswith(('_CONFIG', '_RECOVERY_CONFIG', '_LIBRARY')):
            path = Path(value)
            if path.is_file():
                item.append(hashlib.sha256(path.read_bytes()).hexdigest())
        identity.append(tuple(item))
    return tuple(identity)


def live_hand_config(descriptor, camera_geometry, mode, effective_timing):
    """Bind configured files AND the per-session phase actually used for frames."""
    value={'schema_version':1,'base_config_fingerprint':descriptor['fingerprint'],
        'calibration_file_sha256':descriptor['hashes']['calibration'],
        'camera_geometry_sha256':camera_geometry,'camera_frame':'raw_cam0_channel4',
        'mode':mode,'effective_timing':dict(effective_timing)}
    value['fingerprint']=hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()
    return value


class LiveVioWorker:
    """Own the isolated ORB process across capture sessions in one GUI."""

    def __init__(self):
        self.proc=None
        self.settings=None
        self.log_path=None
        self.last_identity=None
        self._lock=threading.Lock()

    def preload(self):
        from .config_snapshot import resolve_live_config
        config=resolve_live_config()
        self.request_session(None,config.settings,'preload')

    def request_session(self, port, settings, session, *, mode=DEFAULT_MODE,
                        environment=None, library_path=None):
        mode=validate_mode(mode)
        if mode=='hybrid':
            from .hybrid_settings import visual_bootstrap_settings
            settings=visual_bootstrap_settings(settings)
        from .library_manifest import select_library
        explicit_environment=dict(environment or {})
        env=vio_subprocess_environment(recovery_protocol=PROTOCOL)
        if explicit_environment:
            # An explicit profile owns the child policy, including disabled
            # experiments. Do not inherit a previous shell's native switches.
            env={k:v for k,v in env.items() if not k.startswith(('EGOHAND_VIO_', 'EGOHAND_TEST_'))}
        env.update(explicit_environment)
        env=apply_mode_environment(env,mode)
        env['EGOHAND_VIO_RECOVERY_PROTOCOL']=str(PROTOCOL)
        library=Path(library_path).resolve() if library_path is not None else None
        if library is None and env.get('EGOHAND_VIO_LIBRARY'):
            library=Path(env['EGOHAND_VIO_LIBRARY']).resolve()
        if library is None:
            library=select_library(ROOT/'src/egohand/vio/native')
        if mode=='hybrid':
            library=Path(__file__).resolve().parent/'native'/'bin'/'egohand_vio_visual_bootstrap.dll'
        env['EGOHAND_VIO_CONFIG']=str(settings)
        env['EGOHAND_VIO_LIBRARY']=str(library)
        env['PATH']=str(library.parent)+os.pathsep+env.get('PATH','')
        if not library.is_file():
            raise FileNotFoundError(f'指定的 VIO 候选桥接库不存在：{library}')
        identity=(mode, settings, hashlib.sha256(settings.read_bytes()).hexdigest(),
                  library, hashlib.sha256(library.read_bytes()).hexdigest() if library.is_file() else None,
                  _environment_identity(env))
        with self._lock:
            if self.proc is not None and (self.proc.poll() is not None or identity!=self.settings):
                self._close()
            reused=self.proc is not None
            if not reused:
                self.log_path=ROOT/'logs'/f'vio_vs030_worker_{time.time_ns()}.log'
                self.log_path.parent.mkdir(exist_ok=True)
                with self.log_path.open('w',encoding='utf8') as log:
                    self.proc=subprocess.Popen(
                        [str(PYTHON),'-m','egohand.vio.live_subprocess','--live-sessions'],
                        stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,env=env,
                        creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0)|0x00008000,
                    )
                self.settings=identity
                self.last_identity={
                    'mode':mode,'session':session,'pid':getattr(self.proc,'pid',None),
                    'library':str(library),'library_sha256':hashlib.sha256(library.read_bytes()).hexdigest(),
                    'settings':str(settings),'settings_sha256':hashlib.sha256(settings.read_bytes()).hexdigest(),
                    'recovery_config':env.get('EGOHAND_VIO_RECOVERY_CONFIG'),
                    'recovery_config_sha256':(
                        hashlib.sha256(Path(env['EGOHAND_VIO_RECOVERY_CONFIG']).read_bytes()).hexdigest()
                        if env.get('EGOHAND_VIO_RECOVERY_CONFIG') and Path(env['EGOHAND_VIO_RECOVERY_CONFIG']).is_file()
                        else None),
                    'reused':False,
                }
            elif self.last_identity is not None:
                self.last_identity={**self.last_identity,'session':session,'reused':True}
            try:
                self.proc.stdin.write((json.dumps([port,session])+'\n').encode('ascii'))
                self.proc.stdin.flush()
            except OSError as exc:
                self._close()
                raise RuntimeError(f'无法连接 ORB-SLAM3 进程，详见 {self.log_path}') from exc
            return self.proc,reused,self.log_path

    def close(self):
        with self._lock:
            self._close()

    def _close(self):
        proc,self.proc=self.proc,None
        self.settings=None
        if proc is None:return
        try:proc.stdin.close()
        except OSError:pass
        try:proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3)


class LiveVio:
    def __init__(self,source,on_status,timing=None,*,worker=None,mode=DEFAULT_MODE,
                 config_snapshot=None, environment=None, library_path=None):
        self.mode=validate_mode(mode)
        self.use_imu=self.mode!=STEREO
        self.source=source;self.on_status=on_status;self.frames=queue.Queue(maxsize=1)
        self.source_session_id=uuid.uuid4().hex
        self.stop_event=threading.Event();self.thread=None;self.sock=None
        self._owns_worker=worker is None
        self.worker=LiveVioWorker() if worker is None else worker
        self.ready_event=threading.Event();self.ready=False;self.startup_error=None
        self.motion_event=threading.Event();self.motion_samples=0
        self.timing_override=dict(timing) if timing is not None else None
        self.config_snapshot=config_snapshot
        self.vio_environment=dict(environment or {})
        self.vio_library_path=library_path
        self.defer_imu=False
        self._frame_stats_lock=threading.Lock()
        self._frame_stats={'submitted':0,'deferred':0,'replaced':0,'dequeued':0,'restart_discarded':0}
    def start(self, *, defer_imu=False):
        self.stop_event.clear();self.ready_event.clear();self.ready=False;self.startup_error=None
        self.motion_event.clear();self.motion_samples=0
        self.defer_imu=defer_imu
        if self.use_imu and not defer_imu:self.source.enable_imu_capture()
        self.thread=threading.Thread(target=self._run,name='LiveVio',daemon=True);self.thread.start()
    def wait_ready(self,timeout=60.0):
        """Wait for ORB-SLAM3 vocabulary/settings initialization before starting camera streaming."""
        if not self.ready_event.wait(timeout):
            self.startup_error=f'ORB-SLAM3 初始化超时（{timeout:.0f}s）；未启动相机数据流，避免 IMU 队列在加载期间溢出'
            return False
        return bool(self.ready and self.thread is not None and self.thread.is_alive())
    def submit(self,frame):
        submitted_ns=time.perf_counter_ns()
        with self._frame_stats_lock:
            if self.defer_imu and not self.ready:
                self._frame_stats['deferred']+=1
                return
            self._frame_stats['submitted']+=1
            try:self.frames.put_nowait((frame,submitted_ns))
            except queue.Full:
                try:
                    self.frames.get_nowait()
                    self._frame_stats['replaced']+=1
                except queue.Empty:pass
                self.frames.put_nowait((frame,submitted_ns))
    def _take_frame(self,timeout):
        frame,submitted_ns=self.frames.get(timeout=timeout)
        dequeued_ns=time.perf_counter_ns()
        with self._frame_stats_lock:self._frame_stats['dequeued']+=1
        return frame,submitted_ns,dequeued_ns
    def frame_queue_stats(self):
        """Cumulative instance counters, including warmup and restarted sessions.

        These count VIO queue activity, not camera capture losses or accepted poses.
        """
        with self._frame_stats_lock:return dict(self._frame_stats)
    def close(self):
        self.stop_event.set()
        sock=self.sock
        if sock:
            try:sock.shutdown(socket.SHUT_RDWR)
            except OSError:pass
        if self.thread:
            self.thread.join(timeout=12)
            if self.thread.is_alive():
                self.worker.close()
                self.thread.join(timeout=3)
    def _read_message(self):
        length=struct.unpack('<I',_recv(self.sock,4))[0]
        if length>100000:raise ValueError('VIO 响应过大')
        return json.loads(_recv(self.sock,length).decode('utf8'))
    def _commit_recovery(self,result):
        send_message(self.sock,application_feedback(result))
        reply=self._read_message()
        if reply.get('type')!='recovery_committed':
            raise RecoveryProtocolError(reply.get('message','recovery commit missing'))
        return reply.get('recovery_preaccept')
    def _run(self):
        restarts=0
        while not self.stop_event.is_set():
            retry=self._run_session()
            if not retry or self.stop_event.is_set():return
            if restarts>=MAX_WORKER_RESTARTS:
                self.on_status({'type':'error','message':f'ORB-SLAM3 多次异常退出，已停止自动重启；日志：{self.worker.log_path}'})
                return
            restarts+=1
            self.ready=False
            self.on_status({'type':'vio_reset','message':f'ORB-SLAM3 异常退出，正在重启并新建地图（{restarts}/{MAX_WORKER_RESTARTS}）；旧地图位置不可直接连接'})
            if self.stop_event.wait(1.0):return
            try:
                while True:
                    with self._frame_stats_lock:
                        self.frames.get_nowait()
                        self._frame_stats['restart_discarded']+=1
            except queue.Empty:pass
            try:
                if self.use_imu and not self.defer_imu:self.source.enable_imu_capture(allow_running=True)
            except Exception as exc:
                self.on_status({'type':'error','message':f'重启 VIO 时无法重新启用 IMU：{exc}'})
                return

    def _run_session(self):
        listener=None;log=None;audit=None;imu_raw=None;imu_raw_path=None;imu_raw_count=0
        proc=None;worker_failed=False;retry=False;worker_log=None;phase_preparation=None
        shared_transport=None
        try:
            from .config_snapshot import resolve_live_config, load_config_snapshot
            config=self.config_snapshot or resolve_live_config()
            if self.timing_override is not None:
                config=load_config_snapshot(calibration=config.calibration,
                    settings=live_settings(self.timing_override),timing=config.timing_path,
                    quality=config.quality_path,source='live_override',timing_override=self.timing_override)
            geometry=config.validate_geometry(prepare_vs030_vio_geometry(config.calibration))
            from ..calibration.loader import load_calibration
            from .hand_world import calibration_geometry_fingerprint
            from .live_hand_join import live_capture_binding
            descriptor=config.describe()
            camera_geometry=calibration_geometry_fingerprint(
                load_calibration(config.calibration,kalibr_left_camera='cam0'))
            timing=dict(config.timing)
            listener=socket.socket();listener.bind(('127.0.0.1',0));listener.listen(1);listener.settimeout(.5)
            settings=config.settings
            (ROOT/'logs').mkdir(exist_ok=True)
            log_path=ROOT/'logs'/f'vio_vs030_live_{time.time_ns()}.log'
            log_path.with_name(log_path.stem+'_config.json').write_text(
                json.dumps(config.describe(),ensure_ascii=False,indent=2),encoding='utf8')
            log=open(log_path,'w',encoding='utf8')
            audit=open(log_path.with_name(log_path.stem+'_poses.jsonl'),'w',encoding='utf8',buffering=1)
            if self.use_imu:
                imu_raw_path=log_path.with_name(log_path.stem+'_imu_raw.bin')
                imu_raw=open(imu_raw_path,'wb',buffering=0)
            quality_cfg=dict(config.quality)
            continuity=PoseContinuityCorrector(
                enabled=bool(quality_cfg.pop('pose_continuity',False)),
                max_speed_mps=float(quality_cfg.pop('continuity_max_speed_mps',3.0)),
                min_jump_m=float(quality_cfg.pop('continuity_min_jump_m',0.25)),
            )
            # 丢失恢复后重新等待稳定，不将两帧一致当成已验证重定位正确。
            quality=make_quality_gate(self.mode,resume_same_map=False,
                recovery_config=self.vio_environment.get('EGOHAND_VIO_RECOVERY_CONFIG'),**quality_cfg)
            if self.stop_event.is_set():return
            proc,reused,worker_log=self.worker.request_session(
                listener.getsockname()[1],settings.resolve(),log_path.stem,mode=self.mode,
                environment=self.vio_environment,library_path=self.vio_library_path)
            log.write(f'ORB-SLAM3 进程日志：{worker_log}\n')
            identity=getattr(self.worker,'last_identity',None)
            if identity:
                log.write('VIO child identity: '+json.dumps(identity,ensure_ascii=False)+'\n')
            self.on_status({'type':'status','message':('正在复用 ORB-SLAM3 词典，准备新会话…' if reused
                else '正在加载 ORB-SLAM3 词典，定位将在加载完成后启动…'),
                'vio_identity':identity})
            deadline=time.monotonic()+60
            while not self.stop_event.is_set():
                try:self.sock,_=listener.accept();break
                except socket.timeout:
                    if proc.poll() is not None:raise RuntimeError(f'VIO 子进程启动失败，详见 {worker_log}')
                    if time.monotonic()>deadline:raise TimeoutError('VIO 子进程连接超时')
            if self.stop_event.is_set():return
            self.sock.settimeout(45)
            self.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            message=self._read_message()
            if message.get('type')!='ready':
                worker_failed=True
                raise RuntimeError(message.get('message','VIO 初始化失败'))
            recovery_protocol=negotiate_recovery(message,
                requested=int(self.vio_environment.get('EGOHAND_VIO_RECOVERY_PROTOCOL',os.environ.get('EGOHAND_VIO_RECOVERY_PROTOCOL',str(PROTOCOL)))),
                enabled=self.use_imu and self.vio_environment.get('EGOHAND_VIO_DISABLE_RECOVERY',os.environ.get('EGOHAND_VIO_DISABLE_RECOVERY'))!='1')
            shared_transport=negotiate_stereo(message,recovery_protocol,
                requested=self.vio_environment.get('EGOHAND_VIO_SHARED_STEREO',os.environ.get('EGOHAND_VIO_SHARED_STEREO'))=='1')
            localization=LocalizationPipeline(self.mode,quality,continuity=continuity,
                recovery_commit=self._commit_recovery if recovery_protocol==PROTOCOL else None)
            lag_ms=float(timing.get('camera_to_imu_lag_ms',0.0))
            if not np.isfinite(lag_ms) or abs(lag_ms)>1000:raise ValueError('相机—IMU 时间偏移无效')
            alignment=str(timing.get('imu_alignment','timestamp')).strip().lower()
            if alignment not in ('timestamp','lattice'):raise ValueError(f'未知的 IMU 对齐方式：{alignment}')
            drift_ppm=float(timing.get('camera_to_imu_drift_ppm',0.0))
            lattice_reference=timing.get('lattice_reference','packet')
            if lattice_reference not in ('packet','count'):raise ValueError('未知的格点参考方式')
            packet_lattice=alignment=='lattice' and lattice_reference=='packet'
            phase_auto=self.use_imu and timing.get('lattice_phase_mode','manual')=='session_auto'
            if phase_auto and not packet_lattice:
                raise ValueError('自动会话时序核验只支持 IMU 包格点模式')
            image_scale=float(timing.get('image_scale',1.0))
            if not (0.25<=image_scale<=1.0):raise ValueError(f'image_scale 超出范围：{image_scale}')
            if not self.use_imu:
                buffer=None
                mode_text='纯双目视觉'
            elif alignment=='lattice':
                options=dict(sample_rate_hz=float(timing.get('imu_rate_hz',480.0)),
                             camera_to_imu_drift_ppm=drift_ppm,capacity=20000)
                buffer=(PacketLatticeImuBuffer(phase_ns=packet_lattice_phase_ns(
                            {**timing,'lattice_phase_mode':'manual'} if phase_auto or not self.use_imu else timing),**options)
                        if packet_lattice else LatticeImuBuffer(**options))
                mode_text=f'帧格点 {buffer.sample_rate_hz:.1f} Hz · {lattice_reference}'
            else:
                buffer=CandidateImuBuffer(camera_to_imu_lag_ns=round(lag_ms*1e6))
                mode_text=f'时间戳偏移 {lag_ms:+.1f} ms'
            effective_timing={'imu_enabled':self.use_imu,'alignment':alignment,
                'lattice_reference':lattice_reference,'drift_ppm':drift_ppm,
                'camera_to_imu_lag_ms':lag_ms,'image_scale':image_scale,
                'phase_ns':buffer.phase_ns if self.use_imu and packet_lattice else None,
                'sample_rate_hz':getattr(buffer,'sample_rate_hz',None),
                'phase_source':'pending_session_verification' if phase_auto else 'configured_candidate'}
            live_config=live_hand_config(descriptor,camera_geometry,self.mode,effective_timing)
            previous=None
            if phase_auto:
                from .session_phase import SessionPhasePreparation
                camera=geometry.rectifier.P1[:3,:3].copy();camera[:2]*=image_scale
                baseline=-geometry.rectifier.P2[0,3]/geometry.rectifier.P2[0,0]
                phase_preparation=SessionPhasePreparation(camera,baseline,geometry.T_imu_from_orb_left[:3,:3].T)
            if self.stop_event.is_set():return
            if self.use_imu and self.defer_imu:self.source.enable_imu_capture(allow_running=True)
            self.ready=True
            self.ready_event.set()
            self.on_status({'type':'status','mode':self.mode,'message':(f'ORB-SLAM3 已就绪，等待双目与 IMU · {mode_text}（实验）' if self.use_imu else '纯双目视觉已就绪，等待图像')})

            def pull_imu(packets):
                """从独立 IMU 队列取一个 SDK 包；返回是否取到。"""
                nonlocal imu_raw_count, previous
                vio_queue=self.source.imu_vio_queue
                if vio_queue is None:
                    raise RuntimeError(self.source.imu_vio_error or '实时 VIO IMU 队列不可用；检查采集溢出或数据源状态')
                try:item=vio_queue.get(timeout=.05)
                except queue.Empty:return False
                if len(item)==4:
                    imu_device_ts,imu_capture_ns,imu_arrival_ns,payload=item
                    packets.append({'device_ts_raw':int(imu_device_ts),
                                    'capture_ts_ns':int(imu_capture_ns),
                                    'arrival_ts_ns':int(imu_arrival_ns)})
                elif len(item)==2:  # backward-compatible legacy worker during staged rollout
                    imu_device_ts,payload=item
                    imu_capture_ns=0;imu_arrival_ns=0
                    packets.append({'device_ts_raw':int(imu_device_ts),'capture_ts_ns':None,'arrival_ts_ns':None})
                else:
                    raise ValueError('SDK IMU queue record has an unsupported layout')
                if imu_raw is not None:
                    imu_raw.write(IMU_RAW_HEADER.pack(b'IMUR',1,IMU_RAW_HEADER.size,
                        int(imu_device_ts),int(imu_capture_ns),int(imu_arrival_ns),len(payload)))
                    imu_raw.write(payload);imu_raw_count+=1
                try:added=buffer.add(payload,imu_device_ts) if packet_lattice else buffer.add(payload)
                except OverflowError:
                    if phase_preparation is not None:
                        raise RuntimeError('时序核验期间 IMU 缓冲溢出，请重新准备设备')
                    buffer.reset();previous=None;localization.reset_imu_alignment()
                    self.on_status({'type':'status','message':'IMU 缓冲溢出，已重建对齐并等待下一帧'})
                    return False
                if not self.motion_event.is_set():
                    for row in added:
                        if max(abs(row.values[3]),abs(row.values[4]),abs(row.values[5]))>0.15:
                            self.motion_samples+=1
                    if self.motion_samples>=8:self.motion_event.set()
                return True

            while not self.stop_event.is_set():
                try:frame,submitted_ns,dequeued_ns=self._take_frame(timeout=.1)
                except queue.Empty:continue
                stamp=int(frame.metadata['sdk_device_timestamp_raw'])*1000 # 实测帧增量对应微秒；尚未获厂商说明。
                span_s=max(0.0,(stamp-previous)/1e9) if previous is not None else 0.0
                deadline=time.monotonic()+min(max(.6,span_s+.5),8.0)
                imu_sdk_packets=[]
                if previous is not None and stamp<=previous:raise ValueError('相机设备时间非递增')
                if not self.use_imu:
                    measurements=[]
                elif packet_lattice:
                    while buffer.covered_until_ns<stamp and not self.stop_event.is_set() and time.monotonic()<deadline:
                        pull_imu(imu_sdk_packets)
                    if self.stop_event.is_set():break
                    if buffer.covered_until_ns<stamp:
                        self.on_status({'type':'status','message':'IMU 包格点未覆盖相机帧，等待后续数据'})
                        continue
                    if not buffer.rows or buffer.rows[0][0]>stamp:continue
                    try:measurements=[] if previous is None else buffer.interval(previous,stamp)
                    except ValueError as exc:
                        self.on_status({'type':'status','message':str(exc)+'，重建对齐'})
                        previous=None;localization.reset_imu_alignment()
                        continue
                elif alignment=='lattice':
                    need=0 if previous is None else buffer.expected(previous,stamp)
                    while buffer.pending()<need and not self.stop_event.is_set() and time.monotonic()<deadline:
                        pull_imu(imu_sdk_packets)
                    if buffer.pending()<need:
                        self.on_status({'type':'status','message':f'IMU 样本不足（需要 {need}，可用 {buffer.pending()}），重建对齐'})
                        buffer.reset();previous=None;localization.reset_imu_alignment();continue
                    if previous is None:
                        # 首帧不按时间戳裁剪（相机与 IMU 的标签域可能相差数秒）：
                        # 丢弃首帧之前已在缓冲里的样本，从首帧之后到达的样本开始计数对齐。
                        buffer.reset();measurements=[]
                    else:
                        measurements=buffer.consume(previous,stamp)
                        if measurements is None:raise ValueError('IMU 格点样本不足')
                else:
                    while buffer.last_b<stamp and not self.stop_event.is_set() and time.monotonic()<deadline:
                        pull_imu(imu_sdk_packets)
                    if buffer.last_b<stamp:continue
                    measurements=[] if previous is None else buffer.interval(previous,stamp)
                imu_ready_ns=time.perf_counter_ns()
                left,right=geometry.rectifier.remap_pair(frame.right_image,frame.left_image)
                left=np.ascontiguousarray(cv2.cvtColor(left,cv2.COLOR_BGR2GRAY));right=np.ascontiguousarray(cv2.cvtColor(right,cv2.COLOR_BGR2GRAY))
                if image_scale!=1.0:
                    left=cv2.resize(left,None,fx=image_scale,fy=image_scale,interpolation=cv2.INTER_AREA)
                    right=cv2.resize(right,None,fx=image_scale,fy=image_scale,interpolation=cv2.INTER_AREA)
                rectified_ns=time.perf_counter_ns()
                if phase_preparation is not None:
                    verified=phase_preparation.process(stamp,left,right,buffer)
                    if verified is not None:
                        buffer.rephase_before_tracking(round(verified['phase_ms']*1e6))
                        effective_timing={**effective_timing,'phase_ns':buffer.phase_ns,
                            'phase_source':'independently_verified_session',
                            'verification_sha256':hashlib.sha256(json.dumps(verified,sort_keys=True,
                                allow_nan=False).encode()).hexdigest()}
                        live_config=live_hand_config(descriptor,camera_geometry,self.mode,effective_timing)
                        model={'type':'session_phase','schema_version':1,
                               'config_hashes':config.describe()['hashes'],
                               'verified_at_device_ns':stamp,'verification':verified,
                               'message':f"本次时序已确认 {verified['phase_ms']:+d} ms · 等待 IMU 初始化"}
                        log_path.with_name(log_path.stem+'_session_phase.json').write_text(
                            json.dumps(model,ensure_ascii=False,indent=2),encoding='utf8')
                        self.on_status(model)
                        phase_preparation.close();phase_preparation=None
                    else:
                        self.on_status({'type':'status','message':phase_preparation.message})
                    # Always use a fresh frame after asynchronous verification completes.
                    continue
                _send_tracking_frame(self.sock,stamp,left,right,measurements,
                    recovery_protocol=recovery_protocol,
                    reset_imu_alignment=localization.take_imu_alignment_reset(),shared_transport=shared_transport)
                result=self._read_message()
                if shared_transport is not None and result.get('image_transport')!='shared_memory_v1':
                    raise RuntimeError('worker did not confirm shared stereo input')
                worker_return_ns=time.perf_counter_ns()
                if result.get('type')=='error':
                    worker_failed=True
                    if result.get('code')=='VIO_NUMERICAL_FAULT' and result.get('recoverable') is True:
                        raise ConnectionError(result.get('message'))
                    raise RuntimeError(result.get('message'))
                if result.get('matrix') is not None:
                    result['matrix']=(np.array(result['matrix'])@geometry.T_orb_left_from_gui_left).tolist()
                result=localization.process(result,measurements)
                # Additive GUI-only display contract.  This does not change
                # pose admission, the formal matrix, or world-hand joins.
                result['display_pose_contract']={
                    'schema_version':1,
                    'body_frame':'imu_body',
                    'camera_frame':'gui_left_camera',
                    'T_body_from_camera':(
                        getattr(geometry,'T_imu_from_orb_left',np.eye(4)) @ geometry.T_orb_left_from_gui_left
                    ).tolist(),
                    'reference_frame':'map',
                }
                raw_matrix=result.get('raw_matrix')
                if result.get('raw_pose_valid') and raw_matrix is not None:
                    result['display_visual_candidate']={
                        'schema_version':1,'timestamp_ns':result.get('timestamp_ns'),
                        'frame_id':result.get('frame_id'),
                        'reference_id':world_identity(result),
                        'matrix':raw_matrix,
                        'coordinate_frame':'map',
                    }
                result['live_capture']=live_capture_binding(frame,self.source_session_id)
                result['live_config']=dict(live_config)
                admitted_ns=time.perf_counter_ns()
                result['runtime_performance']={
                    'schema_version':1,'clock':'host_perf_counter_ns',
                    'scope':'LiveVio submit to application admission; excludes capture/decode before submit and GUI/hands/recording after admission',
                    'exposure_latency_verified':False,
                    'timing_ms':{
                        'queue_residence':(dequeued_ns-submitted_ns)/1e6,
                        'imu_alignment_wait':(imu_ready_ns-dequeued_ns)/1e6,
                        'rectification':(rectified_ns-imu_ready_ns)/1e6,
                        'worker_roundtrip':(worker_return_ns-rectified_ns)/1e6,
                        'admission':(admitted_ns-worker_return_ns)/1e6,
                        'submit_to_admission':(admitted_ns-submitted_ns)/1e6},
                    'frame_queue':self.frame_queue_stats(),
                    'frame_queue_capacity':1,'frame_queue_policy':'replace_oldest_pending'}
                result['imu_alignment']=alignment if self.use_imu else 'disabled'
                if self.use_imu and alignment=='lattice':
                    result['imu_lattice']={'pending':buffer.pending(),'consumed':buffer.consumed,
                                           'gap_events':buffer.gap_events,'rate_hz':buffer.sample_rate_hz,
                                           'drift_ppm':drift_ppm,'reference':lattice_reference}
                    if packet_lattice:result['imu_lattice']['phase_ms']=buffer.phase_ns/1e6
                elif self.use_imu:
                    result['camera_to_imu_lag_ms']=lag_ms
                md=frame.metadata
                result['sdk_timestamp_audit']={
                    'camera_device_ts_raw':int(md.get('sdk_device_timestamp_raw',0)),
                    'camera_device_ts_to_ns_candidate':'raw * 1000 (microseconds assumption; unverified)',
                    'sdk_clock_domain':'sdk_internal_monotonic_ns',
                    'camera_capture_ts_ns_channel_4':int(md.get('sdk_capture_ts_ns_channel_4',0)),
                    'camera_capture_ts_ns_channel_5':int(md.get('sdk_capture_ts_ns_channel_5',0)),
                    'camera_arrival_ts_ns_channel_4':int(md.get('sdk_arrival_ts_ns_channel_4',0)),
                    'camera_arrival_ts_ns_channel_5':int(md.get('sdk_arrival_ts_ns_channel_5',0)),
                    'camera_capture_pair_delta_ns':int(md.get('sdk_capture_pair_delta_ns',0)),
                    'camera_arrival_pair_delta_ns':int(md.get('sdk_arrival_pair_delta_ns',0)),
                    'camera_capture_to_arrival_ns_channel_4':int(md.get('sdk_capture_to_arrival_ns_channel_4',0)),
                    'camera_capture_to_arrival_ns_channel_5':int(md.get('sdk_capture_to_arrival_ns_channel_5',0)),
                    'imu_sdk_packet_count_drained':len(imu_sdk_packets),
                    'imu_sdk_packet_times':imu_sdk_packets,
                    'imu_raw_archive_path':str(imu_raw_path) if imu_raw_path else None,
                    'imu_raw_archive_records_written':int(imu_raw_count),
                    'timing_ms':{'queue':round((dequeued_ns-submitted_ns)/1e6,1),
                                 'drain':round((rectified_ns-dequeued_ns)/1e6,1),
                                 'orb':round((worker_return_ns-rectified_ns)/1e6,1),
                                 'total':round((worker_return_ns-submitted_ns)/1e6,1)},
                    'timing_semantics_version':2,
                    'timing_scope':'host submit to worker response; orb includes IPC; not exposure latency',
                    'image_scale':image_scale,'orb_config':str(timing.get('orb_config') or ''),

                    'warning':'SDK monotonic timestamps are diagnostic only; device_ts unit/exposure semantics and IMU payload layout remain unverified.',
                }
                audit.write(json.dumps(result,ensure_ascii=False,allow_nan=False)+'\n')
                # 原始数据留在诊断日志；界面只消费门控后的位姿。
                result.pop('raw_matrix',None)
                self.on_status(result);previous=stamp
        except Exception as exc:
            if proc is not None and isinstance(exc,(ConnectionError,EOFError,TimeoutError)):
                worker_failed=True
                retry=bool(self.ready and not self.stop_event.is_set())
            if log:log.write(f'{exc}\n')
            if not self.ready:self.startup_error=str(exc)
            if not self.stop_event.is_set() and not retry:
                detail=f'ORB-SLAM3 连接中断，子进程日志：{worker_log}' if worker_failed else str(exc)
                self.on_status({'type':'error','message':detail})
        finally:
            if phase_preparation is not None:phase_preparation.close()
            # Always unblock startup waiters, including early failure/cancellation.
            self.ready_event.set()
            # 定位停止后不再积压无人消费的独立 IMU 队列，预览仍可继续。
            self.source.imu_vio_queue=None
            if self.sock:self.sock.close();self.sock=None
            if listener:listener.close()
            if shared_transport is not None:shared_transport.close()
            if self._owns_worker or (worker_failed and not self.stop_event.is_set()):self.worker.close()
            if log:log.close()
            if audit:audit.close()
            if imu_raw:
                imu_raw.flush();imu_raw.close()
        return retry
