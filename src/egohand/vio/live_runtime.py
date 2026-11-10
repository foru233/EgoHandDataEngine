"""VS030 实验性实时 VIO：独立进程、显式时序与可观察的失败状态。"""
from collections import deque
from dataclasses import replace
import json
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

from .vs030_imu_candidate import parse_candidate_payload
from .vs030_geometry import prepare_vs030_vio_geometry
from .pose_quality import PoseQualityGate
from .imu_lattice import LatticeImuBuffer, PacketLatticeImuBuffer, packet_lattice_phase_ns

ROOT=Path(__file__).resolve().parents[3]
PYTHON=Path(os.environ.get('EGOHAND_VIO_PYTHON', 'python.exe'))  # 鍙€夛細鎸囧悜宸插畨瑁?egohand-vio 鐜鐨勮В閲婂櫒
CALIBRATION=ROOT/'标定结果/1/data3-camchain-imucam.yaml'
HEADER=struct.Struct('<qIII'); IMU=struct.Struct('<q6f')
# Diagnostic raw packet archive: magic, version, header_size, device_ts, SDK capture/arrival ns, payload_size.
IMU_RAW_HEADER=struct.Struct('<4sHHQQQI')
MAX_IMU_GAP_NS=30_000_000
IMU_HISTORY_KEEP_NS=1_000_000_000

def _recv(sock,n):
    result=bytearray()
    while len(result)<n:
        chunk=sock.recv(n-len(result))
        if not chunk:raise EOFError('VIO 子进程连接已关闭')
        result.extend(chunk)
    return bytes(result)

def _send_tracking_frame(sock, stamp, left, right, measurements):
    packet = b''.join(IMU.pack(*row) for row in measurements)
    sock.sendall(HEADER.pack(int(stamp), left.shape[1], left.shape[0], len(measurements)))
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

class LiveVio:
    def __init__(self,source,on_status,timing=None):
        self.source=source;self.on_status=on_status;self.frames=queue.Queue(maxsize=1)
        self.stop_event=threading.Event();self.thread=None;self.sock=None;self.proc=None
        self.ready_event=threading.Event();self.ready=False;self.startup_error=None
        self.motion_event=threading.Event();self.motion_samples=0
        self.timing_override=dict(timing) if timing is not None else None
        self.defer_imu=False
    def start(self, *, defer_imu=False):
        self.stop_event.clear();self.ready_event.clear();self.ready=False;self.startup_error=None
        self.motion_event.clear();self.motion_samples=0
        self.defer_imu=defer_imu
        if not defer_imu:self.source.enable_imu_capture()
        self.thread=threading.Thread(target=self._run,name='LiveVio',daemon=True);self.thread.start()
    def wait_ready(self,timeout=60.0):
        """Wait for ORB-SLAM3 vocabulary/settings initialization before starting camera streaming."""
        if not self.ready_event.wait(timeout):
            self.startup_error=f'ORB-SLAM3 初始化超时（{timeout:.0f}s）；未启动相机数据流，避免 IMU 队列在加载期间溢出'
            return False
        return bool(self.ready and self.thread is not None and self.thread.is_alive())
    def submit(self,frame):
        if self.defer_imu and not self.ready:return
        try:self.frames.put_nowait(frame)
        except queue.Full:
            try:self.frames.get_nowait()
            except queue.Empty:pass
            self.frames.put_nowait(frame)
    def close(self):
        self.stop_event.set()
        if self.sock:
            try:self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:pass
        if not self.ready and self.proc and self.proc.poll() is None:self.proc.terminate()
        if self.thread:self.thread.join(timeout=12)
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:self.proc.kill();self.proc.wait(timeout=3)
    def _read_message(self):
        length=struct.unpack('<I',_recv(self.sock,4))[0]
        if length>100000:raise ValueError('VIO 响应过大')
        return json.loads(_recv(self.sock,length).decode('utf8'))
    def _run(self):
        listener=None;log=None;audit=None;imu_raw=None;imu_raw_path=None;imu_raw_count=0
        try:
            self.on_status({'type':'status','message':'正在加载 ORB-SLAM3 词典，定位将在加载完成后启动…'})
            geometry=prepare_vs030_vio_geometry(CALIBRATION)
            timing=self.timing_override if self.timing_override is not None else yaml.safe_load((ROOT/'configs/vio/live_timing.yaml').read_text(encoding='utf8'))
            listener=socket.socket();listener.bind(('127.0.0.1',0));listener.listen(1);listener.settimeout(.5)
            env=os.environ.copy();env['PYTHONPATH']=str(ROOT/'src')+os.pathsep+env.get('PYTHONPATH','')
            if timing.get('orb_config'):
                orb_cfg=Path(str(timing['orb_config']))
                env['EGOHAND_VIO_CONFIG']=str(orb_cfg if orb_cfg.is_absolute() else ROOT/orb_cfg)
            (ROOT/'logs').mkdir(exist_ok=True)
            log_path=ROOT/'logs'/f'vio_vs030_live_{time.time_ns()}.log'
            log=open(log_path,'w',encoding='utf8')
            audit=open(log_path.with_name(log_path.stem+'_poses.jsonl'),'w',encoding='utf8',buffering=1)
            imu_raw_path=log_path.with_name(log_path.stem+'_imu_raw.bin')
            imu_raw=open(imu_raw_path,'wb',buffering=0)
            quality=PoseQualityGate(**yaml.safe_load((ROOT/'configs/vio/live_quality.yaml').read_text(encoding='utf8')))
            if self.stop_event.is_set():return
            self.proc=subprocess.Popen([str(PYTHON),'-m','egohand.vio.live_subprocess',str(listener.getsockname()[1])],
                stdout=log,stderr=subprocess.STDOUT,env=env,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0)|0x00008000)
            deadline=time.monotonic()+60
            while not self.stop_event.is_set():
                try:self.sock,_=listener.accept();break
                except socket.timeout:
                    if self.proc.poll() is not None:raise RuntimeError(f'VIO 子进程启动失败，详见 {log_path}')
                    if time.monotonic()>deadline:raise TimeoutError('VIO 子进程连接超时')
            if self.stop_event.is_set():return
            self.sock.settimeout(45)
            self.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            message=self._read_message()
            if message.get('type')!='ready':raise RuntimeError(message.get('message','VIO 初始化失败'))
            lag_ms=float(timing.get('camera_to_imu_lag_ms',0.0))
            if not np.isfinite(lag_ms) or abs(lag_ms)>1000:raise ValueError('相机—IMU 时间偏移无效')
            alignment=str(timing.get('imu_alignment','timestamp')).strip().lower()
            if alignment not in ('timestamp','lattice'):raise ValueError(f'未知的 IMU 对齐方式：{alignment}')
            drift_ppm=float(timing.get('camera_to_imu_drift_ppm',0.0))
            lattice_reference=timing.get('lattice_reference','packet')
            if lattice_reference not in ('packet','count'):raise ValueError('未知的格点参考方式')
            packet_lattice=alignment=='lattice' and lattice_reference=='packet'
            image_scale=float(timing.get('image_scale',1.0))
            if not (0.25<=image_scale<=1.0):raise ValueError(f'image_scale 超出范围：{image_scale}')
            if alignment=='lattice':
                options=dict(sample_rate_hz=float(timing.get('imu_rate_hz',480.0)),
                             camera_to_imu_drift_ppm=drift_ppm,capacity=20000)
                buffer=(PacketLatticeImuBuffer(phase_ns=packet_lattice_phase_ns(timing),**options)
                        if packet_lattice else LatticeImuBuffer(**options))
                mode_text=f'帧格点 {buffer.sample_rate_hz:.1f} Hz · {lattice_reference}'
            else:
                buffer=CandidateImuBuffer(camera_to_imu_lag_ns=round(lag_ms*1e6))
                mode_text=f'时间戳偏移 {lag_ms:+.1f} ms'
            previous=None
            if self.stop_event.is_set():return
            if self.defer_imu:self.source.enable_imu_capture(allow_running=True)
            self.ready=True
            self.ready_event.set()
            self.on_status({'type':'status','message':f'ORB-SLAM3 已就绪，等待双目与 IMU · {mode_text}（实验）'})

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
                    buffer.reset();previous=None
                    self.on_status({'type':'status','message':'IMU 缓冲溢出，已重建对齐并等待下一帧'})
                    return False
                if not self.motion_event.is_set():
                    for row in added:
                        if max(abs(row.values[3]),abs(row.values[4]),abs(row.values[5]))>0.15:
                            self.motion_samples+=1
                    if self.motion_samples>=8:self.motion_event.set()
                return True

            while not self.stop_event.is_set():
                t_frame=time.perf_counter()
                try:frame=self.frames.get(timeout=.1)
                except queue.Empty:continue
                t_dequeued=time.perf_counter()
                stamp=int(frame.metadata['sdk_device_timestamp_raw'])*1000 # 实测帧增量对应微秒；尚未获厂商说明。
                span_s=max(0.0,(stamp-previous)/1e9) if previous is not None else 0.0
                deadline=time.monotonic()+min(max(.6,span_s+.5),8.0)
                imu_sdk_packets=[]
                if previous is not None and stamp<=previous:raise ValueError('相机设备时间非递增')
                if packet_lattice:
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
                        previous=None
                        continue
                elif alignment=='lattice':
                    need=0 if previous is None else buffer.expected(previous,stamp)
                    while buffer.pending()<need and not self.stop_event.is_set() and time.monotonic()<deadline:
                        pull_imu(imu_sdk_packets)
                    if buffer.pending()<need:
                        self.on_status({'type':'status','message':f'IMU 样本不足（需要 {need}，可用 {buffer.pending()}），重建对齐'})
                        buffer.reset();previous=None;continue
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
                left,right=geometry.rectifier.remap_pair(frame.right_image,frame.left_image)
                left=np.ascontiguousarray(cv2.cvtColor(left,cv2.COLOR_BGR2GRAY));right=np.ascontiguousarray(cv2.cvtColor(right,cv2.COLOR_BGR2GRAY))
                if image_scale!=1.0:
                    left=cv2.resize(left,None,fx=image_scale,fy=image_scale,interpolation=cv2.INTER_AREA)
                    right=cv2.resize(right,None,fx=image_scale,fy=image_scale,interpolation=cv2.INTER_AREA)
                t_rect=time.perf_counter()
                _send_tracking_frame(self.sock,stamp,left,right,measurements)
                result=self._read_message()
                t_vio=time.perf_counter()
                if result.get('type')=='error':raise RuntimeError(result.get('message'))
                if result.get('matrix') is not None:
                    result['matrix']=(np.array(result['matrix'])@geometry.T_orb_left_from_gui_left).tolist()
                result=quality.check(result)
                result['imu_alignment']=alignment
                if alignment=='lattice':
                    result['imu_lattice']={'pending':buffer.pending(),'consumed':buffer.consumed,
                                           'gap_events':buffer.gap_events,'rate_hz':buffer.sample_rate_hz,
                                           'drift_ppm':drift_ppm,'reference':lattice_reference}
                    if packet_lattice:result['imu_lattice']['phase_ms']=buffer.phase_ns/1e6
                else:
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
                    'timing_ms':{'queue':round((t_dequeued-t_frame)*1e3,1),
                                 'drain':round((t_rect-t_dequeued)*1e3,1),
                                 'orb':round((t_vio-t_rect)*1e3,1),
                                 'total':round((t_vio-t_frame)*1e3,1)},'image_scale':image_scale,'orb_config':str(timing.get('orb_config') or ''),

                    'warning':'SDK monotonic timestamps are diagnostic only; device_ts unit/exposure semantics and IMU payload layout remain unverified.',
                }
                audit.write(json.dumps(result,ensure_ascii=False,allow_nan=False)+'\n')
                # 原始数据留在诊断日志；界面只消费门控后的位姿。
                result.pop('raw_matrix',None)
                self.on_status(result);previous=stamp
        except Exception as exc:
            if not self.ready:self.startup_error=str(exc)
            if not self.stop_event.is_set():self.on_status({'type':'error','message':str(exc)})
        finally:
            # Always unblock startup waiters, including early failure/cancellation.
            self.ready_event.set()
            # 定位停止后不再积压无人消费的独立 IMU 队列，预览仍可继续。
            self.source.imu_vio_queue=None
            if self.sock:self.sock.close();self.sock=None
            if listener:listener.close()
            if self.proc:
                if self.stop_event.is_set() and not self.ready and self.proc.poll() is None:self.proc.terminate()
                try:self.proc.wait(timeout=8)
                except subprocess.TimeoutExpired:self.proc.terminate();self.proc.wait(timeout=3)
            if log:log.close()
            if audit:audit.close()
            if imu_raw:
                imu_raw.flush();imu_raw.close()
