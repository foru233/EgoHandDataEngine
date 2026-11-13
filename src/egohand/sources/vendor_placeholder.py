"""VS030 USB 网络直连：由本程序控制并接收双通道 UDP 视频。

通道 4/5 左右顺序由用户确认与 VisionViewer 一致；传感器时钟尚未验收。
"""
import logging
import queue
import re
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path
import numpy as np
from .base import StereoDataSource
from ..core.types import SourceMetadata,StereoFrame
from ..calibration.loader import load_calibration

ROOT=Path(__file__).resolve().parents[3]
ADB=ROOT/'tools/android/platform-tools/adb.exe'
CAMERA_SERIAL='<device-serial>'  # 按实际设备修改：adb devices / SDK 发现接口
CAPTURE_INTERFACE='usb0'
CAMERA_UDP_PORT=45679
HOST_UDP_PORT=45680
CONTROL_PORT=45678
CONTROL_TIMEOUT_SECONDS=5.
HEARTBEAT_SECONDS=1.
RECEIVE_BUFFER_BYTES=8*1024*1024
STREAM_CHANNELS=(4,5)  # 用户确认与上位机左右顺序一致；不代表已完成几何标定。
CALIBRATION_PATH=ROOT/'标定结果/2/data3-camchain-imucam-measured.yaml'
KALIBR_LEFT_CAMERA='cam0'  # 用户确认 cam0 为上位机左路，即通道 4。
FRAME_WAIT_SECONDS=10.
MAX_PACKET_BYTES=262144
MAX_CHANNEL_BYTES=8*1024*1024
MAGIC=b'\x90\x80\x55\xff'
log=logging.getLogger(__name__)

class VendorBundleDecoder:
    """按厂家数据包边界输出完整双通道；残缺或异常包直接丢弃。"""
    def __init__(self,decoders):
        self.decoders=decoders;self.pending=None;self.pending_size=0;self.encoded=bytearray();self.bundle=None

    def feed(self,payload):
        import av
        if payload.startswith(MAGIC) and len(payload)>=8:
            kind=struct.unpack_from('<I',payload,4)[0]
            if kind==1 and len(payload)==48:
                self.bundle={'images':{},'seen':set(),'bytes':0,'declared':struct.unpack_from('<I',payload,8)[0]}
                self.pending=None;self.encoded.clear()
            elif kind==4 and len(payload)==16 and self.bundle is not None:
                channel,size=struct.unpack_from('<II',payload,8)
                if self.pending is not None or channel not in self.decoders or channel in self.bundle['seen'] or not 0<size<=MAX_CHANNEL_BYTES:
                    self.bundle=None;self.pending=None;self.encoded.clear()
                else:
                    self.bundle['seen'].add(channel);self.pending=channel;self.pending_size=size;self.encoded.clear()
            elif kind==2 and len(payload)==8 and self.bundle is not None:
                bundle=self.bundle;self.bundle=None
                if self.pending is None and bundle['bytes']==bundle['declared'] and all(k in bundle['images'] for k in STREAM_CHANNELS):
                    return tuple(bundle['images'][k] for k in STREAM_CHANNELS)
                self.pending=None;self.encoded.clear()
            return None
        if self.bundle is None or self.pending not in self.decoders:return None
        self.encoded.extend(payload)
        if len(self.encoded)>self.pending_size:
            self.bundle=None;self.pending=None;self.encoded.clear();return None
        if len(self.encoded)==self.pending_size:
            self.bundle['bytes']+=self.pending_size
            try:
                frames=self.decoders[self.pending].decode(av.Packet(bytes(self.encoded)))
                if len(frames)==1:self.bundle['images'][self.pending]=frames[0].to_ndarray(format='bgr24')
            except av.error.FFmpegError:
                # 可能从 GOP 中途加入或出现丢包，等待后续关键帧。
                log.debug('VS030 H.265 解码等待关键帧',exc_info=True)
            self.pending=None;self.encoded.clear()
        return None

class VendorStereoSource(StereoDataSource):
    def __init__(self):
        self.thread=None;self.running=False;self.queue=queue.Queue(maxsize=1)
        self.error=None;self.index=0;self.fps=30.;self.stop_event=threading.Event();self.calibration=None
        self.control=None;self.data_socket=None;self.camera_ip=None;self.stream_started=False;self.camera_open=False

    def open(self):
        self.calibration=load_calibration(CALIBRATION_PATH,kalibr_left_camera=KALIBR_LEFT_CAMERA) if CALIBRATION_PATH.is_file() else None
        try:import av  # noqa: F401
        except ImportError as exc:raise RuntimeError('缺少 H.265 解码依赖 av，请安装 requirements-vendor.txt') from exc
        if not ADB.is_file():raise RuntimeError('缺少 Android platform-tools，请检查 '+str(ADB))
        result=subprocess.run([str(ADB),'-s',CAMERA_SERIAL,'get-state'],capture_output=True,text=True,timeout=5,
                              creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        if result.returncode or result.stdout.strip()!='device':
            raise RuntimeError('未检测到相机 ADB 设备，请检查 USB 连接及授权')
        address=subprocess.run([str(ADB),'-s',CAMERA_SERIAL,'shell','ip','-4','addr','show',CAPTURE_INTERFACE],
            capture_output=True,text=True,timeout=5,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        candidates=re.findall(r'\binet\s+(169\.254\.\d+\.\d+)/',address.stdout)
        if address.returncode or len(candidates)!=1:raise RuntimeError('无法确定相机 USB 网络地址，请检查 RNDIS 连接')
        self.camera_ip=candidates[0]
        self.index=0;self.error=None;self.stop_event.clear()
        while True:
            try:self.queue.get_nowait()
            except queue.Empty:break

    def start(self):
        try:
            self.control=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            self.control.settimeout(CONTROL_TIMEOUT_SECONDS);self.control.connect((self.camera_ip,CONTROL_PORT))
            host=self.control.getsockname()[0]
            self.data_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            self.data_socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,RECEIVE_BUFFER_BYTES)
            try:self.data_socket.bind((host,HOST_UDP_PORT))
            except OSError as exc:raise RuntimeError('相机 UDP 端口 45680 被占用，请关闭 VisionViewer 或其他采集程序') from exc
            self.data_socket.settimeout(.25)
            # 根据本机上位机实际交互核对：NET 双路 H.265、1600×1200、30 fps。
            state=self._command(4,struct.pack('<IQI',0,time.time_ns()//1_000_000,0))
            if not state or state[0]>4:raise RuntimeError('相机返回未知状态，不能自动切换视频模式')
            if state[0]==4:self._command(32)
            if state[0] in (1,2,3,4):self._command(8)
            self._command(2)
            self._command(6,struct.pack('<4I',0x3004,0,0,0));self.camera_open=True
            self._command(16,bytes(64))
            self._command(20,struct.pack('<4I',1600|(1200<<16),30,0,0))
            self._command(30);self.stream_started=True
            self.data_socket.sendto(b'\x01\x02\x03\x04\x05',(self.camera_ip,CAMERA_UDP_PORT))
            self.running=True;self.thread=threading.Thread(target=self._capture_udp,name='VS030Capture',daemon=True);self.thread.start()
        except Exception:
            self.stop();raise

    @staticmethod
    def _control_packet(command,payload=b''):
        return struct.pack('<8I',0x12345678,0xcafebabe,0,0,0,command,len(payload),0)+payload

    def _command(self,command,payload=b''):
        self.control.send(self._control_packet(command,payload))
        deadline=time.monotonic()+CONTROL_TIMEOUT_SECONDS
        while time.monotonic()<deadline:
            try:data=self.control.recv(65535)
            except socket.timeout:break
            if len(data)<32:continue
            magic,signature,_,_,status,reply,length,_=struct.unpack_from('<8I',data)
            if (magic,signature)!=(0x12345678,0xcafebabe) or reply!=command+1:continue
            if status or len(data)<32+length:raise RuntimeError(f'相机控制命令 {command} 返回错误 {status}')
            return data[32:32+length]
        raise RuntimeError(f'相机控制命令 {command} 超时，请检查 USB 网络连接')

    def _capture_udp(self):
        import av
        decoder=VendorBundleDecoder({channel:av.CodecContext.create('hevc','r') for channel in STREAM_CHANNELS})
        heartbeat=time.perf_counter()
        try:
            while not self.stop_event.is_set():
                if time.perf_counter()>=heartbeat:
                    self.control.send(self._control_packet(4,struct.pack('<IQI',0,time.time_ns()//1_000_000,0)))
                    heartbeat=time.perf_counter()+HEARTBEAT_SECONDS
                try:payload,peer=self.data_socket.recvfrom(65535)
                except socket.timeout:continue
                if peer[0]!=self.camera_ip or peer[1]!=CAMERA_UDP_PORT:continue
                pair=decoder.feed(payload)
                if pair is not None:
                    item=(*pair,time.perf_counter_ns())
                    try:self.queue.put_nowait(item)
                    except queue.Full:
                        try:self.queue.get_nowait()
                        except queue.Empty:pass
                        self.queue.put_nowait(item)
        except (OSError,ValueError,struct.error) as exc:
            if not self.stop_event.is_set():self.error=str(exc);log.warning('VS030 capture stopped: %s',exc)
        finally:self.running=False

    def read(self):
        deadline=time.monotonic()+FRAME_WAIT_SECONDS
        while True:
            if not self.running:
                if self.error:raise RuntimeError('相机预览中断：'+self.error)
                return None
            remaining=deadline-time.monotonic()
            if remaining<=0:raise RuntimeError('相机已连接但未收到完整双通道 H.265 图像，请检查 USB 网络及相机状态')
            try:left,right,stamp=self.queue.get(timeout=min(.2,remaining));break
            except queue.Empty:continue
        if left.shape!=right.shape or left.ndim!=3:raise RuntimeError('相机两个视频通道的尺寸不一致')
        idx=self.index;self.index+=1
        return StereoFrame(idx,stamp,left,right,None,None,self.calibration.calibration_id if self.calibration else None,'vendor_vs030',False,'raw',{
            'real_stereo':True,'synthetic':False,'sync_quality':'UNVERIFIED',
            'timestamp_kind':'host_bundle_decode_completed_monotonic_ns','hardware_sync_precision_verified':False,
            'transport_pairing':'same_udp_bundle_not_sensor_sync','channel_ids':list(STREAM_CHANNELS),
            'left_right_mapping_evidence':'user_confirmed_matches_visionviewer',
            'physical_left_right_verified':False,'device_serial':CAMERA_SERIAL,
        })

    def interrupt_read(self):
        self.stop_event.set();self.running=False

    def stop(self):
        self.interrupt_read()
        if self.thread and self.thread is not threading.current_thread():self.thread.join(timeout=3)
        if self.control and (self.stream_started or self.camera_open):
            try:
                if self.stream_started:self._command(32)
                if self.camera_open:self._command(8)
            except (OSError,RuntimeError) as exc:log.warning('VS030 stop command failed: %s',exc)
        self.stream_started=False;self.camera_open=False
        if self.data_socket:self.data_socket.close();self.data_socket=None
        if self.control:self.control.close();self.control=None

    def close(self):self.stop()
    def get_metadata(self):
        return SourceMetadata('vendor_vs030',real_stereo=True,synchronized=False,real_calibration=self.calibration is not None,
            metric_capable=False,timestamp_kind='host_bundle_decode_completed_monotonic_ns',
            extra={'device':'VS030 / SC535HAI','transport':'direct USB network UDP / H.265',
                   'channels':list(STREAM_CHANNELS),'left_right_mapping_evidence':'user_confirmed_matches_visionviewer',
                   'physical_left_right_verified':False})
    def get_calibration(self):return self.calibration
