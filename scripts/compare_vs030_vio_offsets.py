"""同一段真实采集在内存中分别重放两种偏移；不向磁盘保存图像或 IMU。"""
import json
import os
from pathlib import Path
import queue
import socket
import struct
import subprocess
import time
from collections import Counter
import cv2
from egohand.sources.vendor_xdsdk import XdSdkStereoSource
from egohand.vio.live_runtime import CandidateImuBuffer, HEADER, IMU, PYTHON, ROOT, _recv
from egohand.vio.vs030_geometry import prepare_vs030_vio_geometry
from egohand.vio.vs030_imu_candidate import parse_candidate_payload

CAPTURE_SECONDS=20
MIN_FRAME_INTERVAL_NS=100_000_000
JPEG_QUALITY=95
OFFSETS_MS=(0.,-85.)


def read_message(sock):
    size=struct.unpack('<I',_recv(sock,4))[0]
    if size>100000:raise ValueError('结果长度异常')
    return json.loads(_recv(sock,size))


def replay(frames,packets,lag_ms):
    buffer=CandidateImuBuffer(camera_to_imu_lag_ns=round(lag_ms*1e6))
    for packet in packets:buffer.add(packet)
    listener=socket.socket();listener.bind(('127.0.0.1',0));listener.listen(1);listener.settimeout(30)
    env=os.environ.copy();env['PYTHONPATH']=str(ROOT/'src')+os.pathsep+env.get('PYTHONPATH','')
    log_path=ROOT/'logs'/f'vio_offset_ab_{int(lag_ms)}_{time.time_ns()}.log'
    count=Counter();epochs=set();error=None
    with log_path.open('w',encoding='utf8') as log:
        proc=subprocess.Popen([str(PYTHON),'-m','egohand.vio.live_subprocess',str(listener.getsockname()[1])],stdout=log,stderr=subprocess.STDOUT,env=env,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        conn=None
        try:
            conn,_=listener.accept();conn.settimeout(60);msg=read_message(conn)
            if msg.get('type')!='ready':raise RuntimeError(str(msg))
            print('REPLAY_READY',lag_ms,flush=True)
            previous=None
            for t,left,right in frames:
                if buffer.last_b<t:break
                measures=[] if previous is None else buffer.interval(previous,t)
                l=cv2.imdecode(left,cv2.IMREAD_GRAYSCALE);r=cv2.imdecode(right,cv2.IMREAD_GRAYSCALE)
                if l is None or r is None:raise RuntimeError('内存图像解码失败')
                conn.sendall(HEADER.pack(t,l.shape[1],l.shape[0],len(measures))+l.tobytes()+r.tobytes()+b''.join(IMU.pack(*m) for m in measures))
                response=read_message(conn)
                if response.get('type')=='error':raise RuntimeError(response.get('message'))
                count[response.get('tracking_state','UNKNOWN')]+=1
                count['imu_initialized']+=bool(response.get('imu_initialized'))
                count['pose_valid']+=bool(response.get('pose_valid'))
                epochs.add(response.get('map_epoch'));previous=t
        except Exception as exc:error=str(exc)
        finally:
            if conn:conn.close()
            listener.close()
            try:proc.wait(timeout=10)
            except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
    return {'lag_ms':lag_ms,'counts':dict(count),'map_epochs':len(epochs),'error':error,'log':str(log_path)}


def main():
    cv2.setNumThreads(2)
    source=XdSdkStereoSource();source.enable_imu_capture();frames=[];packets=[];last=0;error=None
    geometry=prepare_vs030_vio_geometry(ROOT/'标定结果/1/data3-camchain-imucam.yaml');rect=geometry.rectifier
    try:
        source.open();source.start();end=time.monotonic()+CAPTURE_SECONDS
        print('CAPTURE_READY',flush=True)
        while time.monotonic()<end:
            frame=source.read()
            if frame is None:break
            while True:
                try:
                    item=source.imu_vio_queue.get_nowait()
                    if len(item)==4: _,_,_,payload=item
                    elif len(item)==2: _,payload=item
                    else: raise ValueError('SDK IMU queue record has an unsupported layout')
                    packets.append(payload)
                except queue.Empty:break
            t=int(frame.metadata['sdk_device_timestamp_raw'])*1000
            if t-last<MIN_FRAME_INTERVAL_NS:continue
            left=cv2.remap(frame.right_image,*rect.maps_left,cv2.INTER_LINEAR)
            right=cv2.remap(frame.left_image,*rect.maps_right,cv2.INTER_LINEAR)
            a=cv2.cvtColor(left,cv2.COLOR_BGR2GRAY);b=cv2.cvtColor(right,cv2.COLOR_BGR2GRAY)
            ok_l,el=cv2.imencode('.jpg',a,[cv2.IMWRITE_JPEG_QUALITY,JPEG_QUALITY])
            ok_r,er=cv2.imencode('.jpg',b,[cv2.IMWRITE_JPEG_QUALITY,JPEG_QUALITY])
            if not (ok_l and ok_r):raise RuntimeError('内存图像压缩失败')
            frames.append((t,el,er));last=t
    except Exception as exc:error=str(exc)
    finally:source.close()
    print('CAPTURE_DONE',len(frames),len(packets),ascii(error),flush=True)
    if error or len(frames)<15:raise RuntimeError('采集不足 '+str(error))
    # 两次重放使用完全相同的帧；起点要覆盖两种偏移所需的 IMU 历史。
    first_imu=parse_candidate_payload(packets[0])[0]
    earliest=max(first_imu.time_a_raw,first_imu.time_b_raw)+max(0,round(-min(OFFSETS_MS)*1e6))+50_000_000
    frames=[frame for frame in frames if frame[0]>=earliest]
    if len(frames)<15:raise RuntimeError('有效公共起始区间不足')
    results=[]
    for lag in OFFSETS_MS:
        print('REPLAY_START',lag,flush=True);results.append(replay(frames,packets,lag))
    report={'frames_captured':len(frames),'imu_packets':len(packets),'sdk':source.sdk_diagnostics,'results':results,
            'note':'相同内存帧和惯性数据分别重放；JPEG 有损且重放速度非实时，不能当作精度验收。'}
    (ROOT/'results/vio/vs030_offset_same_input_ab.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print('RESULT',ascii(report),flush=True)

if __name__=='__main__':main()
