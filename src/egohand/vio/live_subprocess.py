"""隔离运行 ORB-SLAM3；TCP 仅监听本机，数据为未压缩校正图和 IMU。"""
import json
import os
import socket
import struct
import sys
import numpy as np
from pathlib import Path
from egohand.vio.native_bridge import NativeVioBridge, ImuMeasurement

HEADER=struct.Struct('<qIII')
IMU=struct.Struct('<q6f')
ROOT=Path(__file__).resolve().parents[3]

def receive(sock,n):
    data=bytearray(n)
    remaining=memoryview(data)
    while remaining:
        received=sock.recv_into(remaining)
        if not received:raise EOFError('连接已关闭')
        remaining=remaining[received:]
    return data

def send(sock,message):
    data=json.dumps(message,allow_nan=False).encode('utf8')
    sock.sendall(struct.pack('<I',len(data))+data)

def main():
    sock=socket.create_connection(('127.0.0.1',int(sys.argv[1])),timeout=20);sock.settimeout(None)
    try:
        settings=os.environ.get('EGOHAND_VIO_CONFIG') or str(ROOT/'configs/vio/vs030_rectified_preliminary.yaml')
        with NativeVioBridge(ROOT/'third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt',settings) as bridge:
            send(sock,{'type':'ready'})
            while True:
                try:stamp,w,h,n=HEADER.unpack(receive(sock,HEADER.size))
                except EOFError:break
                if w<=0 or h<=0 or w*h>1600*1200 or n>10000:raise ValueError(f'图像尺寸或 IMU 数量异常：{w}x{h} n={n}')
                pixels=w*h
                left=np.frombuffer(receive(sock,pixels),dtype=np.uint8).reshape(h,w)
                right=np.frombuffer(receive(sock,pixels),dtype=np.uint8).reshape(h,w)
                raw=receive(sock,n*IMU.size)
                imu=[]
                for row in IMU.iter_unpack(raw):imu.append(ImuMeasurement(row[0],tuple(row[1:4]),tuple(row[4:7])))
                result=bridge.track_stereo(left,right,stamp,imu)
                send(sock,{'type':'pose','timestamp_ns':stamp,'tracking_state':result.tracking_state_name,
                    'pose_valid':result.pose_valid,'imu_initialized':result.imu_initialized,
                    'map_epoch':result.map_epoch,
                    'matrix':result.T_world_from_rectified_left_camera.tolist() if result.pose_valid else None})
    except Exception as exc:
        try:send(sock,{'type':'error','message':str(exc)})
        except OSError:pass
        raise
    finally:sock.close()

if __name__=='__main__':main()
