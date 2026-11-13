"""隔离运行 ORB-SLAM3；TCP 仅监听本机，数据为未压缩校正图和 IMU。"""
import json
import os
import socket
import struct
import sys
import numpy as np
import yaml
from contextlib import ExitStack
from pathlib import Path
from egohand.vio.native_bridge import NativeVioBridge, ImuMeasurement
from egohand.vio.recovery_coordinator import RecoveryCoordinator
from egohand.vio.recovery_contract import (PROTOCOL, FRAME_CONTEXT, RESET_IMU_ALIGNMENT,
    MAX_MESSAGE, RecoveryProtocolError)

HEADER=struct.Struct('<qIII')
IMU=struct.Struct('<q6f')
ROOT=Path(__file__).resolve().parents[3]
RECOVERY_CONFIG=ROOT/'configs/vio/recovery_experimental.yaml'

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
    if len(data)>MAX_MESSAGE:
        raise RecoveryProtocolError('VIO response exceeds protocol limit')
    sock.sendall(struct.pack('<I',len(data))+data)

def connect(port):
    sock=socket.create_connection(('127.0.0.1',port),timeout=20)
    sock.settimeout(None)
    return sock


def serve(sock, bridge, map_output=None, *, recovery_enabled=None, recovery_protocol=None):
    if recovery_protocol is None:recovery_protocol=int(os.environ.get('EGOHAND_VIO_RECOVERY_PROTOCOL','1'))
    if recovery_protocol not in (1,PROTOCOL):raise RecoveryProtocolError('unknown recovery protocol')
    if recovery_enabled is None:recovery_enabled=recovery_policy_enabled()
    if bridge.map_mode is None and recovery_enabled:
        recovery_path=Path(os.environ.get('EGOHAND_VIO_RECOVERY_CONFIG') or RECOVERY_CONFIG)
        options=yaml.safe_load(recovery_path.read_text(encoding='utf8')) or {}
        if not isinstance(options,dict):raise ValueError('VIO 恢复参数必须是字典')
        options.pop('allow_verified_stereo_bootstrap',None)  # Host quality gate only.
        if options.get('short_recovery') and not hasattr(bridge._dll,'egohand_vio_short_recovery_report'):
            raise ValueError('短时恢复配置需要显式选择支持几何证据的隔离DLL')
        coordinator=RecoveryCoordinator(application_admission=recovery_protocol==PROTOCOL,**options)
    else:coordinator=None
    coordinated=coordinator is not None and recovery_protocol==PROTOCOL
    send(sock,{'type':'ready','recovery_protocol':PROTOCOL if coordinated else 1 if coordinator else 0,
               'recovery_owner':'RecoveryCoordinator' if coordinated else 'legacy'})
    current_stamp=[None]
    if coordinator is not None:
        def check_candidate(T,epoch):
            geometry=bridge.get_short_recovery_report() if coordinator.short_recovery else None
            result=coordinator.check_candidate(T,current_stamp[0],epoch,geometry)
            # An IMU envelope is not an independent visual-geometric check.
            # After this map has tracked, never let ORB's permissive -1 path
            # accept an unverified relocalization into its map. Startup keeps
            # the existing policy; direct recovery is guarded after Track.
            if (result.get('visual_requalification_required') is True and
                result.get('independent_visual_verification')!='provisional_native_geometry'):
                result['native_decision']='rejected_visual_verifier_unavailable'
                return 0
            return {'consistent':1,'inconsistent':0,'unavailable':-1}[result['status']]
        bridge.set_recovery_callback(check_candidate)
    def start_independent_map():
        bridge.start_new_map()
        if coordinator is not None:
            coordinator.reset_for_new_map()
    try:
      while True:
        try:stamp,w,h,n=HEADER.unpack(receive(sock,HEADER.size))
        except EOFError:return
        if map_output is not None and (stamp,w,h,n)==(0,0,0,0):
            bridge.export_map(map_output/'atlas.osa', map_output/'poses.txt')
            send(sock,{'type':'exported'})
            return
        if w<=0 or h<=0 or w*h>1600*1200 or n>10000:raise ValueError(f'图像尺寸或 IMU 数量异常：{w}x{h} n={n}')
        if coordinated:
            flags=FRAME_CONTEXT.unpack(receive(sock,FRAME_CONTEXT.size))[0]
            if flags & ~RESET_IMU_ALIGNMENT:raise RecoveryProtocolError('unknown frame context flags')
            if flags & RESET_IMU_ALIGNMENT:coordinator.reset_imu_alignment()
        pixels=w*h
        left=np.frombuffer(receive(sock,pixels),dtype=np.uint8).reshape(h,w)
        right=np.frombuffer(receive(sock,pixels),dtype=np.uint8).reshape(h,w)
        raw=receive(sock,n*IMU.size)
        imu=[ImuMeasurement(row[0],tuple(row[1:4]),tuple(row[4:7])) for row in IMU.iter_unpack(raw)]
        if coordinator is not None:
            current_stamp[0]=stamp;coordinator.begin_frame(imu);bridge._recovery_callback_error=None
        result=bridge.track_stereo(left,right,stamp,imu)
        if coordinator is not None and bridge._recovery_callback_error:
            coordinator.callback_results.append({'status':'unavailable',
                'reason':'callback_exception: '+bridge._recovery_callback_error,
                'mode':'pre_accept_experimental'})
        snapshot=getattr(result,'frame_snapshot',None)
        # New DLLs publish camera, identity and body in one map-locked snapshot.
        # Legacy fallback is explicitly labelled as unverified consistency.
        if snapshot is not None:
            body=snapshot.get('body_state') if result.pose_valid and result.imu_initialized else None
        else:
            body=bridge.get_candidate_body_state() if result.pose_valid and result.imu_initialized else None
        if body is not None:
            body={k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in body.items()}
        response={'type':'pose','timestamp_ns':stamp,'tracking_state':result.tracking_state_name,
            'pose_valid':result.pose_valid,'imu_initialized':result.imu_initialized,
            'inertial_scale_finalized':result.inertial_scale_finalized,
            'map_epoch':result.map_epoch,
            # 世界系修订号：整图重锚计数。与 map_epoch 独立，epoch 不变时也可能递增；
            # 0 可能来自不支持该扩展的旧 DLL，需结合 capabilities 判断。
            'world_frame_revision':getattr(result,'world_frame_revision',0),
            'body_state_candidate':body,
            'matrix':result.T_world_from_rectified_left_camera.tolist() if result.pose_valid else None}
        counters=bridge.get_optimization_counters() if hasattr(bridge,'get_optimization_counters') else None
        if counters:
            response['optimization_counters']=counters
        world_frame=getattr(result,'world_frame_info',None)
        if world_frame is not None:
            response['world_frame']={key:value for key,value in world_frame.items() if key!='revision'}
        response['map_event_frame']=getattr(result,'map_event_frame',None)
        appearance=getattr(result,'map_appearance',None)
        if appearance is not None:
            response['map_appearance']=appearance
        observation=getattr(result,'recovery_observation',None)
        if observation is not None:response['recovery_observation']=observation
        if snapshot is not None:
            response.update({key:value for key,value in snapshot.items() if key!='body_state'})
        else:
            response['snapshot_consistent']=None
        if coordinator is not None:
            response['recovery_preaccept']=coordinator.finish_frame(response,body)
            if response['recovery_preaccept']['request_new_map']:
                start_independent_map()
                response['automatic_new_map']=True
                print('Recovery timeout: starting a new map',flush=True)
        send(sock,response)
        if coordinated:
            length=struct.unpack('<I',receive(sock,4))[0]
            if length>MAX_MESSAGE:raise RecoveryProtocolError('application acknowledgement too large')
            feedback=json.loads(receive(sock,length).decode('utf8'))
            committed=coordinator.finish_application_frame(feedback)
            if committed['request_new_map']:
                start_independent_map()
                committed['automatic_new_map']=True
                print('Unverified old reference abandoned: scheduled an independent new map',flush=True)
            send(sock,{'type':'recovery_committed','recovery_preaccept':committed})
    finally:
        if coordinator is not None:bridge.set_recovery_callback(None)


def open_bridge(map_mode=None):
    settings=os.environ.get('EGOHAND_VIO_CONFIG') or str(ROOT/'configs/vio/vs030_rectified_preliminary.yaml')
    library=os.environ.get('EGOHAND_VIO_LIBRARY')
    options={'map_mode':map_mode}
    appearance_frames=os.environ.get('EGOHAND_VIO_MAP_APPEARANCE_FRAMES','')
    if appearance_frames:
        options['map_appearance_frames']=tuple(int(v) for v in appearance_frames.split(','))
    if library:
        options['library_path']=Path(library).resolve()
    return NativeVioBridge(ROOT/'third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt',settings,**options)


def recovery_policy_enabled():
    """Diagnostic opt-out; normal sessions retain the experimental recovery policy."""
    return os.environ.get('EGOHAND_VIO_DISABLE_RECOVERY') != '1'


def report_error(sock, exc):
    try:
        message={'type':'error','message':str(exc)}
        if str(exc).startswith('VIO_NUMERICAL_FAULT:'):
            message.update(code='VIO_NUMERICAL_FAULT',recoverable=True)
        send(sock,message)
    except OSError:
        pass


def serve_live_sessions():
    # Closing the parent's control pipe exits the worker even if the GUI crashes.
    with ExitStack() as lifetime:
        bridge=None
        used=False
        for line in sys.stdin:
            port, session=json.loads(line)
            if port is None:
                if bridge is None:
                    bridge=lifetime.enter_context(open_bridge())
                continue
            try:
                sock=connect(port)
            except ConnectionRefusedError:
                continue  # The GUI cancelled before the worker accepted the request.
            with sock as connection:
                print(f'Live VIO session: {session}',flush=True)
                try:
                    if bridge is None:
                        bridge=lifetime.enter_context(open_bridge())
                    if used:
                        bridge.reset()
                    used=True
                    serve(connection,bridge)
                except (EOFError,ConnectionError):
                    continue
                except Exception as exc:
                    report_error(connection,exc)
                    raise


def main():
    if sys.argv[1]=='--live-sessions':
        serve_live_sessions()
        return
    with connect(int(sys.argv[1])) as sock:
        try:
            mode=os.environ.get('EGOHAND_VIO_MAP_MODE')
            output=Path(os.environ['EGOHAND_VIO_MAP_OUTPUT']) if mode=='build' else None
            with open_bridge(mode) as bridge:
                serve(sock,bridge,output)
        except Exception as exc:
            report_error(sock,exc)
            raise

if __name__=='__main__':main()
