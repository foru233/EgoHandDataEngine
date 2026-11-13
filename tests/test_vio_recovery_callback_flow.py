import json
import socket
import struct
import threading
from types import SimpleNamespace
import numpy as np
import pytest
from egohand.vio.live_subprocess import serve, HEADER, IMU


def test_candidate_checked_before_fake_orb_accepts_and_callback_is_cleared():
    class Bridge:
        map_mode=None
        def __init__(self):self.callback=None;self.count=0;self.decisions=[]
        def set_recovery_callback(self,callback):self.callback=callback
        def get_candidate_body_state(self):
            return {'T_world_from_body':np.eye(4),'velocity_world_mps':np.zeros(3),
                    'accel_bias_mps2':np.zeros(3),'gyro_bias_rps':np.zeros(3),'ba2_complete':True}
        def track_stereo(self,left,right,stamp,imu):
            self.count+=1;accepted=True
            if self.count==3:
                candidate=np.eye(4);candidate[0,3]=4
                self.decisions.append(self.callback(candidate,1));accepted=self.decisions[-1]!=0
            body=self.get_candidate_body_state()
            body.update(snapshot_consistent=True,frame_id=self.count,timestamp_ns=stamp,world_frame_id=1)
            snapshot={'snapshot_available':True,'snapshot_consistent':True,'frame_id':self.count,
                      'timestamp_ns':stamp,'world_frame_id':1,'body_state':body,
                      'map_id':0,'reset_id':0,'map_revision':1}
            return SimpleNamespace(tracking_state_name='OK' if accepted else 'LOST',pose_valid=accepted,
                imu_initialized=True,inertial_scale_finalized=True,map_epoch=1,frame_snapshot=snapshot,
                T_world_from_rectified_left_camera=np.eye(4) if accepted else None)
    a,b=socket.socketpair();bridge=Bridge()
    thread=threading.Thread(target=serve,args=(a,bridge));thread.start()
    def message():
        def read(n):
            v=b''
            while len(v)<n:
                x=b.recv(n-len(v));assert x;v+=x
            return v
        return json.loads(read(struct.unpack('<I',read(4))[0]))
    def frame(t,samples):
        payload=b''.join(IMU.pack(x,0,0,9.81,0,0,0) for x in samples)
        b.sendall(HEADER.pack(t,2,2,len(samples))+bytes(8)+payload)
        return message()
    assert message()['type']=='ready'
    frame(1_000_000_000,[])
    frame(2_500_000_000,[1_000_000_000+i*10_000_000 for i in range(1,151)])
    rejected=frame(2_510_000_000,[2_510_000_000])
    assert bridge.decisions==[0] and not rejected['pose_valid']
    assert rejected['recovery_preaccept']['candidates'][0]['status']=='inconsistent'
    b.close();thread.join(timeout=3);a.close()
    assert not thread.is_alive() and bridge.callback is None

@pytest.mark.parametrize('action', ['report_only', 'legacy_new_map'])
def test_quarantine_timeout_policy_at_bridge(tmp_path,monkeypatch,action):
    from egohand.vio import live_subprocess
    config=tmp_path/'recovery.yaml';config.write_text(
        f'stable_seconds: 0\nquarantine_timeout_s: 0.02\ntimeout_action: {action}\n',encoding='utf8')
    monkeypatch.setattr(live_subprocess,'RECOVERY_CONFIG',config)
    class Bridge:
        map_mode=None
        def __init__(self):self.callback=None;self.count=0;self.epoch=0;self.maps=0;self._recovery_callback_error=None
        def set_recovery_callback(self,callback):self.callback=callback
        def start_new_map(self):self.maps+=1;self.epoch=1<<20
        def get_candidate_body_state(self):
            return {'T_world_from_body':np.eye(4),'velocity_world_mps':np.zeros(3),
                    'accel_bias_mps2':np.zeros(3),'gyro_bias_rps':np.zeros(3),'ba2_complete':True}
        def track_stereo(self,left,right,stamp,imu):
            self.count+=1;T=np.eye(4)
            if self.count>1 and not self.maps:T[0,3]=4
            body=self.get_candidate_body_state()
            body.update(snapshot_consistent=True,frame_id=self.count,timestamp_ns=stamp,world_frame_id=self.epoch)
            snapshot={'snapshot_available':True,'snapshot_consistent':True,'frame_id':self.count,
                      'timestamp_ns':stamp,'world_frame_id':self.epoch,'body_state':body,
                      'map_id':self.epoch>>40,'reset_id':(self.epoch>>20)&((1<<20)-1),
                      'map_revision':self.epoch&((1<<20)-1)}
            return SimpleNamespace(tracking_state_name='OK',pose_valid=True,imu_initialized=True,inertial_scale_finalized=True,
                map_epoch=self.epoch,frame_snapshot=snapshot,T_world_from_rectified_left_camera=T)
    a,b=socket.socketpair();bridge=Bridge();thread=threading.Thread(target=serve,args=(a,bridge));thread.start()
    def message():
        def read(n):
            v=b''
            while len(v)<n:
                x=b.recv(n-len(v));assert x;v+=x
            return v
        return json.loads(read(struct.unpack('<I',read(4))[0]))
    def frame(t):
        b.sendall(HEADER.pack(t,2,2,1)+bytes(8)+IMU.pack(t,0,0,9.81,0,0,0))
        return message()
    assert message()['type']=='ready'
    frame(1_000_000_000);frame(1_010_000_000);frame(1_020_000_000)
    switched=frame(1_030_000_000)
    assert switched['recovery_preaccept']['quarantine_timed_out']
    fresh=frame(1_040_000_000)
    if action == 'legacy_new_map':
        assert switched['automatic_new_map'] and bridge.maps==1
        assert fresh['map_epoch']==1<<20 and not fresh['recovery_preaccept']['await_visual_requalification']
    else:
        assert not switched.get('automatic_new_map') and bridge.maps==0
        assert fresh['recovery_preaccept']['await_visual_requalification']
    b.close();thread.join(timeout=3);a.close()
    assert not thread.is_alive()


def test_native_unavailable_callback_is_rejected_after_tracking_was_lost(tmp_path,monkeypatch):
    """After tracking, unverified candidates must not take native's permissive -1 path."""
    from egohand.vio import live_subprocess
    from egohand.vio.pose_quality import apply_recovery_quarantine
    config=tmp_path/'recovery.yaml'
    config.write_text('stable_seconds: 0\nquarantine_timeout_s: 0.02\ntimeout_action: report_only\n',encoding='utf8')
    monkeypatch.setattr(live_subprocess,'RECOVERY_CONFIG',config)
    class LegacyBridge:
        map_mode=None
        def __init__(self):self.callback=None;self.count=0;self.decisions=[];self.maps=0
        def set_recovery_callback(self,callback):self.callback=callback
        def start_new_map(self):self.maps+=1
        def get_candidate_body_state(self):
            # Legacy asynchronous read deliberately has no frame snapshot metadata.
            return {'T_world_from_body':np.eye(4),'velocity_world_mps':np.zeros(3),
                    'accel_bias_mps2':np.zeros(3),'gyro_bias_rps':np.zeros(3),'ba2_complete':True}
        def track_stereo(self,left,right,stamp,imu):
            self.count+=1
            if self.count==3:
                self.decisions.append(self.callback(np.eye(4),1))
            state='LOST' if self.count==2 else 'OK'
            # Native semantics: callback -1 means unavailable, not rejection.
            accepted=state=='OK' and (self.count!=3 or self.decisions[-1]!=0)
            return SimpleNamespace(tracking_state_name=state,pose_valid=accepted,
                imu_initialized=True,inertial_scale_finalized=True,map_epoch=1,
                T_world_from_rectified_left_camera=np.eye(4) if accepted else None)
    a,b=socket.socketpair();b.settimeout(3);bridge=LegacyBridge()
    failures=[]
    def run():
        try:serve(a,bridge)
        except Exception as exc:failures.append(exc)
    thread=threading.Thread(target=run);thread.start()
    def read(n):
        data=b''
        while len(data)<n:
            chunk=b.recv(n-len(data));assert chunk;data+=chunk
        return data
    def message():return json.loads(read(struct.unpack('<I',read(4))[0]))
    def frame(t):
        b.sendall(HEADER.pack(t,2,2,1)+bytes(8)+IMU.pack(t,0,0,9.81,0,0,0))
        return message()
    try:
        assert message()['type']=='ready'
        first=frame(1_000_000_000)
        assert first['pose_valid'] and not first['recovery_preaccept']['anchor_available']
        assert not frame(1_010_000_000)['pose_valid']
        rejected=frame(1_020_000_000)
        assert bridge.decisions==[0] and not rejected['pose_valid']
        assert rejected['recovery_preaccept']['candidates'][0]['native_decision']=='rejected_visual_verifier_unavailable'
        accepted=frame(1_030_000_000)
        assert accepted['recovery_preaccept']['await_visual_requalification']
        filtered=apply_recovery_quarantine(accepted)
        assert not filtered['pose_valid'] and filtered['matrix'] is None
        later=frame(1_060_000_000)
        assert later['recovery_preaccept']['await_visual_requalification']
        assert later['recovery_preaccept']['quarantine_timed_out'] and bridge.maps==0
        assert not apply_recovery_quarantine(later)['pose_valid']
    finally:
        b.close();thread.join(timeout=3);a.close()
    assert not thread.is_alive() and not failures
