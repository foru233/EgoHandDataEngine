"""One coordinator across native callbacks and application admission transactions."""
import copy
import json
import socket
import struct
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from egohand.vio.hybrid_quality import make_quality_gate
from egohand.vio.localization_state import LocalizationPipeline, consumer_map_pose
from egohand.vio.map_identity import decode_map_epoch
from egohand.vio.native_bridge import ImuMeasurement
from egohand.vio.recovery_contract import (PROTOCOL, RecoveryProtocolError,
    application_feedback, negotiate_recovery)
from egohand.vio.recovery_coordinator import RecoveryCoordinator


def frame(t, *, x=0., epoch=0, state='OK', initialized=True, mature=True):
    T=np.eye(4);T[0,3]=x
    return dict(timestamp_ns=t,frame_id=t,map_epoch=epoch,world_frame_id=epoch,
        world_frame_revision=0,**decode_map_epoch(epoch),snapshot_available=True,
        snapshot_consistent=True,tracking_state=state,pose_valid=state=='OK',
        matrix=T.tolist() if state=='OK' else None,imu_initialized=initialized,
        ba2_complete=mature,inertial_scale_finalized=mature,
        body_state_candidate=dict(timestamp_ns=t,frame_id=t,world_frame_id=epoch,
            snapshot_consistent=True,ba2_complete=mature,T_world_from_body=T.tolist(),
            velocity_world_mps=[0.,0.,0.],accel_bias_mps2=[0.,0.,0.],gyro_bias_rps=[0.,0.,0.]))


def sample(t):return ImuMeasurement(t,(0.,0.,9.81),(0.,0.,0.))


def fixture(*, mode='stereo_inertial', stable=0, commit=None):
    c=RecoveryCoordinator(application_admission=True,stable_seconds=99)
    p=LocalizationPipeline(mode,make_quality_gate(mode,stable_seconds=stable),
        recovery_commit=commit or (lambda out:c.finish_application_frame(application_feedback(out))))
    def run(row):
        if p.take_imu_alignment_reset():c.reset_imu_alignment()
        c.begin_frame([sample(row['timestamp_ns'])])
        row=dict(row);row['recovery_preaccept']=c.finish_frame(row,row.get('body_state_candidate'))
        return p.process(row,[(row['timestamp_ns'],0,0,9.81,0,0,0)])
    return c,p,run


def test_application_quality_is_the_only_stability_window_and_anchor_source(monkeypatch):
    from egohand.vio import recovery_monitor
    monkeypatch.setattr(recovery_monitor,'RecoveryMonitor',lambda:pytest.fail('second recovery owner created'))
    c,p,run=fixture(stable=.1)
    assert c.quality is None and p.recovery_monitor is None
    first=run(frame(1_000_000_000))
    assert not first['pose_valid'] and c.predictor.anchor is None
    second=run(frame(1_100_000_000))
    assert second['pose_valid'] and c.predictor.anchor.timestamp_ns==second['timestamp_ns']
    assert second['recovery_preaccept']['stage']=='committed'
    assert second['recovery_ownership']['owner']=='RecoveryCoordinator'
    evidence=second['localization']['recovery_evidence']
    assert evidence['owner']=='RecoveryCoordinator' and evidence['frame_committed']
    assert evidence['monitor']['active'] is False and evidence['source_disagreement'] is None
    assert consumer_map_pose(second) is not None


def test_visual_bootstrap_host_jump_latches_the_same_coordinator_without_imu_anchor():
    c,p,run=fixture(mode='hybrid')
    assert run(frame(1_000_000_000,initialized=False,mature=False))['pose_valid']
    jump=run(frame(1_010_000_000,x=4.,initialized=False,mature=False))
    assert jump['jump_detected'] and not jump['pose_valid']
    assert c.quarantined and c.quarantine_cause=='pose_jump' and c.predictor.anchor is None
    later=run(frame(5_000_000_000,x=4.,initialized=False,mature=False))
    assert not later['pose_valid'] and later['recovery_preaccept']['quarantine_timed_out']
    assert not later['recovery_preaccept']['request_new_map']
    assert later['localization']['state']=='RELOCALIZING'


def test_stable_quality_inside_quarantine_cannot_publish_world_pose():
    c, p, run = fixture(stable=.1)
    run(frame(1_000_000_000))
    assert run(frame(1_100_000_000))['pose_valid']
    c._quarantine('pose_jump', 'unavailable')
    for t in (1_200_000_000, 1_300_000_000, 1_400_000_000):
        out = run(frame(t))
        assert out['quality_gate']['passed'] is True
        assert out['recovery_preaccept']['await_visual_requalification'] is True
        assert not out['pose_valid'] and consumer_map_pose(out) is None
        assert out['localization']['admission']['world_hands_permitted'] is False


def test_loss_and_imu_reset_cannot_clear_single_owner_quarantine_but_new_map_is_distinct():
    c,p,run=fixture()
    run(frame(1_000_000_000))
    run(frame(1_010_000_000,state='LOST'))
    p.reset_imu_alignment()
    recovered=run(frame(1_020_000_000))
    assert c.quarantined and not recovered['pose_valid'] and c.predictor.anchor is None
    assert not p.take_imu_alignment_reset()
    fresh=run(frame(1_030_000_000,epoch=1<<40))
    assert fresh['pose_valid'] and not c.quarantined
    assert not fresh['localization']['coordinate_continuity_verified']
    assert fresh['localization']['odom']['status']=='unavailable'


@pytest.mark.parametrize('field,value', [('timestamp_ns',8),('frame_id',8),('map_epoch',1),
    ('world_frame_revision',None),('session_id','b'*32),('frame_sequence',False),
    ('schema_version',2.),('quality_passed',1),('inertial_anchor_admitted',1)])
def test_mismatched_or_malformed_application_ack_never_updates_anchor(field,value):
    c=RecoveryCoordinator(application_admission=True)
    row=frame(1_000_000_000);c.begin_frame([sample(row['timestamp_ns'])])
    row['recovery_preaccept']=c.finish_frame(row,row['body_state_candidate'])
    row=make_quality_gate('stereo_inertial',stable_seconds=0).check(row);row['mode']='stereo_inertial'
    feedback=application_feedback(row);feedback[field]=value
    with pytest.raises(RecoveryProtocolError):c.finish_application_frame(feedback)
    assert c.predictor.anchor is None
    with pytest.raises(RecoveryProtocolError):c.begin_frame([])


def test_ack_is_consumed_once_and_missing_ack_cannot_seed_next_frame():
    c=RecoveryCoordinator(application_admission=True)
    row=frame(1_000_000_000);c.begin_frame([sample(row['timestamp_ns'])])
    row['recovery_preaccept']=c.finish_frame(row,row['body_state_candidate'])
    assert c.predictor.anchor is None
    with pytest.raises(RecoveryProtocolError):c.begin_frame([])
    row=make_quality_gate('stereo_inertial',stable_seconds=0).check(row);row['mode']='stereo_inertial'
    feedback=application_feedback(row)
    c.finish_application_frame(feedback)
    assert c.predictor.anchor is not None
    with pytest.raises(RecoveryProtocolError):c.finish_application_frame(feedback)


def test_stale_worker_report_is_rejected_before_mutating_application_history():
    c,p,run=fixture();out=run(frame(1_000_000_000))
    anchor=c.predictor.anchor;previous=p.quality.previous
    bad=frame(1_010_000_000)
    bad['recovery_preaccept']=copy.deepcopy(out['recovery_preaccept'])
    bad['recovery_preaccept']['stage']='native'
    with pytest.raises(RecoveryProtocolError):p.process(bad,[])
    assert c.predictor.anchor is anchor and p.quality.previous is previous


def test_single_owner_report_cannot_fall_back_to_the_legacy_monitor():
    c=RecoveryCoordinator(application_admission=True)
    row=frame(1_000_000_000);c.begin_frame([])
    row['recovery_preaccept']=c.finish_frame(row,row['body_state_candidate'])
    p=LocalizationPipeline('stereo_inertial',make_quality_gate('stereo_inertial',stable_seconds=0))
    with pytest.raises(RecoveryProtocolError):p.process(row,[])
    assert p.recovery_monitor.predictor.anchor is None and p.quality.previous is None


def test_normal_negotiation_cannot_silently_downgrade_to_multiple_owners():
    with pytest.raises(RecoveryProtocolError):negotiate_recovery({'type':'ready'})
    with pytest.raises(RecoveryProtocolError):negotiate_recovery({'recovery_protocol':2,'recovery_owner':'other'})
    assert negotiate_recovery({'type':'ready'},requested=1)==1
    assert negotiate_recovery({'recovery_protocol':0},enabled=False)==0
    assert negotiate_recovery({'recovery_protocol':2,'recovery_owner':'RecoveryCoordinator'})==2


@pytest.mark.parametrize('mutation', [lambda r:r['recovery_preaccept'].update(stage='native'),
    lambda r:r['recovery_preaccept'].update(frame_sequence=0),
    lambda r:r['recovery_preaccept'].update(timestamp_ns=0),
    lambda r:r['recovery_preaccept'].update(schema_version=99)])
def test_consumer_never_accepts_uncommitted_or_malformed_report(mutation):
    c,p,run=fixture();out=run(frame(1_000_000_000))
    mutation(out)
    assert consumer_map_pose(out) is None


def test_committed_report_cannot_switch_session_or_fabricate_admission():
    for key,value in [('session_id','b'*32),('application_quality_passed',False),('stage','native')]:
        c=RecoveryCoordinator(application_admission=True)
        def corrupted(out):
            report=c.finish_application_frame(application_feedback(out));report[key]=value;return report
        p=LocalizationPipeline('stereo_inertial',make_quality_gate('stereo_inertial',stable_seconds=0),
                               recovery_commit=corrupted)
        row=frame(1_000_000_000);c.begin_frame([sample(row['timestamp_ns'])])
        row['recovery_preaccept']=c.finish_frame(row,row['body_state_candidate'])
        with pytest.raises(RecoveryProtocolError):p.process(row,[])


@pytest.mark.parametrize('adapter',['live','replay'])
def test_socket_transaction_uses_same_owner_for_host_jump_and_native_callback(adapter,tmp_path,monkeypatch):
    from egohand.vio import live_subprocess
    from egohand.vio.live_runtime import LiveVio, _send_tracking_frame
    from egohand.vio.replay_runtime import ReplayBridge
    config=tmp_path/'recovery.yaml';config.write_text('stable_seconds: 99\ntimeout_action: report_only\n')
    monkeypatch.setattr(live_subprocess,'RECOVERY_CONFIG',config)
    class Bridge:
        map_mode=None
        def __init__(self):self.callback=None;self.decisions=[];self.count=0
        def set_recovery_callback(self,callback):self.callback=callback
        def track_stereo(self,left,right,stamp,imu):
            self.count+=1
            row=frame(stamp,x=4. if self.count>=2 else 0.,initialized=False,mature=False)
            if self.count==3:
                self.decisions.append(self.callback(np.eye(4),0))
                row['pose_valid']=self.decisions[-1]!=0
                row['tracking_state']='OK' if row['pose_valid'] else 'LOST'
            snapshot={k:v for k,v in row.items() if k in
                ('frame_id','world_frame_id','map_id','reset_id','map_revision','snapshot_available','snapshot_consistent')}
            return SimpleNamespace(tracking_state_name=row['tracking_state'],pose_valid=row['pose_valid'],
                imu_initialized=False,inertial_scale_finalized=False,map_epoch=0,frame_snapshot=snapshot,
                T_world_from_rectified_left_camera=np.asarray(row['matrix']))
    server,client=socket.socketpair();client.settimeout(3);bridge=Bridge();errors=[]
    def worker():
        try:live_subprocess.serve(server,bridge,recovery_protocol=PROTOCOL)
        except Exception as exc:errors.append(exc)
    thread=threading.Thread(target=worker);thread.start()
    if adapter=='replay':
        host=ReplayBridge(tmp_path/'unused.log');host.sock=client;host.recovery_protocol=PROTOCOL
        read=host._read_message;commit=host.commit_recovery
        track=lambda t,reset:host.track(t,np.zeros((2,2),np.uint8),np.zeros((2,2),np.uint8),[],reset_imu_alignment=reset)
    else:
        host=LiveVio(object(),lambda _:None);host.sock=client
        read=host._read_message;commit=host._commit_recovery
        def track(t,reset):
            _send_tracking_frame(client,t,np.zeros((2,2),np.uint8),np.zeros((2,2),np.uint8),[],
                recovery_protocol=PROTOCOL,reset_imu_alignment=reset)
            return read()
    try:
        ready=read();assert ready['recovery_protocol']==PROTOCOL
        p=LocalizationPipeline('hybrid',make_quality_gate('hybrid',stable_seconds=0),recovery_commit=commit)
        first=p.process(track(1_000_000_000,False),[])
        assert first['pose_valid'] and p.recovery_monitor is None
        jump=p.process(track(1_010_000_000,False),[])
        assert not jump['pose_valid'] and jump['recovery_preaccept']['quarantine_cause']=='pose_jump'
        p.reset_imu_alignment()
        rejected=p.process(track(1_020_000_000,p.take_imu_alignment_reset()),[])
        assert bridge.decisions==[0] and not rejected['pose_valid']
        assert rejected['recovery_preaccept']['await_visual_requalification']
    finally:
        client.close();thread.join(timeout=3);server.close()
    assert not thread.is_alive() and not errors and bridge.callback is None


def test_disconnect_before_ack_keeps_anchor_uncommitted_and_clears_native_callback(tmp_path,monkeypatch):
    from egohand.vio import live_subprocess
    from egohand.vio.live_runtime import _send_tracking_frame, _recv
    observed=[]
    class Coordinator(RecoveryCoordinator):
        def __init__(self,**options):super().__init__(**options);observed.append(self)
    monkeypatch.setattr(live_subprocess,'RecoveryCoordinator',Coordinator)
    config=tmp_path/'recovery.yaml';config.write_text('timeout_action: report_only\n')
    monkeypatch.setattr(live_subprocess,'RECOVERY_CONFIG',config)
    class Bridge:
        map_mode=None
        callback=None
        def set_recovery_callback(self,callback):self.callback=callback
        def track_stereo(self,left,right,stamp,imu):
            row=frame(stamp)
            snapshot={**{key:row[key] for key in ('frame_id','map_id','reset_id','map_revision',
                'snapshot_available','snapshot_consistent','world_frame_id')},'body_state':row['body_state_candidate']}
            return SimpleNamespace(tracking_state_name='OK',pose_valid=True,imu_initialized=True,
                inertial_scale_finalized=True,map_epoch=0,frame_snapshot=snapshot,
                T_world_from_rectified_left_camera=np.eye(4))
    server,client=socket.socketpair();client.settimeout(3);bridge=Bridge();errors=[]
    def worker():
        try:live_subprocess.serve(server,bridge,recovery_protocol=PROTOCOL)
        except EOFError:pass
        except Exception as exc:errors.append(exc)
    thread=threading.Thread(target=worker);thread.start()
    def read():return json.loads(_recv(client,struct.unpack('<I',_recv(client,4))[0]))
    try:
        assert read()['type']=='ready'
        image=np.zeros((2,2),np.uint8)
        _send_tracking_frame(client,1_000_000_000,image,image,[(1_000_000_000,0,0,9.81,0,0,0)],
                             recovery_protocol=PROTOCOL)
        assert read()['recovery_preaccept']['stage']=='native'
        assert observed[0].predictor.anchor is None
    finally:
        client.close();thread.join(timeout=3);server.close()
    assert not thread.is_alive() and not errors and bridge.callback is None
    assert observed[0].predictor.anchor is None
