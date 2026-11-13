"""Exercise asynchronous capture identity, admission, bounds and writer lifetime."""
import copy
from dataclasses import replace
import json
import queue
import time
from types import SimpleNamespace

import numpy as np
import pytest

from egohand.core.types import Hand3D, StereoFrame, StereoHandResult, SourceMetadata, result_from_dict
from egohand.vio.live_hand_join import LiveLocalizationHandJoin, live_capture_binding
from egohand.vio.hand_world import calibration_geometry_fingerprint
from test_recorded_localization_hand_join import admitted_pose


@pytest.fixture
def live_context(calibration):
    tick = [0.]
    session = 'e' * 32
    join = LiveLocalizationHandJoin(calibration, source_session_id=session, clock=lambda: tick[0], max_pending=2)

    def sample(index=1):
        image = np.zeros((4, 4, 3), np.uint8)
        sdk_stamp = 1_000_000_000 + index * 33_333_000
        host_stamp = 9_000_000_000 + index * 34_000_000
        frame = StereoFrame(index, host_stamp, image, image, calibration_id=calibration.calibration_id,
            source_type='vendor_vs030', metadata={'sdk_sequence': index + 42,
                'sdk_device_timestamp_raw': sdk_stamp // 1000,
                'timestamp_kind': 'host_bundle_decode_completed_monotonic_ns',
                'transport_pairing': 'sdk_sequence_and_equal_device_timestamp',
                'acquired_perf_counter_ns': time.perf_counter_ns()})
        hand = Hand3D('Left', 1., 17)
        hand.joints[0].valid = True
        hand.joints[0].xyz_m = np.array([.1, .2, 1.])
        result = StereoHandResult(index, host_stamp, calibration.calibration_id, 'vendor_vs030',
                                  hands=[hand], session_id='hand-session')
        pose = admitted_pose(sdk_stamp)
        pose['live_capture'] = live_capture_binding(frame, session)
        pose['live_config'] = {'schema_version': 1, 'fingerprint': 'a' * 64,
            'calibration_file_sha256': 'b' * 64, 'camera_frame': 'raw_cam0_channel4',
            'camera_geometry_sha256': calibration_geometry_fingerprint(calibration)}
        return frame, result, pose
    return SimpleNamespace(join=join, sample=sample, calibration=calibration, tick=tick)


@pytest.mark.parametrize('pose_first', [False, True])
def test_exact_capture_joins_both_arrival_orders_without_equating_clocks(live_context, pose_first):
    c = live_context
    frame, result, pose = c.sample()
    attachment = object()
    assert result.timestamp_ns != pose['timestamp_ns']
    if pose_first:
        assert c.join.add_pose(pose) == []
    ready = c.join.offer(result, frame, calibration=c.calibration, attachment=attachment)
    if not pose_first:
        assert ready == []
        ready = c.join.add_pose(pose)
    [(saved, sink)] = ready
    assert sink is attachment and saved.world_hands['status'] == 'available'
    np.testing.assert_allclose(saved.world_hands['hands'][0]['xyz_world_m'][0], [.4, .2, 1.])
    np.testing.assert_array_equal(saved.hands[0].joints[0].xyz_m, [.1, .2, 1.])
    assert saved.timestamp_ns == frame.timestamp_ns
    assert saved.world_hands['pose_timestamp_ns'] == pose['timestamp_ns']
    assert saved.world_hands['join']['sdk_sequence'] == frame.metadata['sdk_sequence']
    assert saved.world_hands['join']['exposure_synchrony_verified'] is False
    assert saved.world_hands['world_reference']['cross_frame_merge_permitted'] is False
    assert result.world_hands is None and not c.join.pending
    assert c.join.add_pose(pose) == [] and c.join.flush() == []


@pytest.mark.parametrize('area,field,value', [
    ('capture', 'source_session_id', 'f' * 32),
    ('capture', 'source_frame_id', 2),
    ('capture', 'source_frame_id', True),
    ('capture', 'hand_timestamp_ns', 10),
    ('capture', 'sdk_sequence', 99),
    ('capture', 'sdk_device_timestamp_raw', 999),
    ('capture', 'hand_timestamp_kind', 'sdk_device'),
    ('capture', 'transport_pairing', 'nearest_timestamp'),
    ('capture', 'exposure_synchrony_verified', True),
    ('config', 'fingerprint', 'invalid'),
    ('config', 'calibration_file_sha256', None),
    ('config', 'camera_geometry_sha256', 'f' * 64),
    ('config', 'camera_frame', 'orb_left'),
    ('pose', 'timestamp_ns', 9_034_000_000),
    ('report', 'stage', 'native'),
    ('report', 'timestamp_ns', 0),
])
def test_wrong_capture_clock_calibration_or_commit_never_yields_world(live_context, area, field, value):
    c = live_context
    frame, result, pose = c.sample()
    target = {'capture': pose['live_capture'], 'config': pose['live_config'],
              'pose': pose, 'report': pose['recovery_preaccept']}[area]
    target[field] = value
    assert c.join.offer(result, frame, calibration=c.calibration) == []
    ready = c.join.add_pose(pose) + c.join.flush()
    assert len(ready) == 1 and ready[0][0].world_hands['status'] == 'unavailable'
    assert ready[0][0].world_hands['hands'] == []


@pytest.mark.parametrize('fault', ['quarantine', 'not_admitted', 'unknown_contract', 'legacy_only'])
def test_existing_consumer_admission_is_authoritative(live_context, fault):
    c = live_context
    frame, result, pose = c.sample()
    if fault == 'quarantine':
        pose['recovery_preaccept']['await_visual_requalification'] = True
    elif fault == 'not_admitted':
        pose['localization']['admission']['world_hands_permitted'] = False
    elif fault == 'unknown_contract':
        pose['localization']['contract_version'] = 999
    else:
        pose.pop('localization')
    c.join.offer(result, frame, calibration=c.calibration)
    [(saved, _)] = c.join.add_pose(pose)
    assert saved.world_hands['status'] == 'unavailable' and not saved.world_hands['hands']
    np.testing.assert_array_equal(saved.hands[0].joints[0].xyz_m, result.hands[0].joints[0].xyz_m)


def test_ordered_release_waits_for_earlier_result_and_expires_missing_exact_pair(live_context):
    c = live_context
    one, two = c.sample(1), c.sample(2)
    assert c.join.offer(one[1], one[0], calibration=c.calibration) == []
    assert c.join.offer(two[1], two[0], calibration=c.calibration) == []
    assert c.join.add_pose(two[2]) == []
    c.tick[0] = 2.1
    ready = c.join.drain()
    assert [r.frame_id for r, _ in ready] == [1, 2]
    assert ready[0][0].world_hands['reason'] == 'localization_join_timeout'
    assert ready[1][0].world_hands['status'] == 'available'
    assert c.join.add_pose(one[2]) == []  # no late rewrite after writer release


def test_capacity_flush_duplicates_and_source_mutation_preserve_camera_results(live_context):
    c = live_context
    samples = [c.sample(i) for i in (1, 2, 3)]
    for frame, result, _ in samples[:2]:
        assert c.join.offer(result, frame, calibration=c.calibration) == []
    samples[0][1].hands[0].joints[0].xyz_m[:] = 99  # buffered result is a value copy
    ready = c.join.offer(samples[2][1], samples[2][0], calibration=c.calibration)
    assert len(ready) == 1 and ready[0][0].world_hands['reason'] == 'localization_join_capacity'
    np.testing.assert_array_equal(ready[0][0].hands[0].joints[0].xyz_m, [.1, .2, 1.])
    assert len(c.join.pending) == 2
    ready = c.join.flush('recording_stopped')
    assert [r.frame_id for r, _ in ready] == [2, 3]
    assert all(r.world_hands['reason'] == 'recording_stopped' for r, _ in ready)
    assert c.join.flush() == []
    duplicate = c.join.offer(samples[2][1], samples[2][0], calibration=c.calibration)
    assert duplicate[0][0].world_hands['reason'] == 'non_monotonic_join'


def test_pose_cache_bound_and_duplicate_cannot_replace_buffered_geometry(live_context):
    c = live_context
    for index in range(1, 11):
        c.join.add_pose(c.sample(index)[2])
    assert len(c.join.poses) == 2 and c.join.stats['pose_evicted'] == 8
    frame, result, pose = c.sample(10)
    pose['matrix'][0][3] = 10  # duplicate is not allowed to replace the first pose
    c.join.add_pose(pose)
    [(saved, _)] = c.join.offer(result, frame, calibration=c.calibration)
    assert saved.world_hands['status'] == 'available'
    np.testing.assert_allclose(saved.world_hands['hands'][0]['xyz_world_m'][0], [.4, .2, 1.])


def test_config_is_latched_and_explicit_session_reset_flushes_old_pending(live_context):
    c = live_context
    c.join.add_pose(c.sample(1)[2])
    frame, result, pose = c.sample(2)
    pose['live_config']['fingerprint'] = 'c' * 64
    assert c.join.offer(result, frame, calibration=c.calibration) == []
    assert c.join.add_pose(pose) == []
    [(saved, _)] = c.join.flush('localization_session_reset', reset=True)
    assert saved.world_hands['reason'] == 'localization_session_reset'
    assert not c.join.poses
    frame, result, pose = c.sample(3)
    pose['live_config']['fingerprint'] = 'c' * 64
    c.join.add_pose(pose)
    assert c.join.offer(result, frame, calibration=c.calibration)[0][0].world_hands['status'] == 'available'


def test_actual_live_descriptor_binds_verified_phase_and_mode_without_mutating_input():
    from egohand.vio.live_runtime import live_hand_config
    descriptor={'fingerprint':'a'*64,'hashes':{'calibration':'b'*64}}
    timing={'phase_ns':52_000_000,'phase_source':'independently_verified_session'}
    first=live_hand_config(descriptor,'c'*64,'stereo_inertial',timing)
    second=live_hand_config(descriptor,'c'*64,'stereo_inertial',{**timing,'phase_ns':106_000_000})
    other=live_hand_config(descriptor,'c'*64,'hybrid',timing)
    assert first['fingerprint']!=second['fingerprint']!=other['fingerprint']
    assert first['base_config_fingerprint']==descriptor['fingerprint']
    assert 'fingerprint' not in timing


@pytest.mark.parametrize('target,field,value',[
    ('result','coordinate_frame','rectified'), ('result','unit','mm'),
    ('result','source_type','external'), ('result','session_id',''),
    ('result','calibration_id','other'), ('result','timestamp_ns',1),
    ('frame','frame_id',True), ('frame','calibration_id','other'),
])
def test_hand_side_identity_and_geometry_are_checked(live_context,target,field,value):
    c=live_context
    frame,result,_=c.sample()
    setattr({'frame':frame,'result':result}[target],field,value)
    [(saved,_)]=c.join.offer(result,frame,calibration=c.calibration)
    assert saved.world_hands['status']=='unavailable' and saved.world_hands['hands']==[]


def worker_context(c, recorder):
    from egohand.gui.workers import PipelineWorker
    class Engine:
        detectors = None
        rectifier = None
        last_timestamp = None
        calibration = c.calibration
        def process(self, frame, **kwargs):
            self.last_frame = frame
            return c.sample(frame.frame_id)[1]
    source = SimpleNamespace(loop=False, get_metadata=lambda: SourceMetadata('vendor_vs030'))
    worker = PipelineWorker(source)
    worker.engine, worker.recorder, worker._live_world_join = Engine(), recorder, c.join
    return worker


def test_worker_real_writer_stop_flush_and_export_readback(live_context, tmp_path):
    from egohand.storage.recorder import AsyncRecorder
    from egohand.storage.exporter import export_all, load_json
    c = live_context
    recorder = AsyncRecorder(tmp_path / 'recording', SourceMetadata('vendor_vs030'), c.calibration)
    worker = worker_context(c, recorder)
    try:
        frame, _, pose = c.sample(1)
        preview = worker._process_frame(frame, time.perf_counter())
        assert preview.world_hands['status'] == 'pending' and recorder.writer.manifest['result_count'] == 0
        worker._vio_status(pose)  # producer callback doesn't write or mutate the pending result
        assert len(c.join.pending) == 1
        worker._drain_live_world_join()
        assert preview.world_hands['status']=='available'
        assert worker.history[-1].world_hands['status']=='available'
        assert worker.output[-1][1].world_hands['status']=='available'
        worker._process_frame(c.sample(2)[0], time.perf_counter())
        worker.command('record_stop')
        worker._commands()
        assert worker.recorder is None and not c.join.pending and not recorder.thread.is_alive()
        assert all(r.world_hands['status']!='pending' for r in worker.history)
    finally:
        if recorder.thread.is_alive():recorder.close()
    rows = [result_from_dict(json.loads(line)) for line in
            (tmp_path / 'recording/results/frames.jsonl').read_text(encoding='utf8').splitlines()]
    assert [r.frame_id for r in rows] == [1, 2]
    assert rows[0].world_hands['status'] == 'available'
    assert rows[1].world_hands['reason'] == 'recording_stopped'
    assert all(r.world_hands['status'] != 'pending' for r in rows)
    destination = export_all(rows, tmp_path / 'export')
    restored = load_json(destination / 'results.json')
    assert restored[0].world_hands['join']['recovery_session_id'] == pose['recovery_preaccept']['session_id']
    for result in restored:
        np.testing.assert_array_equal(result.hands[0].joints[0].xyz_m, [.1, .2, 1.])
        assert result.world_hands['world_reference']['cross_frame_merge_permitted'] is False


def test_worker_pause_flushes_before_writer_pause_and_reset_never_reuses_old_pose(live_context):
    c = live_context
    class Sink:
        accepting = True
        paused = False
        error = None
        started = time.perf_counter()
        bytes_written = 0
        queue = queue.Queue()
        def __init__(self):self.rows = []
        def submit(self, kind, value, offline):
            if kind == 'result':
                assert not self.paused
                self.rows.append(copy.deepcopy(value))
        def set_paused(self, value):self.paused = value
    sink = Sink()
    worker = worker_context(c, sink)
    worker._process_frame(c.sample(1)[0], time.perf_counter())
    worker.command('record_pause', True)
    worker._commands()
    assert sink.paused and sink.rows[0].world_hands['reason'] == 'recording_paused'
    worker._process_frame(c.sample(2)[0], time.perf_counter())
    worker._vio_status({'type': 'vio_reset'})
    worker._drain_live_world_join()
    worker._vio_status(c.sample(1)[2])
    worker._drain_live_world_join()
    assert len(sink.rows) == 1 and not c.join.pending and not c.join.poses


def test_worker_mailbox_overflow_is_bounded_and_explicit(live_context):
    c = live_context
    worker = worker_context(c, None)
    worker._process_frame(c.sample(1)[0], time.perf_counter())
    worker._live_pose_mailbox = queue.Queue(maxsize=1)
    worker._vio_status(c.sample(2)[2])
    worker._vio_status(c.sample(3)[2])
    assert worker._live_pose_mailbox_overflow.is_set() and worker._live_pose_mailbox.qsize() == 1
    worker._drain_live_world_join()
    assert not c.join.pending and c.join.stats['unavailable'] == 1


def test_live_runtime_publishes_binding_from_actual_dequeued_frame(live_context,tmp_path,monkeypatch):
    """Actual LiveVio loop + admission/commit; explicit fake IPC/geometry, no DLL."""
    from egohand.vio import live_runtime
    from egohand.vio.recovery_contract import application_feedback
    from egohand.vio.recovery_coordinator import RecoveryCoordinator
    c=live_context
    frame,_,pose=c.sample()
    native={k:copy.deepcopy(v) for k,v in pose.items() if k in (
        'type','timestamp_ns','frame_id','map_epoch','map_id','reset_id','map_revision',
        'world_frame_id','world_frame_revision','snapshot_available','snapshot_consistent',
        'tracking_state','matrix','pose_valid','imu_initialized','ba1_complete','ba2_complete',
        'inertial_scale_finalized')}
    coordinator=RecoveryCoordinator(application_admission=True)
    coordinator.begin_frame([])
    native['recovery_preaccept']=coordinator.finish_frame(native,None)
    config=SimpleNamespace(calibration=tmp_path/'calibration.yaml',settings=tmp_path/'settings.yaml',
        quality={'stable_seconds':0},timing={'image_scale':1.,'imu_alignment':'timestamp'},
        validate_geometry=lambda geometry:geometry,
        describe=lambda:{'fingerprint':'a'*64,'hashes':{'calibration':'b'*64}})
    geometry=SimpleNamespace(T_orb_left_from_gui_left=np.eye(4),
        rectifier=SimpleNamespace(remap_pair=lambda left,right:(left,right)))
    class Connection:
        def __init__(self):self.sent=[]
        def settimeout(self,_):pass
        def setsockopt(self,*_):pass
        def sendall(self,data):self.sent.append(bytes(data))
        def close(self):pass
    connection=Connection()
    class Listener:
        def bind(self,_):pass
        def listen(self,_):pass
        def settimeout(self,_):pass
        def getsockname(self):return ('127.0.0.1',12345)
        def accept(self):return connection,('127.0.0.1',1)
        def close(self):pass
    worker=SimpleNamespace(request_session=lambda *a,**k:(SimpleNamespace(poll=lambda:None),False,tmp_path/'worker.log'))
    monkeypatch.setattr(live_runtime,'ROOT',tmp_path)
    monkeypatch.setattr(live_runtime.socket,'socket',Listener)
    monkeypatch.setattr(live_runtime,'prepare_vs030_vio_geometry',lambda _:geometry)
    import egohand.vio.config_snapshot as snapshots
    import egohand.calibration.loader as loader
    monkeypatch.setattr(snapshots,'resolve_live_config',lambda:config)
    monkeypatch.setattr(loader,'load_calibration',lambda *a,**k:c.calibration)
    messages=[]
    vio=live_runtime.LiveVio(SimpleNamespace(),messages.append,worker=worker,mode='stereo')
    vio.submit(frame)
    replies=iter([{'type':'ready','recovery_protocol':2,'recovery_owner':'RecoveryCoordinator'},native])
    def read_message():
        value=next(replies)
        if value.get('type')=='pose':vio.stop_event.set()
        return value
    monkeypatch.setattr(vio,'_read_message',read_message)
    monkeypatch.setattr(vio,'_commit_recovery',lambda row:coordinator.finish_application_frame(application_feedback(row)))
    assert vio._run_session() is False
    published=next(m for m in messages if m.get('type')=='pose')
    assert published['live_capture']==live_capture_binding(frame,vio.source_session_id)
    assert published['timestamp_ns']==frame.metadata['sdk_device_timestamp_raw']*1000
    assert published['timestamp_ns']!=frame.timestamp_ns
    assert published['live_config']['camera_geometry_sha256']==calibration_geometry_fingerprint(c.calibration)
    assert published['live_config']['effective_timing']['imu_enabled'] is False
    assert published['recovery_preaccept']['stage']=='committed'
    assert len(connection.sent)>=3 and vio.frame_queue_stats()['dequeued']==1
