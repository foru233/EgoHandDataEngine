"""Sensor selection must not leak across sessions or require absent IMU data."""
from types import SimpleNamespace
import time

import numpy as np

from egohand.vio.modes import apply_mode_environment
from egohand.vio.live_runtime import LiveVio


def test_child_sensor_environment_is_explicit_and_does_not_modify_parent():
    parent = {'EGOHAND_VIO_STEREO_ONLY': '1', 'PATH': 'original'}
    inertial = apply_mode_environment(parent, 'stereo_inertial')
    visual = apply_mode_environment(parent, 'stereo')
    assert inertial['EGOHAND_VIO_STEREO_ONLY'] == '0'
    assert visual['EGOHAND_VIO_STEREO_ONLY'] == '1'
    assert visual['EGOHAND_VIO_DISABLE_RECOVERY'] == '1'
    assert parent == {'EGOHAND_VIO_STEREO_ONLY': '1', 'PATH': 'original'}


def test_visual_live_start_and_restart_never_enable_imu(monkeypatch):
    # A camera without IMU hooks is sufficient; both initial start and restart work.
    vio = LiveVio(object(), lambda _message: None, mode='stereo')
    outcomes = iter([True, False])
    monkeypatch.setattr(vio, '_run_session', lambda: next(outcomes))
    monkeypatch.setattr(vio.stop_event, 'wait', lambda _: False)
    vio.start()
    vio.thread.join(timeout=2)
    assert not vio.thread.is_alive()


def test_recorded_worker_publishes_pose_with_its_camera_frame(monkeypatch, tmp_path):
    from egohand.gui.workers import PipelineWorker
    from egohand.vio import replay_runtime
    from egohand.core.types import SourceMetadata, StereoFrame, StereoHandResult
    class Source:
        recording_dir = tmp_path
        def get_metadata(self):return SourceMetadata('vs030_recorded')
        def frame_from_pair(self, pair):
            image = np.zeros((4, 4, 3), np.uint8)
            return StereoFrame(pair.index, pair.device_ts*1000, image, image)
    class Engine:
        detectors = None
        rectifier = None
        last_timestamp = None
        def process(self, frame, **kwargs):
            self.last_frame = frame
            return StereoHandResult(frame.frame_id, frame.timestamp_ns, None, 'vs030_recorded')
    class Replay:
        def __init__(self, directory, **kwargs):self.kwargs = kwargs
        def run(self):
            for index in range(3):
                pair = SimpleNamespace(index=index, device_ts=100+index)
                pose = {'type':'pose', 'timestamp_ns':pair.device_ts*1000, 'matrix':None}
                self.kwargs['on_frame'](pair, pose)
                self.kwargs['on_status'](pose)
            return {'status':'completed'}
    monkeypatch.setattr(replay_runtime, 'ReplayVio', Replay)
    worker = PipelineWorker(Source(), live_vio=True, vio_mode='stereo',vio_output_dir=tmp_path/'out')
    worker.engine = Engine()
    statuses = []
    worker.vio_status.connect(statuses.append)
    worker._run_recorded_vio(time.perf_counter())
    frame, result, stats = worker.output[-1]
    assert frame.timestamp_ns == result.timestamp_ns == result.diagnostics['vio_pose']['timestamp_ns']
    assert result.frame_id == 2 and stats['dropped_frames'] == 0
    assert len(worker.history) == 3
    assert not statuses  # Poses travel with the frame, not a racing Qt signal.
    from egohand.storage.replay import load_recorded_results
    saved, truncated = load_recorded_results(tmp_path/'out')
    assert len(saved) == 3 and not truncated
    assert saved[-1].diagnostics['vio_pose']['timestamp_ns'] == frame.timestamp_ns


def test_recorded_explicit_calibration_applies_to_raw_pixels_and_keeps_original_id(monkeypatch,tmp_path,calibration):
    from egohand.gui.workers import PipelineWorker
    from egohand.vio import replay_runtime
    from egohand.core.types import SourceMetadata,StereoFrame,StereoHandResult
    seen=[]
    class Source:
        recording_dir=tmp_path
        def get_metadata(self):return SourceMetadata('vs030_recorded')
        def frame_from_pair(self,pair):
            image=np.zeros((4,4,3),np.uint8)
            return StereoFrame(pair.index,pair.device_ts*1000,image,image,calibration_id='old-saved')
    class Engine:
        detectors=None;rectifier=None;last_timestamp=None
        def process(self,frame,**kwargs):
            seen.append(frame);self.last_frame=frame
            return StereoHandResult(frame.frame_id,frame.timestamp_ns,frame.calibration_id,'vendor_vs030')
    class Replay:
        def __init__(self,*args,**kwargs):self.kwargs=kwargs
        def run(self):
            self.kwargs['on_frame'](SimpleNamespace(index=0,device_ts=100),{'type':'pose','timestamp_ns':100000})
            return {'status':'completed'}
    monkeypatch.setattr(replay_runtime,'ReplayVio',Replay)
    worker=PipelineWorker(Source(),offline=True,calibration=calibration,vio_output_dir=tmp_path/'out')
    worker.engine=Engine();worker.engine.calibration=calibration
    worker._run_recorded_vio(time.perf_counter())
    assert seen[0].calibration_id==calibration.calibration_id
    assert seen[0].metadata['recorded_calibration_id']=='old-saved'
    assert seen[0].metadata['calibration_override_explicit'] is True
