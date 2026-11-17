import json
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from egohand.calibration.schema import synthetic_calibration
from egohand.core.types import StereoFrame
from egohand.storage.depth_results import DepthResultWriter,DepthResultReader,DEPTH_FILE
from egohand.visualization.stereo_depth import DepthCache,compute_depth


@pytest.mark.parametrize('vertical',[False,True])
def test_metric_stereo_depth_and_reference_geometry(vertical):
    cal=synthetic_calibration(320,240)
    rng=np.random.default_rng(24)
    left=rng.integers(0,256,(240,320,3),dtype=np.uint8)
    right=np.zeros_like(left)
    if vertical:
        cal.t_right_left=np.array([0.,.064,0.]);right[16:]=left[:-16]
    else:right[:,:-16]=left[:,16:]
    frame=StereoFrame(7,123,left,right,calibration_id=cal.calibration_id,source_type='synthetic')
    depth,info=compute_depth(frame,cal,DepthCache())
    valid=depth>0
    # SGBM and its reverse check each lose a 96-pixel search border.
    assert valid.mean()>.2
    assert np.median(depth[valid])==pytest.approx(600*.064/16,rel=.02)
    assert info['reference_camera']==('right' if vertical else 'left')
    assert info['disparity_axis']==int(vertical)
    assert info['quantity']=='rectified_optical_z' and info['unit']=='m'
    assert depth.shape==(240,320) and depth.dtype==np.float32


def test_depth_sidecar_roundtrip_identity_and_calibration_binding(tmp_path,monkeypatch):
    import egohand.storage.depth_results as module
    cal=synthetic_calibration();result_file=tmp_path/'frames.jsonl';result_file.write_text('original result')
    frame=SimpleNamespace(frame_id=9,timestamp_ns=123,calibration_id=cal.calibration_id)
    expected=np.array([[0.,1.25],[2.5,3.75]],np.float32)
    info={'shape':[2,2],'unit':'m','reference_camera':'right'}
    monkeypatch.setattr(module,'compute_depth',lambda *args:(expected,info))
    writer=DepthResultWriter(tmp_path,cal,input_pixels='test')
    writer.write(frame);writer.close(result_file,completed=True)
    reader=DepthResultReader(tmp_path,result_file,cal)
    try:
        actual,metadata=reader.read(frame)
        np.testing.assert_array_equal(actual,expected)
        assert metadata['status']=='available' and metadata['unit']=='m'
        assert reader.read(SimpleNamespace(frame_id=9,timestamp_ns=124,calibration_id=cal.calibration_id)) is None
        with pytest.raises(ValueError,match='身份不匹配'):
            reader.read(SimpleNamespace(frame_id=9,timestamp_ns=123,calibration_id='other'))
    finally:reader.close()
    with pytest.raises(FileExistsError):DepthResultWriter(tmp_path,cal,input_pixels='test')
    changed=replace(cal,K_left=cal.K_left*1.01)
    with pytest.raises(ValueError,match='不匹配'):DepthResultReader(tmp_path,result_file,changed)
    result_file.write_text('changed result')
    with pytest.raises(ValueError,match='不匹配'):DepthResultReader(tmp_path,result_file,cal)


def test_failed_depth_frame_remains_explicit_not_fake_zero_map(tmp_path,monkeypatch):
    import egohand.storage.depth_results as module
    cal=synthetic_calibration();result_file=tmp_path/'frames.jsonl';result_file.write_text('result')
    def fail(*args):raise ValueError('missing stereo')
    monkeypatch.setattr(module,'compute_depth',fail)
    writer=DepthResultWriter(tmp_path,cal,input_pixels='test')
    frame=SimpleNamespace(frame_id=0,timestamp_ns=1,calibration_id=cal.calibration_id)
    assert writer.write(frame)['status']=='unavailable'
    writer.close(result_file,completed=True)
    reader=DepthResultReader(tmp_path,result_file,cal)
    try:
        assert reader.metadata['status']=='partial'
        depth,info=reader.read(frame)
        assert depth is None and info['reason']=='missing stereo'
    finally:reader.close()


def test_offline_worker_calculation_persists_depth(tmp_path,monkeypatch,qtbot):
    from egohand.gui.workers import PipelineWorker
    from egohand.core.types import StereoHandResult
    from egohand.vio.hand_world import RecordedLocalizationHandJoin
    import egohand.vio.replay_runtime as runtime
    import egohand.storage.depth_results as module
    cal=synthetic_calibration(320,240)
    frame=StereoFrame(1,123,np.zeros((240,320,3),np.uint8),np.zeros((240,320,3),np.uint8),calibration_id=cal.calibration_id,source_type='recorded')
    source=SimpleNamespace(recording_dir=tmp_path/'original',frame_from_pair=lambda pair:frame)
    w=PipelineWorker(source,offline=True,vio_output_dir=tmp_path/'computed',save_depth=True)
    w.engine=SimpleNamespace(calibration=cal,session_id='test')
    monkeypatch.setattr(RecordedLocalizationHandJoin,'from_recording',lambda *args,**kwargs:None)
    monkeypatch.setattr(module,'compute_depth',lambda *args:(np.ones((2,2),np.float32),{'shape':[2,2],'unit':'m'}))
    def process(frame,started):
        w.processed+=1
        return StereoHandResult(frame.frame_id,frame.timestamp_ns,cal.calibration_id,'recorded')
    monkeypatch.setattr(w,'_process_frame',process)
    monkeypatch.setattr(w,'_commands',lambda:None)
    class Replay:
        def __init__(self,*args,**kwargs):self.on_frame=kwargs['on_frame']
        def run(self):self.on_frame(None,{});return {'status':'completed'}
    monkeypatch.setattr(runtime,'ReplayVio',Replay)
    w._run_recorded_vio(0.)
    root=tmp_path/'computed'
    assert (root/DEPTH_FILE).is_file()
    result=json.loads((root/'results/frames.jsonl').read_text())
    assert result['diagnostics']['stereo_depth']['status']=='available'
    assert json.loads((root/'reprocessing.json').read_text())['stereo_depth']['count']==1
    reader=DepthResultReader(root,root/'results/frames.jsonl',cal)
    try:np.testing.assert_array_equal(reader.read(frame)[0],np.ones((2,2),np.float32))
    finally:reader.close()
