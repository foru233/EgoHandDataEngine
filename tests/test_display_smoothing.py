from dataclasses import replace
import numpy as np
from egohand.visualization.display_smoothing import DisplaySmoother

def test_display_filter_reduces_jitter_without_mutating_raw_or_filling_invalid(engine,source):
    first=engine.process(source.read());first=replace(first,diagnostics={**first.diagnostics,'synthetic':False})
    filt=DisplaySmoother();filt.update(first)
    d=first.left_detections[0]
    moved=replace(d,pixel_landmarks=d.pixel_landmarks+4.)
    h=first.hands[0];joints=[replace(j,xyz_m=j.xyz_m+np.array([.01,0,0])) for j in h.joints]
    joints[1]=replace(joints[1],valid=False,xyz_m=np.full(3,np.nan))
    second=replace(first,timestamp_ns=first.timestamp_ns+50_000_000,left_detections=[moved],hands=[replace(h,joints=joints)])
    shown=filt.update(second)
    assert 0<shown.left_detections[0].pixel_landmarks[0,0]-d.pixel_landmarks[0,0]<4
    assert 0<shown.hands[0].joints[0].xyz_m[0]-h.joints[0].xyz_m[0]<.01
    assert not shown.hands[0].joints[1].valid and np.isnan(shown.hands[0].joints[1].xyz_m).all()
    np.testing.assert_allclose(second.left_detections[0].pixel_landmarks,moved.pixel_landmarks)
    np.testing.assert_allclose(second.hands[0].joints[0].xyz_m,joints[0].xyz_m)
    assert shown.diagnostics['display_smoothed'] and 'display_smoothed' not in second.diagnostics

def test_filter_reset_after_loss_or_session_change(engine,source):
    a=engine.process(source.read());a=replace(a,diagnostics={**a.diagnostics,'synthetic':False})
    filt=DisplaySmoother();filt.update(a)
    d=a.left_detections[0];moved=replace(d,pixel_landmarks=d.pixel_landmarks+10)
    late=replace(a,timestamp_ns=a.timestamp_ns+1_000_000_000,left_detections=[moved])
    np.testing.assert_allclose(filt.update(late).left_detections[0].pixel_landmarks,moved.pixel_landmarks)
    other=replace(late,session_id='new',timestamp_ns=late.timestamp_ns+1,left_detections=[d])
    np.testing.assert_allclose(filt.update(other).left_detections[0].pixel_landmarks,d.pixel_landmarks)

def test_smoothing_when_cross_frame_ids_are_disabled(engine,source):
    engine.config['enable_3d_tracking']=False
    first=engine.process(source.read());first=replace(first,diagnostics={**first.diagnostics,'synthetic':False})
    h=first.hands[0];first=replace(first,hands=[h])
    filt=DisplaySmoother();filt.update(first)
    moved=[replace(j,xyz_m=j.xyz_m+np.array([.01,0,0])) for j in h.joints]
    second=replace(first,timestamp_ns=first.timestamp_ns+50_000_000,hands=[replace(h,joints=moved)])
    shown=filt.update(second)
    assert 0<shown.hands[0].joints[0].xyz_m[0]-h.joints[0].xyz_m[0]<.01
    assert shown.hands[0].track_id==-1
