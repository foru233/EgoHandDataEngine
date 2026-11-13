from pathlib import Path
import numpy as np
import pytest
from egohand.core.types import HandDetection
from egohand.detection.mediapipe_detector import (
    MediaPipeHandDetector,DEFAULT_RESCUE,restore_crop_detection,same_hand_pixels)


def hand(uv=None):
    uv=np.column_stack((np.linspace(20,60,21),np.linspace(30,90,21))) if uv is None else uv
    return HandDetection('Left',.9,np.column_stack((uv/[200,200],np.full(21,.1))),uv)


def test_crop_restores_pixels_normalized_z_and_bbox():
    d=hand();out=restore_crop_detection(d,(80,100,200,200),(600,800,3))
    np.testing.assert_allclose(out.pixel_landmarks,d.pixel_landmarks+[80,100])
    np.testing.assert_allclose(out.normalized_landmarks[:,:2],out.pixel_landmarks/[800,600])
    np.testing.assert_allclose(out.normalized_landmarks[:,2],.025)
    assert out.valid.all() and out.origin=='mediapipe_tiled_rescue'
    np.testing.assert_allclose(d.pixel_landmarks[0],[20,30])


def test_overlapping_crops_deduplicate_without_handedness_identity():
    a=hand();b=hand(a.pixel_landmarks+1);b.handedness='Right'
    assert same_hand_pixels(a,b)
    assert not same_hand_pixels(a,hand(a.pixel_landmarks+[180,0]))


def detector_stub(full):
    d=MediaPipeHandDetector.__new__(MediaPipeHandDetector)
    d.num_hands=2;d.rescue_config=dict(DEFAULT_RESCUE)
    calls=[]
    def infer(image,timestamp_ns=0,image_mode=False):
        calls.append((timestamp_ns,image_mode))
        return [hand()] if full else []
    d._detect_single=infer
    return d,calls


def test_successful_full_frame_never_changes_or_runs_tiles():
    d,calls=detector_stub(True)
    result=d.detect(np.zeros((480,640,3),np.uint8),123)
    assert len(result)==1 and calls==[(123,False)]
    assert d.last_detection_diagnostics['crop_attempts']==0


def test_zero_detection_uses_image_mode_for_tiles_and_no_invented_hands():
    d,calls=detector_stub(False)
    assert not d.detect(np.zeros((480,640,3),np.uint8),456)
    assert calls==[(456,False)]+[(456,True)]*4
    assert not d.last_detection_diagnostics['used_ground_truth']
    d.rescue_config['enabled']=False;calls.clear()
    assert not d.detect(np.zeros((480,640,3),np.uint8),789)
    assert calls==[(789,False)]


def test_partial_rescue_keeps_full_observation_and_deduplicates_label_flip():
    d,_=detector_stub(True)
    d.rescue_config['retry_partial']=True
    original=hand();duplicate=hand(original.pixel_landmarks+1);duplicate.handedness='Right'
    extra=hand(original.pixel_landmarks+[150,0]);extra.handedness='Right'
    crops=[]
    def infer(image,timestamp_ns=0,image_mode=False):
        if not image_mode:return [original]
        crops.append(image.shape)
        return [duplicate] if len(crops)==1 else [extra]
    d._detect_single=infer
    result=d.detect(np.zeros((480,640,3),np.uint8),123)
    assert result[0] is original and len(result)==2 and len(crops)==2
    np.testing.assert_allclose(result[1].pixel_landmarks,extra.pixel_landmarks+[224,0])
    assert d.last_detection_diagnostics['rescue_hands']==1
    assert result[1].origin=='mediapipe_tiled_rescue' and result[1].valid.all()


def test_partial_retry_cadence_never_reuses_old_points_or_skips_main_video_frame():
    d,_=detector_stub(True)
    d.rescue_config.update(retry_partial=True,partial_retry_interval_frames=3)
    calls=[]
    def infer(image,timestamp_ns=0,image_mode=False):
        calls.append((timestamp_ns,image_mode))
        return [] if image_mode else [hand()]
    d._detect_single=infer
    for stamp in range(1,5):
        assert len(d.detect(np.zeros((480,640,3),np.uint8),stamp))==1
    assert [stamp for stamp,crop in calls if not crop]==[1,2,3,4]
    assert [stamp for stamp,crop in calls if crop]==[1]*4+[4]*4


def test_complete_two_hand_frame_does_not_run_partial_rescue():
    d,_=detector_stub(True);d.rescue_config['retry_partial']=True
    calls=[]
    def infer(image,timestamp_ns=0,image_mode=False):
        calls.append(image_mode)
        return [hand(),hand(hand().pixel_landmarks+[150,0])]
    d._detect_single=infer
    assert len(d.detect(np.zeros((480,640,3),np.uint8),123))==2
    assert calls==[False]


def test_current_hand_roi_restores_pixels_retains_originals_and_context_label():
    d,_=detector_stub(True)
    d.rescue_config.update(enabled=False,refine_full_frame=True)
    original=hand(hand().pixel_landmarks+[100,100])
    observed=[]
    def infer(image,timestamp_ns=0,image_mode=False):
        if not image_mode:return [original]
        observed.append((image.shape,timestamp_ns))
        # The deterministic 128px crop begins at x=76,y=96 for this hand.
        candidate=hand(original.pixel_landmarks-[76,96]+1)
        candidate.handedness='Right'
        return [candidate]
    d._detect_single=infer
    result=d.detect(np.zeros((480,640,3),np.uint8),123)
    assert len(observed)==1 and observed[0][1]==123
    assert result[0].origin=='mediapipe_roi_refinement'
    assert result[0].handedness==original.handedness
    np.testing.assert_allclose(result[0].pixel_landmarks,original.pixel_landmarks+1)
    assert d.last_detection_diagnostics['full_frame_observations'][0] is original
    np.testing.assert_allclose(original.pixel_landmarks,hand().pixel_landmarks+[100,100])


def test_roi_refinement_rejects_observation_shared_by_two_original_hands():
    d,_=detector_stub(True);d.rescue_config.update(enabled=False,refine_full_frame=True)
    originals=[hand(hand().pixel_landmarks+[100,100]),hand(hand().pixel_landmarks+[102,100])]
    def infer(image,timestamp_ns=0,image_mode=False):
        if not image_mode:return originals.copy()
        # First crop x=76 and second x=78; both candidates are spatially
        # indistinguishable from two originals, so neither may replace a hand.
        return [hand(hand().pixel_landmarks+[24,4])]
    d._detect_single=infer
    result=d.detect(np.zeros((480,640,3),np.uint8),123)
    assert all(a is b for a,b in zip(result,originals))
    assert d.last_detection_diagnostics['refined_hands']==0

def test_realtime_rescue_rotates_tiles_without_changing_pixel_coordinates():
    d,_=detector_stub(False);d.rescue_config['max_crops_per_frame']=1
    image=np.zeros((480,640,3),np.uint8);image[:240,:320]=17;image[:240,320:]=41
    image[240:,:320]=83;image[240:,320:]=137;observed=[]
    def infer(crop,timestamp_ns=0,image_mode=False):
        if image_mode:observed.append(int(crop[crop.shape[0]//2,crop.shape[1]//2,0]))
        return []
    d._detect_single=infer
    for i in range(4):d.detect(image,i)
    assert observed==[17,41,83,137]


@pytest.mark.assets
def test_real_hot3d_visible_hand_rescued_without_lower_thresholds():
    import cv2
    from egohand.sources.hot3d import HOT3DSource
    root=Path(__file__).resolve().parents[1]
    clip=root/'samples/date/hot3d/train_aria/clip-001851.tar';model=root/'models/hand_landmarker.task'
    if not clip.is_file() or not model.is_file():pytest.skip('Actual HOT3D/official model unavailable')
    s=HOT3DSource(clip);s.open();s.start()
    d=MediaPipeHandDetector(model,rescue_config={'enabled':False})
    try:
        frame=s.read();image=np.ascontiguousarray(np.rot90(frame.left_image,3))
        assert d.detect(image)==[]
        d.rescue_config['enabled']=True
        ds=d.detect(image)
        assert ds and all(x.valid.sum()>=16 for x in ds)
        assert d.last_detection_diagnostics['rescue_hands']>=1
        assert all(x.origin=='mediapipe_tiled_rescue' for x in ds)
        assert not d.last_detection_diagnostics['used_ground_truth']
    finally:d.close();s.close()


@pytest.mark.assets
def test_video_rescue_does_not_reuse_video_timestamp_for_crops():
    model=Path('models/hand_landmarker.task')
    if not model.is_file():pytest.skip('Official model unavailable')
    d=MediaPipeHandDetector(model,mode='VIDEO')
    try:
        image=np.zeros((256,256,3),np.uint8)
        assert not d.detect(image,1_000_000_000)
        assert d._image_landmarker is not None
        assert not d.detect(image,1_033_000_000)
        with pytest.raises(ValueError,match='递增'):d.detect(image,1_033_000_000)
    finally:d.close()
