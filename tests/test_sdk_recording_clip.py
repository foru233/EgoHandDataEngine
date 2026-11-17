from pathlib import Path
import hashlib
import json
import pytest
from egohand.storage.recording_clip import save_sdk_clip,save_sdk_clip_time_range
from egohand.storage.sdk_recording import iter_paired_frames,iter_imu_packets
from egohand.sources.sdk_recorded import SdkRecordedStereoSource


def test_real_sdk_clip_preserves_pairs_and_imu(tmp_path):
    source=Path('data/ori/hand/session_20261110_111807_489019700')
    if not (source/'manifest.json').exists():pytest.skip('real recording unavailable')
    original=list(iter_paired_frames(source,limit=46))
    dest=save_sdk_clip(source,tmp_path/'clip',20,45)
    clipped=list(iter_paired_frames(dest))
    assert [p.index for p in clipped]==list(range(26))
    index=[json.loads(line) for line in (dest/'raw/camera_frames.jsonl').read_text(encoding='utf8').splitlines()]
    assert [(r['frame_id'],r['timestamp_ns']) for r in index]==[(p.index,p.device_ts*1000) for p in clipped]
    assert [(p.sequence,p.device_ts) for p in clipped]==[(p.sequence,p.device_ts) for p in original[20:46]]
    for a,b in zip(clipped,original[20:46]):
        assert (a.left_bgr==b.left_bgr).all() and (a.right_bgr==b.right_bgr).all()
    first,last=clipped[0].device_ts,clipped[-1].device_ts
    expected=[(p.device_ts,hashlib.sha256(p.payload).digest()) for p in iter_imu_packets(source)
              if first-2_000_000<=p.device_ts<=last+2_000_000]
    actual=[(p.device_ts,hashlib.sha256(p.payload).digest()) for p in iter_imu_packets(dest)]
    assert actual==expected and actual[0][0]<=first<=last<=actual[-1][0]
    replay=SdkRecordedStereoSource(dest);replay.open();replay.start()
    try:
        replay.seek(12);frame=replay.read()
        assert frame.frame_id==12 and frame.timestamp_ns==clipped[12].device_ts*1000
    finally:replay.close()


def test_trim_dialog_preview_follows_each_handle(qtbot,monkeypatch):
    import recording_trim
    source=Path('data/ori/hand/session_20261110_111807_489019700')
    if not (source/'manifest.json').exists():pytest.skip('real recording unavailable')
    original=recording_trim.iter_paired_frames;requests=[]
    def requested(path,**kwargs):
        requests.append(kwargs.get('min_device_ts'))
        yield from original(path,**kwargs)
    monkeypatch.setattr(recording_trim,'iter_paired_frames',requested)
    dialog=recording_trim.RecordingTrimDialog(source);qtbot.addWidget(dialog)
    try:
        dialog.show();qtbot.waitUntil(lambda:dialog.save.isEnabled(),timeout=30000)
        assert 'QDialog#recordingTrim { background: #0D1118;' in dialog.styleSheet()
        assert dialog.image.objectName()=='trimPreview'
        assert len(requests)==1 and dialog.times
        dialog.start_slider.setValue(25)
        qtbot.waitUntil(lambda:dialog.save.isEnabled(),timeout=30000)
        assert '起点预览' in dialog.info.text() and not dialog.image.pixmap().isNull()
        dialog.end_slider.setValue(45)
        qtbot.waitUntil(lambda:dialog.save.isEnabled(),timeout=30000)
        assert '终点预览' in dialog.info.text() and len(requests)==3
        dialog.start_slider.setValue(10);dialog.start_slider.setValue(15);dialog.start_slider.setValue(20)
        qtbot.waitUntil(lambda:dialog.save.isEnabled(),timeout=30000)
        assert len(requests)==4
        assert abs(dialog._selection['start']-dialog.times[20])<=250_000
    finally:dialog.reject()


def test_time_range_uses_exact_displayed_frames(tmp_path):
    source=Path('data/ori/hand/session_20261110_111807_489019700')
    if not (source/'manifest.json').exists():pytest.skip('real recording unavailable')
    pairs=list(iter_paired_frames(source,limit=46))
    dest=save_sdk_clip_time_range(source,tmp_path/'timed_clip',pairs[20].device_ts,pairs[45].device_ts)
    assert [(x.sequence,x.device_ts) for x in iter_paired_frames(dest)]==[(x.sequence,x.device_ts) for x in pairs[20:46]]
