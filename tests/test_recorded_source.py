import json
import numpy as np
import cv2
import pytest
from egohand.storage.recorder import Recorder,AsyncRecorder
from egohand.sources.recorded import RecordedStereoSource,read_jsonl_recover
from egohand.sources.vendor_placeholder import VendorStereoSource

def test_roundtrip(source,tmp_path):
    f=source.read();path=tmp_path/"r"
    w=Recorder(path,source.get_metadata(),source.get_calibration());w.write_raw(f);w.close()
    s=RecordedStereoSource(path,reuse_detections=True);s.open();s.start()
    q=s.read()
    np.testing.assert_equal(f.left_image,q.left_image)
    assert q.left_timestamp_ns==f.left_timestamp_ns
    assert q.timestamp_ns==f.timestamp_ns
    assert q.calibration_id==f.calibration_id
    assert q.left_detections[0].origin=="synthetic_projection"
    s.seek(0);s.pause();assert s.read() is None;s.resume();assert s.read().frame_id==f.frame_id
    s.close()

def test_interrupted_tail(tmp_path):
    p=tmp_path/"frames.jsonl";p.write_text('{"frame_id":1}\n{"frame_id":',encoding="utf-8")
    rows,truncated=read_jsonl_recover(p)
    assert rows==[{"frame_id":1}] and truncated

def test_unsynced_images(tmp_path):
    for side in ("l","r"):cv2.imwrite(str(tmp_path/f"{side}.png"),np.zeros((20,30,3),np.uint8))
    p=tmp_path/"manifest.json";p.write_text(json.dumps({"frames":[{"left":"l.png","right":"r.png","timestamp_ns":1}]}))
    s=RecordedStereoSource(p);s.open();s.start()
    f=s.read();assert not f.synchronized and not s.get_metadata().metric_capable
    s.close()

def test_vendor():
    with pytest.raises(NotImplementedError,match="真实相机适配器尚未配置"):VendorStereoSource().open()

def test_async_stop(source,tmp_path):
    w=AsyncRecorder(tmp_path/"async",source.get_metadata(),source.get_calibration())
    assert w.submit("raw",source.read(),True)
    w.close()
    assert not w.thread.is_alive()
    assert json.loads((tmp_path/"async/manifest.json").read_text())["raw_count"]==1
