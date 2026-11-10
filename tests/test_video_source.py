import csv
import json
import cv2
import numpy as np
from egohand.sources.recorded import RecordedStereoSource

def videos(tmp_path):
    for side,offset in (("left",0),("right",50)):
        w=cv2.VideoWriter(str(tmp_path/f"{side}.avi"),cv2.VideoWriter_fourcc(*"MJPG"),30,(64,48))
        assert w.isOpened()
        for i in range(5):w.write(np.full((48,64,3),offset+i*10,np.uint8))
        w.release()

def test_video_fps_is_not_sync(tmp_path):
    videos(tmp_path)
    manifest={"left_video":"left.avi","right_video":"right.avi","metadata":{"real_stereo":True,"synchronized":True,"synchronization_evidence":"untrusted fps equality"}}
    p=tmp_path/"manifest.json";p.write_text(json.dumps(manifest))
    s=RecordedStereoSource(p);s.open();s.start()
    f=s.read();assert not f.synchronized and s.metadata.timestamp_kind=="estimated_from_fps"
    assert f.left_timestamp_ns is None
    s.seek(3);assert s.read().frame_id==3
    s.close();assert not s.captures

def test_sensor_csv_and_offset(tmp_path):
    videos(tmp_path)
    with (tmp_path/"ts.csv").open("w",newline="") as f:
        w=csv.writer(f);w.writerow(["timestamp_ns","left_timestamp_ns","right_timestamp_ns"])
        for i in range(5):w.writerow([1_000_000_000+i*33_333_333,1_000_000_000+i*33_333_333,1_002_000_000+i*33_333_333])
    p=tmp_path/"manifest.json"
    p.write_text(json.dumps({"left_video":"left.avi","right_video":"right.avi","timestamps_csv":"ts.csv","right_timestamp_offset_ns":-2_000_000,"metadata":{"real_stereo":True,"synchronized":True,"synchronization_evidence":"USER_ASSERTED test clock offset","timestamp_kind":"sensor"}}))
    s=RecordedStereoSource(p,loop=True);s.open();s.start()
    frames=[s.read() for _ in range(6)]
    assert frames[0].left_timestamp_ns==frames[0].right_timestamp_ns
    assert frames[0].metadata["original_right_timestamp_ns"]==1_002_000_000
    assert frames[5].frame_id==0
    s.close()
