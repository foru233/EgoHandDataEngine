from dataclasses import replace
import numpy as np
from egohand.sources.synthetic import SyntheticStereoSource

def run(engine,scenario,frames=1):
    s=SyntheticStereoSource(scenario,frames=frames);s.open();s.start()
    return [engine.process(s.read()) for _ in range(frames)]

def test_order_and_tracks(engine):
    results=run(engine,"two_static",8)
    assert all([h.track_id for h in r.hands]==[1,2] for r in results)
    assert all(sum(h.valid_joint_count for h in r.hands)==42 for r in results)

def test_crossing(engine):
    results=run(engine,"crossing",100)
    assert all([h.track_id for h in r.hands]==[1,2] for r in results)

def test_one_missing(engine):
    r=run(engine,"inconsistent_count")[0]
    assert sorted(h.valid_joint_count for h in r.hands)==[0,21]

def test_all_unmatched(engine):
    r=run(engine,"wrong_correspondence")[0]
    assert not any(h.valid_joint_count for h in r.hands)

def test_ambiguous(engine,source):
    f=source.read()
    a=f.left_detections[0];b=f.right_detections[0]
    f=replace(f,left_detections=[a,a],right_detections=[b,b])
    r=engine.process(f)
    assert r.diagnostics["ambiguous"]
    assert not any(h.valid_joint_count for h in r.hands)

def test_wrong_handedness(engine,source):
    f=source.read()
    f.right_detections[0].handedness="Right"
    f.right_detections[1].handedness="Left"
    r=engine.process(f)
    assert sum(h.valid_joint_count for h in r.hands)==42
