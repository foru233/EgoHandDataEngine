import copy
from dataclasses import replace
import numpy as np
from egohand.core.pipeline import EgoHandEngine


def test_duplicate_coverage_evidence_must_not_select_wrong_candidate(engine,source):
    f=source.read();left=copy.deepcopy(f.left_detections[0])
    correct=copy.deepcopy(f.right_detections[0]);correct.valid[[1,2,3,4,8]]=False
    wrong=copy.deepcopy(f.right_detections[0]);wrong.pixel_landmarks[:,0]+=2
    wrong.handedness='Right';wrong.handedness_score=1.
    matches,ambiguous=engine.associator.associate([left],[correct,wrong])
    assert not any(m.right_index==1 for m in matches)
    assert ambiguous or any(m.right_index==0 for m in matches)


def test_old_id_not_restored_after_long_gap_without_motion_history(engine,source):
    result=engine.process(source.read());old=result.hands[0].track_id
    h=copy.deepcopy(result.hands[0]);engine.tracks.assign([h],result.timestamp_ns+250_000_000)
    assert h.track_id!=old
    assert h.tracking_status=='new_track'


def test_one_view_missing_explicit_unmatched(engine,source):
    f=source.read();r=engine.process(replace(f,right_detections=[]))
    assert all(h.valid_joint_count==0 and h.tracking_status=='unmatched' for h in r.hands)


def test_swap_order_is_not_swap_identity(engine,source):
    f=source.read();matches,ambiguous=engine.associator.associate(f.left_detections,f.right_detections[::-1])
    assert not ambiguous and {(m.left_index,m.right_index) for m in matches}=={(0,1),(1,0)}


def test_nearly_overlapping_hands_rejected(engine,source):
    f=source.read();a=copy.deepcopy(f.left_detections[0]);b=copy.deepcopy(f.right_detections[0])
    matches,ambiguous=engine.associator.associate([a,copy.deepcopy(a)],[b,copy.deepcopy(b)])
    assert ambiguous and not matches


def test_short_occlusion_with_independent_history_can_keep_track(engine,source):
    a=engine.process(source.read());b=engine.process(source.read())
    h=copy.deepcopy(b.hands[0]);engine.tracks.assign([],b.timestamp_ns+33_333_333)
    engine.tracks.assign([h],b.timestamp_ns+66_666_666)
    assert h.track_id==b.hands[0].track_id and h.tracking_status=='tracked'


def test_expired_track_creates_new_id(engine,source):
    r=engine.process(source.read());h=copy.deepcopy(r.hands[0]);old=h.track_id
    engine.tracks.assign([h],r.timestamp_ns+1_000_000_000)
    assert h.track_id!=old and h.tracking_status=='new_track'


def test_association_diagnostics_are_costs_not_probabilities(engine,source):
    f=source.read();engine.associator.associate(f.left_detections,f.right_detections)
    d=engine.associator.diagnostics
    assert d['algorithm_version']=='hungarian-independent-evidence-v2'
    assert d['legacy_disparity_weight_ignored']
    assert d['calibrated_probability'] is False
    assert len(d['candidates'])==4
