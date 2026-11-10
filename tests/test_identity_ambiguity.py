import copy
import numpy as np

def test_identity_ambiguity_does_not_create_new_tracks(engine,source):
    result=engine.process(source.read())
    tracker=engine.tracks
    next_id=tracker.next_id
    center=np.mean([t["center"] for t in tracker.tracks.values()],axis=0)
    hand=copy.deepcopy(result.hands[0])
    delta=center-np.mean([j.xyz_m for j in hand.joints],axis=0)
    for j in hand.joints:j.xyz_m+=delta
    hands=[copy.deepcopy(hand),copy.deepcopy(hand)]
    tracker.assign(hands,result.timestamp_ns+100_000_000)
    assert all(h.tracking_status=="identity_ambiguous" for h in hands)
    assert all(h.valid_joint_count==0 and h.track_id==-1 for h in hands)
    assert tracker.next_id==next_id
