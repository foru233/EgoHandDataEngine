from types import SimpleNamespace
from egohand.vio.hand_identity_stabilizer import HandIdentityStabilizer


def hand(label, track=1):
    return SimpleNamespace(handedness=label,track_id=track,joints=[])


def test_window_resists_isolated_wrong_label_and_corrects_wrong_initial_label():
    s=HandIdentityStabilizer()
    labels=[]
    for i,label in enumerate(['Left','Right','Right','Right','Right','Right','Left','Right']):
        stable,_=s.update([hand(label)],i*33_000_000)
        labels.append(stable[0].handedness)
    assert labels[:5]==['Left']*5
    assert labels[5:]==['Right']*3


def test_window_resets_after_gap_and_does_not_invent_unknown_side():
    s=HandIdentityStabilizer()
    assert s.update([hand('Right')],1)[0][0].handedness=='Right'
    assert s.update([hand('Left')],1_000_000_000)[0][0].handedness=='Left'
    s.reset()
    assert s.update([hand('Unknown')],1)[0][0].handedness=='Unknown'


def test_two_tracks_remain_distinct():
    s=HandIdentityStabilizer()
    values,_=s.update([hand('Left',1),hand('Right',2)],1)
    assert [h.handedness for h in values]==['Left','Right']


def test_continuous_track_does_not_switch_despite_wrong_label_votes():
    s=HandIdentityStabilizer()
    def observed(label,x):
        return SimpleNamespace(handedness=label,track_id=3,
            joints=[SimpleNamespace(valid=True,xyz_m=[x,0.,.5])])
    labels=[]
    for i,label in enumerate(['Right']+['Left']*9):
        labels.append(s.update([observed(label,i*.003)],i*33_000_000)[0][0].handedness)
    assert labels==['Right']*10
