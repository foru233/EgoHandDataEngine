"""Display-only temporal left/right identity stabilization."""
from dataclasses import dataclass
from collections import deque, Counter
import numpy as np


@dataclass(frozen=True)
class StableHand:
    source: object
    handedness: str

    def __getattr__(self, name):
        return getattr(self.source, name)


class HandIdentityStabilizer:
    def __init__(self, max_wrist_jump_m=.18, window_size=7, switch_votes=5):
        self.max_wrist_jump_m = float(max_wrist_jump_m)
        if not 1 <= switch_votes <= window_size:
            raise ValueError('invalid handedness vote thresholds')
        self.window_size=window_size;self.switch_votes=switch_votes
        self.by_track = {}
        self.last_wrist = {}
        self.votes = {}

    def reset(self):
        self.by_track.clear(); self.last_wrist.clear(); self.votes.clear()
        self._timestamp_ns=None

    def update(self, hands, timestamp_ns=None):
        previous=getattr(self,'_timestamp_ns',None)
        if timestamp_ns is not None:
            if previous is not None and (timestamp_ns <= previous or timestamp_ns-previous > 350_000_000):
                self.reset()
            self._timestamp_ns=timestamp_ns
        stable=[]; used=set(); current={}
        for hand in hands or []:
            track=int(getattr(hand,'track_id',-1) or -1)
            label=self.by_track.get(track) if track>0 else None
            raw=getattr(hand,'handedness','Unknown')
            wrist=None
            joints=getattr(hand,'joints',())
            if joints and getattr(joints[0],'valid',False):
                value=np.asarray(getattr(joints[0],'xyz_m',[np.nan]*3),dtype=float)
                if value.shape==(3,) and np.isfinite(value).all():wrist=value
            prior=self.last_wrist.get(label)
            continuous=(wrist is not None and prior is not None and
                        np.linalg.norm(wrist-prior)<=self.max_wrist_jump_m)
            if track>0 and raw in ('Left','Right'):
                votes=self.votes.setdefault(track,deque(maxlen=self.window_size))
                votes.append(raw)
                counts=Counter(votes)
                if label in ('Left','Right'):
                    opposite='Right' if label=='Left' else 'Left'
                    if not continuous and counts[opposite]>=self.switch_votes and opposite not in used:
                        label=opposite
            if label is None and wrist is not None:
                candidates=[(float(np.linalg.norm(wrist-v)),k) for k,v in self.last_wrist.items() if k not in used]
                if candidates and min(candidates)[0] <= self.max_wrist_jump_m:label=min(candidates)[1]
            if label not in ('Left','Right') or label in used:
                label=raw if raw in ('Left','Right') and raw not in used else 'Unknown'
            if track>0 and label in ('Left','Right'):self.by_track[track]=label
            used.add(label); current[label]=wrist
            stable.append(StableHand(hand,label))
        self.last_wrist={k:v for k,v in current.items() if v is not None}
        return stable, {'stable_labels': [h.handedness for h in stable]}
