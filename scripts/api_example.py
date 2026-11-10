"""Minimal public API example. Explicit synthetic provenance, no Qt dependency."""
from egohand import EgoHandEngine
from egohand.sources.synthetic import SyntheticStereoSource

source=SyntheticStereoSource("two_static",frames=1)
engine=EgoHandEngine()
try:
    engine.start(source)
    result=engine.process_next()
    print(result.reconstruction_status,result.coordinate_frame,result.unit)
    for hand in result.hands:
        wrist=hand.joints[0]
        print(hand.track_id,hand.handedness,"wrist",wrist.xyz_m,"valid",wrist.valid)
finally:engine.close()
