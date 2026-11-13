"""Measured candidate selection must retain geometric gates and original ties."""
import copy
from dataclasses import replace
import numpy as np


class Observations:
    def __init__(self,original,current):
        self.original=original;self.current=current
        self.last_detection_diagnostics={'full_frame_observations':original}
    def detect(self,image,timestamp_ns):return self.current


def process(engine,source,mutate):
    frame=source.read()
    original=copy.deepcopy([frame.left_detections,frame.right_detections])
    current=copy.deepcopy(original)
    mutate(original,current)
    engine.config.update(stereo_refinement_selection=True,use_epipolar_error=True)
    engine.detectors=[Observations(a,b) for a,b in zip(original,current)]
    result=engine.process(replace(frame,left_detections=None,right_detections=None))
    return result,original,current


def test_roi_with_bad_epipolar_geometry_cannot_replace_valid_original(engine,source):
    def corrupt(original,current):
        for hand in current[1]:hand.pixel_landmarks[:,1]+=40
    result,original,current=process(engine,source,corrupt)
    assert sum(h.valid_joint_count for h in result.hands)==42
    selection=result.diagnostics['stereo_detection_selection']
    assert selection['selected_left']=='full_frame' and selection['selected_right']=='full_frame'
    for a,b in zip(result.right_detections,original[1]):
        np.testing.assert_array_equal(a.pixel_landmarks,b.pixel_landmarks)


def test_roi_can_recover_measured_missing_joints_without_relaxing_gate(engine,source):
    def missing(original,current):
        for hand in original[1]:hand.valid[:5]=False
    result,_,_=process(engine,source,missing)
    assert sum(h.valid_joint_count for h in result.hands)==42
    selection=result.diagnostics['stereo_detection_selection']
    assert selection['selected_right']=='roi' and not selection['geometry_thresholds_changed']
    assert min(a['valid_joints'] for a in selection['alternatives'])<42


def test_no_variant_passes_stereo_geometry_means_no_valid_coordinates(engine,source):
    def corrupt(original,current):
        for variants in (original,current):
            for hand in variants[1]:hand.pixel_landmarks[:,1]+=40
    result,_,_=process(engine,source,corrupt)
    assert not any(j.valid for h in result.hands for j in h.joints)
