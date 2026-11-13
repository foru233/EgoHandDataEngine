from dataclasses import replace
import cv2
import numpy as np
from egohand.core.pipeline import EgoHandEngine
from egohand.stereo.triangulation import JointTriangulator
from egohand.calibration.schema import synthetic_calibration
from egohand.sources.synthetic import SyntheticStereoSource

def test_reprojection_rejection(engine,source):
    r=engine.rectifier;f=r.rectify(source.read())
    a,b=f.left_detections[0],f.right_detections[0]
    uv=b.pixel_landmarks.copy();uv[:,1]+=10
    cfg={**engine.config,"max_epipolar_px":20.,"max_reprojection_px":1.,'use_reprojection_error':True}
    js=JointTriangulator(r,cfg).triangulate(a,replace(b,pixel_landmarks=uv))
    assert all(j.status=="REPROJECTION_REJECTED" for j in js)

def test_epipolar_rejection(engine,source):
    r=engine.rectifier;f=r.rectify(source.read())
    a,b=f.left_detections[0],f.right_detections[0]
    uv=b.pixel_landmarks.copy();uv[:,1]+=15
    js=JointTriangulator(r,{**engine.config,'use_epipolar_error':True}).triangulate(a,replace(b,pixel_landmarks=uv))
    assert all(j.status=="EPIPOLAR_REJECTED" for j in js)

def test_unbounded_depth_keeps_positive_depth_and_optional_limits(engine,source):
    r=engine.rectifier;f=r.rectify(source.read());a,b=f.left_detections[0],f.right_detections[0]
    for z,left_x in ((.1,500.),(5.,320.)):
        l=np.tile([left_x,240.],(21,1));q=l.copy();q[:,0]-=600*.064/z
        left=replace(a,pixel_landmarks=l);right=replace(b,pixel_landmarks=q)
        js=JointTriangulator(r,{**engine.config,'min_depth_m':None,'max_depth_m':None}).triangulate(left,right)
        assert all(j.valid for j in js)
        np.testing.assert_allclose([j.xyz_m[2] for j in js],z,atol=1e-6)
        bounded=JointTriangulator(r,{**engine.config,'min_depth_m':.15,'max_depth_m':2.}).triangulate(left,right)
        assert all(j.status=='DEPTH_RANGE_REJECTED' for j in bounded)

def test_epipolar_switch_keeps_reprojection_gate(engine,source):
    r=engine.rectifier;f=r.rectify(source.read())
    a,b=f.left_detections[0],f.right_detections[0]
    uv=b.pixel_landmarks.copy();uv[:,1]+=4
    shifted=replace(b,pixel_landmarks=uv)
    on=JointTriangulator(r,{**engine.config,'use_epipolar_error':True}).triangulate(a,shifted)
    off=JointTriangulator(r,{**engine.config,'use_epipolar_error':False}).triangulate(a,shifted)
    assert all(j.status=='EPIPOLAR_REJECTED' for j in on)
    assert all(j.valid and j.epipolar_error_px>3 for j in off)
    uv=b.pixel_landmarks.copy();uv[:,1]+=15
    off=JointTriangulator(r,{**engine.config,'use_epipolar_error':False,'use_reprojection_error':True}).triangulate(a,replace(b,pixel_landmarks=uv))
    assert all(j.status=='REPROJECTION_REJECTED' for j in off)

def test_reprojection_switch_independent(engine,source):
    r=engine.rectifier;f=r.rectify(source.read())
    a,b=f.left_detections[0],f.right_detections[0]
    uv=b.pixel_landmarks.copy();uv[:,1]+=10
    cfg={**engine.config,'use_epipolar_error':False,'use_reprojection_error':False}
    joints=JointTriangulator(r,cfg).triangulate(a,replace(b,pixel_landmarks=uv))
    assert all(j.valid and j.reprojection_error_px>3 and j.quality_score is None for j in joints)
    cfg['use_epipolar_error']=True
    joints=JointTriangulator(r,cfg).triangulate(a,replace(b,pixel_landmarks=uv))
    assert all(j.status=='EPIPOLAR_REJECTED' for j in joints)

def test_temporal_joint_outlier():
    s=SyntheticStereoSource("single_static",frames=2);s.open();s.start()
    e=EgoHandEngine({"max_speed_m_s":.4,"temporal_slack_m":.005},s.calibration)
    first=e.process(s.read())
    f=s.read()
    # Move one fingertip by 4 cm in the same calibrated projection.
    for d in (f.left_detections[0],f.right_detections[0]):
        d.pixel_landmarks[8,0]+=.04*600/.65
    second=e.process(f)
    assert second.hands[0].track_id==first.hands[0].track_id
    assert second.hands[0].joints[8].status=="TEMPORAL_OUTLIER"
    assert not second.hands[0].joints[8].valid

def test_vertical_stereo():
    c=synthetic_calibration();c.t_right_left=np.array([0,-.064,0])
    s=SyntheticStereoSource("single_static",frames=1);s.calibration=c;s.open();s.start()
    e=EgoHandEngine(calibration=c);f=s.read();r=e.process(f)
    assert e.rectifier.disparity_axis==1
    assert r.hands[0].valid_joint_count==21
    np.testing.assert_allclose([j.xyz_m for j in r.hands[0].joints],f.metadata["ground_truth_xyz_m"][0],atol=1e-5)

def test_unequal_intrinsics_and_fisheye():
    c=synthetic_calibration();c.distortion_model="fisheye"
    c.D_left=np.array([.01,-.002,0,0]);c.D_right=np.array([-.01,.001,0,0])
    c.K_right=c.K_right.copy();c.K_right[0,0]=570;c.K_right[1,1]=585
    c.R_right_left=cv2.Rodrigues(np.array([.01,.02,.005]))[0]
    s=SyntheticStereoSource("single_static",frames=1);s.calibration=c;s.open();s.start()
    e=EgoHandEngine(calibration=c);f=s.read();r=e.process(f)
    assert r.hands[0].valid_joint_count==21
    np.testing.assert_allclose([j.xyz_m for j in r.hands[0].joints],f.metadata["ground_truth_xyz_m"][0],atol=1e-5)

def test_track_expiration(engine,source):
    a=engine.process(source.read())
    f=source.read();f.timestamp_ns+=2_000_000_000
    f.left_timestamp_ns=f.right_timestamp_ns=f.timestamp_ns
    b=engine.process(f)
    assert set(h.track_id for h in a.hands).isdisjoint(h.track_id for h in b.hands)
