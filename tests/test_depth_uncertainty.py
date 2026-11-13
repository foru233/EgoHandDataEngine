from dataclasses import replace
import numpy as np
import pytest
from egohand.stereo.triangulation import JointTriangulator
from egohand.stereo.uncertainty import estimate_uncertainty,rectified_depth_std
from egohand.calibration.rectifier import StereoRectifier
from egohand.calibration.schema import synthetic_calibration
import cv2

def observe(c,xyz):
    r=StereoRectifier(c);xyz=np.asarray(xyz,dtype=float).reshape(-1,3)
    points=[]
    for side in ('left','right'):
        R,t=(np.eye(3),np.zeros(3)) if side=='left' else (c.R_right_left,c.t_right_left)
        K,D=(c.K_left,c.D_left) if side=='left' else (c.K_right,c.D_right)
        fn=cv2.fisheye.projectPoints if c.distortion_model=='fisheye' else cv2.projectPoints
        inputs=xyz.reshape(-1,1,3) if c.distortion_model=='fisheye' else xyz
        uv=fn(inputs,cv2.Rodrigues(R)[0],t,K,D)[0].reshape(-1,2)
        points.append(r.points(uv,side))
    return r,*points

def test_farther_depth_and_noise_increase_uncertainty():
    r,l,q=observe(synthetic_calibration(),[[0,0,z] for z in (.3,.6,1.2,2.4)])
    u=estimate_uncertainty(r.P1,r.P2,l,q,r.R1)
    assert all(a.depth_std_m<b.depth_std_m for a,b in zip(u,u[1:]))
    a=estimate_uncertainty(r.P1,r.P2,l,q,r.R1,{'assumed_pixel_std':.5})
    b=estimate_uncertainty(r.P1,r.P2,l,q,r.R1,{'assumed_pixel_std':2.})
    np.testing.assert_allclose([x.depth_std_m*4 for x in a],[x.depth_std_m for x in b])
    d=l[:,0]-q[:,0]
    for item,disparity in zip(u,d):assert item.depth_std_m==pytest.approx(rectified_depth_std(600,.064,disparity,np.sqrt(2)*1.5),rel=1e-5)

@pytest.mark.parametrize('geometry',['horizontal','vertical','rotated','fisheye'])
def test_general_geometry_covariance(geometry):
    c=synthetic_calibration()
    if geometry=='vertical':c.t_right_left=np.array([0,-.064,0])
    if geometry in ('rotated','fisheye'):c.R_right_left=cv2.Rodrigues(np.array([.01,.03,-.02]))[0]
    if geometry=='fisheye':c.distortion_model='fisheye';c.D_left=np.array([.01,0,0,0]);c.D_right=np.array([-.01,0,0,0])
    r,l,q=observe(c,[[.03,.02,.8]])
    u=estimate_uncertainty(r.P1,r.P2,l,q,r.R1)[0]
    assert u.status=='ESTIMATED_ASSUMED_PIXEL_NOISE'
    assert u.position_std_m>=u.depth_std_m>0
    assert np.linalg.eigvalsh(u.position_covariance_m2).min()>=-1e-12

def test_degenerate_and_nonfinite_do_not_claim_zero(calibration):
    r,l,q=observe(calibration,[[0,0,.6]])
    for right in (l,np.array([[np.nan,0.]]),l+np.array([[-1e-5,0]])):
        u=estimate_uncertainty(r.P1,r.P2,l,right,r.R1)[0]
        assert u.depth_std_m is None and u.position_covariance_m2 is None
        assert u.status!='ESTIMATED_ASSUMED_PIXEL_NOISE'
    u=estimate_uncertainty(r.P1,r.P1,l,l,r.R1)[0]
    assert u.depth_std_m is None

def test_opt_in_rejection_and_disabled_compatibility(engine,source):
    f=engine.rectifier.rectify(source.read());a,b=f.left_detections[0],f.right_detections[0]
    tri=lambda cfg:JointTriangulator(engine.rectifier,{**engine.config,'uncertainty':cfg}).triangulate(a,b)
    base=tri({'enabled':False});diagnostic=tri({'mode':'diagnostic'});rejected=tri({'mode':'reject','max_depth_std_m':1e-6,'max_position_std_m':1e-6})
    np.testing.assert_allclose([j.xyz_m for j in base],[j.xyz_m for j in diagnostic],atol=1e-12)
    assert all(j.valid for j in base+diagnostic)
    assert all(not j.valid and j.status=='HIGH_UNCERTAINTY' and j.depth_std_m>0 for j in rejected)
    assert all(j.depth_std_m is None and j.uncertainty_status=='DISABLED' for j in base)

def test_systematic_bias_is_not_detected_by_fixed_random_noise(engine,source):
    f=engine.rectifier.rectify(source.read());a,b=f.left_detections[0],f.right_detections[0]
    uv=b.pixel_landmarks.copy();uv[:,0]+=5
    js=JointTriangulator(engine.rectifier,engine.config).triangulate(a,replace(b,pixel_landmarks=uv))
    error=abs(js[0].xyz_m[2]-.65)
    assert error>js[0].depth_std_m
    assert js[0].uncertainty_status=='ESTIMATED_ASSUMED_PIXEL_NOISE'
    # This is an intentional remaining systematic_bias failure mode, not a pass
    # for bias detection: covariance is conditional on the assumed noise model.

def test_systematic_bias_can_hide_behind_zero_reprojection(engine,source):
    f=engine.rectifier.rectify(source.read())
    a,b=f.left_detections[0],f.right_detections[0]
    shifted=b.pixel_landmarks.copy();shifted[:,0]+=5.
    # 此用例验证启用重投影质量分后，系统偏差仍可能被低残差掩盖。
    joints=JointTriangulator(engine.rectifier,{**engine.config,'use_reprojection_error':True}).triangulate(a,replace(b,pixel_landmarks=shifted))
    errors=[np.linalg.norm(j.xyz_m-f.metadata['ground_truth_xyz_m'][0,j.joint_id]) for j in joints if j.valid]
    assert np.mean(errors)>.05
    assert max(j.reprojection_error_px for j in joints if j.valid)<1e-8
    assert min(j.quality_score for j in joints if j.valid)>.999

def test_numerical_covariance_available_without_changing_geometry(engine,source):
    f=engine.rectifier.rectify(source.read())
    joints=JointTriangulator(engine.rectifier,engine.config).triangulate(f.left_detections[0],f.right_detections[0])
    assert all(j.valid for j in joints)
    assert all(j.depth_std_m is not None and j.depth_std_m>0 for j in joints)
    assert all(j.uncertainty_status=='ESTIMATED_ASSUMED_PIXEL_NOISE' for j in joints)
    np.testing.assert_allclose(joints[0].position_covariance_m2,np.asarray(joints[0].position_covariance_m2).T,atol=1e-12)
