import numpy as np
from egohand.sources.synthetic import SyntheticStereoSource

def test_exact(engine,source):
    f=source.read();r=engine.process(f)
    assert len(r.hands)==2
    for i,h in enumerate(r.hands):
        assert h.valid_joint_count==21
        np.testing.assert_allclose([j.xyz_m for j in h.joints],f.metadata["ground_truth_xyz_m"][i],atol=1e-5)
    assert r.coordinate_frame=="camera_left_optical" and r.unit=="m"

def test_noise_propagation(engine):
    s=SyntheticStereoSource("single_static",frames=10,noise_px=.25);s.open();s.start()
    errors=[]
    for _ in range(10):
        f=s.read();r=engine.process(f)
        for j in r.hands[0].joints:
            if j.valid: errors.append(np.linalg.norm(j.xyz_m-f.metadata["ground_truth_xyz_m"][0,j.joint_id]))
    # sigma_Z ~= Z^2/(f*b)*sqrt(2)*sigma_px, not a fixed mm promise.
    sigma=.65**2/(600*.064)*np.sqrt(2)*.25
    assert np.sqrt(np.mean(np.square(errors)))<3*sigma

def test_zero_disparity(engine):
    s=SyntheticStereoSource("zero_disparity",frames=1);s.open();s.start()
    # 启用极线约束以避免两只手发生跨手错误配对；零视差仍由几何门限拒绝。
    engine.config['use_epipolar_error']=True
    r=engine.process(s.read())
    assert sum(h.valid_joint_count for h in r.hands)==0

def test_negative_depth(engine,source):
    r=engine.rectifier;f=r.rectify(source.read())
    a,b=f.left_detections[0],f.right_detections[0]
    from egohand.stereo.triangulation import JointTriangulator
    js=JointTriangulator(r,engine.config).triangulate(b,a)
    assert all(j.status=="NEGATIVE_DEPTH" for j in js)

def test_far_points_are_not_rejected_by_distance_and_nonfinite_are_rejected(engine,source):
    from dataclasses import replace
    from egohand.stereo.triangulation import JointTriangulator
    r=engine.rectifier;f=r.rectify(source.read());a,b=f.left_detections[0],f.right_detections[0]
    uv=a.pixel_landmarks.copy();uv[:,0]-=.5
    js=JointTriangulator(r,engine.config).triangulate(a,replace(b,pixel_landmarks=uv))
    # 默认不设置距离上下限；满足视差和几何约束的远点仍可输出。
    assert any(j.valid for j in js)
    uv[:]=np.nan
    js=JointTriangulator(r,engine.config).triangulate(a,replace(b,pixel_landmarks=uv))
    assert not any(j.valid for j in js)
