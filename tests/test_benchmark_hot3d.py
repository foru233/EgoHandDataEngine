import copy
import json
from pathlib import Path
import tarfile
import numpy as np
import pytest
from egohand.benchmarks.hot3d import HOT3DBenchmarkAdapter,HOT3DStereoGeometry,HOT3D_MAPPING,CANONICAL_MP,transform
from egohand.benchmarks.types import BenchmarkBlocked
from egohand.core.types import HandDetection
from egohand.stereo.triangulation import JointTriangulator
from egohand.core.pipeline import default_config

ROOT=Path(__file__).resolve().parents[1]/'samples/date/hot3d'


@pytest.fixture
def real_sample():
    if not (ROOT/'prepared/clip-001849/umetrack_world.json').exists():pytest.skip('Actual official HOT3D training archive/FK cache unavailable')
    return next(HOT3DBenchmarkAdapter(ROOT,max_frames=1,clips=['clip-001849']).iter_samples())


def test_mapping_is_canonical20_not_mediapipe21():
    assert len(HOT3D_MAPPING)==20 and len(set(CANONICAL_MP))==20
    assert 1 not in CANONICAL_MP
    assert CANONICAL_MP[:6]==(4,8,12,16,20,0)
    assert all(m.comparable and m.mapping_reason for m in HOT3D_MAPPING)


def test_real_stereo_grayscale_and_sensor_timestamps(real_sample):
    s=real_sample
    assert s.provenance['selected_streams']==['1201-1','1201-2']
    assert s.provenance['official_hash_verified']
    assert s.left_image.shape==s.right_image.shape==(480,640,3)
    assert np.array_equal(s.left_image[:,:,0],s.left_image[:,:,2])
    assert not np.array_equal(s.left_image,s.right_image)
    assert abs(s.left_timestamp_ns-s.right_timestamp_ns)==12
    assert not s.provenance['hardware_sync_precision_verified']


def test_real_fisheye624_world_device_and_coordinate_conversion(real_sample):
    s=real_sample;g=s.native_annotations['geometry'];cams=s.native_annotations['cameras']
    assert g.extrinsic_check_max_abs<1e-12
    assert .13<g.baseline_m<.15
    assert all(type(c).__name__=='OVR624CameraModel' for c in g.native)
    points=s.ground_truth_xyz_m
    left=g.virtual_to_original(points)
    right=left@g.R.T+g.t
    world=left@transform(cams['1201-1']['T_world_from_camera'])[:3,:3].T+transform(cams['1201-1']['T_world_from_camera'])[:3,3]
    Twr=transform(cams['1201-2']['T_world_from_camera'])
    independent=(world-Twr[:3,3])@Twr[:3,:3]
    assert np.max(np.abs(right-independent))<1e-12
    assert np.max(np.abs(left@g.R_virtual_left.T-points))<1e-12


def test_detection_pinhole_points_reproject_to_same_rectified_pixels(real_sample):
    g=real_sample.native_annotations['geometry']; xyz=np.vstack((real_sample.ground_truth_xyz_m,real_sample.ground_truth_xyz_m[:1]))
    original=g.virtual_to_original(xyz)
    upright=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
    for side in (0,1):
        camera=original if side==0 else original@g.R.T+g.t
        rays=camera@upright.T
        hp=rays@g.K.T; uv=hp[:,:2]/hp[:,2,None]
        d=HandDetection('Left',1.,uv/[640,480],uv,valid=np.ones(len(uv),bool))
        mapped=g.rectifier.detection_to_rectified([d],('left','right')[side])[0]
        expected,_=g.project_virtual(xyz,side)
        np.testing.assert_allclose(mapped.pixel_landmarks,expected,atol=1e-8)


def test_rectified_display_roi_maps_to_containing_detection_region(real_sample):
    rectifier=real_sample.native_annotations['geometry'].rectifier
    roi=(.3,.45,.3,.3); w,h=640,480
    for i,side in enumerate(('left','right')):
        mapped=rectifier.rectified_roi_to_detection(roi,side)
        x,y,rw,rh=roi
        corners=np.array([[x*w,y*h],[(x+rw)*w,y*h],[(x+rw)*w,(y+rh)*h],[x*w,(y+rh)*h]])
        hom=np.column_stack((corners,np.ones(4)))@np.linalg.inv(rectifier.undistorted_to_rectified[i]).T
        xy=hom[:,:2]/hom[:,2,None]
        inside=(xy[:,0]>=0)&(xy[:,0]<=w)&(xy[:,1]>=0)&(xy[:,1]<=h)
        lo=np.array(mapped[:2])*[w,h];hi=(np.array(mapped[:2])+mapped[2:])*[w,h]
        assert np.all(xy[inside]>=lo-1e-8) and np.all(xy[inside]<=hi+1e-8)


def test_real_oracle_only_uses_projected_pixels(real_sample):
    s=real_sample;g=s.native_annotations['geometry']
    uv=[np.full((21,2),np.nan),np.full((21,2),np.nan)]
    masks=[]
    for side in (0,1):
        p,valid=g.project_virtual(s.ground_truth_xyz_m,side);uv[side][list(CANONICAL_MP)]=p
        mask=np.zeros(21,bool);mask[list(CANONICAL_MP)]=valid;masks.append(mask)
    ds=[HandDetection('Left',1.,u/[640,480],u,valid=m,origin='oracle_2d') for u,m in zip(uv,masks)]
    # Triangulator receives two pixel observations, never 3D ground truth.
    joints=JointTriangulator(g.rectifier,default_config()).triangulate(*ds)
    pairs=[(j,s.ground_truth_xyz_m[CANONICAL_MP.index(j.joint_id)]) for j in joints if j.valid]
    assert len(pairs)>=3
    assert max(np.linalg.norm(j.xyz_m-truth) for j,truth in pairs)<1e-10
    assert not joints[1].valid


def test_wrong_extrinsic_direction_rejected(real_sample):
    cams=copy.deepcopy(real_sample.native_annotations['cameras'])
    cams['1201-2']['calibration']['T_device_from_camera']['translation_xyz'][0]+=.01
    with pytest.raises(BenchmarkBlocked,match='extrinsics disagree'):
        HOT3DStereoGeometry(cams,ROOT/'official-hand-toolkit')


def test_test_clip_has_no_fabricated_ground_truth():
    path=ROOT/'train_aria/clip-003365.tar'
    if not path.exists():pytest.skip('Actual official HOT3D test archive unavailable')
    s=next(HOT3DBenchmarkAdapter(ROOT,max_frames=1,clips=['clip-003365']).iter_samples())
    assert s.provenance['official_path'].startswith('test_')
    assert s.ground_truth_xyz_m is None and not s.native_annotations['gt_hands']
    with tarfile.open(path) as t:assert not any(n.endswith('.hands.json') for n in t.getnames())


def test_hand_transform_matches_official_quaternion_convention():
    import importlib.util
    p=Path(__file__).resolve().parents[1]/'scripts/convert_hot3d_umetrack.py'
    spec=importlib.util.spec_from_file_location('converter',p);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    j={'quaternion_wxyz':[.5,.5,.5,.5],'translation_xyz':[.1,-.2,.3]}
    assert np.allclose(module.rigid_transform(j),transform(j),atol=1e-15)


def test_upright_rotation_restores_native_pixel_centers():
    from egohand.benchmarks.hot3d import restore_clockwise_pixels
    original=np.array([[0,0],[639,0],[639,479],[0,479],[320.25,100.75]])
    clockwise=np.column_stack((479-original[:,1],original[:,0]))
    assert np.array_equal(restore_clockwise_pixels(clockwise,480),original)


def test_official_fisheye624_inverse_roundtrip(real_sample):
    g=real_sample.native_annotations['geometry'];xyz=real_sample.ground_truth_xyz_m
    original=g.virtual_to_original(xyz)
    for side in (0,1):
        native=original if side==0 else original@g.R.T+g.t
        pixels=g.native[side].eye_to_window(native)
        virtual,mask=g.native_pixels_to_virtual(pixels,side)
        expected,_=g.project_virtual(xyz,side)
        assert mask.any()
        assert np.max(np.linalg.norm(virtual[mask]-expected[mask],axis=1))<1e-6


def test_dynamic_aria_calibration_not_frozen_at_frame_zero():
    ids=['clip-001849/000000','clip-001849/000062']
    samples=list(HOT3DBenchmarkAdapter(ROOT,max_frames=2,clips=['clip-001849'],frame_ids=ids).iter_samples())
    assert len(samples)==2
    assert samples[0].calibration.calibration_id!=samples[1].calibration.calibration_id
    assert all(s.native_annotations['geometry'].extrinsic_check_max_abs<1e-12 for s in samples)
