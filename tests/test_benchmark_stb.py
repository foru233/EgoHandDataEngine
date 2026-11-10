from pathlib import Path
import numpy as np
import cv2
import pytest
from scipy.io import savemat
from egohand.benchmarks.stb import STBBenchmarkAdapter,stb_joint_mapping
from egohand.benchmarks.types import BenchmarkBlocked

def test_explicit_semantic_mapping():
    m=stb_joint_mapping();assert len(m)==21
    assert sum(x.comparable for x in m)==17
    assert not m[0].comparable and m[0].mediapipe_index is None
    assert all(not m[i].comparable for i in (17,18,19))
    assert m[20].mediapipe_index==4 and m[1].mediapipe_index==17 and m[13].mediapipe_index==5

def test_stb_format_fixture_not_real_dataset(tmp_path):
    d=tmp_path/'images/B1Counting';d.mkdir(parents=True);(tmp_path/'labels').mkdir()
    xyz=np.full((3,21,2),100.,dtype=float);xyz[2]=600.
    savemat(tmp_path/'labels/B1Counting_BB.mat',{'handPara':xyz})
    for i in range(2):
        for side in ('left','right'):cv2.imwrite(str(d/f'BB_{side}_{i}.png'),np.zeros((480,640,3),np.uint8))
    adapter=STBBenchmarkAdapter(tmp_path,max_frames=1);s=next(adapter.iter_samples())
    assert s.calibration is None and s.provenance['calibration_status']=='CALIBRATION_MAPPING_UNVERIFIED'
    assert s.left_timestamp_ns is None and s.right_timestamp_ns is None
    assert s.provenance['sync_quality']=='DATASET_DOCUMENTED'
    np.testing.assert_allclose(s.ground_truth_xyz_m[:,2],.6)
    assert s.mapped_ground_truth()[1].sum()==17

def test_missing_data_is_blocked(tmp_path):
    with pytest.raises(BenchmarkBlocked):list(STBBenchmarkAdapter(tmp_path).iter_samples())

def test_actual_official_stb_assets():
    root=Path('samples/date/stb')
    if not list((root/'images').glob('*/BB_left_*')):pytest.skip('BLOCKED_BY_DATA: official STB BB image pairs unavailable')
    s=next(STBBenchmarkAdapter(root,max_frames=1).iter_samples())
    assert s.left_image.shape==s.right_image.shape
