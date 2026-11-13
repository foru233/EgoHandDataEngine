"""Run only in isolated egohand-vio Python with a TEST_HOOKS DLL; never a deployment DLL."""
import argparse
import ctypes
import json
import os
from pathlib import Path
import sys
import tempfile
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from egohand.vio.native_bridge import NativeVioBridge


def main():
    os.environ.pop('EGOHAND_VIO_ALLOW_TEST_HOOKS',None)
    p=argparse.ArgumentParser()
    p.add_argument('--library',type=Path,required=True)
    a=p.parse_args()
    with tempfile.TemporaryDirectory(prefix='vio_fault_test_') as directory:
        folder=Path(directory)
        config=folder/'orb.yaml'
        source=ROOT/'configs/vio/vs030_rectified_half_calibration2.yaml'
        config.write_text(source.read_text(encoding='utf8')+'\nloopClosing: 0\n',encoding='utf8')
        try:
            NativeVioBridge(ROOT/'third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt',config,
                            library_path=a.library,map_mode='build')
        except RuntimeError as exc:
            assert '故障注入' in str(exc)
        else:
            raise AssertionError('test library allowed into a normal capture process')
        os.environ['EGOHAND_VIO_ALLOW_TEST_HOOKS']='1'
        with NativeVioBridge(ROOT/'third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt',config,
                             library_path=a.library,map_mode='build') as bridge:
            inject=bridge._dll.egohand_vio_test_inject_numerical_fault
            inject.argtypes=[];inject.restype=None
            count=bridge._dll.egohand_vio_numerical_fault_count
            count.argtypes=[];count.restype=ctypes.c_uint64
            assert count()==0
            inject()
            assert count()==1
            image=np.zeros((600,800),dtype=np.uint8)
            for _ in range(2):
                try:
                    bridge.track_stereo(image,image,1_000_000_000,[])
                except RuntimeError as exc:
                    assert str(exc).startswith('VIO_NUMERICAL_FAULT:')
                else:
                    raise AssertionError('invalid worker returned a frame')
            atlas=folder/'existing.osa';atlas.write_bytes(b'original map stays intact')
            try:
                bridge.export_map(atlas,folder/'poses.txt')
            except RuntimeError as exc:
                assert str(exc).startswith('VIO_NUMERICAL_FAULT:')
            else:
                raise AssertionError('invalid worker exported a map')
            assert atlas.read_bytes()==b'original map stays intact'
            assert not (folder/'poses.txt').exists()
            assert count()==1
    print(json.dumps({'injected_fault_detected':True,'repeated_pose_rejected':True,
                      'export_rejected':True,'existing_atlas_unchanged':True,
                      'test_library_blocked_by_default':True}))


if __name__=='__main__':main()
