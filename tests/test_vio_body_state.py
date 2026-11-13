import ctypes
import numpy as np
import pytest
from egohand.vio.native_bridge import NativeVioBridge


def test_body_state_preserves_pose_velocity_and_bias():
    class FakeDll:
        def egohand_vio_get_body_state(self,handle,T,v,b,ba2):
            for i,x in enumerate(np.eye(4).reshape(-1)):T[i]=x
            for i,x in enumerate((1,2,3)):v[i]=x
            for i,x in enumerate((.1,.2,.3,.4,.5,.6)):b[i]=x
            ctypes.cast(ba2,ctypes.POINTER(ctypes.c_int))[0]=1
            return 0
    bridge=NativeVioBridge.__new__(NativeVioBridge)
    bridge._dll=FakeDll();bridge._handle=ctypes.c_void_p(1);bridge._closed=False
    state=bridge.get_candidate_body_state()
    np.testing.assert_allclose(state['T_world_from_body'],np.eye(4))
    np.testing.assert_allclose(state['velocity_world_mps'],[1,2,3])
    np.testing.assert_allclose(state['accel_bias_mps2'],[.1,.2,.3],rtol=1e-6)
    np.testing.assert_allclose(state['gyro_bias_rps'],[.4,.5,.6],rtol=1e-6)
    assert state['ba2_complete']


def test_body_state_unavailable_is_not_an_anchor():
    class FakeDll:
        def egohand_vio_get_body_state(self,*args):return 1
    bridge=NativeVioBridge.__new__(NativeVioBridge)
    bridge._dll=FakeDll();bridge._handle=ctypes.c_void_p(1);bridge._closed=False
    assert bridge.get_candidate_body_state() is None
    bridge._closed=True
    with pytest.raises(RuntimeError):bridge.get_candidate_body_state()


def test_recovery_callback_exceptions_are_unavailable_not_consistent():
    class FakeDll:
        def __init__(self):self.callback=None
        def egohand_vio_set_recovery_callback(self,handle,callback,context):self.callback=callback;return 0
    bridge=NativeVioBridge.__new__(NativeVioBridge)
    bridge._dll=FakeDll();bridge._handle=ctypes.c_void_p(1);bridge._closed=False
    bridge._recovery_callback_ref=None;bridge._recovery_callback_error=None
    def fail(*args):raise ValueError('test error')
    bridge.set_recovery_callback(fail)
    matrix=(ctypes.c_float*16)(*np.eye(4).reshape(-1))
    assert bridge._dll.callback(matrix,1,None)==-1 and 'test error' in bridge._recovery_callback_error
    bridge.set_recovery_callback(None)
    assert bridge._recovery_callback_ref is None

def test_abi_mismatch_is_rejected_before_use():
    class FakeDll:
        def egohand_vio_result_size(self):return 64
    bridge=NativeVioBridge.__new__(NativeVioBridge)
    bridge._dll=FakeDll()
    with pytest.raises(RuntimeError,match='ABI'):bridge._check_abi()

def test_abi_without_size_export_is_rejected():
    """无法确认结构体布局时必须拒绝，避免错误解析位姿。"""
    class FakeDll:pass
    bridge=NativeVioBridge.__new__(NativeVioBridge)
    bridge._dll=FakeDll()
    with pytest.raises(RuntimeError,match='ABI'):bridge._check_abi()
