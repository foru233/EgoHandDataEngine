import numpy as np
import pytest
from egohand.vio.imu_loss_prediction import BodyState,integrate_loss,LossRecoveryPredictor
from egohand.vio.native_bridge import ImuMeasurement

def test_static_and_constant_velocity():
    state=BodyState(1_000_000_000,np.eye(3),np.zeros(3),np.array([1.,0.,0.]),np.zeros(3),np.zeros(3),'map')
    samples=[ImuMeasurement(1_000_000_000+i*10_000_000,(0,0,9.81),(0,0,0)) for i in range(11)]
    result=integrate_loss(state,samples,1_100_000_000)
    np.testing.assert_allclose(result.position_world_m,[.1,0,0],atol=1e-8)
    np.testing.assert_allclose(result.velocity_world_mps,[1,0,0],atol=1e-8)
    with pytest.raises(ValueError):integrate_loss(state,samples[1:],1_100_000_000)
    with pytest.raises(ValueError):integrate_loss(state,samples[::3],1_090_000_000)

def test_loss_recovery_uses_anchor_full_interval_and_map_identity():
    state=BodyState(1_000_000_000,np.eye(3),np.zeros(3),np.array([1.,0.,0.]),np.zeros(3),np.zeros(3),'map1')
    predictor=LossRecoveryPredictor();predictor.set_trusted_state(state)
    for i in range(11):predictor.push(ImuMeasurement(1_000_000_000+i*10_000_000,(0,0,9.81),(0,0,0)))
    T=np.eye(4);T[0,3]=.1
    assert predictor.evaluate(T,1_100_000_000,'map1')['status']=='consistent'
    T[0,3]=4
    assert predictor.evaluate(T,1_100_000_000,'map1')['status']=='inconsistent'
    assert predictor.evaluate(T,1_100_000_000,'map2')['reason']=='map_changed'
    assert predictor.evaluate(T,1_110_000_000,'map1')['status']=='unavailable'
    with pytest.raises(ValueError):predictor.push(ImuMeasurement(1_100_000_000,(0,0,9.81),(0,0,0)))

def test_bias_compensation_and_rotation_during_loss():
    state=BodyState(0,np.eye(3),np.zeros(3),np.zeros(3),np.array([.2,0,0]),np.array([0,0,.1]),'map')
    samples=[ImuMeasurement(i*10_000_000,(.2,0,9.81),(0,0,1.1)) for i in range(11)]
    result=integrate_loss(state,samples,100_000_000)
    np.testing.assert_allclose(result.position_world_m,[0,0,0],atol=1e-7)
    np.testing.assert_allclose(result.rotation_world_from_body[:2,:2],[[np.cos(.1),-np.sin(.1)],[np.sin(.1),np.cos(.1)]],atol=1e-7)
