import cv2
import numpy as np
from egohand.vio.visual_rotation import solve_relative_rotation


def test_stereo_pnp_rotation_with_known_motion_and_mismatches():
    rng=np.random.default_rng(42)
    points=rng.uniform([-1,-.7,1.5],[1,.7,4],size=(150,3))
    k=np.array([[200.,0,400],[0,200,300],[0,0,1]])
    rv=np.array([.04,-.07,.02]);translation=np.array([.03,-.01,.02])
    pixels=cv2.projectPoints(points,rv,translation,k,None)[0].reshape(-1,2)
    pixels[:25]=rng.uniform([0,0],[800,600],size=(25,2))
    result=solve_relative_rotation(points,pixels,k)
    assert result['valid'] and result['inliers']>=120
    expected=cv2.Rodrigues(rv)[0].T
    np.testing.assert_allclose(result['previous_from_current_rotation'],expected,atol=1e-5)


def test_no_stereo_geometry_cannot_be_a_valid_rotation():
    result=solve_relative_rotation(np.zeros((50,3)),np.zeros((50,2)),np.eye(3))
    assert not result['valid'] and 'previous_from_current_rotation' not in result
