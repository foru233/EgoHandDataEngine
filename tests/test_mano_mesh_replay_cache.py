import numpy as np
from types import SimpleNamespace
from egohand.gui.vio_panel import TrajectoryView
from delivery_views import WorldHandOverlay


def test_mesh_cache_refreshes_when_other_vertices_or_camera_move(qtbot):
    view=TrajectoryView();qtbot.addWidget(view)
    overlay=WorldHandOverlay(view)
    mesh={'vertices':[[0.,0.,1.],[.1,0.,1.],[0.,.1,1.]],'faces':[[0,1,2]]}
    overlay.update_mano_meshes([mesh],np.eye(4))
    first=overlay.mesh_items[0].opts['meshdata'].vertexes().copy()
    changed={'vertices':[[0.,0.,1.],[.2,0.,1.],[0.,.1,1.]],'faces':[[0,1,2]]}
    overlay.update_mano_meshes([changed],np.eye(4))
    second=overlay.mesh_items[0].opts['meshdata'].vertexes().copy()
    assert not np.array_equal(first,second)
    moved=np.eye(4);moved[0,3]=.3
    overlay.update_mano_meshes([changed],moved)
    third=overlay.mesh_items[0].opts['meshdata'].vertexes()
    np.testing.assert_allclose(third[:,0]-second[:,0],.3,atol=1e-6)


def test_mano_mesh_uses_same_track_wrist_without_mutating_the_fit(qtbot):
    view=TrajectoryView();qtbot.addWidget(view)
    overlay=WorldHandOverlay(view)
    mesh={'track_id':7,'joints20':[[.1,0.,1.]],
          'vertices':[[.1,0.,1.],[.2,0.,1.],[.1,.1,1.]],'faces':[[0,1,2]]}
    hand=SimpleNamespace(track_id=7,joints=[SimpleNamespace(valid=True,xyz_m=np.array([.3,0.,1.]))])
    overlay.update_mano_meshes([mesh],np.eye(4),[hand])
    vertices=overlay.mesh_items[0].opts['meshdata'].vertexes().copy()
    np.testing.assert_allclose(vertices[0],[.3,1.,0.],atol=1e-6)
    assert mesh['vertices'][0]==[.1,0.,1.]
    # Duplicate IDs are ambiguous, so do not attach to either wrist.
    overlay.update_mano_meshes([mesh],np.eye(4),[hand,hand])
    vertices=overlay.mesh_items[0].opts['meshdata'].vertexes()
    np.testing.assert_allclose(vertices[0],[.1,1.,0.],atol=1e-6)
