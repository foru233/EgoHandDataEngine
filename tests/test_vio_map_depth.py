import numpy as np

from egohand.vio.map_depth import VoxelEvidence


def observe_surface(evidence, depth):
    target = np.array([.05, .05, depth])
    for x in (-.135, .05, .235):
        pose = np.eye(4)
        pose[0, 3] = x
        evidence.observe((target - pose[:3, 3])[None, :], pose)


def test_occlusion_does_not_remove_surface():
    baseline = {(0, 0, 11)}
    evidence = VoxelEvidence(size_m=.1)
    observe_surface(evidence, .62)
    _, _, removed = evidence.differences(baseline, set())
    assert not removed


def test_depth_behind_old_surface_confirms_removal():
    evidence = VoxelEvidence(size_m=.1)
    observe_surface(evidence, 1.42)
    _, unknown, removed = evidence.differences({(0, 0, 11)}, set())
    assert (0, 0, 14) in unknown
    assert (0, 0, 11) in removed


def test_repeated_observations_at_same_camera_position_are_not_multiple_views():
    evidence = VoxelEvidence(size_m=.1)
    for _ in range(5):
        evidence.observe(np.array([[.02, .02, 1.42]]), np.eye(4))
    assert evidence.resolved() == (set(), set())
