from egohand.vio.map_identity import PresetMapIdentity, decode_map_epoch


def pose(map_id=7, reset=0, revision=3, valid=True):
    return {'map_epoch': (map_id << 40) | (reset << 20) | revision,
            'pose_valid': valid, 'tracking_state': 'OK'}


def test_nonzero_loaded_map_id_and_initial_nonzero_revision():
    identity = PresetMapIdentity(7)
    message = pose()
    assert decode_map_epoch(message['map_epoch']) == {'map_id': 7, 'reset_id': 0, 'map_revision': 3}
    assert identity.rejection(message) is None
    assert identity.located_epoch == message['map_epoch']
    assert identity.rejection(pose(valid=False)) is None


def test_wrong_map_never_pins_and_changed_world_never_rejoins():
    identity = PresetMapIdentity(7)
    assert identity.rejection(pose(map_id=8))
    assert identity.located_epoch is None
    assert identity.rejection(pose()) is None
    assert identity.rejection(pose(revision=4))
    assert identity.rejection(pose(reset=1))
    assert identity.located_epoch == pose()['map_epoch']


def test_uninitialized_or_inconsistent_snapshot_does_not_pin():
    identity = PresetMapIdentity(7)
    assert identity.rejection(pose(valid=False)) is None
    assert identity.located_epoch is None
    assert identity.rejection({**pose(), 'snapshot_consistent': False})
    assert identity.located_epoch is None
