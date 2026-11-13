from egohand.vio.map_revision_audit import MapRevisionAudit


def row(epoch, world=0, valid=True):
    return {'timestamp_ns':123,'map_epoch':epoch,'world_frame_revision':world,
            'pose_valid':valid,'quality_status':'waiting' if not valid else 'accepted'}


def test_internal_revision_logs_actual_decision_without_claiming_continuity():
    audit=MapRevisionAudit()
    assert audit.observe(row(1)) is None
    decision=audit.observe(row(2,valid=False))
    assert decision['event']=='map_revision_changed'
    assert decision['gate_action']=='rejected'
    assert decision['coordinate_continuity']=='unverified'
    assert audit.observe(row(2)) is None


def test_new_map_world_change_and_unknown_are_distinct():
    audit=MapRevisionAudit();audit.observe(row(1))
    assert audit.observe(row(1<<40))['event']=='map_or_reset_changed'
    assert audit.observe(row(1<<40,world=1))['event']=='world_revision_changed'
    assert audit.observe(row(None,world=1))['event']=='identity_unknown'
