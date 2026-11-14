import pytest
from scripts.run_vs030_stability_matrix import END, FRAMES, START, summarize


def frames():
    return [{'pose_valid': False, 'tracking_state': 'OK', 'timestamp_ns': i * 100_000_000,
             'replay': {'frame_index': i},
             'controlled_visual_dropout': {'input_frame_index': i, 'active': START <= i < END}}
            for i in range(FRAMES)]


def test_uses_last_pre_mask_pose_and_refuses_unknown_identity():
    rows = frames()
    rows[START - 1]['pose_valid'] = True
    rows[END + 2]['pose_valid'] = True
    result = summarize(rows)
    assert result['first_post_mask_frame'] == END + 2
    assert result['readmission_delay_s'] == pytest.approx(0.2)
    assert result['same_reference_as_pre_mask'] is None
    for i in (START - 1, END + 2):
        rows[i].update(map_id=1, reset_id=0, map_revision=2, reference_generation=3, world_frame_revision=0)
    assert summarize(rows)['same_reference_as_pre_mask'] is True
    rows[END + 2]['reset_id'] = 1
    assert summarize(rows)['same_reference_as_pre_mask'] is False


def test_rejects_incorrect_dropout_window():
    rows = frames()
    rows[START]['controlled_visual_dropout']['active'] = False
    with pytest.raises(ValueError, match='dropout marker'):
        summarize(rows)
