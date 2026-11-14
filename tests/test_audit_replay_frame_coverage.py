import pytest
from scripts.audit_replay_frame_coverage import account


def test_packet_identities_report_leading_interior_trailing_and_mismatch():
    index = [{'sink': sink, 'sequence': i, 'device_timestamp_raw': 100 + i}
             for i in range(5) for sink in (4, 5)]
    poses = [{'timestamp_ns': (100 + i) * 1000, 'replay': {'sequence': i, 'device_ts_raw': 100 + i}}
             for i in (1, 3)]
    poses[1]['timestamp_ns'] += 1
    report = account(index, poses)
    assert report['paired_packet_identities'] == 5
    assert report['missing']['leading'] == [0]
    assert report['missing']['interior'] == [2]
    assert report['missing']['trailing'] == [4]
    assert report['replay_timestamp_mismatch_sequences'] == [3]


def test_duplicate_sequences_fail_closed():
    rows = [{'sink': 4, 'sequence': 1, 'device_timestamp_raw': 3}] * 2
    with pytest.raises(ValueError, match='duplicate camera sequence'):
        account(rows, [])
