import struct
import pytest
from egohand.vio.vs030_imu_candidate import parse_candidate_payload


def test_candidate_layout_rejects_corrupt_or_nonmonotonic_payload():
    row = struct.pack('<QQ6f', 100, 200, 1, 2, 3, 4, 5, 6)
    assert parse_candidate_payload(row)[0].values == (1, 2, 3, 4, 5, 6)
    with pytest.raises(ValueError): parse_candidate_payload(row + b'x')
    with pytest.raises(ValueError): parse_candidate_payload(row + row)
    with pytest.raises(ValueError): parse_candidate_payload(struct.pack('<QQ6f', 1, 2, float('nan'), 2, 3, 4, 5, 6))
