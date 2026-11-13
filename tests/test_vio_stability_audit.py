"""Audit tests protect classification against false jumps across missing observations."""
import importlib.util
from pathlib import Path

import numpy as np

spec = importlib.util.spec_from_file_location("audit_vio_stability", Path(__file__).parents[1] / "scripts/audit_vio_stability.py")
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def pose(t, x, epoch=1, valid=True):
    m = np.eye(4); m[0, 3] = x
    return {"timestamp_ns": int(t * 1e9), "raw_pose_valid": valid,
            "raw_matrix": m.tolist() if valid else None, "pose_valid": False,
            "map_epoch": epoch, "tracking_state": "OK" if valid else "LOST"}


def test_long_gap_never_classified_as_short_gap_jump():
    report, _, pairs = audit.audit_rows([pose(0, 0), pose(.1, 0, valid=False), pose(4, 20)])
    assert len(pairs) == 1 and not pairs[0]["record_adjacent"]
    assert not pairs[0]["short_gap"] and not pairs[0]["investigate_jump"]
    assert report["pairs"]["same_exact_epoch_short_gap"]["count"] == 0


def test_exact_epoch_and_map_id_are_different_classifications():
    e = 2 << 40
    report, _, _ = audit.audit_rows([pose(0, 0, e), pose(.1, 1, e + 1), pose(.2, 2, e + 1)])
    assert report["pairs"]["same_exact_epoch_short_gap"]["investigate_jump_count"] == 1
    assert report["pairs"]["changed_epoch_short_gap"]["investigate_jump_count"] == 1
    assert report["pairs"]["same_map_id_short_gap"]["investigate_jump_count"] == 2


def test_short_gap_available_pose_pair_retains_record_adjacency():
    report, _, pairs = audit.audit_rows([pose(0, 0), pose(.03, 0, valid=False), pose(.1, 1)])
    assert report["pairs"]["consecutive_records_with_raw_matrix"]["count"] == 0
    assert report["pairs"]["same_exact_epoch_short_gap"]["investigate_jump_count"] == 1
    assert pairs[0]["previous_line"] == 1 and pairs[0]["current_line"] == 3


def test_time_reversal_and_invalid_matrix_are_explicit():
    bad = pose(.2, 1); bad["raw_matrix"][0][0] = float("nan")
    report, _, pairs = audit.audit_rows([pose(.1, 0), pose(0, 1), bad])
    assert report["non_monotonic_frame_intervals"] == 1
    assert report["raw_valid_count"] == 3 and report["raw_matrix_available_count"] == 2
    assert pairs[0]["speed_mps"] is None and not pairs[0]["investigate_jump"]


def test_epoch_decode_preserves_all_components():
    assert audit.map_identity((7 << 40) | (3 << 20) | 9) == {
        "map_id": 7, "reset_revision": 3, "big_change_revision": 9}


def test_worker_events_not_invented_exit_codes_or_timestamps(tmp_path):
    path = tmp_path / "worker.log"
    path.write_text("Live VIO session: x\nstart VIBA 2\nend VIBA 2\nRelocalized!!\n", encoding="utf8")
    result = audit.audit_worker(path)
    assert result["event_counts"]["ba2_end"] == 1
    assert result["native_abnormal_exit_count"] is None
    assert all(e["timestamp_ns"] is None for e in result["events"])


def test_wall_throughput_and_resources_require_observed_samples():
    rows=[pose(0,0),pose(.1,.01)]
    report,_,_=audit.audit_rows(rows)
    assert report['wall_throughput_fps'] is None and report['resource_samples']==0
    rows[0].update(timing={'elapsed_s':20.,'track_ms':10.},resources={'rss_bytes':100,'threads':3})
    rows[1].update(timing={'elapsed_s':20.5,'track_ms':20.},resources={'rss_bytes':110,'threads':3})
    report,_,_=audit.audit_rows(rows)
    assert report['wall_throughput_fps']==2
    assert report['rss_observed_trend']['delta_bytes']==10
