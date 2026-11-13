import importlib.util
from pathlib import Path


spec = importlib.util.spec_from_file_location('hand_timing_audit',
    Path(__file__).resolve().parents[1] / 'scripts/audit_hand_timings.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def test_strata_do_not_treat_missing_detection_as_no_hand():
    rows = [
        {'processing_time_ms': 10, 'diagnostics': {'detected_hands_left': 1,
            'detected_hands_right': 1, 'valid_joints': 17, 'triangulation_ms': 2}},
        {'processing_time_ms': 20, 'diagnostics': {'detected_hands_left': 0,
            'detected_hands_right': 0, 'valid_joints': 0}},
        {'processing_time_ms': float('nan'), 'diagnostics': {}},
    ]
    result = audit.summarize(rows)['groups']
    assert result['all']['frames'] == 3
    assert result['all']['engine_total_ms']['count'] == 2
    assert result['either_detector']['frames'] == result['both_detectors']['frames'] == 1
    assert result['valid_3d_joint']['frames'] == result['no_detected_hands']['frames'] == 1
    assert result['valid_3d_joint']['triangulation_ms']['p50'] == 2
    assert result['no_detected_hands']['triangulation_ms']['p50'] is None
