import importlib.util
import json
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location(
    'recovery_shadow_gt', Path(__file__).resolve().parents[1] / 'scripts/audit_recovery_shadow_gt.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def test_refuses_unanchored_runs_and_tampered_frames(tmp_path):
    plan = tmp_path / 'plan.json'
    frames = tmp_path / 'frames.jsonl'
    plan.write_text('{}', encoding='utf8')
    frames.write_text('{}\n', encoding='utf8')
    (tmp_path / 'run_summary.json').write_text(json.dumps({
        'frames_sha256': audit.runner.sha(frames), 'plan_sha256': audit.runner.sha(plan)}), encoding='utf8')
    (tmp_path / 'recovery_gt_metrics.json').write_text(json.dumps({
        'pre_mask_admitted': False, 'threshold_comparison_assessable': False}), encoding='utf8')
    with pytest.raises(ValueError, match='same-reference'):
        audit.analyze(tmp_path)
    frames.write_text('{"changed":true}\n', encoding='utf8')
    with pytest.raises(ValueError, match='provenance'):
        audit.analyze(tmp_path)
