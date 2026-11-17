import json
import subprocess
import sys
from pathlib import Path

import numpy as np


def test_stage_checks_frames_and_preserves_existing_candidate(tmp_path):
    script = Path(__file__).resolve().parents[1] / 'scripts' / 'stage_smpl_fit_candidate.py'
    (tmp_path / 'derived_pose.jsonl').write_text(
        json.dumps({'frame_id': 5, 'timestamp_ns': 12}) + '\n' +
        json.dumps({'frame_id': 6, 'timestamp_ns': 13}) + '\n', encoding='utf8')
    report = tmp_path / 'report.json'
    report.write_text(json.dumps([{'index': 0, 'frame_id': 5, 'valid': True},
                                  {'index': 1, 'valid': False}]), encoding='utf8')
    vertices = tmp_path / 'source.npy'
    array = np.ones((2, 6890, 3), dtype=np.float32)
    np.save(vertices, array)
    cmd = [sys.executable, str(script), str(report), str(vertices), str(tmp_path)]
    subprocess.run(cmd, check=True, capture_output=True)
    assert np.load(tmp_path / 'body_vertices_valid.npy').tolist() == [True, False]
    assert np.load(tmp_path / 'body_vertices.npy')[0, 0].tolist() == [1, -1, 1]
    assert subprocess.run(cmd, capture_output=True).returncode != 0
    assert np.load(tmp_path / 'body_vertices.npy')[0, 0].tolist() == [1, -1, 1]
