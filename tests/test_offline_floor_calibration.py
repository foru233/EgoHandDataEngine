import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np


def test_floor_calibration_keeps_head_and_bones_and_bends_knees_forward(tmp_path):
    root = Path(__file__).resolve().parents[1]
    def frame(index, shift):
        joints = {'Head': [0., -.7-shift, 0.], 'Pelvis': [0., .075, 0.]}
        for side, x in [('Left', -.1), ('Right', .1)]:
            joints.update({side+'Hip': [x, .075, 0.], side+'Knee': [x, .525, 0.],
                           side+'Ankle': [x, .975, 0.]})
        return {'frame_id': index, 'body': {'reference_key': ['segment'], 'joints': joints,
            'diagnostics': {'pelvis_ground_adjustment_m': shift, 'ground_y_m': 1.,
                            'pelvis_forward_world': [0., 0., 1.]}}}
    source = tmp_path/'input.jsonl'; output = tmp_path/'output.jsonl'
    rows = [frame(0, 0.), frame(1, .2)]
    source.write_text(''.join(json.dumps(r)+'\n' for r in rows), encoding='utf8')
    env = os.environ.copy(); env['PYTHONPATH'] = str(root/'src')
    subprocess.run([sys.executable, str(root/'scripts/audit_floor_calibration.py'),
                    str(source), str(output)], check=True, capture_output=True, env=env)
    result = [json.loads(x) for x in output.read_text(encoding='utf8').splitlines()]
    for before, after in zip(rows, result):
        body = after['body']; j = body['joints']
        assert j['Head'] == before['body']['joints']['Head']
        assert body['diagnostics']['experimental_floor_is_observed'] is False
        for side in ('Left', 'Right'):
            h,k,a = (np.asarray(j[side+part]) for part in ('Hip','Knee','Ankle'))
            assert np.isclose(np.linalg.norm(k-h), .45)
            assert np.isclose(np.linalg.norm(a-k), .45)
            assert np.isclose(a[1], body['diagnostics']['ground_y_m']-.025)
            assert k[2] >= -1e-9
