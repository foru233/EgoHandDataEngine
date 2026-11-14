"""Prepare an isolated TUM-VI version of the audited public recovery runner."""
import difflib
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'handoff/vio_recovery_bridge_v4_20261113/benchmark_application_recovery.py'
DEST = ROOT / 'handoff/vio_gate_optimization_20261114/tumvi_recovery_runner.py'


def replace_once(text, before, after):
    if text.count(before) != 1:
        raise ValueError(f'expected one anchor: {before[:100]}')
    return text.replace(before, after, 1)


def main():
    original = SOURCE.read_text(encoding='utf-8')
    text = original
    text = replace_once(text, "V3 = BASE  # Explicit v4 candidate profile; shared native DLL is unchanged.",
                        "V3 = BASE / 'vs030_184714_stability_matrix_20261114'  # isolated stable-window profiles")
    text = replace_once(text, "LIBRARY = BASE.parent / 'vio_recovery_bridge_20261113/bin/egohand_vio_short_recovery_candidate.dll'",
                        "LIBRARY = BASE / 'fisheye_calibration_candidate/src/egohand/vio/native/bin/egohand_vio_fisheye_candidate.dll'")
    text = replace_once(text, "sequence = 'V1_02_medium'", "sequence = 'room1'")
    text = replace_once(text, "str(V3 / 'recovery_candidate.yaml')",
                        "str(V3 / ('recovery_' + str(args.stable_seconds).replace('.', '_') + '.yaml'))")
    text = replace_once(text, "(ROOT / 'configs/vio/live_quality.yaml').read_text(encoding='utf8')",
                        "(V3 / ('quality_' + str(args.stable_seconds).replace('.', '_') + '.yaml')).read_text(encoding='utf8')")
    start = text.index('    # ORB Settings::precomputeRectificationMaps')
    end = text.index('    snapshot_calibration_errors = []', start)
    text = text[:start] + '''    # TUM-VI uses raw Kannala-Brandt camera1, not EuRoC's rectified pinhole.
    # T_b_c1 maps camera coordinates into body; its inverse maps body to camera.
    T_camera_from_body = np.linalg.inv(np.array(config['IMU.T_b_c1']['data']).reshape(4, 4))
''' + text[end:]
    text = replace_once(text, "'actual admitted rectified camera transform times calibrated T_rectified_camera_from_body'",
                        "'actual admitted raw fisheye camera transform times calibrated T_camera_from_body'")
    text = replace_once(text, "p.add_argument('--frames', type=int, default=1000)",
                        "p.add_argument('--frames', type=int, default=1000)\n    p.add_argument('--stable-seconds', type=float, choices=[0.5, 1.0, 1.5], default=1.5)")
    text = replace_once(text, "'sequence': sequence, 'kind': args.kind, 'frames': args.frames,",
                        "'sequence': sequence, 'kind': args.kind, 'stable_seconds': args.stable_seconds, 'frames': args.frames,")
    with DEST.open('x', encoding='utf-8') as f:
        f.write(text)
    patch = ''.join(difflib.unified_diff(original.splitlines(True), text.splitlines(True),
                                         fromfile=str(SOURCE), tofile=str(DEST)))
    DEST.with_suffix('.patch').write_text(patch, encoding='utf-8')
    report = {'source': str(SOURCE), 'source_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
              'runner': str(DEST), 'runner_sha256': hashlib.sha256(DEST.read_bytes()).hexdigest(),
              'scope': 'TUM room1 development sequence, raw fisheye, fixed calibration candidate; GT scoring separate',
              'warning': 'Native recovery geometry assumptions for raw fisheye must be assessed; not VS030 certification.'}
    DEST.with_name('tumvi_recovery_runner_provenance.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
