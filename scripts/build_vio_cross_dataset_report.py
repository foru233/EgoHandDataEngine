"""Build an evidence-indexed cross-dataset VIO validation report.

The report is deliberately conservative: it aggregates existing immutable
artifacts and preserves coverage/qualification limits instead of turning a
successful replay into an accuracy or recovery claim.
"""
from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HANDOFF = ROOT / 'handoff/vio_gate_optimization_20261114'


def read_json(path):
    path = Path(path)
    return json.loads(path.read_text(encoding='utf-8')) if path.is_file() else None


def compact_metrics(value):
    if not isinstance(value, dict):
        return None
    keep = ('count', 'rmse', 'mean', 'median', 'p95', 'max')
    return {key: value[key] for key in keep if key in value}


def trajectory_summary(path):
    data = read_json(path)
    if data is None:
        return None
    out = {'artifact': str(Path(path).relative_to(ROOT))}
    coverage = data.get('coverage', {})
    out['coverage'] = {key: coverage[key] for key in
                       ('expected_frames', 'estimate_available', 'gt_available', 'paired',
                        'estimate_fraction', 'paired_fraction') if key in coverage}
    if 'ate_translation_m' in data:
        out['ate_translation_m'] = compact_metrics(data['ate_translation_m'])
    if 'rpe_by_horizon_s' in data:
        out['rpe_by_horizon_s'] = {
            str(horizon): {
                'translation_m': compact_metrics(metrics.get('translation_m')),
                'rotation_deg': compact_metrics(metrics.get('rotation_deg')),
            } for horizon, metrics in data['rpe_by_horizon_s'].items()
        }
    if 'scored_body_pairs' in data:
        out['scored_body_pairs'] = data['scored_body_pairs']
        out['unscored_reasons'] = data.get('unscored_reasons', {})
        out['first_phase_times'] = data.get('first_phase_times', {})
        segments = data.get('reference_segments', [])
        out['reference_segments'] = [{key: segment[key] for key in
                                      ('reference_identity', 'phase', 'pairs', 'first_sequence',
                                       'last_sequence', 'span_s') if key in segment}
                                     for segment in segments]
        if segments:
            segment = max(segments, key=lambda item: item.get('pairs', 0))
            out['largest_reference_segment'] = {
                'pairs': segment.get('pairs', 0),
                'first_sequence': segment.get('first_sequence'),
                'last_sequence': segment.get('last_sequence'),
                'ate_translation_m': compact_metrics(segment.get('initial_aligned_ape_translation_m')),
                'rpe_translation_m': compact_metrics(segment.get('consecutive_rpe_translation_m')),
                'rpe_rotation_deg': compact_metrics(segment.get('consecutive_rpe_rotation_deg')),
            }
    return out


def recovery_summary(path):
    data = read_json(path)
    if data is None:
        return None
    folder = Path(path).parent
    run = read_json(folder / 'run_summary.json') or {}
    counts = run.get('counts', {})
    return {
        'artifact': str(Path(path).relative_to(ROOT)),
        'kind': data.get('kind'),
        'mask': data.get('mask'),
        'mask_seconds': data.get('mask_seconds'),
        'pre_mask_admitted': data.get('pre_mask_admitted'),
        'native_recovery': data.get('native_recovery'),
        'first_geometry': data.get('first_geometry'),
        'application_readmission': data.get('application_readmission'),
        'same_reference_as_pre_mask': data.get('same_reference_as_pre_mask'),
        'threshold_comparison_assessable': data.get('threshold_comparison_assessable'),
        'independent_reference_readmission': data.get('independent_reference_readmission'),
        'new_map_requests': data.get('new_map_requests', []),
        'step_rejection_count': len(data.get('step_rejections', [])),
        'consecutive_admitted_rpe_translation_m': compact_metrics(data.get('consecutive_admitted_rpe_translation_m')),
        'consecutive_admitted_rpe_rotation_deg': compact_metrics(data.get('consecutive_admitted_rpe_rotation_deg')),
        'run_counts': counts,
        'camera_body_snapshot_crosscheck': data.get('camera_body_snapshot_crosscheck'),
        'absolute_accuracy_verified': data.get('vs030_absolute_accuracy_verified') is True,
        'false_recovery_nonincrease_established': data.get('statistical_false_recovery_rate_nonincrease_established') is True,
    }


def vs030_summary(path):
    data = read_json(path)
    if data is None:
        return None
    return {
        'artifact': str(Path(path).relative_to(ROOT)),
        'input_frames': (data.get('replay_frame_accounting') or {}).get('decoded'),
        'processed_frames': (data.get('replay_frame_accounting') or {}).get('processed'),
        'pose_frames': data.get('pose_frames'),
        'tracking': data.get('tracking'),
        'pose_admitted': data.get('pose_admitted'),
        'loss_episodes': data.get('loss_episodes'),
        'jump_detected_frames': data.get('jump_detected_frames'),
        'quarantined_frames': data.get('quarantined_frames'),
        'pose_available_ratio': data.get('pose_available_ratio'),
        'runtime_metrics': data.get('runtime_metrics'),
        'offline_processing_fps': data.get('offline_processing_fps'),
        'real_time_30fps_acceptance': data.get('real_time_30fps_acceptance'),
        'device_validation': data.get('device_validation'),
        'vs030_absolute_accuracy_verified': data.get('vs030_absolute_accuracy_verified'),
    }


def public_catalog():
    return [
        {'dataset': 'EuRoC MAV', 'sequence': 'MH_01_easy', 'split': 'development',
         'gt': True, 'sensor': 'stereo + IMU',
         'reason': 'historical trajectory and gate work already used for development'},
        {'dataset': 'EuRoC MAV', 'sequence': 'V1_02_medium', 'split': 'development',
         'gt': True, 'sensor': 'stereo + IMU',
         'reason': 'used by prior candidate and recovery experiments'},
        {'dataset': 'TUM-VI', 'sequence': 'room1', 'split': 'development',
         'gt': True, 'sensor': 'stereo fisheye + IMU',
         'reason': 'calibration loader and recovery runner development sequence'},
        {'dataset': 'TUM-VI', 'sequence': 'room2', 'split': 'development',
         'gt': True, 'sensor': 'stereo fisheye + IMU',
         'reason': 'used for corrected-fisheye and recovery development'},
        {'dataset': 'TUM-VI', 'sequence': 'corridor4', 'split': 'development',
         'gt': True, 'sensor': 'stereo fisheye + IMU',
         'reason': 'used for corrected-fisheye gate coverage development'},
        {'dataset': 'TUM-VI', 'sequence': 'room4', 'split': 'development',
         'gt': True, 'sensor': 'stereo fisheye + IMU',
         'reason': 'historical trajectory and debugging work already used'},
        {'dataset': 'TUM-VI', 'sequence': 'room3_holdout', 'split': 'final_test_candidate',
         'gt': True, 'sensor': 'stereo fisheye + IMU',
         'reason': 'fixed-revision bag; full control coverage run completed; not used for parameter selection'},
        {'dataset': 'KITTI Raw/Odometry', 'sequence': None, 'split': 'unavailable',
         'gt': None, 'sensor': 'not present locally',
         'reason': 'no local sequence downloaded; KITTI OXTS/reference conventions need a separate adapter before Stereo-Inertial scoring'},
        {'dataset': 'Hilti SLAM Challenge', 'sequence': None, 'split': 'unavailable',
         'gt': None, 'sensor': 'not present locally',
         'reason': 'no local sequence or validated adapter; not substituted with an unsuitable sensor modality'},
    ]


def markdown(report):
    lines = [
        '# VIO 跨数据集联合验证报告',
        '',
        f"生成时间：{report['generated_at_utc']}",
        '',
        '本报告只索引已有可复核产物；SE(3) 评估不拟合尺度或测试集时间偏移，缺帧和 GT 缺口保留在覆盖统计中。',
        '',
        '## 数据集与划分',
        '',
        '| 数据集 | 序列 | 划分 | 传感器 | GT | 说明 |',
        '|---|---|---|---|---|---|',
    ]
    for item in report['dataset_catalog']:
        gt = '有' if item['gt'] is True else ('无' if item['gt'] is False else '未评估')
        lines.append(f"| {item['dataset']} | {item['sequence'] or '—'} | {item['split']} | {item['sensor']} | {gt} | {item['reason']} |")
    lines += ['', '## 已有 GT 轨迹评估', '', '| 产物 | 配对帧 | ATE RMSE (m) | RPE 0.5 s RMSE (m) | 限制 |', '|---|---:|---:|---:|---|']
    for item in report['trajectory_evaluations']:
        largest = item.get('largest_reference_segment') or {}
        ate = (item.get('ate_translation_m') or largest.get('ate_translation_m') or {}).get('rmse', '—')
        rpe = (item.get('rpe_by_horizon_s', {}).get('0.5', {}).get('translation_m')
               or largest.get('rpe_translation_m') or {}).get('rmse', '—')
        paired = item.get('coverage', {}).get('paired', item.get('scored_body_pairs', '—'))
        lines.append(f"| `{item['artifact']}` | {paired} | {ate} | {rpe} | 事后 SE(3) 对齐或参考段限制 |")
    lines += ['', '## 恢复/遮挡事件', '', '| 产物 | 预遮挡准入 | 同参考恢复 | 独立新图重新准入 | 可比较 | 运行计数 |', '|---|---|---|---|---|---|']
    for item in report['recovery_evaluations']:
        lines.append('| `{artifact}` | {pre_mask_admitted} | {same_reference_as_pre_mask} | {independent_reference_readmission} | {threshold_comparison_assessable} | {run_counts} |'.format(**item))
    lines += ['', '## VS030 本机链路', '', '| 产物 | 帧数 | 丢失事件 | 隔离帧 | 准入帧 | 离线 FPS | 设备验收 |', '|---|---:|---:|---:|---:|---:|---|']
    for item in report['vs030_runs']:
        tracking = item.get('tracking') or {}
        lines.append(f"| `{item['artifact']}` | {item.get('pose_frames','—')} | {item.get('loss_episodes','—')} | {item.get('quarantined_frames','—')} | {item.get('pose_admitted','—')} | {item.get('offline_processing_fps','—')} | {item.get('device_validation','—')} |")
    lines += ['', '## 当前结论', '']
    for key, value in report['claims'].items():
        lines.append(f'- {key}：**{value}**')
    lines += ['', '## 尚缺证据', '']
    lines.extend(f'- {note}' for note in report['open_requirements'])
    return '\n'.join(lines) + '\n'


def main():
    inventory = read_json(HANDOFF / 'data_inventory_20261114.json') or {}
    integrity = read_json(ROOT / 'samples10-15gb/VIO/TUM-VI/room3_holdout/integrity_validation.json') or {}
    metadata = read_json(ROOT / 'samples10-15gb/VIO/TUM-VI/room3_holdout/metadata_extraction.json') or {}
    trajectory_paths = [
        HANDOFF / 'euroc_mh01_existing_trajectory_eval_v2.json',
        HANDOFF / 'tum_room4_existing_trajectory_eval_v2.json',
        HANDOFF / 'tumvi_room1_native_baseline_20261114/trajectory_metrics.json',
        HANDOFF / 'fisheye_fixed_room1_native/ground_truth_metrics.json',
        HANDOFF / 'fisheye_fixed_room2_native/ground_truth_metrics.json',
    ]
    trajectories = [trajectory_summary(path) for path in trajectory_paths]
    trajectories = [item for item in trajectories if item]
    recoveries = []
    for path in sorted(HANDOFF.rglob('recovery_gt_metrics.json')):
        item = recovery_summary(path)
        if item:
            recoveries.append(item)
    vs030 = []
    for path in sorted(HANDOFF.glob('vs030*/validation.json')):
        item = vs030_summary(path)
        if item:
            vs030.append(item)
    matrix = read_json(HANDOFF / 'vs030_184714_stability_matrix_20261114/matrix_summary.json')
    report = {
        'schema_version': 1,
        'generated_at_utc': datetime.now(timezone.utc).isoformat(),
        'scope': 'all indexed local recordings and locally available public GT artifacts; no claim is broadened beyond its evidence',
        'inventory': {'counts': inventory.get('counts', {}), 'recordings': len(inventory.get('recordings', [])),
                      'other_recording_manifests': len(inventory.get('other_recording_manifests', [])),
                      'duplicate_groups': len(inventory.get('same_sdk_payload_sha256', [])),
                      'errors': inventory.get('errors', [])},
        'room3_holdout_integrity': integrity,
        'room3_holdout_metadata': metadata,
        'dataset_catalog': public_catalog(),
        'trajectory_evaluations': trajectories,
        'recovery_evaluations': recoveries,
        'vs030_runs': vs030,
        'vs030_stability_matrix': matrix,
        'claims': {
            'all_local_recordings_scanned': bool(inventory) and not inventory.get('errors'),
            'all_applicable_public_gt_sequences_evaluated': False,
            'absolute_accuracy_verified_for_vs030': False,
            'recovery_success_verified_on_same_map': False,
            'false_recovery_rate_nonincrease_established': False,
            'final_test_set_frozen': False,
            'official_stable_window_changed': False,
        },
        'open_requirements': [
            'KITTI/Hilti are catalogued as unavailable locally; an adapter and a licensed, synchronized Stereo-Inertial sequence are still needed before using either for scoring.',
            'The native short-recovery geometry ABI currently returns structured unavailable; no candidate run may claim visual recovery.',
            'VS030 has no independent external pose ground truth, so its gate, hand-world and GUI evidence cannot establish ATE/RPE or false-recovery rates.',
            'Room3 control coverage is complete but its synthetic dropout was already inside an isolated state; it is not a valid same-reference recovery event.',
            'Final cross-dataset gate thresholds and split freeze remain pending repeated independent events and a native geometry solver.',
        ],
    }
    out_json = HANDOFF / 'CROSS_DATASET_REPORT_20261114.json'
    out_md = HANDOFF / 'CROSS_DATASET_REPORT_20261114.md'
    out_json.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    out_md.write_text(markdown(report), encoding='utf-8')
    print(json.dumps({'json': str(out_json), 'markdown': str(out_md),
                      'trajectory_evaluations': len(trajectories),
                      'recovery_evaluations': len(recoveries), 'vs030_runs': len(vs030)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
