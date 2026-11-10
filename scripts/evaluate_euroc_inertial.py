"""对比 EuRoC IMU 系轨迹；刚体对齐，不估计尺度或时间偏移。"""
import argparse
from decimal import Decimal
import json
from pathlib import Path

import numpy as np
import os
import subprocess
from evo.core import metrics, sync
from evo.tools import file_interface
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
GT = ROOT / 'samples/date/VIO/EuRoC/MH_01_easy/MH_01_easy_gt_openvins.tum'

def main():
    p = argparse.ArgumentParser()
    p.add_argument('run_directory', type=Path)
    p.add_argument('--dataset', choices=('euroc', 'tumvi'), default='euroc')
    p.add_argument('--stereo', action='store_true', help='输入为 EuRoC 校正左相机轨迹，先转换至 IMU 体坐标')
    args = p.parse_args()
    folder = args.run_directory.resolve()
    source = folder / 'CameraTrajectory.txt'
    out = folder / 'CameraTrajectory_seconds.tum'
    lines = []
    for line in source.read_text(encoding='utf8').splitlines():
        fields = line.split()
        if fields:
            if len(fields) != 8:
                raise ValueError('ORB-SLAM3 位姿行必须有 8 列')
            lines.append(str(Decimal(fields[0]) / Decimal(1_000_000_000)) + ' ' + ' '.join(fields[1:]))
    out.write_text('\n'.join(lines) + '\n', encoding='utf8')
    if args.stereo:
        # Settings::precomputeRectificationMaps 使用 Stereo.T_c1_c2 的逆。
        # 原工程 OpenCV 4.11 与 C++ 一致；子进程隔离 conda 的 DLL 路径。
        env = os.environ.copy()
        env['PATH'] = os.pathsep.join(p for p in env['PATH'].split(os.pathsep) if 'egohand-vio' not in p.lower())
        transform_json = subprocess.check_output([str(ROOT / '.venv/Scripts/python.exe'), str(ROOT / 'scripts/euroc_rectified_transform.py'), args.dataset], env=env, text=True)
        T_rect_body = np.asarray(json.loads(transform_json), dtype=float)
        converted = []
        for line in lines:
            fields = line.split()
            T_world_rect = np.eye(4)
            T_world_rect[:3,:3] = Rotation.from_quat([float(x) for x in fields[4:8]]).as_matrix()
            T_world_rect[:3,3] = [float(x) for x in fields[1:4]]
            T_world_body = T_world_rect @ T_rect_body
            values = list(T_world_body[:3,3]) + list(Rotation.from_matrix(T_world_body[:3,:3]).as_quat())
            converted.append(fields[0] + ' ' + ' '.join(f'{v:.12g}' for v in values))
        out = folder / 'BodyTrajectory_seconds.tum'
        out.write_text('\n'.join(converted) + '\n', encoding='utf8')
    ground_truth = GT if args.dataset == 'euroc' else ROOT / 'results/vio/tumvi_room4_gt_from_bag.tum'
    ref = file_interface.read_tum_trajectory_file(str(ground_truth))
    est = file_interface.read_tum_trajectory_file(str(out))
    matched_ref, matched_est = sync.associate_trajectories(ref, est, max_diff=0.005)
    rotation, translation, scale = matched_est.align(matched_ref, correct_scale=False)
    report = {'gt_source': str(ground_truth), 'estimate_source': str(source),
              'estimate_frame': 'ORB-SLAM3 body/IMU' if not args.stereo else ('rectified camera converted to IMU body using R1 and T_b_c1' if args.dataset == 'euroc' else 'fisheye camera converted to IMU body using T_b_c1'), 'gt_frame': 'EuRoC state estimate sensor frame' if args.dataset == 'euroc' else 'calibrated bag MoCap in IMU frame',
              'alignment': 'SE(3), all associated samples; no scale correction',
              'max_time_difference_s': 0.005, 'time_offset_s': 0,
              'input_poses': len(lines), 'matched_poses': matched_est.num_poses,
              'scale_applied': float(scale), 'rotation': rotation.tolist(), 'translation_m': translation.tolist(),
              'caveat': ('OpenVINS sidecar is a public derivative; remaining ground-truth timing and provenance uncertainty applies. ' if args.dataset == 'euroc' else 'Calibrated TUM-VI room bag contains IMU-frame MoCap poses; source paper specifies temporal alignment. ') + 'RPE uses consecutive associated frames.'}
    for name, metric in [('ape_translation_m', metrics.APE(metrics.PoseRelation.translation_part)),
                         ('rpe_translation_m', metrics.RPE(metrics.PoseRelation.translation_part)),
                         ('rpe_rotation_deg', metrics.RPE(metrics.PoseRelation.rotation_angle_deg))]:
        metric.process_data((matched_ref, matched_est))
        report[name] = {str(key): float(value) for key, value in metric.get_all_statistics().items()}
    (folder / 'evo_metrics.json').write_text(json.dumps(report, indent=2), encoding='utf8')
    print(json.dumps(report, indent=2))

if __name__ == '__main__':
    main()
