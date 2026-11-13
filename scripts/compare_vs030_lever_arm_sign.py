"""用同一 VS030 原始录制比较 cam1 杠杆臂的两种 X 符号。"""
import argparse
import json
import os
from pathlib import Path
import re
import sys

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from egohand.vio import replay_runtime as rr  # noqa: E402
from egohand.vio.vs030_geometry import prepare_vs030_vio_geometry  # noqa: E402


def candidate_config(output, x_m):
    """只修改 IMU.T_b_c1 的 X 平移，其他参数保持逐字一致。"""
    source = ROOT / 'configs/vio/vs030_rectified_half_calibration2.yaml'
    fs = cv2.FileStorage(str(source), cv2.FILE_STORAGE_READ)
    try:
        matrix = fs.getNode('IMU.T_b_c1').mat()
    finally:
        fs.release()
    matrix[0, 3] = x_m
    values = ', '.join(repr(float(x)) for x in matrix.reshape(-1))
    text, count = re.subn(r'(?ms)(^IMU\.T_b_c1:.*?\bdata:\s*\[)[^\]]*(\])',
                          lambda match: match[1] + ' ' + values + ' ' + match[2],
                          source.read_text(encoding='utf8'))
    if count != 1:
        raise ValueError('ORB 参数格式无效')
    output.write_text(text, encoding='utf8')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frames', type=int, default=450)
    parser.add_argument('--stride', type=int, default=2)
    args = parser.parse_args()
    output = ROOT / 'results/vio/lever_arm_sign_comparison'
    output.mkdir(parents=True, exist_ok=True)
    # 两组始终使用同一套结果 2 双目校正与时间参数；不修改录制或实时配置。
    geometry = prepare_vs030_vio_geometry(ROOT / '标定结果/2/data3-camchain-imucam-measured.yaml')
    rr.prepare_vs030_vio_geometry = lambda _path: geometry
    original_track = rr.ReplayBridge.track

    def track(self, stamp, left, right, measurements):
        left = cv2.resize(left, (800, 600), interpolation=cv2.INTER_AREA)
        right = cv2.resize(right, (800, 600), interpolation=cv2.INTER_AREA)
        return original_track(self, stamp, left, right, measurements)

    rr.ReplayBridge.track = track
    candidates = [('geometry_consistent', -.03485), ('opposite', .03485)]
    sessions = ['session_20261110_110729_776644600', 'session_20261110_112020_689246900']
    results = []
    for session in sessions:
        for name, x in candidates:
            config = output / f'{name}.yaml'
            candidate_config(config, x)
            os.environ['EGOHAND_VIO_CONFIG'] = str(config)
            directory = output / session / name
            report = rr.ReplayVio(ROOT / 'data/imu' / session, output_dir=directory,
                timing={'imu_alignment': 'lattice', 'lattice_reference': 'packet', 'lattice_phase_ms': 97.0},
                max_frames=args.frames, stride=args.stride).run()
            item = {'session': session, 'candidate': name, 'cam1_x_m': x,
                    'status': report['status'], 'frames': report['frames'],
                    'map_epochs': report['map_epochs'], 'reset_events': report['reset_events'],
                    'error': report['error'], 'output': str(directory)}
            results.append(item)
            (output / 'summary.json').write_text(json.dumps(results, indent=2, ensure_ascii=False), encoding='utf8')
            print(json.dumps(item, ensure_ascii=True), flush=True)
            if report['status'] != 'completed':
                raise SystemExit('回放失败，停止比较')


if __name__ == '__main__':
    main()
