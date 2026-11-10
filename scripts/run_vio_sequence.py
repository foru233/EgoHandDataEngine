"""离线运行官方示例；输出隔离，超时及非零退出均明确报告。"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
ORB = ROOT / 'third_party/ORB_SLAM3'
ENV = Path(os.environ.get('VIO_CONDA', str(Path.home() / 'miniconda3' / 'envs' / 'egohand-vio')))
DATA = ROOT / 'samples/date/VIO/converted'

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=('euroc_stereo', 'euroc_inertial', 'tum_stereo', 'tum_inertial'))
    parser.add_argument('--timeout', type=int, default=1800)
    args = parser.parse_args()
    tum = args.mode.startswith('tum_')
    inertial = args.mode.endswith('inertial')
    sequence = DATA / ('TUM-VI/room4' if tum else 'EuRoC/MH_01_easy')
    kind = 'Stereo-Inertial' if inertial else 'Stereo'
    exe = ORB / 'Examples' / kind / (('stereo_inertial_' if inertial else 'stereo_') + ('tum_vi.exe' if tum else 'euroc.exe'))
    # 使用转换数据目录中的官方标定副本；各模式必须分别核对配置。
    settings = sequence / 'calibration' / ('TUM-VI.yaml' if tum else 'EuRoC.yaml') if inertial else ROOT / 'configs/vio' / ('tum_stereo.yaml' if tum else 'euroc_stereo.yaml')
    stamps = sequence / ('timestamps_inertial.txt' if inertial else 'timestamps.txt')
    command = [str(exe), str(ORB / 'Vocabulary/ORBvoc.txt'), str(settings)]
    if tum:
        command += [str(sequence / 'mav0/cam0/data'), str(sequence / 'mav0/cam1/data'), str(stamps)]
        if inertial:
            command += [str(sequence / 'mav0/imu0/data.csv')]
    else:
        command += [str(sequence), str(stamps)]
    output = ROOT / 'results/vio' / (args.mode + '_full_' + time.strftime('%Y%m%d_%H%M%S'))
    output.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env['PATH'] = str(ENV / 'Library/bin') + os.pathsep + env['PATH']
    start = time.monotonic()
    report = {'command': command, 'timeout_s': args.timeout}
    with (output / 'run.log').open('wb') as log:
        try:
            proc = subprocess.run(command, cwd=output, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout)
            report.update(exit_code=proc.returncode, status='exited')
        except subprocess.TimeoutExpired:
            report.update(exit_code=None, status='timeout')
    report['elapsed_s'] = time.monotonic() - start
    (output / 'run.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report), flush=True)
    return 0 if report.get('exit_code') == 0 else 1

if __name__ == '__main__':
    raise SystemExit(main())
