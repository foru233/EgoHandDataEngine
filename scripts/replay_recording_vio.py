"""CLI：离线回放一个 VS030 录制目录，产出 poses.jsonl / run.log / replay.json。

示例：
    .\\.venv\\Scripts\\python.exe scripts\\replay_recording_vio.py --recording recordings\\session_xxx
    .\\.venv\\Scripts\\python.exe scripts\\replay_recording_vio.py --recording <dir> --lag-ms -85 ^
        --alignment lattice --stride 2 --max-frames 200

退出码：0 = 回放完成（即使 VIO 未初始化，只要整段跑完）；1 = 失败/取消/参数非法。
"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))


def build_parser():
    parser = argparse.ArgumentParser(
        description='VS030 录制目录 → 确定性离线 VIO 回放（ORB-SLAM3 子进程 + 原始 IMU 对齐）',
    )
    parser.add_argument('--recording', required=True, help='录制目录（含 raw/sdk_records.bin）')
    parser.add_argument('--output', help='输出目录；默认 <recording>/replay_vio/replay_<unixns>')
    parser.add_argument('--config-snapshot', help='显式完整配置快照目录；供缺少快照的旧录制使用')
    parser.add_argument('--lag-ms', type=float, help='覆盖 camera_to_imu_lag_ms（默认取录制/仓库配置）')
    parser.add_argument('--alignment', choices=('timestamp', 'lattice'),
                        help='覆盖 IMU 对齐方式（默认取录制/仓库配置）')
    parser.add_argument('--lattice-reference', choices=('packet', 'count'), help='包头格点 / 旧计数格点')
    parser.add_argument('--lattice-phase-ms', type=float, help='包格点相位，正值把样本时间后移')
    parser.add_argument('--stride', type=int, default=1, help='每 N 帧处理 1 帧（默认 1）')
    parser.add_argument('--max-frames', type=int, default=None, help='最多处理的帧数（默认全部）')
    parser.add_argument('--start-stamp-ns', type=int, help='起始相机绝对时间（含，纳秒）；首帧空 IMU')
    parser.add_argument('--end-stamp-ns', type=int, help='结束相机绝对时间（含，纳秒）')
    parser.add_argument('--quiet', action='store_true', help='不打印逐帧状态，只打印最终摘要')
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.stride < 1:
        raise SystemExit('--stride 必须 >= 1')
    if args.max_frames is not None and args.max_frames < 1:
        raise SystemExit('--max-frames 必须 >= 1')
    if any(value is not None and not 0 < value < 2**63
           for value in (args.start_stamp_ns, args.end_stamp_ns)):
        raise SystemExit('起止时间必须为正整数纳秒')
    if args.start_stamp_ns is not None and args.end_stamp_ns is not None and args.start_stamp_ns > args.end_stamp_ns:
        raise SystemExit('--start-stamp-ns 必须 <= --end-stamp-ns')
    recording = Path(args.recording).resolve()
    if not recording.is_dir():
        raise SystemExit(f'录制目录不存在：{recording}')

    from egohand.vio.replay_runtime import ReplayVio  # 延迟 import：缺少依赖时仍是清晰报错

    timing = {}
    if args.lag_ms is not None:
        timing['camera_to_imu_lag_ms'] = args.lag_ms
    if args.alignment is not None:
        timing['imu_alignment'] = args.alignment
    if args.lattice_reference is not None:
        timing['lattice_reference'] = args.lattice_reference
    if args.lattice_phase_ms is not None:
        timing['lattice_phase_ms'] = args.lattice_phase_ms

    def on_status(message):
        if args.quiet:
            return
        kind = message.get('type', 'status')
        if kind == 'pose':
            return  # 逐帧位姿已写入 poses.jsonl，终端只显示控制/进度消息
        print(f'[{kind}] {message.get("message", "")}', flush=True)

    config_options = {}
    if args.config_snapshot:
        from egohand.vio.config_snapshot import resolve_recording_config
        config_options['config_snapshot'] = resolve_recording_config(Path(args.config_snapshot))
    vio = ReplayVio(
        recording,
        output_dir=args.output,
        timing=timing or None,
        stride=args.stride,
        max_frames=args.max_frames,
        start_stamp_ns=args.start_stamp_ns,
        end_stamp_ns=args.end_stamp_ns,
        on_status=on_status,
        **config_options,
    )
    try:
        report = vio.run()
    except KeyboardInterrupt:
        vio.cancel()
        print('回放被中断', file=sys.stderr)
        return 1

    frames = report.get('frames', {})
    summary = {
        'status': report.get('status'),
        'replay_json': (report.get('output') or {}).get('replay'),
        'poses_jsonl': (report.get('output') or {}).get('poses'),
        'run_log': (report.get('output') or {}).get('run_log'),
        'frames': frames,
        'image_range': report.get('image_range'),
        'map_epochs': report.get('map_epochs'),
        'reset_events': report.get('reset_events'),
        'imu': report.get('imu'),
        'warnings': report.get('warnings'),
        'imu_coverage_exhausted': report.get('imu_coverage_exhausted'),
        'elapsed_s': report.get('elapsed_s'),
        'error': report.get('error'),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if report.get('status') != 'completed':
        print(f"回放未完成：{report.get('error')}", file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
