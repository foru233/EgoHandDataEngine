#!/usr/bin/env python3
"""
VIO 稳定性自动诊断工具

扫描位姿 JSONL 日志、worker 日志和录制数据，生成统计报告、事件时序图和跳变分析。
用法: python scripts/diagnose_vio_stability.py --pose-log logs/vio_vs030_live_*.jsonl
"""
import json
import argparse
from pathlib import Path
from collections import defaultdict, Counter
from dataclasses import dataclass
from typing import Optional, List, Tuple
import numpy as np
from datetime import datetime

@dataclass
class PoseRecord:
    """单条位姿记录"""
    line_num: int
    timestamp_ns: int
    relative_time_s: float
    tracking_state: str
    pose_valid: bool
    imu_initialized: bool
    inertial_scale_finalized: bool
    map_id: int
    map_revision: int
    map_epoch: int
    position: Optional[np.ndarray]
    matrix: Optional[np.ndarray]
    quality_status: Optional[str]
    invalid_reason: Optional[str]

@dataclass
class JumpEvent:
    """跳变事件"""
    from_idx: int
    to_idx: int
    from_time: float
    to_time: float
    delta_time: float
    delta_position: float
    delta_rotation_deg: float
    same_map_epoch: bool
    map_epoch_change: Tuple[int, int]

class VIOStabilityAnalyzer:
    def __init__(self, pose_log_path: str):
        self.pose_log_path = Path(pose_log_path)
        self.records: List[PoseRecord] = []
        self.jumps: List[JumpEvent] = []
        self.session_start_ns: Optional[int] = None

    def load_poses(self):
        """加载位姿日志"""
        print(f"Loading {self.pose_log_path}...")
        with open(self.pose_log_path, 'r', encoding='utf-8') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    data = json.loads(line)
                    ts_ns = int(data['timestamp_ns'])
                    if self.session_start_ns is None:
                        self.session_start_ns = ts_ns

                    rel_time = (ts_ns - self.session_start_ns) / 1e9

                    matrix = None
                    position = None
                    if 'matrix' in data and data['matrix']:
                        matrix = np.array(data['matrix'], dtype=float)
                        if matrix.shape == (4, 4):
                            position = matrix[:3, 3]

                    record = PoseRecord(
                        line_num=line_num,
                        timestamp_ns=ts_ns,
                        relative_time_s=rel_time,
                        tracking_state=data.get('tracking_state', 'UNKNOWN'),
                        pose_valid=data.get('pose_valid', False),
                        imu_initialized=data.get('imu_initialized', False),
                        inertial_scale_finalized=data.get('inertial_scale_finalized', False),
                        map_id=data.get('map_id', 0),
                        map_revision=data.get('map_revision', 0),
                        map_epoch=data.get('map_epoch', 0),
                        position=position,
                        matrix=matrix,
                        quality_status=data.get('quality_status'),
                        invalid_reason=data.get('invalid_reason')
                    )
                    self.records.append(record)
                except (json.JSONDecodeError, KeyError, ValueError) as e:
                    print(f"  Warning: skipping line {line_num}: {e}")

        print(f"Loaded {len(self.records)} records")
        return len(self.records)

    def detect_jumps(self, position_threshold_m: float = 0.5,
                     max_time_gap_s: float = 0.2,
                     include_cross_epoch: bool = True):
        """检测位姿跳变

        Args:
            position_threshold_m: 位置变化阈值(米)
            max_time_gap_s: 最大时间间隔(秒)，超过此值不检测
            include_cross_epoch: 是否包含跨 epoch 的跳变
        """
        print(f"\nDetecting jumps (threshold={position_threshold_m}m, "
              f"max_gap={max_time_gap_s}s, cross_epoch={include_cross_epoch})...")

        valid_with_pose = [r for r in self.records
                          if r.pose_valid and r.position is not None]

        for i in range(len(valid_with_pose) - 1):
            r1, r2 = valid_with_pose[i], valid_with_pose[i+1]

            dt = r2.relative_time_s - r1.relative_time_s
            if dt > max_time_gap_s:
                continue

            # 如果不包含跨 epoch 跳变，跳过不同 epoch 的相邻帧
            if not include_cross_epoch and r1.map_epoch != r2.map_epoch:
                continue

            dp = np.linalg.norm(r2.position - r1.position)

            # 计算旋转变化
            R1, R2 = r1.matrix[:3, :3], r2.matrix[:3, :3]
            dR = R2 @ R1.T
            trace_val = np.trace(dR)
            # 限制 trace 范围避免 arccos 域错误
            trace_val = np.clip(trace_val, -1.0, 3.0)
            angle_rad = np.arccos((trace_val - 1) / 2)
            angle_deg = np.degrees(angle_rad)

            if dp >= position_threshold_m:
                jump = JumpEvent(
                    from_idx=self.records.index(r1),
                    to_idx=self.records.index(r2),
                    from_time=r1.relative_time_s,
                    to_time=r2.relative_time_s,
                    delta_time=dt,
                    delta_position=dp,
                    delta_rotation_deg=angle_deg,
                    same_map_epoch=(r1.map_epoch == r2.map_epoch),
                    map_epoch_change=(r1.map_epoch, r2.map_epoch)
                )
                self.jumps.append(jump)

        print(f"Found {len(self.jumps)} jumps "
              f"({sum(1 for j in self.jumps if j.same_map_epoch)} same-epoch)")
        return self.jumps

    def generate_statistics(self) -> dict:
        """生成统计报告"""
        stats = {}

        # 基础统计
        stats['total_records'] = len(self.records)
        stats['duration_s'] = self.records[-1].relative_time_s if self.records else 0

        # 跟踪状态分布
        tracking_states = Counter(r.tracking_state for r in self.records)
        stats['tracking_states'] = dict(tracking_states)

        # 有效位姿统计
        native_valid = sum(1 for r in self.records if r.pose_valid)
        quality_valid = sum(1 for r in self.records
                           if r.pose_valid and r.quality_status == '可信度待验证')
        stats['native_valid_count'] = native_valid
        stats['quality_valid_count'] = quality_valid
        stats['native_valid_rate'] = native_valid / len(self.records) if self.records else 0

        # IMU 初始化统计
        imu_init_count = sum(1 for r in self.records if r.imu_initialized)
        ba2_count = sum(1 for r in self.records if r.inertial_scale_finalized)
        stats['imu_initialized_count'] = imu_init_count
        stats['ba2_completed_count'] = ba2_count

        # 地图切换统计
        map_epochs = [r.map_epoch for r in self.records]
        unique_epochs = len(set(map_epochs))
        epoch_changes = sum(1 for i in range(len(map_epochs)-1)
                           if map_epochs[i] != map_epochs[i+1])
        stats['unique_map_epochs'] = unique_epochs
        stats['epoch_change_count'] = epoch_changes

        # 跳变统计
        stats['total_jumps'] = len(self.jumps)
        same_epoch_jumps = [j for j in self.jumps if j.same_map_epoch]
        stats['same_epoch_jumps'] = len(same_epoch_jumps)

        if self.jumps:
            jump_distances = [j.delta_position for j in self.jumps]
            stats['jump_distance_max'] = float(np.max(jump_distances))
            stats['jump_distance_p95'] = float(np.percentile(jump_distances, 95))
            stats['jump_distance_median'] = float(np.median(jump_distances))

        # 门控拒绝原因
        if any(r.invalid_reason for r in self.records):
            invalid_reasons = Counter(r.invalid_reason for r in self.records
                                     if r.invalid_reason)
            stats['invalid_reasons'] = dict(invalid_reasons)

        return stats

    def print_report(self, stats: dict):
        """打印统计报告"""
        print("\n" + "="*60)
        print("VIO 稳定性诊断报告")
        print("="*60)
        print(f"\n位姿日志: {self.pose_log_path}")
        print(f"总记录数: {stats['total_records']}")
        print(f"持续时间: {stats['duration_s']:.1f} 秒")

        print(f"\n原生跟踪状态分布:")
        for state, count in stats['tracking_states'].items():
            pct = 100 * count / stats['total_records']
            print(f"  {state}: {count} ({pct:.1f}%)")

        print(f"\n有效位姿:")
        print(f"  原生有效: {stats['native_valid_count']} "
              f"({100*stats['native_valid_rate']:.1f}%)")
        print(f"  门控有效: {stats['quality_valid_count']}")

        print(f"\n惯性状态:")
        print(f"  IMU 已初始化: {stats['imu_initialized_count']}")
        print(f"  BA2 已完成: {stats['ba2_completed_count']}")

        print(f"\n地图切换:")
        print(f"  唯一 epoch 数: {stats['unique_map_epochs']}")
        print(f"  epoch 变化次数: {stats['epoch_change_count']}")

        print(f"\n跳变检测:")
        print(f"  总跳变: {stats['total_jumps']}")
        print(f"  同 epoch 内跳变: {stats['same_epoch_jumps']}")

        if stats['total_jumps'] > 0:
            print(f"  跳变距离最大: {stats['jump_distance_max']:.3f} m")
            print(f"  跳变距离 P95: {stats['jump_distance_p95']:.3f} m")
            print(f"  跳变距离中位: {stats['jump_distance_median']:.3f} m")

        if 'invalid_reasons' in stats:
            print(f"\n门控拒绝原因:")
            for reason, count in stats['invalid_reasons'].items():
                print(f"  {reason}: {count}")

        print("\n" + "="*60)

    def export_report(self, output_dir: Path):
        """导出详细报告"""
        output_dir.mkdir(parents=True, exist_ok=True)

        stats = self.generate_statistics()

        # 保存统计 JSON
        stats_file = output_dir / "statistics.json"
        with open(stats_file, 'w', encoding='utf-8') as f:
            json.dump(stats, f, indent=2, ensure_ascii=False)
        print(f"\nStatistics saved to {stats_file}")

        # 保存跳变详情
        if self.jumps:
            jumps_file = output_dir / "jumps.csv"
            with open(jumps_file, 'w', encoding='utf-8') as f:
                f.write("from_idx,to_idx,from_time_s,to_time_s,delta_time_s,"
                       "delta_position_m,delta_rotation_deg,same_epoch,"
                       "epoch_from,epoch_to\n")
                for j in self.jumps:
                    f.write(f"{j.from_idx},{j.to_idx},{j.from_time:.3f},"
                           f"{j.to_time:.3f},{j.delta_time:.4f},"
                           f"{j.delta_position:.4f},{j.delta_rotation_deg:.2f},"
                           f"{j.same_map_epoch},{j.map_epoch_change[0]},"
                           f"{j.map_epoch_change[1]}\n")
            print(f"Jump details saved to {jumps_file}")

        return stats_file

def main():
    parser = argparse.ArgumentParser(
        description="VIO 稳定性自动诊断工具"
    )
    parser.add_argument(
        '--pose-log',
        required=True,
        help="位姿 JSONL 日志路径"
    )
    parser.add_argument(
        '--jump-threshold',
        type=float,
        default=0.5,
        help="跳变检测阈值(米，默认0.5)"
    )
    parser.add_argument(
        '--include-cross-epoch',
        action='store_true',
        default=True,
        help="包含跨 epoch 跳变(默认启用)"
    )
    parser.add_argument(
        '--output-dir',
        default='results/vio/stability_baseline',
        help="输出目录"
    )

    args = parser.parse_args()

    analyzer = VIOStabilityAnalyzer(args.pose_log)
    analyzer.load_poses()
    analyzer.detect_jumps(
        position_threshold_m=args.jump_threshold,
        include_cross_epoch=args.include_cross_epoch
    )

    stats = analyzer.generate_statistics()
    analyzer.print_report(stats)

    output_dir = Path(args.output_dir)
    analyzer.export_report(output_dir)

    print(f"\n诊断完成。详细报告已保存至 {output_dir}")

if __name__ == '__main__':
    main()
