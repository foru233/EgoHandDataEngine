#!/usr/bin/env python3
"""
增强版 VIO 稳定性诊断 - 检测所有类型的跳变和异常

专门检测:
1. 跨 epoch 的大跳变 (VIBA2 优化导致)
2. 同 epoch 内的逐帧跳变
3. 有效位姿之间的大间隔
4. 频繁的地图重置
"""
import json
import argparse
from pathlib import Path
from collections import defaultdict, Counter
from dataclasses import dataclass
from typing import Optional, List, Tuple
import numpy as np

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
    from_line: int
    to_line: int
    from_time: float
    to_time: float
    delta_time: float
    delta_position: float
    delta_rotation_deg: float
    same_map_epoch: bool
    map_epoch_change: Tuple[int, int]
    ba2_transition: bool  # BA2 状态是否发生变化

class EnhancedVIOAnalyzer:
    def __init__(self, pose_log_path: str):
        self.pose_log_path = Path(pose_log_path)
        self.records: List[PoseRecord] = []
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

        print(f"Loaded {len(self.records)} records, duration {self.records[-1].relative_time_s:.1f}s")
        return len(self.records)

    def detect_all_jumps(self, position_threshold_m: float = 0.5):
        """检测所有类型的跳变，包括跨大时间间隔的"""
        print(f"\n检测跳变 (阈值={position_threshold_m}m，无时间间隔限制)...")

        jumps = []
        valid_with_pose = [(i, r) for i, r in enumerate(self.records)
                          if r.pose_valid and r.position is not None]

        print(f"有效位姿: {len(valid_with_pose)}/{len(self.records)}")

        for i in range(len(valid_with_pose) - 1):
            idx1, r1 = valid_with_pose[i]
            idx2, r2 = valid_with_pose[i+1]

            dt = r2.relative_time_s - r1.relative_time_s
            dp = np.linalg.norm(r2.position - r1.position)

            # 计算旋转变化
            R1, R2 = r1.matrix[:3, :3], r2.matrix[:3, :3]
            dR = R2 @ R1.T
            trace_val = np.clip(np.trace(dR), -1.0, 3.0)
            angle_rad = np.arccos((trace_val - 1) / 2)
            angle_deg = np.degrees(angle_rad)

            # BA2 状态变化检测
            ba2_transition = (r1.inertial_scale_finalized != r2.inertial_scale_finalized)

            if dp >= position_threshold_m:
                jump = JumpEvent(
                    from_idx=idx1,
                    to_idx=idx2,
                    from_line=r1.line_num,
                    to_line=r2.line_num,
                    from_time=r1.relative_time_s,
                    to_time=r2.relative_time_s,
                    delta_time=dt,
                    delta_position=dp,
                    delta_rotation_deg=angle_deg,
                    same_map_epoch=(r1.map_epoch == r2.map_epoch),
                    map_epoch_change=(r1.map_epoch, r2.map_epoch),
                    ba2_transition=ba2_transition
                )
                jumps.append(jump)

        # 按跳变距离排序
        jumps.sort(key=lambda j: j.delta_position, reverse=True)

        print(f"检测到 {len(jumps)} 次跳变:")
        print(f"  同 epoch 内: {sum(1 for j in jumps if j.same_map_epoch)}")
        print(f"  跨 epoch: {sum(1 for j in jumps if not j.same_map_epoch)}")
        print(f"  BA2 转变时: {sum(1 for j in jumps if j.ba2_transition)}")

        if jumps:
            print(f"\n前 10 大跳变:")
            for i, j in enumerate(jumps[:10], 1):
                epoch_str = "same" if j.same_map_epoch else f"{j.map_epoch_change[0]}→{j.map_epoch_change[1]}"
                ba2_str = " [BA2↑]" if j.ba2_transition else ""
                print(f"  {i}. 第{j.from_line}→{j.to_line}行: {j.delta_position:.3f}m, "
                      f"Δt={j.delta_time:.3f}s, epoch={epoch_str}{ba2_str}")

        return jumps

    def analyze_step_distribution(self):
        """分析相邻有效位姿之间的步长分布"""
        print(f"\n分析相邻位姿步长分布...")

        valid_with_pose = [r for r in self.records
                          if r.pose_valid and r.position is not None]

        steps = []
        for i in range(len(valid_with_pose) - 1):
            r1, r2 = valid_with_pose[i], valid_with_pose[i+1]
            if r1.map_epoch == r2.map_epoch:  # 只统计同 epoch 内
                dt = r2.relative_time_s - r1.relative_time_s
                if dt < 1.0:  # 只统计 1 秒内的相邻帧
                    dp = np.linalg.norm(r2.position - r1.position)
                    steps.append((dt, dp, dp/dt if dt > 0 else 0))

        if steps:
            dts, dps, speeds = zip(*steps)
            print(f"同 epoch、<1s 间隔的相邻步长统计 (n={len(steps)}):")
            print(f"  位置变化: P50={np.median(dps):.4f}m, P95={np.percentile(dps, 95):.4f}m, "
                  f"max={np.max(dps):.4f}m")
            print(f"  速度: P50={np.median(speeds):.4f}m/s, P95={np.percentile(speeds, 95):.4f}m/s")
            print(f"  时间间隔: P50={np.median(dts):.4f}s")

            # 检测异常大步长
            large_steps = [(dt, dp) for dt, dp, _ in steps if dp > 0.5]
            if large_steps:
                print(f"  ⚠ {len(large_steps)} 个 >0.5m 的同 epoch 步长 (可能异常)")
        else:
            print("  无同 epoch 相邻位姿样本")

    def generate_report(self, output_dir: Path, jumps: List[JumpEvent]):
        """生成完整报告"""
        output_dir.mkdir(parents=True, exist_ok=True)

        # 统计信息
        stats = {
            'log_file': str(self.pose_log_path),
            'total_records': len(self.records),
            'duration_s': self.records[-1].relative_time_s if self.records else 0,
            'tracking_states': dict(Counter(r.tracking_state for r in self.records)),
            'native_valid': sum(1 for r in self.records if r.pose_valid),
            'imu_initialized': sum(1 for r in self.records if r.imu_initialized),
            'ba2_completed': sum(1 for r in self.records if r.inertial_scale_finalized),
            'unique_epochs': len(set(r.map_epoch for r in self.records)),
            'epoch_changes': sum(1 for i in range(len(self.records)-1)
                               if self.records[i].map_epoch != self.records[i+1].map_epoch),
            'total_jumps': len(jumps),
            'same_epoch_jumps': sum(1 for j in jumps if j.same_map_epoch),
            'cross_epoch_jumps': sum(1 for j in jumps if not j.same_map_epoch),
            'ba2_transition_jumps': sum(1 for j in jumps if j.ba2_transition)
        }

        if jumps:
            stats['max_jump_m'] = float(max(j.delta_position for j in jumps))
            stats['jump_p95_m'] = float(np.percentile([j.delta_position for j in jumps], 95))

        # 保存 JSON
        with open(output_dir / 'enhanced_stats.json', 'w', encoding='utf-8') as f:
            json.dump(stats, f, indent=2, ensure_ascii=False)

        # 保存跳变详情
        if jumps:
            with open(output_dir / 'jumps_detailed.csv', 'w', encoding='utf-8') as f:
                f.write("from_line,to_line,from_time_s,to_time_s,delta_time_s,"
                       "delta_position_m,delta_rotation_deg,same_epoch,"
                       "epoch_from,epoch_to,ba2_transition\n")
                for j in jumps:
                    f.write(f"{j.from_line},{j.to_line},{j.from_time:.3f},"
                           f"{j.to_time:.3f},{j.delta_time:.4f},"
                           f"{j.delta_position:.4f},{j.delta_rotation_deg:.2f},"
                           f"{j.same_map_epoch},{j.map_epoch_change[0]},"
                           f"{j.map_epoch_change[1]},{j.ba2_transition}\n")

        print(f"\n报告已保存至: {output_dir}")
        return stats

def main():
    parser = argparse.ArgumentParser(description="增强版 VIO 稳定性诊断")
    parser.add_argument('--pose-log', required=True, help="位姿 JSONL 日志")
    parser.add_argument('--threshold', type=float, default=0.5, help="跳变阈值(米)")
    parser.add_argument('--output-dir', default='results/vio/enhanced_baseline',
                       help="输出目录")

    args = parser.parse_args()

    analyzer = EnhancedVIOAnalyzer(args.pose_log)
    analyzer.load_poses()

    jumps = analyzer.detect_all_jumps(position_threshold_m=args.threshold)
    analyzer.analyze_step_distribution()

    output_dir = Path(args.output_dir)
    analyzer.generate_report(output_dir, jumps)

if __name__ == '__main__':
    main()
