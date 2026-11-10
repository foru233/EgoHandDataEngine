"""按相机帧格点重采样的 IMU 缓冲（计数锁定）。

背景：VS030 的 IIO 驱动时间戳由 IMU 内部 RC tick 合成，实测相对相机时间以
约 0.3%（3000 ppm）线性漂移；厂商在 MCAP 路径用 `exposure_start + i*T/N`
的帧格点重定时，UDP 实时路径没有做。这里用“每个相机帧间隔消费
round(dt * rate_cam) 个样本”的方式把 IMU 锁到相机时间轴：

- 对齐只依赖相机帧时间与样本计数，不依赖会漂移的 IMU 时间戳；
- 驱动时间戳只用于包内/包间递增与缺口检测；
- 每个间隔内的样本按 `start + (i+0.5)*dt/n` 均匀铺开，时间严格落在
  `(start, end)` 内，满足 ORB-SLAM3 对 IMU 时间的要求。

这不是精度验收：缺口、丢样与真实采样时刻仍需厂商协议确认。
"""
from collections import deque
import math

from .vs030_imu_candidate import parse_candidate_payload

MAX_IMU_GAP_NS = 30_000_000
# Positive means samples occurred later than the frame label in the UDP packet header.
DEFAULT_PACKET_PHASE_MS = 97.0


def packet_lattice_phase_ns(timing):
    phase_ms = float(timing.get('lattice_phase_ms', DEFAULT_PACKET_PHASE_MS))
    if not math.isfinite(phase_ms) or abs(phase_ms) > 1000:
        raise ValueError('IMU 包格点相位无效')
    return round(phase_ms * 1e6)


class PacketLatticeImuBuffer:
    """Place each SDK IMU packet on the camera clock using its own device_ts."""

    def __init__(self, *, sample_rate_hz, camera_to_imu_drift_ppm, phase_ns, capacity=20000):
        self.sample_rate_hz = float(sample_rate_hz)
        if not math.isfinite(self.sample_rate_hz) or self.sample_rate_hz <= 0:
            raise ValueError('IMU 采样率必须为正数')
        if not abs(camera_to_imu_drift_ppm) < 100000:
            raise ValueError('IMU 漂移参数异常')
        self.period_ns = 1e9 * (1 + camera_to_imu_drift_ppm * 1e-6) / self.sample_rate_hz
        self.phase_ns = phase_ns
        self.capacity = capacity
        self.rows = deque()
        self.consumed = 0
        self.gap_events = 0
        self.reset()

    def reset(self):
        self.rows.clear()
        self.covered_until_ns = 0
        self._last_stamp = 0
        self._last_header = 0

    def pending(self):
        return len(self.rows)

    def add(self, payload, device_ts):
        records = parse_candidate_payload(payload)
        device_ts = int(device_ts)
        if device_ts <= self._last_header:
            raise ValueError('IMU 包头 device_ts 缺失或非严格递增')
        if len(self.rows) + len(records) > self.capacity:
            raise OverflowError('IMU 缓冲超限')
        start = device_ts * 1000 + self.phase_ns
        # Keep the sample count. A short packet is never padded with invented samples.
        placed = [(start + round((index + .5) * self.period_ns), *row.values)
                  for index, row in enumerate(records)]
        stamps = [self._last_stamp, *(row[0] for row in placed)]
        if any(b <= a for a, b in zip(stamps, stamps[1:])):
            raise ValueError('IMU 包格点重叠或时间非严格递增')
        self.gap_events += sum(a > 0 and b - a > 1.5 * self.period_ns
                               for a, b in zip(stamps, stamps[1:]))
        self.gap_events += sum(b.time_a_raw - a.time_a_raw > 1.5 * self.period_ns
                               for a, b in zip(records, records[1:]))
        self.rows.extend(placed)
        self._last_stamp = stamps[-1]
        self._last_header = device_ts
        # The header uses integer microseconds. Allow its rounding at a packet boundary.
        self.covered_until_ns = self._last_stamp + round(self.period_ns / 2) + 1000
        return records

    def interval(self, start, end):
        while self.rows and self.rows[0][0] <= start:
            self.rows.popleft()
        rows = []
        while self.rows and self.rows[0][0] <= end:
            rows.append(self.rows.popleft())
        stamps = [start, *(row[0] for row in rows), end]
        if not rows or any(b - a > MAX_IMU_GAP_NS for a, b in zip(stamps, stamps[1:])):
            raise ValueError('IMU 包格点未覆盖帧间隔或存在采样缺口')
        self.consumed += len(rows)
        return rows


class LatticeImuBuffer:
    def __init__(self, *, sample_rate_hz=480.0, camera_to_imu_drift_ppm=3000.0, capacity=60000):
        if not sample_rate_hz > 0:
            raise ValueError('IMU 采样率必须为正数')
        if not (abs(camera_to_imu_drift_ppm) < 100_000):
            raise ValueError('IMU 漂移参数异常')
        self.sample_rate_hz = float(sample_rate_hz)
        # 相机时间轴上的真实采样率：IMU 时钟慢 drift_ppm 时，每相机秒的样本更少。
        self.rate_per_ns = self.sample_rate_hz / (1.0 + camera_to_imu_drift_ppm * 1e-6) / 1e9
        self.rows = deque()
        self.last_a = 0
        self.last_b = 0
        self.carry = 0.0
        self.capacity = int(capacity)
        self.consumed = 0
        self.gap_events = 0

    def pending(self):
        return len(self.rows)

    def reset(self):
        """丢弃当前缓冲并重新开始对齐（用于数据不足/溢出后的显式重建）。"""
        self.rows.clear()
        self.last_a = 0
        self.last_b = 0
        self.carry = 0.0

    def add(self, payload):
        added = []
        for row in parse_candidate_payload(payload):
            if row.time_a_raw <= self.last_a or row.time_b_raw <= self.last_b:
                raise ValueError('IMU 时间非递增')
            if self.last_a and row.time_a_raw - self.last_a > MAX_IMU_GAP_NS:
                self.gap_events += 1
            self.rows.append(row)
            added.append(row)
            self.last_a = row.time_a_raw
            self.last_b = row.time_b_raw
        if len(self.rows) > self.capacity:
            raise OverflowError('IMU 缓冲超限；实时处理未跟上采样')
        return added

    def trim_before(self, timestamp_ns):
        """丢弃早于指定相机时间的样本（只用于会话起点对齐）。"""
        while self.rows and self.rows[0].time_a_raw < timestamp_ns:
            self.rows.popleft()

    def skip_span(self, span_ns):
        """跳过一段没有相机帧的相机时间（保持相位），返回实际丢弃数。"""
        span_ns = int(span_ns)
        if span_ns <= 0:
            return 0
        count = int(self.carry + span_ns * self.rate_per_ns + 1e-9)
        if count <= 0:
            return 0
        self.carry += span_ns * self.rate_per_ns - count
        dropped = 0
        for _ in range(min(count, len(self.rows))):
            self.rows.popleft()
            dropped += 1
        return dropped

    def expected(self, start_ns, end_ns):
        """该帧间隔应消费的样本数（含小数进位，保证长期不累积误差）。"""
        span = int(end_ns) - int(start_ns)
        if span <= 0:
            raise ValueError('相机时间必须递增')
        return int(self.carry + span * self.rate_per_ns + 1e-9)

    def consume(self, start_ns, end_ns):
        """消费该间隔的样本并返回 (t, ax, ay, az, gx, gy, gz) 列表。

        样本不足时返回 None（调用方应继续收包后重试），不抛异常。
        """
        start_ns = int(start_ns)
        end_ns = int(end_ns)
        count = self.expected(start_ns, end_ns)
        if count <= 0:
            return []
        if len(self.rows) < count:
            return None
        self.carry += (end_ns - start_ns) * self.rate_per_ns - count
        span = end_ns - start_ns
        out = []
        for index in range(count):
            row = self.rows.popleft()
            stamp = start_ns + int((index + 0.5) * span / count)
            out.append((stamp, *row.values))
        self.consumed += count
        return out
