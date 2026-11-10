"""诊断新版相机 IMU 候选布局，不能据此宣称字段语义或同步已经验收。"""
from dataclasses import dataclass
import math
import struct

# 仅为实测得到的候选布局：两个 uint64 加六个 float，小端、每条 40 字节。
_RECORD = struct.Struct('<QQ6f')


@dataclass(frozen=True)
class CandidateImuRecord:
    time_a_raw: int
    time_b_raw: int
    values: tuple[float, ...]


def parse_candidate_payload(payload):
    if not payload or len(payload) % _RECORD.size:
        raise ValueError('IMU 载荷长度与候选布局不符')
    records = []
    previous = None
    for a, b, ax, ay, az, gx, gy, gz in _RECORD.iter_unpack(payload):
        values = (ax, ay, az, gx, gy, gz)
        if not a or not b or not all(map(math.isfinite, values)):
            raise ValueError('候选 IMU 数据包含无效值')
        if previous and (a <= previous[0] or b <= previous[1]):
            raise ValueError('候选 IMU 时钟在包内不递增')
        records.append(CandidateImuRecord(a, b, values))
        previous = a, b
    return tuple(records)
