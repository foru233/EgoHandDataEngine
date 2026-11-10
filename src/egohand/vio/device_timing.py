"""为原生设备时间输入提供可审计门控，不估计主机与设备之间的时钟映射。"""
from dataclasses import dataclass
import math
from .native_bridge import ImuMeasurement

@dataclass(frozen=True)
class SensorFrameTime:
    left_ns: int
    right_ns: int
    clock_domain: str
    origin: str
    evidence: str

@dataclass(frozen=True)
class TimedImu:
    measurement: ImuMeasurement
    clock_domain: str
    origin: str
    evidence: str

@dataclass(frozen=True)
class VioInterval:
    frame_ns: int
    stereo_delta_ns: int
    imu_samples: tuple[ImuMeasurement,...]
    discarded_before_first_frame: int

class DeviceTimeGate:
    """拒绝未知曝光时间、混合时钟、重复和缺失 IMU 区间。

    调用方必须先按可证实的方式把独立陀螺仪/加速度计样本组合为同一采样时刻；
    本类不按数组下标配对、不插值、不猜测 Kalibr 偏移，也不使用主机接收时间。
    """
    def __init__(self,*,max_stereo_delta_ns,max_imu_gap_ns,max_imu_buffer=8192):
        if max_stereo_delta_ns<0 or max_imu_gap_ns<=0 or max_imu_buffer<1:
            raise ValueError('invalid timing limits')
        self.max_stereo_delta_ns=max_stereo_delta_ns;self.max_imu_gap_ns=max_imu_gap_ns
        self.max_imu_buffer=max_imu_buffer;self.samples=[];self.last_imu=None
        self.last_frame=None;self.clock_domain=None

    def push_imu(self,sample):
        if sample.origin!='device_sensor_sample' or not sample.evidence or not sample.clock_domain:
            raise ValueError('IMU sampling timestamp provenance is unverified')
        ts=sample.measurement.timestamp_ns
        if not isinstance(ts,int) or ts<=0 or (self.last_imu is not None and ts<=self.last_imu):
            raise ValueError('IMU timestamps must be strictly increasing')
        vals=(*sample.measurement.acceleration_mps2,*sample.measurement.angular_velocity_rps)
        if len(vals)!=6 or not all(math.isfinite(v) for v in vals):raise ValueError('invalid IMU values')
        if self.clock_domain and sample.clock_domain!=self.clock_domain:raise ValueError('clock domain mismatch')
        if len(self.samples)>=self.max_imu_buffer:raise OverflowError('IMU buffer full; no samples silently discarded')
        self.clock_domain=sample.clock_domain;self.last_imu=ts;self.samples.append(sample)

    def frame_interval(self,frame):
        if frame.origin not in ('device_exposure_start','device_exposure_midpoint') or not frame.evidence:
            raise ValueError('camera exposure timestamp provenance is unverified')
        if not frame.clock_domain or (self.clock_domain and frame.clock_domain!=self.clock_domain):
            raise ValueError('clock domain mismatch')
        if any(not isinstance(t,int) or t<=0 for t in (frame.left_ns,frame.right_ns)):
            raise ValueError('invalid camera timestamp')
        if abs(frame.left_ns-frame.right_ns)>self.max_stereo_delta_ns:
            raise ValueError('stereo exposure delta exceeds limit')
        if self.last_frame is not None and frame.left_ns<=self.last_frame:
            raise ValueError('camera timestamps must be strictly increasing')
        n=sum(s.measurement.timestamp_ns<=frame.left_ns for s in self.samples)
        selected=self.samples[:n]
        if self.last_frame is None:
            # 首帧无 IMU；记录明确丢弃的启动前样本数。
            result=VioInterval(frame.left_ns,abs(frame.left_ns-frame.right_ns),(),n)
        else:
            if not selected:raise ValueError('no IMU samples in frame interval')
            ts=[self.last_frame]+[s.measurement.timestamp_ns for s in selected]+[frame.left_ns]
            if any(t<=self.last_frame for t in ts[1:-1]) or any(b-a>self.max_imu_gap_ns for a,b in zip(ts,ts[1:])):
                raise ValueError('IMU interval incomplete or sample gap too large')
            result=VioInterval(frame.left_ns,abs(frame.left_ns-frame.right_ns),tuple(s.measurement for s in selected),0)
        self.samples=self.samples[n:];self.last_frame=frame.left_ns;self.clock_domain=frame.clock_domain
        return result
