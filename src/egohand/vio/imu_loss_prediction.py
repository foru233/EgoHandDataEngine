"""丢失区间 IMU 惯性外推原语；调用方必须提供可信体坐标状态和 SI 数据。"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class BodyState:
    timestamp_ns: int
    rotation_world_from_body: np.ndarray
    position_world_m: np.ndarray
    velocity_world_mps: np.ndarray
    accel_bias_mps2: np.ndarray
    gyro_bias_rps: np.ndarray
    map_id: str


def _rotation_vector(w):
    angle=float(np.linalg.norm(w))
    if angle<1e-12:return np.eye(3)
    x,y,z=w/angle
    k=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
    return np.eye(3)+np.sin(angle)*k+(1-np.cos(angle))*(k@k)


def integrate_loss(state, samples, end_ns, *, gravity_world=(0.,0.,-9.81), max_gap_ns=20_000_000, max_duration_ns=2_000_000_000):
    """中点积分；样本须覆盖起止时刻，缺口不补造。返回体位姿、速度，不声称精度。"""
    start=int(state.timestamp_ns);end=int(end_ns)
    if not state.map_id or end<=start or end-start>max_duration_ns:raise ValueError('无效地图或预测时间区间')
    R=np.asarray(state.rotation_world_from_body,dtype=float).copy()
    p=np.asarray(state.position_world_m,dtype=float).copy();v=np.asarray(state.velocity_world_mps,dtype=float).copy()
    ba=np.asarray(state.accel_bias_mps2,dtype=float);bg=np.asarray(state.gyro_bias_rps,dtype=float);g=np.asarray(gravity_world,dtype=float)
    if R.shape!=(3,3) or any(x.shape!=(3,) for x in (p,v,ba,bg,g)) or not all(np.isfinite(x).all() for x in (R,p,v,ba,bg,g)) or not np.allclose(R.T@R,np.eye(3),atol=1e-5) or np.linalg.det(R)<.999:raise ValueError('可信状态无效')
    rows=list(samples)
    if len(rows)<2:raise ValueError('IMU 样本不足')
    ts=[int(s.timestamp_ns) for s in rows]
    if ts[0]>start or ts[-1]<end or any(b<=a or b-a>max_gap_ns for a,b in zip(ts,ts[1:])):raise ValueError('IMU 区间未覆盖或存在缺口')
    for i in range(len(rows)-1):
        left=max(start,ts[i]);right=min(end,ts[i+1])
        if right<=left:continue
        a0=np.asarray(rows[i].acceleration_mps2,dtype=float);a1=np.asarray(rows[i+1].acceleration_mps2,dtype=float)
        w0=np.asarray(rows[i].angular_velocity_rps,dtype=float);w1=np.asarray(rows[i+1].angular_velocity_rps,dtype=float)
        if any(x.shape!=(3,) or not np.isfinite(x).all() for x in (a0,a1,w0,w1)):raise ValueError('IMU 数据无效')
        alpha=((left+right)/2-ts[i])/(ts[i+1]-ts[i]);dt=(right-left)*1e-9
        omega=(1-alpha)*w0+alpha*w1-bg;acc=(1-alpha)*a0+alpha*a1-ba
        mid=R@_rotation_vector(omega*dt*.5);world_acc=mid@acc+g
        p=p+v*dt+.5*world_acc*dt*dt;v=v+world_acc*dt;R=R@_rotation_vector(omega*dt)
    return BodyState(end,R,p,v,ba.copy(),bg.copy(),state.map_id)


class LossRecoveryPredictor:
    """缓存最后可信体状态及连续 IMU；不可用时返回明确原因，不自动放行。"""
    def __init__(self, *, max_gap_ns=20_000_000, max_duration_ns=2_000_000_000,
                 position_floor_m=.30, position_growth_mps2=2.0,
                 angle_floor_deg=25., angle_growth_deg_s=90.):
        self.max_gap_ns=int(max_gap_ns);self.max_duration_ns=int(max_duration_ns)
        self.position_floor_m=float(position_floor_m);self.position_growth_mps2=float(position_growth_mps2)
        self.angle_floor_deg=float(angle_floor_deg);self.angle_growth_deg_s=float(angle_growth_deg_s)
        if min(self.max_gap_ns,self.max_duration_ns,self.position_floor_m,self.position_growth_mps2,self.angle_floor_deg,self.angle_growth_deg_s)<=0:
            raise ValueError('预测参数必须为正数')
        self.anchor=None;self.samples=[]

    def set_trusted_state(self,state):
        if not isinstance(state,BodyState) or not state.map_id:raise ValueError('缺少可信状态或地图')
        self.anchor=BodyState(int(state.timestamp_ns),np.array(state.rotation_world_from_body,copy=True),
            np.array(state.position_world_m,copy=True),np.array(state.velocity_world_mps,copy=True),
            np.array(state.accel_bias_mps2,copy=True),np.array(state.gyro_bias_rps,copy=True),state.map_id)
        self.samples=[]

    def push(self,sample):
        if self.anchor is None:raise ValueError('尚无可信起点')
        t=int(sample.timestamp_ns)
        if self.samples and (t<=self.samples[-1].timestamp_ns or t-self.samples[-1].timestamp_ns>self.max_gap_ns):
            raise ValueError('IMU 时间非递增或存在缺口')
        if t>self.anchor.timestamp_ns+self.max_duration_ns+self.max_gap_ns:raise OverflowError('预测区间超过上限')
        self.samples.append(sample)

    def predict(self, timestamp_ns, map_id, *, max_endpoint_hold_ns=0):
        """给出短时间 IMU 外推体状态；调用者须单独标明它不是视觉测量。"""
        if self.anchor is None:raise ValueError('no_trusted_anchor')
        if map_id!=self.anchor.map_id:raise ValueError('map_changed')
        samples=list(self.samples);hold_ns=0
        if samples and samples[-1].timestamp_ns<int(timestamp_ns):
            hold_ns=int(timestamp_ns)-int(samples[-1].timestamp_ns)
            if hold_ns<=max_endpoint_hold_ns:
                # 仅对极短的帧末尾作零阶保持，并在结果中明确报告。
                from .native_bridge import ImuMeasurement
                last=samples[-1]
                samples.append(ImuMeasurement(int(timestamp_ns),last.acceleration_mps2,last.angular_velocity_rps))
        pred=integrate_loss(self.anchor,samples,timestamp_ns,max_gap_ns=self.max_gap_ns,max_duration_ns=self.max_duration_ns)
        return pred,hold_ns

    def evaluate(self,candidate_world_from_body, timestamp_ns, map_id, *, max_endpoint_hold_ns=0):
        try:
            pred,hold_ns=self.predict(timestamp_ns,map_id,max_endpoint_hold_ns=max_endpoint_hold_ns)
        except ValueError as exc:return {'status':'unavailable','reason':str(exc)}
        T=np.asarray(candidate_world_from_body,dtype=float)
        if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1],atol=1e-6) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-5) or np.linalg.det(T[:3,:3])<.999:
            return {'status':'unavailable','reason':'invalid_candidate'}
        dt=(int(timestamp_ns)-self.anchor.timestamp_ns)*1e-9
        distance=float(np.linalg.norm(T[:3,3]-pred.position_world_m))
        relative=pred.rotation_world_from_body.T@T[:3,:3]
        angle=float(np.degrees(np.arccos(np.clip((np.trace(relative)-1)/2,-1,1))))
        # 经验包络，不是协方差或统计置信区间，需用实际噪声与真值标定。
        position_limit=self.position_floor_m+.5*self.position_growth_mps2*dt*dt
        angle_limit=self.angle_floor_deg+self.angle_growth_deg_s*dt
        return {'status':'consistent' if distance<=position_limit and angle<=angle_limit else 'inconsistent',
                'position_residual_m':distance,'rotation_residual_deg':angle,'duration_s':dt,
                'position_limit_m':position_limit,'rotation_limit_deg':angle_limit,
                'endpoint_hold_ns':hold_ns,
                'limits_kind':'heuristic_not_covariance'}
