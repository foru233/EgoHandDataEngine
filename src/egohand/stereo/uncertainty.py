# 三维条件协方差：只传播校正后二维像素的假定随机噪声，不代表已校准置信概率。

"""Conditional first-order pixel-noise propagation, never a calibrated confidence."""
from dataclasses import dataclass
import cv2
import numpy as np

# 默认回退值集中在文件前面，便于独立调用；引擎实际优先使用 configs/stereo.yaml。
# 1.5px 是校正后像素随机噪声假设；没有建模标定、时序、误配或系统偏差。
DEFAULT_UNCERTAINTY={
    "enabled":True,                   # False 时返回 DISABLED，不计算协方差。
    "mode":"diagnostic",             # reject 的正式拒绝在三角化模块执行。
    "assumed_pixel_std":1.5,          # 每个二维坐标分量的假定标准差，px。
    "method":"numerical_jacobian",    # 一般投影矩阵下的数值误差传播。
    "jacobian_step_px":.01,           # 中心差分步长，px；也计算半步长。
    "stability_rtol":.02,             # 两种步长下 Jacobian 的相对差容差。
    "max_jacobian_condition":1e8,      # 奇异值比超过此值视为退化。
    "max_depth_std_m":.03,            # reject 模式的 Z 标准差上限，m。
    "max_position_std_m":.03,         # reject 模式的最大主轴标准差上限，m。
    "reject_unavailable":True,        # reject 模式是否拒绝不可用不确定性。
}

# 不可用时 std/协方差为 None，并通过 status 区分退化、数值不稳定或禁用。
@dataclass
class UncertaintyEstimate:
    depth_std_m: float | None = None
    position_std_m: float | None = None
    position_covariance_m2: np.ndarray | None = None
    status: str = "INVALID_INPUT"

# 平行校正双目的近似对照 σZ≈fBσd/d²；一般旋转/垂直/鱼眼情形走下面的 Jacobian。
def rectified_depth_std(focal_px,baseline_m,disparity_px,sigma_disparity_px):
    values=np.asarray([focal_px,baseline_m,disparity_px,sigma_disparity_px],dtype=float)
    if not np.isfinite(values).all() or focal_px<=0 or baseline_m<=0 or sigma_disparity_px<=0 or abs(disparity_px)<1e-9:return None
    return float(focal_px*baseline_m*sigma_disparity_px/disparity_px**2)

# 批量三角化并转回原左相机坐标；数值差分必须和主三角化的坐标约定相同。
def _solve(P1,P2,observations,R1):
    h=cv2.triangulatePoints(P1,P2,observations[:,:2].T.copy(),observations[:,2:].T.copy()).T
    xyz=np.full((len(h),3),np.nan)
    good=np.isfinite(h).all(axis=1)&(np.abs(h[:,3])>1e-12)
    xyz[good]=(h[good,:3]/h[good,3,None])@R1
    return xyz

# q=(uL,vL,uR,vR)，J=∂X/∂q，ΣX≈JΣuvJᵀ；标定、对应关系和时刻视为固定。
def estimate_uncertainty(P1,P2,left_uv,right_uv,R1,config=None,pixel_covariance=None):
    """Return one covariance per observation in ORIGINAL left optical metres.

    UV covariance is expressed in rectified detector pixels. Calibration,
    association, timing, and systematic detector bias are held fixed.
    """
    cfg={**DEFAULT_UNCERTAINTY,**(config or {})}
    observations=np.column_stack((np.asarray(left_uv,dtype=float),np.asarray(right_uv,dtype=float)))
    if observations.ndim!=2 or observations.shape[1]!=4:raise ValueError('UV observations must be Nx2 per camera')
    result=[UncertaintyEstimate() for _ in observations]
    if not cfg['enabled']:return [UncertaintyEstimate(status='DISABLED') for _ in observations]
    if cfg['method']!='numerical_jacobian':raise ValueError('Only numerical_jacobian is implemented for general stereo geometry')
    sigma=float(cfg['assumed_pixel_std']);step=float(cfg['jacobian_step_px'])
    if not np.isfinite([sigma,step]).all() or sigma<=0 or step<=0:raise ValueError('Pixel std and Jacobian step must be finite and positive')
    # 默认四维独立同方差像素噪声；显式传入时要求有限、对称、半正定的 4×4 协方差。
    S=np.eye(4)*sigma*sigma if pixel_covariance is None else np.asarray(pixel_covariance,dtype=float)
    if S.shape!=(4,4) or not np.isfinite(S).all() or not np.allclose(S,S.T) or np.linalg.eigvalsh(S).min()<-1e-12:raise ValueError('Pixel covariance must be finite symmetric PSD 4x4')
    P1,P2,R1=map(lambda x:np.asarray(x,dtype=float),(P1,P2,R1))
    if P1.shape!=(3,4) or P2.shape!=(3,4) or R1.shape!=(3,3) or not all(np.isfinite(p).all() for p in (P1,P2,R1)):return result
    indices=np.flatnonzero(np.isfinite(observations).all(axis=1))
    if not len(indices):return result
    obs=observations[indices]
    base=_solve(P1,P2,obs,R1)
    # 中心差分分别扰动四个像素坐标；批量处理关节，避免逐点反复创建投影矩阵。
    def jacobian(h):
        J=np.empty((len(obs),3,4))
        for k in range(4):
            plus=obs.copy();minus=obs.copy();plus[:,k]+=h;minus[:,k]-=h
            J[:,:,k]=(_solve(P1,P2,plus,R1)-_solve(P1,P2,minus,R1))/(2*h)
        return J
    # 用 h 和 h/2 两种步长比较稳定性，步长变小不应令导数发生显著改变。
    coarse=jacobian(step);fine=jacobian(step/2)
    for k,index in enumerate(indices):
        status='DEGENERATE_GEOMETRY'
        if not np.isfinite(base[k]).all() or base[k,2]<=0:
            result[index]=UncertaintyEstimate(status=status);continue
        J=fine[k]
        if not np.isfinite(J).all() or not np.isfinite(coarse[k]).all():
            result[index]=UncertaintyEstimate(status='UNSTABLE_JACOBIAN');continue
        relative=np.linalg.norm(J-coarse[k])/max(np.linalg.norm(J),1e-15)
        # 用奇异值检查局部可观测性；近零视差或退化时不能输出看似精确的零协方差。
        sv=np.linalg.svd(J,compute_uv=False)
        if relative>cfg['stability_rtol']:
            result[index]=UncertaintyEstimate(status='UNSTABLE_JACOBIAN');continue
        if sv[-1]<1e-12 or sv[0]/sv[-1]>cfg['max_jacobian_condition']:
            result[index]=UncertaintyEstimate(status=status);continue
        # 对称化只消除浮点误差；若特征值明显为负仍判无效，不强行修成“可信”。
        covariance=J@S@J.T;covariance=(covariance+covariance.T)/2
        eig=np.linalg.eigvalsh(covariance)
        if not np.isfinite(covariance).all() or eig.min()<-1e-12 or eig.max()<=0:
            result[index]=UncertaintyEstimate(status='INVALID_COVARIANCE');continue
        result[index]=UncertaintyEstimate(float(np.sqrt(max(covariance[2,2],0))),float(np.sqrt(eig.max())),covariance,'ESTIMATED_ASSUMED_PIXEL_NOISE')
    return result
