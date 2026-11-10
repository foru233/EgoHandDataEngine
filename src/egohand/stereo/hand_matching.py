# 左右手关联与跨帧身份管理；调试参数集中在 configs/stereo.yaml 前部。
# 立体匹配代价与跨帧身份代价不同；两处共用 ambiguity_margin，但都不是概率。

from dataclasses import dataclass
import itertools
import numpy as np
from scipy.optimize import linear_sum_assignment
from .geometry_validation import ReconstructionValidator

@dataclass
class Match:
    left_index: int
    right_index: int
    cost: float
    joints: list

# 将三角化/骨架检查作为硬门控，再用 Hungarian 求解左右视图的全局分配。
class StereoHandAssociator:
    """Gated Hungarian assignment, explicit unmatched and global ambiguity test."""
    def __init__(self,triangulator,config):
        self.tri,self.cfg=triangulator,config
        self.validator=ReconstructionValidator(config)

    # 逐对构造候选；diagnostics 保存拒绝原因、分量及替代解差值，便于调参定位问题。
    def associate(self,left,right,tracks=(),timestamp_ns=0):
        self.tri.elapsed_ms=0.0
        self.diagnostics={'algorithm_version':'hungarian-independent-evidence-v2',
            'legacy_disparity_weight_ignored':True,'calibrated_probability':False,
            'candidates':[],'accepted_pairs':[],'alternative_margins':[]}
        n,m=len(left),len(right)
        if not n or not m: return [],False
        # 大数是禁止匹配的数值哨兵，不是普通可调代价；门控失败的候选不能被强行选中。
        costs=np.full((n,m),1e6)
        joints={}
        w=self.cfg["weights"]
        for i,a in enumerate(left):
            for j,b in enumerate(right):
                js=self.validator.validate(self.tri.triangulate(a,b))
                good=[p for p in js if p.valid]
                joints[i,j]=js
                detail={'left_index':i,'right_index':j,'valid_joints':len(good),
                        'status_counts':{s:sum(str(p.status)==s for p in js) for s in sorted({str(p.status) for p in js})},
                        'rejected_by_geometry':len(good)<self.cfg['min_valid_joints']}
                self.diagnostics['candidates'].append(detail)
                detail['joint_diagnostics']=[{'joint_id':p.joint_id,
                    'left_available':bool(a.valid[k] and np.isfinite(a.pixel_landmarks[k]).all()),
                    'right_available':bool(b.valid[k] and np.isfinite(b.pixel_landmarks[k]).all()),
                    'geometry_valid':bool(p.valid),'status':str(p.status),
                    'epipolar_error_px':p.epipolar_error_px,'reprojection_error_px':p.reprojection_error_px}
                    for k,p in enumerate(js)]
                if len(good)<self.cfg["min_valid_joints"]: continue
                epi=np.median([p.epipolar_error_px for p in good])/self.cfg["max_epipolar_px"]
                # 覆盖损失只计一次；disparity 已参与点级门控，不再重复加权同一证据。
                structure=1-len(good)/21
                label=float(a.handedness!=b.handedness)*min(a.handedness_score,b.handedness_score)
                center=np.mean([p.xyz_m for p in good],axis=0)
                temporal=0.
                if tracks:
                    distances=[np.linalg.norm(center-t["center"])/(.05+self.cfg["max_speed_m_s"]*max((timestamp_ns-t["timestamp_ns"])/1e9,0)) for t in tracks]
                    temporal=min(min(distances),2.)
                # Disparity already gates triangulation. Count valid-geometry
                # coverage once; its old duplicate weight was not independent evidence.
                costs[i,j]=(w["epipolar"]*epi if self.cfg.get('use_epipolar_error',True) else 0)+w["handedness"]*label+w["temporal"]*temporal+w["geometry"]*structure
                detail.update(cost=float(costs[i,j]),epipolar_used=self.cfg.get('use_epipolar_error',True),components={'epipolar':float(epi),
                    'geometry_coverage_loss':float(structure),'handedness':float(label),'temporal':float(temporal)})
        # Each side can independently remain unmatched, with a finite penalty.
        # 补入虚拟行/列表示左右各自可不匹配；放弃一对真实左右手会付两侧 unmatched_cost。
        def solve(matrix):
            padded=np.full((n+m,n+m),1e6)
            padded[:n,:m]=matrix
            for i in range(n): padded[i,m+i]=self.cfg["unmatched_cost"]
            for j in range(m): padded[n+j,j]=self.cfg["unmatched_cost"]
            padded[n:,m:]=0
            rows,cols=linear_sum_assignment(padded)
            pairs={(int(a),int(b)) for a,b in zip(rows,cols) if a<n and b<m and matrix[a,b]<1e5}
            return pairs,float(padded[rows,cols].sum())
        pairs,total=solve(costs)
        ambiguous=False
        disputed=set()
        # 逐一禁止已选边并重求最优解；替代解总成本差小于 ambiguity_margin 时拒绝争议边。
        for pair in pairs:
            alternative=costs.copy()
            alternative[pair]=1e6
            other,alt_total=solve(alternative)
            self.diagnostics['alternative_margins'].append({'pair':list(pair),'margin':alt_total-total})
            if alt_total-total<self.cfg["ambiguity_margin"]:
                ambiguous=True
                disputed|=pairs-other
        self.diagnostics.update(cost_matrix=costs.tolist(),total_cost=total,ambiguous=ambiguous,
            accepted_pairs=[list(p) for p in sorted(pairs-disputed)])
        return [Match(i,j,float(costs[i,j]),joints[i,j]) for i,j in sorted(pairs-disputed)],ambiguous

# 仅维护短期三维轨迹；不是人的身份识别模型，证据不足时允许新建 ID 或拒绝。
class HandTrackManager:
    def __init__(self,config):
        self.cfg=config
        self.tracks={}
        self.next_id=1
    # 只保留时间差非负且未超过 track_timeout_s 的历史，防止时间倒退引用未来轨迹。
    def active(self,ts):
        self.tracks={k:v for k,v in self.tracks.items() if 0<=(ts-v["timestamp_ns"])/1e9<=self.cfg["track_timeout_s"]}
        return list(self.tracks.values())
    # 先全局分配旧 ID；身份存在接近的替代解时拒绝有效三维点，不静默交换 ID。
    def assign(self,hands,ts):
        self.active(ts)
        valid=[h for h in hands if h.valid_joint_count>=self.cfg["min_valid_joints"]]
        old=list(self.tracks.items())
        centers=[np.mean([j.xyz_m for j in h.joints if j.valid],axis=0) for h in valid]
        assignments={}
        identity_ambiguous=set()
        if valid and old:
            C=np.zeros((len(valid),len(old)))
            for i,h in enumerate(valid):
                for k,(_,t) in enumerate(old):
                    dt=(ts-t["timestamp_ns"])/1e9
                    bound=self.cfg["max_speed_m_s"]*dt+self.cfg["temporal_slack_m"]
                    dist=np.linalg.norm(centers[i]-t["center"])
                    C[i,k]=dist/max(bound,1e-9)+(.35 if h.handedness!=t["handedness"] else 0) if dist<=bound else 1e6
                    # 长失联需要至少两次历史、速度外推残差与去平移姿态一致，不能只看中心最近。
                    if dt>self.cfg.get('reacquisition_gap_s',.1):
                        # Long-gap nearest neighbor alone cannot establish identity.
                        xyz=np.array([j.xyz_m if j.valid else np.full(3,np.nan) for j in h.joints])
                        common=np.isfinite(xyz).all(axis=1)&np.isfinite(t['xyz']).all(axis=1)
                        enough=common.sum()>=self.cfg['min_valid_joints'] and t.get('observations',1)>=2
                        if enough:
                            pose=xyz[common]-xyz[common].mean(axis=0)
                            old_pose=t['xyz'][common]-t['xyz'][common].mean(axis=0)
                            pose_error=np.sqrt(np.mean(np.sum((pose-old_pose)**2,axis=1)))
                            prediction=t['center']+t.get('velocity',np.zeros(3))*dt
                            residual=np.linalg.norm(centers[i]-prediction)
                            enough=pose_error<=self.cfg.get('reacquisition_pose_rms_m',.03) and residual<=self.cfg.get('reacquisition_prediction_m',.05)
                        if not enough:C[i,k]=1e6
            n,m=C.shape
            def solve(matrix):
                padded=np.full((n+m,n+m),1e6)
                padded[:n,:m]=matrix
                # 这里的 1.5 是旧 ID 分配的固定新建代价；与立体匹配配置 unmatched_cost 不是同一项。
                for i in range(n):padded[i,m+i]=1.5
                for k in range(m):padded[n+k,k]=0
                padded[n:,m:]=0
                rows,cols=linear_sum_assignment(padded)
                return {(int(i),int(k)) for i,k in zip(rows,cols) if i<n and k<m and matrix[i,k]<1e5},float(padded[rows,cols].sum())
            pairs,total=solve(C)
            for i,k in pairs:
                alternate=C.copy();alternate[i,k]=1e6
                other,value=solve(alternate)
                if value-total<self.cfg["ambiguity_margin"]:
                    identity_ambiguous.update(a for a,b in pairs.symmetric_difference(other))
            for i,k in pairs:
                if i not in identity_ambiguous:assignments[i]=old[k][0]
        validator=ReconstructionValidator(self.cfg)
        for i,h in enumerate(valid):
            # 身份歧义显式标记为无效，不靠新建一条虚假连续轨迹掩盖交换风险。
            if i in identity_ambiguous:
                from ..core.status import JointStatus
                from .geometry_validation import reject
                for j in h.joints:
                    if j.valid:reject(j,JointStatus.IDENTITY)
                h.track_id=-1;h.valid_joint_count=0;h.tracking_status="identity_ambiguous"
                continue
            track=assignments.get(i)
            if track is not None:
                previous=self.tracks[track]
                validator.validate(h.joints,previous["xyz"],(ts-previous["timestamp_ns"])/1e9)
                h.valid_joint_count=sum(j.valid for j in h.joints)
            else:
                track=self.next_id
                self.next_id+=1
            h.track_id=track
            h.tracking_status="tracked" if i in assignments else "new_track"
            if h.valid_joint_count>=self.cfg["min_valid_joints"]:
                xyz=np.array([j.xyz_m for j in h.joints])
                center=np.nanmean(xyz,axis=0);prior=self.tracks.get(track)
                dt=(ts-prior['timestamp_ns'])/1e9 if prior else 0
                # 只用实际保留下来的观测更新速度；时间为秒、位移为米，不能混用纳秒。
                velocity=(center-prior['center'])/dt if prior and dt>0 else np.zeros(3)
                self.tracks[track]={"center":center,"xyz":xyz,"timestamp_ns":ts,"handedness":h.handedness,
                    'observations':prior.get('observations',1)+1 if prior else 1,'velocity':velocity}
        return hands
