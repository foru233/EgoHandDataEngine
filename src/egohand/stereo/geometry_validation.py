# 骨长与跨帧位移检查；阈值见 stereo.yaml，拒绝时统一清除有效坐标与质量分数。

import numpy as np
from ..detection.landmarks import BONES
from ..core.status import JointStatus

# 保留具体拒绝原因；不能以 (0,0,0) 作为无效三维点，否则下游容易误当测量。
def reject(joint,status):
    joint.valid=False
    joint.status=status
    joint.xyz_m[:]=np.nan
    joint.quality_score=None

class ReconstructionValidator:
    def __init__(self,config):
        self.cfg=config

    # 骨长检查只比较两端均有效的骨；时序检查仅在有历史且 dt>0 秒时运行。
    def validate(self,joints,previous=None,dt=None):
        bad=set()
        for a,b in BONES:
            if joints[a].valid and joints[b].valid:
                length=np.linalg.norm(joints[a].xyz_m-joints[b].xyz_m)
                if not self.cfg["min_bone_m"]<=length<=self.cfg["max_bone_m"]:
                    bad.add(b)
        for i in bad: reject(joints[i],JointStatus.BONE)
        # 位移容差 = max_speed_m_s × dt + temporal_slack_m；不是速度滤波或轨迹预测值。
        if previous is not None and dt is not None and dt>0:
            for i,j in enumerate(joints):
                if j.valid and np.isfinite(previous[i]).all() and np.linalg.norm(j.xyz_m-previous[i])>self.cfg["max_speed_m_s"]*dt+self.cfg["temporal_slack_m"]:
                    reject(j,JointStatus.TEMPORAL)
        return joints
