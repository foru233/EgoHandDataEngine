"""仅供录制回放验收：强制拒绝每个视觉候选，以验证原生回调真正参与接受前判定。"""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from egohand.vio import live_subprocess

class ForcedRejectCoordinator(live_subprocess.RecoveryCoordinator):
    def check_candidate(self,T,t,epoch):
        result=super().check_candidate(T,t,epoch)
        result['status']='inconsistent';result['reason']='test_forced_reject'
        return result

live_subprocess.RecoveryCoordinator=ForcedRejectCoordinator
live_subprocess.main()
