"""仅验收自动新地图：在真实录制处理中人为注入隔离，不表示故障来自这段数据。"""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from egohand.vio import live_subprocess
class InjectQuarantine(live_subprocess.RecoveryCoordinator):
 def __init__(self,**kwargs):super().__init__(**kwargs);self.frames=0
 def finish_frame(self,result,body):
  self.frames+=1
  if self.frames==80:self.quarantined=True
  return super().finish_frame(result,body)
live_subprocess.RecoveryCoordinator=InjectQuarantine
live_subprocess.main()
