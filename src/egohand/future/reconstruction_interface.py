from typing import Protocol
from ..core.types import StereoHandResult

class TemporalHandOptimizer(Protocol):
    def optimize(self,result:StereoHandResult) -> StereoHandResult: ...

class MANOHandReconstructor(Protocol):
    def reconstruct(self,result:StereoHandResult): ...

class ObjectTracker(Protocol):
    def process(self,frame): ...

class RobotRetargetingAdapter(Protocol):
    def retarget(self,result:StereoHandResult): ...
