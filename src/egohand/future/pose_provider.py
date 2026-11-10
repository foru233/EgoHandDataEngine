from typing import Protocol
import numpy as np

class CameraPoseProvider(Protocol):
    def camera_to_world(self,timestamp_ns:int) -> np.ndarray: ...
