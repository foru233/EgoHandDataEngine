import cv2
import numpy as np
from .base import StereoDataSource
from ..core.types import StereoFrame,SourceMetadata

class SingleImageSource(StereoDataSource):
    """Real-image 2D inspection. Right pane is explicitly empty, never a fake pair."""
    def __init__(self,path): self.path=path;self.running=False;self.index=0
    def open(self):
        self.image=cv2.imread(str(self.path))
        if self.image is None: raise ValueError("真实图片无法读取")
        self.index=0
    def start(self): self.running=True
    def read(self):
        if not self.running or self.index: return None
        self.index+=1
        return StereoFrame(0,1_000_000_000,self.image.copy(),np.zeros_like(self.image),1_000_000_000,None,source_type="real_single_image",metadata={"synthetic":False,"timestamp_kind":"not_sensor_timestamp","single_image":True})
    def stop(self): self.running=False
    def close(self): self.stop()
    def get_metadata(self): return SourceMetadata("real_single_image",timestamp_kind="not_sensor_timestamp")
    def get_calibration(self): return None
