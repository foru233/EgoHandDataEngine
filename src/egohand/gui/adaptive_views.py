"""Size the viewports from the image ratio and the available workspace area."""
from PySide6.QtCore import Qt,QTimer
from PySide6.QtWidgets import QSplitter

class AdaptiveViews(QSplitter):
    def __init__(self,cameras,scene,parent=None):
        super().__init__(Qt.Horizontal,parent)
        self.cameras=cameras;self.scene=scene
        self.image_ratio=4/3
        self.mode="auto"
        self.setChildrenCollapsible(False);self.setHandleWidth(10)
        self.addWidget(cameras);self.addWidget(scene)
        self.reflow_timer=QTimer(self);self.reflow_timer.setSingleShot(True)
        self.reflow_timer.timeout.connect(self.reflow)
        self.reflow_timer.start(0)
    def resizeEvent(self,event):
        super().resizeEvent(event)
        if hasattr(self,"reflow_timer"):self.reflow_timer.start(35)
    def set_image_ratio(self,ratio):
        if ratio>0 and abs(self.image_ratio-ratio)>.001:
            self.image_ratio=ratio;self.reflow_timer.start(0)
    def set_mode(self,mode):
        self.mode=mode;self.reflow()
    def reflow(self):
        width,height=self.width(),self.height()
        if width<100 or height<100:return
        header=46;gap=self.handleWidth()
        camera_height=(width-gap)/2/self.image_ratio+header
        # A wide, short workspace works best with the stereo pair stacked at
        # left and a substantial 3D viewport at right. Tall windows can stack rows.
        top=(height-camera_height-gap>=max(240,height*.4)) if self.mode=="auto" else self.mode=="top"
        if top:
            self.setOrientation(Qt.Vertical);self.cameras.setOrientation(Qt.Horizontal)
            camera_height=round(min(max(190,camera_height),height*.6))
            self.setSizes([camera_height,max(180,height-camera_height-gap)])
            self.cameras.setSizes([width//2,width//2])
        else:
            self.setOrientation(Qt.Horizontal);self.cameras.setOrientation(Qt.Vertical)
            camera_width=round(max(264,min(width*.63,(height-gap-2*header)/2*self.image_ratio)))
            self.setSizes([camera_width,max(270,width-camera_width-gap)])
            self.cameras.setSizes([height//2,height//2])
