from PySide6.QtWidgets import QGroupBox,QVBoxLayout,QLabel

class CalibrationPanel(QGroupBox):
    def __init__(self,parent=None):
        super().__init__("双目几何标定",parent)
        layout=QVBoxLayout(self)
        layout.setContentsMargins(0,12,0,0);layout.setSpacing(12)
        self.label=QLabel("等待数据源启动后检查标定");self.label.setWordWrap(True)
        layout.addWidget(self.label)
        text=QLabel("标定描述左右相机的成像参数及相对位置，用于双目校正和三维重建。SHOW3D 从数据集逐帧读取标定。")
        text.setProperty("role","hint")
        text.setWordWrap(True);layout.addWidget(text)
