from PySide6.QtWidgets import QWidget,QVBoxLayout,QComboBox,QLineEdit,QCheckBox,QSpinBox,QPushButton
from .components import label
from ..sources.synthetic import SCENARIOS

class SourcePanel(QWidget):
    def __init__(self,parent=None):
        super().__init__(parent)
        layout=QVBoxLayout(self);layout.setContentsMargins(0,0,0,0);layout.setSpacing(12)
        self.kind=QComboBox();self.kind.addItems(["几何模拟","录制 / 外部清单","真实单图 · 2D","VS030 · 双通道预览","HOT3D Clips","SHOW3D · v3 双目","DexYCB · 多相机 RGB"])
        self.scenario=QComboBox()
        for title,key in zip(("单手静止","单手靠近","单手远离","双手静止","双手交叉","双手不同深度","部分关节缺失","检测数量不一致","错误关节对应","零视差 / 退化","无效标定","时间戳异常"),SCENARIOS):self.scenario.addItem(title,key)
        self.scenario.setCurrentIndex(4)
        self.kind.setAccessibleName("数据源类型")
        self.scenario.setAccessibleName("模拟场景")
        self.scenario_title=label("模拟场景","muted")
        layout.addWidget(self.scenario_title);layout.addWidget(self.scenario)
        self.path_title=label("源文件路径","muted")
        self.path=QLineEdit();self.path.setPlaceholderText("选择图像或 manifest.json")
        self.path.setAccessibleName("源文件路径")
        layout.addWidget(self.path_title);layout.addWidget(self.path)
        self.source_hint=label("","hint");self.source_hint.setWordWrap(True);layout.addWidget(self.source_hint)
        layout.addSpacing(4)
        self.offline=QCheckBox("离线完整处理")
        self.offline.setToolTip("勾选：尽快逐帧处理，队列满时等待，不主动丢帧；未勾选：优先低延迟，允许丢弃旧待处理帧。修改在下次开始时生效；本选项不会自动录制。")
        self.loop=QCheckBox("循环回放")
        self.reuse=QCheckBox("复用保存的二维观测")
        self.reuse.setChecked(True)
        self.seek_index=QSpinBox();self.seek_index.setRange(0,10_000_000)
        self.seek_index.setAccessibleName("定位到帧")
        for w in (self.offline,self.loop,self.reuse):layout.addWidget(w)
        self.hot3d_mode=QComboBox();self.hot3d_mode.addItem("官方真值（GT）","ground_truth");self.hot3d_mode.addItem("算法预测","prediction")
        self.hot3d_mode.setAccessibleName("数据集查看内容")
        self.directory_button=QPushButton("选择 HOT3D 目录")
        layout.addWidget(self.directory_button);layout.addWidget(self.hot3d_mode)
        self.update_kind(0)
    def update_kind(self,kind):
        self.hot3d_mode.setVisible(kind in (4,5));self.directory_button.setVisible(kind in (4,5,6));self.directory_button.setText({4:"选择 HOT3D 目录",5:"选择 SHOW3D 目录",6:"选择 DexYCB 目录"}.get(kind,"选择目录"))
        for w in (self.scenario,self.scenario_title):w.setVisible(kind==0)
        for w in (self.path,self.path_title):w.setVisible(kind in (1,2,4,5,6))
        self.loop.setVisible(kind in (1,4,5,6));self.reuse.setVisible(kind==1);self.offline.setVisible(kind in (0,1,4,5,6))
        self.source_hint.setText(("使用真值二维投影验证几何链路。合成结果不代表真实设备测量。","选择原生录制或外部数据清单。有效标定与同步依据齐全时才进行米制重建。","单图缺少双目标定，无法进行极线校正，关键点检测不可用。","USB 直连采集；请关闭 VisionViewer。同步由用户确认，曝光时间差尚未测量。","打开 tar 或目录；双灰度图 1201-1/1201-2。官方真值与算法预测分开查看；测试集真值可能不可用。","选择 SHOW3D 根目录或已验收的单个场景目录；仅加载 v3 双目验收完成的场景。官方已去畸变单色 PinholePlane，在极线校正图上检测；GT 与预测严格分离。","选择解压后的 DexYCB 根目录、受试者目录或单个序列；自动使用固定的两路 RGB 相机及官方标定。")[kind])

