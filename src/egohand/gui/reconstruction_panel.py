from PySide6.QtWidgets import QGroupBox,QFormLayout,QDoubleSpinBox,QSpinBox,QCheckBox,QPushButton,QLabel,QComboBox,QWidget,QVBoxLayout

DETECTOR_KEYS=('algorithm','num_hands','min_hand_detection_confidence','min_hand_presence_confidence','min_tracking_confidence','box_threshold','keypoint_threshold')

class ReconstructionPanel(QGroupBox):
    def __init__(self,parent=None):
        super().__init__('检测设置',parent)
        layout=QFormLayout(self);layout.setContentsMargins(0,8,0,0);layout.setVerticalSpacing(9)
        self.algorithm=QComboBox();self.algorithm.addItem('MediaPipe Hand','mediapipe');self.algorithm.addItem('RTMPose-Hand','rtmpose')
        layout.addRow('检测算法',self.algorithm)
        self.num_hands=QSpinBox();self.num_hands.setRange(1,2);self.num_hands.setValue(2);layout.addRow('最多手数',self.num_hands)
        self.confidence=[];self.mp_page=QWidget();mp=QFormLayout(self.mp_page);mp.setContentsMargins(0,0,0,0)
        for title in ('掌部检测阈值','手部存在阈值','跟踪阈值'):
            spin=QDoubleSpinBox();spin.setRange(0,1);spin.setSingleStep(.05);spin.setValue(.5);self.confidence.append(spin);mp.addRow(title,spin)
        layout.addRow(self.mp_page)
        self.rtm_page=QWidget();rtm=QFormLayout(self.rtm_page);rtm.setContentsMargins(0,0,0,0);self.rtm_confidence=[]
        for title,value,tip in [('手框阈值',.3,'RTMDet 手框最低分数'),('关节阈值',.3,'低于阈值的关节点不参与三维')]:
            spin=QDoubleSpinBox();spin.setRange(0,1);spin.setSingleStep(.05);spin.setValue(value);spin.setToolTip(tip);self.rtm_confidence.append(spin);rtm.addRow(title,spin)
        layout.addRow(self.rtm_page)
        self.numbers=QCheckBox('显示关节编号');self.epilines=QCheckBox('显示极线与匹配观测')
        self.use_epipolar=QCheckBox('使用极线误差约束')
        self.use_epipolar.setToolTip('关闭后不按极线误差拒绝关节或参与匹配打分；重投影、深度等其他门控仍生效。')
        layout.addRow(self.use_epipolar)
        self.epipolar_hint=QLabel('关闭后仍会计算误差；重投影等其他门控仍生效。')
        self.epipolar_hint.setWordWrap(True);self.epipolar_hint.setProperty('role','hint');layout.addRow(self.epipolar_hint)
        self.use_reprojection=QCheckBox('使用重投影误差约束')
        self.use_reprojection.setToolTip('关闭后仍计算误差，不按重投影误差拒绝或计入质量分；其他门控继续生效。')
        layout.addRow(self.use_reprojection)
        self.use_3d_tracking=QCheckBox('三维跨帧 ID 跟踪')
        self.use_3d_tracking.setToolTip('开启后按三维运动与姿态关联跨帧身份；关闭后只进行逐帧双目重建。')
        layout.addRow(self.use_3d_tracking)
        self.display_smoothing=QCheckBox('稳定显示（不改变结果）')
        self.display_smoothing.setChecked(True)
        self.display_smoothing.setToolTip('只平滑屏幕上的二维点和可区分的三维骨架；原始结果、几何门控和导出数据不改变。')
        layout.addRow(self.display_smoothing)
        self.apply_button=QPushButton('应用检测设置');self.apply_button.setMinimumHeight(36);self.apply_button.setProperty('role','primary');layout.addRow(self.apply_button)
        self.apply_hint=QLabel('调整后点击应用。');self.apply_hint.setWordWrap(True);self.apply_hint.setProperty('role','hint');layout.addRow(self.apply_hint)
        self._applied=self.settings();self._busy=False
        for spin in (self.num_hands,*self.confidence,*self.rtm_confidence):spin.valueChanged.connect(self._changed)
        self.algorithm.currentIndexChanged.connect(self._changed);self._changed()
    def settings(self):
        return dict(zip(DETECTOR_KEYS,(self.algorithm.currentData(),self.num_hands.value(),*(x.value() for x in self.confidence),*(x.value() for x in self.rtm_confidence))))
    def _changed(self,*_):
        is_mp=self.algorithm.currentData()=='mediapipe';self.mp_page.setVisible(is_mp);self.rtm_page.setVisible(not is_mp)
        dirty=self.settings()!=self._applied;self.apply_button.setEnabled(dirty and not self._busy)
        if dirty and not self._busy:self.apply_hint.setText('有未应用修改。')
    def set_busy(self,value):
        self._busy=value;self._changed()
        if value:self.apply_hint.setText('正在后台应用检测设置。')
    def mark_applied(self,settings,message):
        self._applied={key:settings[key] for key in DETECTOR_KEYS};self._busy=False;self.apply_hint.setText(message);self._changed()
