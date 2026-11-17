"""Native Studio workspace composition. Processing stays in PipelineWorker."""
import math
from PySide6.QtCore import Qt,QTimer
from PySide6.QtGui import QAction,QKeySequence,QColor,QPainter,QLinearGradient
from PySide6.QtWidgets import QWidget,QFrame,QVBoxLayout,QHBoxLayout,QGridLayout,QLabel,QSplitter,QScrollArea,QTabWidget,QMenu,QToolButton,QComboBox,QSizePolicy,QStackedWidget,QGroupBox,QGraphicsOpacityEffect
from .components import label,badge,button,StatCard,ViewportCard,HandReadout
from .icons import icon
from .theme import THEMES,stylesheet
from .source_panel import SourcePanel
from .recording_panel import RecordingPanel
from .calibration_panel import CalibrationPanel
from .reconstruction_panel import ReconstructionPanel
from .adaptive_views import AdaptiveViews
from ..visualization.image_view import ImageView
from ..visualization.skeleton_view import SkeletonView
from .shared.presentation import session_state

class MotionRail(QWidget):
    """A small, low-cost ambient signal rail controlled by the motion switch."""
    def __init__(self,parent=None):
        super().__init__(parent)
        self.setFixedHeight(6)
        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        self._phase=0.35
        self._active=True
        self._background=QColor("#0E141D")
        self._accent=QColor("#5FE0C5")

    def set_theme(self,tokens):
        self._background=QColor(tokens["canvas"])
        self._accent=QColor(tokens["accent"])
        self.update()

    def set_active(self,active):
        self._active=bool(active)
        self.update()

    def advance(self):
        if not self._active:return
        self._phase=(self._phase+0.012)%1.0
        self.update()

    def paintEvent(self,event):
        painter=QPainter(self)
        painter.fillRect(self.rect(),self._background)
        if not self._active:return
        width=max(1,self.width())
        center=int((self._phase*1.35-0.15)*width)
        gradient=QLinearGradient(center-150,0,center+150,0)
        glow=QColor(self._accent);glow.setAlpha(0)
        edge=QColor(self._accent);edge.setAlpha(90)
        core=QColor(self._accent);core.setAlpha(245)
        gradient.setColorAt(0.0,glow);gradient.setColorAt(0.38,edge)
        gradient.setColorAt(0.5,core);gradient.setColorAt(0.62,edge);gradient.setColorAt(1.0,glow)
        painter.fillRect(self.rect(),gradient)

def build_workspace(w):
    root=QWidget();root.setObjectName("applicationRoot")
    layout=QVBoxLayout(root);layout.setContentsMargins(0,0,0,0);layout.setSpacing(0)
    header=QFrame();header.setObjectName("appHeader")
    h=QHBoxLayout(header);h.setContentsMargins(16,9,16,9);h.setSpacing(12)
    w.brand_mark=QLabel();w.brand_mark.setFixedSize(35,35);h.addWidget(w.brand_mark)
    brand=QVBoxLayout();brand.setSpacing(0);brand.addWidget(label("EgoHand",name="brand"));brand.addWidget(label("STUDIO / 双目手部工作台",name="brandEdition"));h.addLayout(brand)
    h.addStretch()
    w.provenance_badge=badge("合成数据","warning");w.provenance_badge.setToolTip("Synthetic Geometry · 几何验证，不是设备实测")
    w.state_badge=badge("就绪");h.addWidget(w.provenance_badge);h.addWidget(w.state_badge)
    w.settings_button=button("配置",lambda:None,"layers","ghost","展开 / 收起工作配置")
    w.settings_button.setCheckable(True);w.settings_button.setChecked(True);h.addWidget(w.settings_button)
    w.motion_toggle=QToolButton();w.motion_toggle.setText("动效");w.motion_toggle.setCheckable(True);w.motion_toggle.setChecked(True)
    w.motion_toggle.setToolTip("开启 / 关闭品牌呼吸动效");w.motion_toggle.setAccessibleName("界面动效")
    w.motion_toggle.setCursor(Qt.PointingHandCursor);h.addWidget(w.motion_toggle)
    w.theme=QComboBox();w.theme.addItems(["深色","浅色"]);w.theme.setAccessibleName("界面主题");w.theme.setFixedWidth(88)
    h.addWidget(w.theme);layout.addWidget(header)
    w.motion_rail=MotionRail();layout.addWidget(w.motion_rail)
    content=QWidget();v=QVBoxLayout(content);v.setContentsMargins(16,10,16,8);v.setSpacing(10)
    command=QFrame();command.setObjectName("commandBar");c=QHBoxLayout(command);c.setContentsMargins(10,10,10,10);c.setSpacing(8)

    def command_section(title, eyebrow):
        section=QFrame();section.setObjectName("commandSection")
        section_layout=QVBoxLayout(section);section_layout.setContentsMargins(11,7,11,7);section_layout.setSpacing(5)
        heading=QHBoxLayout();heading.setContentsMargins(0,0,0,0);heading.setSpacing(7)
        heading.addWidget(label(eyebrow,name="commandEyebrow"));heading.addWidget(label(title,name="commandTitle"));heading.addStretch()
        section_layout.addLayout(heading)
        row=QHBoxLayout();row.setContentsMargins(0,0,0,0);row.setSpacing(6);section_layout.addLayout(row)
        return section,row

    def command_divider():
        divider=QFrame();divider.setObjectName("commandDivider");divider.setFixedWidth(1);return divider

    w.source_panel=SourcePanel()
    source_section,source_row=command_section("数据源","INPUT")
    w.source_panel.kind.setMinimumWidth(174);source_row.addWidget(w.source_panel.kind)
    w.browse_button=button("打开数据",w.browse_source,"folder","ghost");source_row.addWidget(w.browse_button)
    c.addWidget(source_section,2)
    c.addWidget(command_divider())

    runtime_section,runtime_row=command_section("运行控制","SESSION")
    w.start_button=button("开始采集",w.start_pipeline,"play","primary","开始采集 / 重新处理  ·  Ctrl+Enter");w.start_button.setMinimumWidth(118)
    w.pause_button=button("",w.toggle_pause,"pause",tooltip="暂停 / 继续  ·  Space");w.pause_button.setFixedWidth(42)
    w.stop_button=button("",w.request_pipeline_stop,"stop",tooltip="停止处理");w.stop_button.setFixedWidth(42)
    runtime_row.addWidget(w.start_button);runtime_row.addWidget(w.pause_button);runtime_row.addWidget(w.stop_button)
    c.addWidget(runtime_section,1)
    c.addWidget(command_divider())

    output_section,output_row=command_section("结果输出","OUTPUT")
    w.export_button=QToolButton();w.export_button.setText("导出");w.export_button.setToolButtonStyle(Qt.ToolButtonTextBesideIcon);w.export_button.setIcon(icon("export"));w.export_button.setProperty("icon_name","export")
    w.export_button.setAccessibleName("导出结果和截图");w.export_button.setCursor(Qt.PointingHandCursor);w.export_button.setMinimumHeight(38)
    menu=QMenu(w.export_button)
    menu.addAction("导出当前帧",w.export_current);menu.addAction("导出缓存 / 已有结果",w.export_results)
    w.export_button.setMenu(menu);w.export_button.setPopupMode(QToolButton.InstantPopup);output_row.addWidget(w.export_button)
    w.results_button=button("回看结果",w.open_results,"frame","ghost","打开已保存的结果")
    output_row.addWidget(w.results_button)
    c.addWidget(output_section,1)
    v.addWidget(command)
    metric_strip=QFrame();metric_strip.setObjectName("metricStrip")
    kpis=QHBoxLayout(metric_strip);kpis.setContentsMargins(0,0,0,0);kpis.setSpacing(0)
    w.stat_fps=StatCard("处理帧率","FPS","等待数据流")
    w.stat_joints=StatCard("有效三维关节","/ 42","仅统计双目有效观测")
    w.stat_error=StatCard("平均重投影误差","px","等待几何重建")
    w.stat_latency=StatCard("处理延迟","ms","采集交付 → 处理完成")
    for card in (w.stat_fps,w.stat_joints,w.stat_error,w.stat_latency):kpis.addWidget(card,1)
    v.addWidget(metric_strip)

    outer=QSplitter(Qt.Horizontal);outer.setChildrenCollapsible(False);outer.setHandleWidth(12)
    cameras=QSplitter(Qt.Vertical);cameras.setChildrenCollapsible(False);cameras.setHandleWidth(10)
    w.left_view=ImageView("等待左相机图像");w.right_view=ImageView("等待右相机图像")
    w.left_card=ViewportCard("左相机","LEFT",w.left_view)
    w.right_card=ViewportCard("右相机","RIGHT",w.right_view)
    # 状态文字不应撑大视图最小宽度；窄窗口时完整内容仍可通过提示查看。
    for card in (w.left_card,w.right_card):
        card.meta.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Preferred)
    for view in (w.left_view,w.right_view):
        view.setToolTip('算法预测：相同颜色和编号表示同一条已接受双目轨迹；黄色 unmatched 表示尚未配对。Left / Right 是模型手侧预测，并非配对结果。')
    cameras.addWidget(w.left_card);cameras.addWidget(w.right_card)
    w.skeleton=SkeletonView();w.skeleton.joint_selected.connect(w.show_joint)
    scene=QWidget();scene_row=QVBoxLayout(scene);scene_row.setContentsMargins(0,0,0,0);scene_row.setSpacing(0)
    scene_tools=QHBoxLayout();scene_tools.setContentsMargins(10,2,10,2);scene_tools.setSpacing(8);scene_row.addLayout(scene_tools)
    scene_row.addWidget(w.skeleton,1)
    readout=QWidget();r=QVBoxLayout(readout);r.setContentsMargins(0,0,0,0);r.setSpacing(10)
    r.addWidget(label("手腕坐标","muted"))
    w.left_hand=HandReadout("左手 · LEFT");w.right_hand=HandReadout("右手 · RIGHT")
    r.addWidget(w.left_hand);r.addWidget(w.right_hand);r.addStretch()
    r.addWidget(label("X 向右 · Y 向下 · Z 向前","hint"))
    from .vio_panel import VioPanel
    w.vio_panel=VioPanel();w.vio_panel.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored)
    w.scene_stack=QStackedWidget();w.scene_stack.addWidget(scene);w.scene_stack.addWidget(w.vio_panel)
    w.scene_card=ViewportCard("三维视图","OPTICAL · m",w.scene_stack,"")
    w.scene_card.meta.hide()
    from PySide6.QtWidgets import QCheckBox
    w.show_ground_truth=QCheckBox('显示真值');w.show_ground_truth.setToolTip('紫色小球为官方标注；仅在语义映射已确认时连接骨架。')
    w.show_ground_truth.toggled.connect(w.ground_truth_toggled);scene_tools.addWidget(w.show_ground_truth)
    mano_button=QToolButton();w.mano_button=mano_button;mano_button.setText('MANO');mano_button.setToolTip('预测网格和官方真值网格分别控制；进度与失败原因显示在底部状态栏。')
    mano_menu=QMenu(mano_button)
    w.mano_prediction_action=mano_menu.addAction('显示预测 MANO');w.mano_prediction_action.setCheckable(True)
    w.mano_truth_action=mano_menu.addAction('显示官方真值 MANO');w.mano_truth_action.setCheckable(True)
    w.mano_prediction_action.toggled.connect(w.mano_toggled);w.mano_truth_action.toggled.connect(w.mano_toggled)
    mano_button.setMenu(mano_menu);mano_button.setPopupMode(QToolButton.InstantPopup);scene_tools.addWidget(mano_button)
    w.rotation_center=QComboBox();w.rotation_center.setAccessibleName('三维旋转中心')
    for title,mode in (('围绕相机','camera'),('围绕手部','hand'),('围绕范围中心','range')):
        w.rotation_center.addItem(title,mode)
    w.rotation_center.setToolTip('选择旋转中心：手部、左相机原点，或相机与手部范围的中心。')
    w.rotation_center.currentIndexChanged.connect(lambda _:w.skeleton.set_rotation_center_mode(w.rotation_center.currentData()))
    scene_tools.addWidget(w.rotation_center);scene_tools.addStretch()
    w.scene_selector=QComboBox();w.scene_selector.addItems(['手部重建','相机位姿']);w.scene_selector.setAccessibleName('切换三维视图')
    w.scene_selector.setMinimumWidth(100);w.scene_card.header.addWidget(w.scene_selector)
    def switch_scene(index):
        w.scene_stack.setCurrentIndex(index);w.show_ground_truth.setVisible(index==0);w.mano_button.setVisible(index==0)
        w.joint_info.setVisible(index==0)
        w.scene_card.tag.setText('WORLD · m' if index else 'GT · m' if w.latest and w.latest.diagnostics.get('result_kind')=='official_ground_truth' else 'OPTICAL · m')
    w.scene_selector.currentIndexChanged.connect(switch_scene)
    for title,callback,name,tip in (("",lambda:(w.skeleton if w.scene_stack.currentIndex()==0 else w.vio_panel).zoom_by(.8),"plus","放大三维视图"),("",lambda:(w.skeleton if w.scene_stack.currentIndex()==0 else w.vio_panel).zoom_by(1.25),"minus","缩小三维视图"),("",lambda:w.reset_scene_view() if w.scene_stack.currentIndex()==0 else w.vio_panel.reset_camera(),"reset","重置三维视角")):
        b=button(title,callback,name,"ghost",tip)
        if not title:b.setFixedWidth(36)
        w.scene_card.header.addWidget(b)
    w.joint_info=label("点击关节查看坐标  ·  拖动旋转 / 中键平移 / 滚轮缩放","hint")
    w.joint_info.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Fixed)
    w.joint_info.setFixedHeight(28);w.joint_info.setContentsMargins(14,0,14,0)
    w.scene_card.body.addWidget(w.joint_info)
    w.camera_pose_readout=label('相机 6D 位姿 · 等待实时 VIO')
    w.camera_pose_readout.setWordWrap(True)
    w.camera_pose_readout.setContentsMargins(14,3,14,6)
    w.camera_pose_readout.setTextInteractionFlags(Qt.TextSelectableByMouse)
    w.camera_pose_readout.setToolTip('左相机相对 VIO 世界坐标系的位姿；RPY 为固定轴 XYZ 欧拉角。位姿无效时不显示旧值。')
    w.vio_panel.live_pose_readout.connect(w.camera_pose_readout.setText)
    w.scene_card.body.addWidget(w.camera_pose_readout)
    views=AdaptiveViews(cameras,w.scene_card);w.adaptive_views=views
    layout_choice=QComboBox();layout_choice.addItems(["自动布局","双目在左","双目在上"])
    layout_choice.setAccessibleName("视图区布局");layout_choice.setFixedWidth(106)
    layout_choice.currentIndexChanged.connect(lambda i:views.set_mode(("auto","left","top")[i]))
    w.scene_card.header.addWidget(layout_choice)

    inspector=QFrame();inspector.setObjectName("inspector");inspector.setMinimumWidth(280);inspector.setMaximumWidth(340)
    w.inspector=inspector;w.settings_button.toggled.connect(inspector.setVisible)
    ins=QVBoxLayout(inspector);ins.setContentsMargins(16,16,16,16);ins.setSpacing(12)
    ins.addWidget(label("工作配置",name="sectionTitle"));ins.addWidget(label("按当前数据源调整采集与重建","hint"))
    w.inspector_tabs=QTabWidget();w.inspector_tabs.tabBar().setStyleSheet('QTabBar::tab { padding: 8px 4px; min-width: 24px; }');ins.addWidget(w.inspector_tabs,1)
    def tab(title):
        page=QWidget();col=QVBoxLayout(page);col.setContentsMargins(0,16,0,12);col.setSpacing(12)
        scroll=QScrollArea();scroll.setWidgetResizable(True);scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff);scroll.setWidget(page)
        w.inspector_tabs.addTab(scroll,title)
        return col
    source=tab("采集");source.addWidget(w.source_panel)
    source.addStretch()
    # 录制独立成页：录制面板内容较高，放采集页会和数据源设置挤在一起，窄栏下文字被截断。
    recording=tab("录制")
    w.recording_panel=RecordingPanel();recording.addWidget(w.recording_panel)
    w.recording_panel.start_requested.connect(w.start_recording)
    w.recording_panel.pause_requested.connect(w.pause_recording)
    w.recording_panel.stop_requested.connect(w.stop_recording)
    w.recording_panel.replay_requested.connect(w.replay_recording)
    recording.addStretch()
    detection=tab("检测")
    w.reconstruction_panel=ReconstructionPanel()
    model_button=button("加载 MediaPipe 模型",w.load_model,"model")
    detection.addWidget(model_button)
    w.reconstruction_panel.algorithm.currentIndexChanged.connect(lambda:model_button.setVisible(w.reconstruction_panel.algorithm.currentData()=='mediapipe'))
    w.reconstruction_panel.apply_button.clicked.connect(w.apply_detection_settings)
    w.reconstruction_panel.use_epipolar.toggled.connect(w.epipolar_setting_changed)
    w.reconstruction_panel.use_reprojection.toggled.connect(w.reprojection_setting_changed)
    w.reconstruction_panel.use_3d_tracking.toggled.connect(w.tracking_3d_setting_changed)
    from .roi_panel import install_roi_controls
    from ..core.pipeline import ROOT
    detection.addWidget(w.reconstruction_panel)
    detection.addStretch()
    roi_tab=tab("区域");install_roi_controls(w,roi_tab,ROOT/'configs/roi.yaml');roi_tab.addStretch()
    calibration=tab("标定");w.calibration_panel=CalibrationPanel()
    w.calibration_panel.layout().addWidget(button("加载双目标定",w.load_calibration,"layers"));calibration.addWidget(w.calibration_panel)
    w.vio_calibration_panel=QGroupBox('相机—IMU 标定');vio_cal_layout=QVBoxLayout(w.vio_calibration_panel)
    vio_cal_layout.setContentsMargins(0,12,0,0);vio_cal_layout.setSpacing(10)
    w.vio_calibration_status=label('未加载 · 位姿轨迹需要相机与 IMU 的外参','hint');w.vio_calibration_status.setWordWrap(True)
    w.vio_calibration_status.setTextInteractionFlags(Qt.TextSelectableByMouse);vio_cal_layout.addWidget(w.vio_calibration_status)
    vio_cal_layout.addWidget(button('加载相机—IMU 标定',w.vio_panel.select_calibration,'layers'))
    def show_vio_calibration_status(text):
        w.vio_calibration_status.setText(text)
        w.vio_calibration_status.setToolTip(w.vio_panel.calibration_path or text)
    w.vio_panel.calibration_changed.connect(show_vio_calibration_status)
    calibration.addWidget(w.vio_calibration_panel);calibration.addStretch()
    coordinates=tab("坐标");coordinates.addWidget(w.reconstruction_panel.numbers);coordinates.addWidget(w.reconstruction_panel.epilines);coordinates.addWidget(readout);coordinates.addStretch()
    diagnostic=tab("诊断")
    w.metrics=label("Ready\nModel Not Loaded\n未接入真实相机","mono");w.metrics.setWordWrap(True);w.metrics.setTextInteractionFlags(Qt.TextSelectableByMouse)
    diagnostic.addWidget(w.metrics);diagnostic.addStretch()
    w.input_status=label("二维输入   真值投影（模拟）","hint")
    w.geometry_status=label("三维坐标   左相机光学坐标 · m","hint")
    ins.addWidget(w.input_status);ins.addWidget(w.geometry_status)
    outer.addWidget(views);outer.addWidget(inspector);outer.setSizes([1100,320]);outer.setStretchFactor(0,1);outer.setStretchFactor(1,0)
    v.addWidget(outer,1)
    bottom=QHBoxLayout();bottom.setSpacing(8)
    w.frame_label=label("FRAME  —","mono");w.frame_label.setMinimumWidth(110);bottom.addWidget(w.frame_label)
    w.banner=label("Ready · 选择数据源后开始",name="statusBanner");w.banner.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Fixed);w.banner.setMinimumHeight(30);bottom.addWidget(w.banner,1)
    bottom.addWidget(button("单帧",w.step_frame,"step","ghost"))
    bottom.addWidget(label("帧","muted"));w.source_panel.seek_index.setFixedWidth(92);bottom.addWidget(w.source_panel.seek_index)
    bottom.addWidget(button("定位",w.seek_frame,"seek","ghost"));v.addLayout(bottom)
    layout.addWidget(content,1);w.setCentralWidget(root)
    w.statusBar().showMessage("就绪  ·  EgoHand Studio")
    w.statusBar().addPermanentWidget(label("相机坐标 · 米   |   CPU","hint"))
    motion_effect=QGraphicsOpacityEffect(w.brand_mark);w.brand_mark.setGraphicsEffect(motion_effect)
    w._motion_phase=0.0;w._motion_effect=motion_effect
    w.motion_timer=QTimer(w);w.motion_timer.setInterval(70)
    def animate_motion():
        w._motion_phase=(w._motion_phase+0.16)%(math.tau)
        motion_effect.setOpacity(0.88+0.12*(0.5+0.5*math.sin(w._motion_phase)))
        w.motion_rail.advance()
    w.motion_timer.timeout.connect(animate_motion)
    w.motion_toggle.toggled.connect(lambda enabled:(w.motion_timer.start() if enabled else w.motion_timer.stop(), motion_effect.setOpacity(1.0), w.motion_rail.set_active(enabled)))
    if w.motion_toggle.isChecked():w.motion_timer.start()
    if w.width()<1250:w.settings_button.setChecked(False)
    w.theme.currentIndexChanged.connect(lambda index:apply_theme(w,"light" if index else "dark"))
    w.source_panel.kind.currentIndexChanged.connect(w.source_changed)
    w.source_panel.directory_button.clicked.connect(w.browse_hot3d_directory)
    w.source_panel.hot3d_mode.currentIndexChanged.connect(w.hot3d_mode_changed)
    for shortcut,callback in (("Ctrl+Return",w.start_pipeline),("Space",w.toggle_pause),("Ctrl+O",w.browse_source)):
        action=QAction(w);action.setShortcut(QKeySequence(shortcut));action.triggered.connect(callback);w.addAction(action)
    sync_controls(w)

def apply_theme(w,name):
    w.theme_name=name;t=THEMES[name]
    w.setStyleSheet(stylesheet(name))
    w.brand_mark.setPixmap(icon("logo",t["accent"],35).pixmap(35,35))
    w.motion_rail.set_theme(t)
    for b in w.findChildren(__import__('PySide6.QtWidgets',fromlist=['QAbstractButton']).QAbstractButton):
        key=b.property("icon_name")
        if key:b.setIcon(icon(key,t["on_accent"] if b.property("role")=="primary" else t["text"]))
    w.left_view.set_theme(t);w.right_view.set_theme(t);w.skeleton.set_theme(t);w.vio_panel.set_theme(t)
    w.left_hand.title.setStyleSheet(f"color:{t['left_hand']};font-weight:600;")
    w.right_hand.title.setStyleSheet(f"color:{t['right_hand']};font-weight:600;")

def sync_controls(w):
    status=session_state(w.worker,paused=w.paused,has_result=w.latest is not None)
    running,stopping,active=status.running,status.stopping,status.active
    w.start_button.setText('开始回放' if w.source_panel.kind.currentIndex() in (4,5,6) else '开始采集')
    w.start_button.setEnabled(not stopping)
    w.pause_button.setEnabled(active);w.stop_button.setEnabled(active)
    pending=w.recording_pending
    panel=getattr(w,"recording_panel",None)
    if panel is not None:
        # 数据源未运行时不能开始录制。
        panel.set_available(active)
        panel.set_state(w.recording,w.recording_paused,pending)
    w.export_button.setEnabled(w.latest is not None or bool(w.saved_results))
    w.browse_button.setEnabled(w.source_panel.kind.currentIndex() in (1,2,4,5,6))
    w.state_badge.setText(status.text)
    w.state_badge.setProperty("tone","accent" if active and not w.paused else None)
    w.state_badge.style().unpolish(w.state_badge);w.state_badge.style().polish(w.state_badge)

def present_result(w,result,stats=None,frame=None):
    d=result.diagnostics;synthetic=d.get("synthetic",False)
    w.provenance_badge.setText("合成数据" if synthetic else "真实图像 · 2D" if result.reconstruction_status=="2D Only" else "双目观测")
    w.provenance_badge.setToolTip("Synthetic Geometry · 非设备实测" if synthetic else result.reconstruction_status)
    if d.get('sync_quality')=='USER_CONFIRMED_SAME_BUNDLE':
        w.provenance_badge.setText('双目 · 用户确认同步')
        w.provenance_badge.setToolTip('用户确认设备同步；同组配对，曝光时间差未测量。')
    if stats:
        w.stat_fps.update_value(f"{stats['processing_fps']:.1f}",f"读取 {stats['read_fps']:.1f} FPS  ·  丢帧 {stats['dropped_frames']}")
        w.stat_latency.update_value(f"{stats['latency_ms']:.1f}")
    else:
        w.stat_fps.update_value("—","已有结果回放")
        w.stat_latency.update_value("—","不对回放数据估算采集延迟")
    valid=sum(h.valid_joint_count for h in result.hands)
    total=max(len(result.left_detections),len(result.right_detections))*21
    w.stat_joints.unit.setText(f"/ {total}" if total else "关节")
    w.stat_joints.update_value(str(valid) if result.reconstruction_status!="2D Only" else "—","合成几何验证" if synthetic else "二维模式 · 不发布米制坐标" if result.reconstruction_status=="2D Only" else "有效双目观测")
    error=d.get("mean_reprojection_error_px")
    w.stat_error.update_value(f"{error:.3f}" if error is not None else "—")
    if error is not None and error<1e-3:w.stat_error.description.setText("小于 0.001 px" if not synthetic else "无噪声合成投影")
    else:w.stat_error.description.setText("左右视图平均几何残差")
    w.frame_label.setText(f"FRAME  {result.frame_id:06d}")
    kind=d.get("result_kind")
    w.stat_joints.findChildren(QLabel)[0].setText('官方真值关节' if kind=='official_ground_truth' else '有效三维关节')
    w.scene_card.tag.setText('WORLD · m' if w.scene_stack.currentIndex()==1 else 'GT · m' if kind=='official_ground_truth' else 'OPTICAL · m')
    if kind in ("official_ground_truth","ground_truth_unavailable"):
        w.provenance_badge.setText("官方真值 · GT" if kind=="official_ground_truth" else "官方真值不可用")
        w.stat_joints.setToolTip("官方标注数量，不是算法双目测量结果")
        if kind=='official_ground_truth':w.stat_joints.unit.setText(f'/ {len(result.hands)*20}')
    elif kind=='algorithm_prediction':w.provenance_badge.setText('SHOW3D · 算法预测' if d.get('show3d_sequence_id') else 'HOT3D · 算法预测')
    for name,readout in (("Left",w.left_hand),("Right",w.right_hand)):
        readout.update_hand(next((h for h in result.hands if h.handedness==name),None))
    w.input_status.setText("二维输入   "+("真值投影（模拟）" if synthetic else "外部 / 保存的观测" if d.get("left_detection_origin")=="precomputed" else "MediaPipe · CPU" if d.get("left_detection_origin")=="mediapipe" else "RTMPose · CPU" if d.get("left_detection_origin")=="rtmpose" else "模型未加载"))
    if d.get("result_kind")=="official_ground_truth":
        w.input_status.setText("官方 UmeTrack 真值 · 非算法预测")
        for readout in (w.left_hand,w.right_hand):
            readout.track.setText('官方标注')
            if '—' not in readout.xyz.text():readout.valid.setText('20 点 · 官方真值')
    if d.get("show3d_sequence_id"):
        comparison=d.get("show3d_gt_comparison") or {}
        mpjpe=comparison.get("MPJPE_mm")
        metric_text=(f" · GT MPJPE {mpjpe:.1f} mm ({comparison['compared_joints']} joints)"
                     if mpjpe is not None else " · GT MPJPE N/A" if kind=="algorithm_prediction" else " · Official GT")
        w.geometry_status.setText("SHOW3D headset0 光学坐标 · m"+metric_text)
        w.banner.setToolTip(d["show3d_sequence_id"]+" / "+str(d["show3d_frame_index"])+" · official v3")
    if d.get("hot3d_sequence_id"):
        w.geometry_status.setText("原始左光学坐标 · m")
        w.banner.setToolTip(d["hot3d_sequence_id"]+" / "+d["hot3d_frame_key"])
    if frame:
        w.adaptive_views.set_image_ratio(frame.left_image.shape[1]/frame.left_image.shape[0])
        for card,img in ((w.left_card,frame.left_image),(w.right_card,frame.right_image)):
            text=f"{img.shape[1]} × {img.shape[0]}  ·  "+("已校正" if frame.image_geometry=="rectified" else "原始图像")
            card.meta.setText(text);card.meta.setToolTip(text)
    sync_controls(w)
