"""Customer delivery shell. Processing and presentation remain in egohand shared modules."""
import sys
import os
from pathlib import Path
import time
import json
import subprocess
from collections import OrderedDict,deque
from math import ceil
import numpy as np
import yaml
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QColor, QLinearGradient, QPainter, QVector3D, QDoubleValidator
from PySide6.QtWidgets import (QMainWindow,QWidget,QVBoxLayout,QHBoxLayout,QLabel,QPushButton,
    QComboBox,QFileDialog,QSlider,QSizePolicy,QBoxLayout,QToolButton,QDoubleSpinBox,QFrame,QScrollArea,QMenu,QWidgetAction)
from egohand.core.pipeline import ROOT
from egohand.gui.shared.session import SessionController
from egohand.gui.shared.sources import SourceRequest,create_source
from egohand.gui.shared.presentation import display_stereo,clear_stereo
from egohand.gui.theme import THEMES,stylesheet
from egohand.gui.vio_panel import VioPanel
from egohand.vio.live_runtime import LiveVioWorker
from egohand.vio.live_recovery import LiveRecoveryProfile
from egohand.vio.local_segments import consumer_display_pose as consumer_map_pose,display_reference_key
from egohand.vio.hand_pose_binding import consumer_result_display_pose
from egohand.body.body_pose_estimator import BodyPoseEstimator
from egohand.body.body_model import BodyPose
from egohand.body.smpl_runtime import GroundReference,request_from_result,pose_from_reply
from egohand.body.smpl_mount import HEAD_ANCHOR_VERSION, DEFAULT_CAMERA_FOREHEAD_OFFSET_MM
from egohand.gui.smpl_controller import SmplController
from egohand.storage.replay import load_recorded_results,SavedResultReplay
from egohand.visualization.display_smoothing import DisplaySmoother

sys.path.insert(0,str(Path(__file__).resolve().parent))
from delivery_views import (BodyPoseOverlay,DepthCache,DepthTask,ImageTile,WorldHandOverlay,WorldPanel,DISPLAY_WORLD_ROTATION,
    display_sized,fit_view,set_items_visible)

DELIVERY_ROOT=Path(__file__).resolve().parent
DEFAULT_IMPORT_ROOT=ROOT/'data'
DEFAULT_RECORD_ROOT=DEFAULT_IMPORT_ROOT/'ori'
DEFAULT_RESULT_ROOT=DEFAULT_IMPORT_ROOT/'count'
STYLE_SHEET=stylesheet('dark')+'''
QLabel#tileCaption{color:#E8EDF5;background:rgba(13,17,24,185);border-radius:3px;padding:3px 7px;font-size:12px;}
QWidget#worldOverlay{background:rgba(13,17,24,135);border-radius:4px;}
QLabel#compactTitle{font-size:16px;font-weight:600;} QLabel#compactMeta{color:#A2B0C2;font-size:12px;}
QFrame#controlGroup{background:#151B25;border:1px solid #273140;border-radius:6px;}
QFrame#controlGroup[section="record"]{background:#151F29;}
QFrame#controlGroup[section="compute"]{background:#1A1D29;}
QLabel#groupLabel{color:#8697AD;font-size:12px;padding-right:4px;}
QToolButton#viewToggle{background:transparent;border:1px solid transparent;color:#8697AD;padding:3px 8px;
    font-size:12px;border-radius:6px;} QToolButton#viewToggle:hover{color:#E8EDF5;}
QToolButton#viewToggle:checked{color:#5FE0C5;border-color:#20463F;}
QPushButton{min-height:28px;padding:0 9px;font-weight:400;border-radius:6px;}
QToolButton#displayMenu{padding:0 9px;border-radius:6px;}
QComboBox,QDoubleSpinBox{min-height:24px;padding:1px 8px;border-radius:6px;}
QPushButton#start{border-color:#5FE0C5;color:#5FE0C5;font-weight:600;}'''


class CompactLabel(QLabel):
    """One-line information with full details available on hover."""
    def __init__(self,text='',parent=None):
        super().__init__(parent)
        self._full_text=''
        self.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Preferred)
        self.setMinimumWidth(0)
        self.setText(text)

    def setText(self,text):
        self._full_text=str(text)
        self.setToolTip(self._full_text)
        self._elide()

    def _elide(self):
        super().setText(self.fontMetrics().elidedText(self._full_text,Qt.ElideRight,max(0,self.width())))

    def text(self):
        return self._full_text

    def resizeEvent(self,event):
        super().resizeEvent(event)
        self._elide()


class MotionRail(QWidget):
    """Visible ambient motion for the delivery shell, controlled by one switch."""
    def __init__(self,parent=None):
        super().__init__(parent)
        self.setObjectName('motionRail')
        self.setFixedHeight(6)
        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        self._phase=.35
        self._active=True
        self._background=QColor('#0E141D')
        self._accent=QColor('#5FE0C5')

    def set_theme(self,tokens):
        self._background=QColor(tokens['canvas'])
        self._accent=QColor(tokens['accent'])
        self.update()

    def set_active(self,active):
        self._active=bool(active)
        self.update()

    def advance(self):
        if not self._active:return
        self._phase=(self._phase+.012)%1.0
        self.update()

    def paintEvent(self,event):
        painter=QPainter(self)
        painter.fillRect(self.rect(),self._background)
        if not self._active:return
        width=max(1,self.width())
        center=int((self._phase*1.35-.15)*width)
        gradient=QLinearGradient(center-180,0,center+180,0)
        transparent=QColor(self._accent);transparent.setAlpha(0)
        edge=QColor(self._accent);edge.setAlpha(90)
        core=QColor(self._accent);core.setAlpha(245)
        gradient.setColorAt(0.0,transparent);gradient.setColorAt(.38,edge)
        gradient.setColorAt(.5,core);gradient.setColorAt(.62,edge);gradient.setColorAt(1.0,transparent)
        painter.fillRect(self.rect(),gradient)


class ClientMainWindow(QMainWindow):
    def __init__(self, *, recovery_profile=None, live_recovery_profile=None, replay_output_dir=None):
        super().__init__();self.setWindowTitle('EgoHand DELIVERY · v4 交付 R1 · VS030')
        self.recovery_profile=recovery_profile;self.live_recovery_profile=live_recovery_profile;self.replay_output_dir=replay_output_dir
        if recovery_profile:self.setWindowTitle('EgoHand · '+recovery_profile.label)
        elif live_recovery_profile:self.setWindowTitle('EgoHand · '+live_recovery_profile.label)
        area=self.screen().availableGeometry();self.resize(min(1600,area.width()-40),min(950,area.height()-60));self.setMinimumSize(900,620)
        self.session=SessionController(self);self.latest=None;self.recording=False;self.pending_record=False
        self.paused=False;self._paused_vio_reset=None;self._paused_vio_world=None
        # 词典只在 GUI 生命周期内加载一次：把常驻 VIO worker 交给每个会话复用。
        # 不传 worker 时 LiveVio/ReplayVio 会自建子进程并在结束时销毁，
        # 每次开始都要重新读取 145 MB 的 ORBvoc.txt。
        self.live_vio_worker=LiveVioWorker()
        self.recording_path=None;self.recording_source=None;self.saved=[];self.generation=0;self.depth_task=None;self.depth_at=0
        self.last_frame_at=0;self.current_frame=None;self.last_status='未连接';self._closing=False
        self._pose_fitted=False;self._world_pose=None;self._has_native=None
        # Bounded same-session display join; never use the newest pose for a different hand frame.
        self._hand_poses=OrderedDict();self._hand_pose_world=None
        self._displayed_hand_stamp=None;self._displayed_hands=None
        self._displayed_hand_has_pose=False
        self._live_hand_result=None;self._live_hand_age_ms=None;self._live_hand_pose=None
        self.body_estimator=BodyPoseEstimator();self.body_pose=None;self._body_status='WAITING_HEAD'
        self.ground_reference=GroundReference();self._ground_override=False
        self.smpl_controller=SmplController(self);self._smpl_request_key=None;self._smpl_request_id=None
        self.smpl_controller.completed.connect(self._on_smpl_ready)
        self.smpl_controller.failed.connect(self._on_smpl_failed)
        self._saved_body_advance=None
        self._vio_pose_times=deque(maxlen=40);self._last_vio_message=None;self._last_vio_latency_ms=None
        self.saved_replay=None
        self._saved_depth_loaded=False
        self._saved_rectifier=None
        self._saved_last_index=None
        self.result_output_parent=None
        self.record_output_parent=DEFAULT_RECORD_ROOT
        self._computed_output=None;self._computed_options=None;self._cache_jobs=[]
        self._recording_mesh_pending=None
        self.saved_timer=QTimer(self);self.saved_timer.setSingleShot(True);self.saved_timer.timeout.connect(self._advance_saved)
        self.saved_timer.setTimerType(Qt.PreciseTimer)
        self.smoother=DisplaySmoother(yaml.safe_load((ROOT/'configs/app.yaml').read_text(encoding='utf8')).get('display_smoothing'))
        self.image_aspect=4/3
        self.depth_cache=DepthCache()
        self._pending_saved_depth=None
        self._saved_depth_advance=None
        self._raw_cache=None;self._raw_shown=None
        self._build()
        self._paint_latest_pair=os.environ.get('EGOHAND_GUI_PAINT_LATEST_PAIR','1')=='1'
        self.preview.before_paint=self._prepare_live_hand_paint
        self.session.ready.connect(self.on_ready);self.session.error.connect(self.show_error)
        self.session.startup_status.connect(self.set_status);self.session.vio_status.connect(self.on_vio)
        self.session.recording_state_changed.connect(self.on_recording);self.session.finished.connect(self.on_finished)
        self.session.localized_hand_result.connect(self.on_localized_hands)
        self.timer=QTimer(self);self.timer.timeout.connect(self.poll);self.timer.start(40)
        self.cache_timer=QTimer(self);self.cache_timer.timeout.connect(self._poll_cache_jobs);self.cache_timer.start(1000)
        self.motion_timer=QTimer(self);self.motion_timer.setInterval(70);self.motion_timer.timeout.connect(self.motion_rail.advance);self.motion_timer.start()
        self.motion_toggle.toggled.connect(self._set_motion_enabled)
        self.setStyleSheet(STYLE_SHEET)
        for widget in (self.source,self.open_button,self.output_button,self.start_button,self.compute_button,
                       self.results_button,self.pause_button,self.stop_button,self.record_button,self.capture_button,
                       self.record_output_button,self.trim_button,self.view_menu_button,self.hand_filter_toggle,
                       self.body_gate_toggle,self.ground_adjust,self.camera_forehead_offset):
            widget.setFixedHeight(32)
        bar_height=max(self.record_controls.sizeHint().height(),self.compute_controls.sizeHint().height())
        self.record_controls.setFixedHeight(bar_height);self.compute_controls.setFixedHeight(bar_height)
        self._toolbar_wide=None;self._layout_toolbar()
        for view in (self.left_view,self.right_view,self.depth_view,self.raw_left_view,self.raw_right_view):
            view.set_theme(THEMES['dark'])
        self.vio_panel.set_theme(THEMES['dark'])
        self.motion_rail.set_theme(THEMES['dark'])
        self.sync_controls()

    def _set_motion_enabled(self,enabled):
        self.motion_rail.set_active(enabled)
        if enabled:self.motion_timer.start()
        else:self.motion_timer.stop()

    # ---------- 布局 ----------
    def _build(self):
        root=QWidget();root.setObjectName('applicationRoot');self.setCentralWidget(root)
        layout=QVBoxLayout(root);layout.setContentsMargins(8,5,8,5);layout.setSpacing(4)
        head=QHBoxLayout();head.setSpacing(8);self.header_layout=head
        title=QLabel('EgoHand');title.setObjectName('compactTitle');head.addWidget(title)
        edition=QLabel('v4 · R1');edition.setObjectName('compactMeta');head.addWidget(edition)
        head.addStretch()
        self.state_label=CompactLabel('未连接');self.state_label.setObjectName('compactMeta');self.state_label.setAlignment(Qt.AlignRight|Qt.AlignVCenter);head.addWidget(self.state_label,1)
        self.motion_toggle=QToolButton();self.motion_toggle.setText('动效');self.motion_toggle.setCheckable(True);self.motion_toggle.setChecked(True)
        self.motion_toggle.setAccessibleName('界面动效');self.motion_toggle.setToolTip('开启 / 关闭顶部动态信号轨道');self.motion_toggle.setCursor(Qt.PointingHandCursor)
        head.addWidget(self.motion_toggle)
        layout.addLayout(head)
        self.motion_rail=MotionRail();layout.addWidget(self.motion_rail)
        self.toolbar_scroll=QScrollArea()
        self.toolbar_scroll.setAccessibleName('操作栏')
        self.toolbar_scroll.setFrameShape(QFrame.NoFrame)
        self.toolbar_scroll.setWidgetResizable(True)
        self.toolbar_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.toolbar_scroll.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Fixed)
        self.command_strip=QWidget()
        toolbar=QHBoxLayout(self.command_strip);toolbar.setContentsMargins(0,0,0,0);toolbar.setSpacing(0)
        self.compute_controls,selection=self._control_group('计算')
        self.compute_controls.setProperty('section','compute')
        self.record_controls,files=self._control_group('录制')
        self.record_controls.setProperty('section','record')
        self.source=QComboBox();self.source.setAccessibleName('数据源');self.source.addItem('实时 · VS030','vendor');self.source.addItem('录制回放','recorded')
        # Customer UI always uses the delivered v4 profile. Legacy algorithms
        # remain available in the research GUI, not in this source selector.
        self.mode=QComboBox(root);self.mode.setAccessibleName('定位算法');self.mode.hide()
        self.mode.addItem('v4 · 录制','offline_recovery_v4')
        self.mode.addItem('v4 · 实时','live_recovery_v4')
        self.mode.setCurrentIndex(self.mode.findData('live_recovery_v4'))
        self.source.currentIndexChanged.connect(self._source_mode_changed)
        if self.recovery_profile:
            self.source.clear();self.source.addItem('录制回放 · 离线候选','recorded')
            self.mode.clear();self.mode.addItem('双目＋IMU · 恢复候选 v4','stereo_inertial')
        elif self.live_recovery_profile:
            self.mode.clear();self.mode.addItem('双目＋IMU · v4 实时恢复 · 交付 R1','stereo_inertial')
        self.mode.setToolTip('混合模式只运行一个IMU_STEREO实例；允许视觉先建图，IMU初始化并通过门控后优先使用VIO。定位精度仍待验收，仅停止后切换。')
        self.source.setMinimumWidth(132);self.source.setMaximumWidth(190)
        files.addWidget(self.source)
        self.open_button=self._button('打开录制',self.import_recording);selection.addWidget(self.open_button)
        self.results_button=self._button('查看计算结果',self.open_results)
        self.output_button=self._button('结果目录',self.choose_result_output)
        self.output_button.setToolTip('选择离线计算结果的父目录；每次计算在其中创建新文件夹，不覆盖录制')
        selection.addWidget(self.output_button)
        self.start_button=self._button('连接',self.start_pipeline);self.start_button.setObjectName('start');files.addWidget(self.start_button)
        self.compute_button=self._button('计算',self.start_computation);self.compute_button.setObjectName('start');selection.addWidget(self.compute_button)
        selection.addWidget(self.results_button)
        self.pause_button=self._button('暂停',self.toggle_pause)
        self.pause_button.setToolTip('回放暂停在当前帧；实时模式暂停画面和手部处理，采集、定位和正在进行的原始录制在后台继续。')
        files.addWidget(self.pause_button)
        self.stop_button=self._button('停止',self.stop_pipeline);files.addWidget(self.stop_button)
        self.record_button=self._button('录制',self.toggle_recording);files.addWidget(self.record_button)
        self.capture_button=self._button('截图',self.save_screenshot);files.addWidget(self.capture_button)
        self.record_output_button=self._button('录制位置',self.choose_record_output);files.addWidget(self.record_output_button)
        self.trim_button=self._button('截取录制',self.trim_recording);files.addWidget(self.trim_button)
        self.record_output_button.setToolTip('录制位置：'+str(self.record_output_parent))
        self.toolbar_scroll.setWidget(self.command_strip)
        layout.addWidget(self.toolbar_scroll)
        self.source_label=CompactLabel('实时 · VS030');self.source_label.setObjectName('compactMeta');head.insertWidget(2,self.source_label,1)
        self.output_label=QLabel('结果位置：默认 '+str(self.replay_output_dir or DEFAULT_RESULT_ROOT))
        self.output_label.setObjectName('compactMeta');self.output_label.setWordWrap(True);self.output_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.output_label.hide()
        self.output_button.setToolTip(self.output_label.text())
        if self.recovery_profile:
            self.profile_label=QLabel(self.recovery_profile.label+' · 仅处理离线录制')
            self.profile_label.hide();title.setToolTip(self.recovery_profile.label)
        elif self.live_recovery_profile:
            self.profile_label=QLabel(self.live_recovery_profile.label+' · 全分辨率 · 实机精度待验收')
            self.profile_label.hide();title.setToolTip(self.live_recovery_profile.label)

        # 三列：左 3D（吸收多余空间）、中 深度/校正左/校正右（单列）、右 原始左右。
        self.columns=QWidget();self.column_layout=QHBoxLayout(self.columns)
        self.column_layout.setContentsMargins(0,0,0,0);self.column_layout.setSpacing(0)
        self.world_column=self._world_panel()
        # Processing options change results; visibility controls only change
        # presentation. Keep them in different functional zones.
        selection.addWidget(self.hand_filter_toggle);selection.addWidget(self.body_gate_toggle)
        selection.addWidget(self.ground_controls)
        self.camera_mount_controls,mount_layout=self._control_group('相机至额头')
        self.camera_forehead_offset=QDoubleSpinBox()
        self.camera_forehead_offset.setAccessibleName('相机到额头垂直距离')
        self.camera_forehead_offset.setRange(0.,100.);self.camera_forehead_offset.setDecimals(1)
        self.camera_forehead_offset.setSingleStep(.5);self.camera_forehead_offset.setSuffix(' mm')
        self.camera_forehead_offset.setValue(DEFAULT_CAMERA_FOREHEAD_OFFSET_MM)
        self.camera_forehead_offset.setFixedWidth(92);self.camera_forehead_offset.setKeyboardTracking(False)
        self.camera_forehead_offset.setToolTip('相机光心在额头上方的安装距离。只影响下一次离线计算，已有结果不变。')
        mount_layout.addWidget(self.camera_forehead_offset)
        selection.insertWidget(selection.indexOf(self.results_button)+1,self.camera_mount_controls)
        self.view_menu=QMenu(self);self.view_action=QWidgetAction(self.view_menu)
        self.view_action.setDefaultWidget(self.view_controls);self.view_menu.addAction(self.view_action)
        self.view_menu_button=QToolButton();self.view_menu_button.setText('显示')
        self.view_menu_button.setObjectName('displayMenu')
        self.view_menu_button.setAccessibleName('显示设置');self.view_menu_button.setMenu(self.view_menu)
        self.view_menu_button.setPopupMode(QToolButton.InstantPopup);files.addWidget(self.view_menu_button)
        self.command_groups=(self.compute_controls,self.record_controls)
        for group in self.command_groups:toolbar.addWidget(group,0,Qt.AlignVCenter)
        toolbar.addStretch()
        self._layout_toolbar()
        self.middle_column=self._middle_column()
        self.right_column=self._right_column()
        self.column_layout.addWidget(self.world_column,1)
        self.column_layout.addWidget(self.middle_column,0)
        self.column_layout.addWidget(self.right_column,0)
        self.column_widgets=[self.world_column,self.middle_column,self.right_column]
        layout.addWidget(self.columns,1)

        self.saved_controls=QWidget();saved_bar=QHBoxLayout(self.saved_controls);saved_bar.setContentsMargins(0,0,0,0)
        self.saved_play=self._button('播放',self.toggle_saved_playback);saved_bar.addWidget(self.saved_play)
        self.saved_speed=QComboBox();self.saved_speed.setAccessibleName('结果回放倍速');self.saved_speed.setEditable(True)
        self.saved_speed.setInsertPolicy(QComboBox.NoInsert);self.saved_speed.setFixedWidth(82)
        self.saved_speed.setToolTip('可输入 0.05–32 倍；所有档位逐帧播放。「最快」以本机可显示的速度播放。')
        for speed in (.25,.5,1.,1.5,2.,4.,8.,16.,32.):self.saved_speed.addItem(f'{speed:g}×',speed)
        self.saved_speed.addItem('最快')
        self.saved_speed.setCurrentIndex(self.saved_speed.findData(1.))
        validator=QDoubleValidator(.05,32.,2,self);validator.setNotation(QDoubleValidator.StandardNotation)
        self.saved_speed.lineEdit().setValidator(validator);self.saved_speed.lineEdit().editingFinished.connect(self._normalize_saved_speed)
        self.saved_speed.activated.connect(self._saved_speed_changed)
        saved_bar.addWidget(self.saved_speed)
        self.result_slider=QSlider(Qt.Horizontal);self.result_slider.setAccessibleName('已有结果帧')
        # Decoder seeks can require replaying HEVC from the start. Do not
        # launch a seek for each mouse movement during a drag.
        self.result_slider.setTracking(False)
        self.result_slider.sliderPressed.connect(self._saved_seek_started)
        self.result_slider.sliderMoved.connect(self._saved_seek_preview)
        self.result_slider.sliderReleased.connect(self._saved_seek_finished)
        self.result_slider.valueChanged.connect(self.display_saved);saved_bar.addWidget(self.result_slider,1)
        self.saved_controls.hide()
        self.footer=QLabel('帧 —   ·   采集 — FPS   ·   手部 — FPS   ·   VIO — FPS');self.footer.setObjectName('compactMeta');self.footer.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Preferred);layout.addWidget(self.footer)
        layout.removeWidget(self.footer)
        bottom=QHBoxLayout();bottom.setContentsMargins(0,0,0,0);bottom.setSpacing(8)
        bottom.addWidget(self.saved_controls,1);bottom.addWidget(self.footer,1)
        self.footer.setAlignment(Qt.AlignRight|Qt.AlignVCenter)
        self.footer.setMinimumWidth(300)
        self.body_meta.setSizePolicy(QSizePolicy.Maximum,QSizePolicy.Preferred)
        bottom.addWidget(self.body_meta)
        layout.addLayout(bottom)
        self.statusBar().hide()
        self.statusBar().messageChanged.connect(self.state_label.setText)

    def _control_group(self,title):
        group=QFrame();group.setObjectName('controlGroup')
        group.setSizePolicy(QSizePolicy.Maximum,QSizePolicy.Fixed)
        row=QHBoxLayout(group);row.setContentsMargins(6,3,6,3);row.setSpacing(4)
        label=QLabel(title);label.setObjectName('groupLabel');label.setSizePolicy(QSizePolicy.Maximum,QSizePolicy.Preferred);row.addWidget(label)
        return group,row

    def _world_panel(self):
        """左列：一个三维视图，同时显示相机位姿、轨迹与手部位姿。"""
        panel=WorldPanel();box=QVBoxLayout(panel);box.setContentsMargins(0,0,0,0);box.setSpacing(0)
        self.view_controls,bar=self._control_group('显示')
        self.trajectory_toggle=self._toggle('轨迹',True,'显示/隐藏相机轨迹线')
        self.camera_toggle=self._toggle('相机',True,'显示/隐藏当前相机坐标轴')
        self.hands_toggle=self._toggle('手部',True,'显示/隐藏世界系手部骨架')
        self.body_gate_toggle=self._toggle('严格人体',False,'人体严格模式：头部、左手、右手必须是同帧可信观测')
        self.body_mesh_toggle=self._toggle('人体网格',True,'显示人体表面网格')
        self.hand_mesh_toggle=self._toggle('手部网格',True,'显示 MANO 手部网格')
        # The legacy orange sparse skeleton is no longer part of the customer
        # view; SMPL is the sole body representation. Keep the control hidden
        # for compatibility with existing signal wiring.
        self.body_skeleton_toggle=self._toggle('人体骨架',False,'SMPL 官方骨架，与人体网格使用同一姿态')
        self.hand_skeleton_toggle=self._toggle('手部骨架',True,'显示手部骨架线和关节')
        self.body_toggle=self.body_mesh_toggle
        self.mano_toggle=self.hand_mesh_toggle
        self.hand_filter_toggle=self._toggle('手部滤波',True,'平滑三维关节并写入后续录制结果；快速运动精度仍待验证')
        self.hand_filter_toggle.toggled.connect(lambda enabled:self.session.command('hand_filter',enabled))
        self.fit_button=QToolButton();self.fit_button.setText('复位');self.fit_button.setObjectName('viewToggle')
        self.fit_button.setToolTip('把视角重新套到轨迹与手部的包围盒')
        self.fit_button.clicked.connect(self.reset_world_view)
        for widget in (self.trajectory_toggle,self.camera_toggle,self.body_mesh_toggle,self.hand_mesh_toggle,self.body_skeleton_toggle,self.hand_skeleton_toggle,self.fit_button):bar.addWidget(widget)
        self.pivot_choice=QComboBox();self.pivot_choice.setAccessibleName('三维旋转基准点')
        for title,key in (('自由视角','free'),('相机','camera'),('世界原点','world'),('手部中心','hands'),('人体中心','body')):
            self.pivot_choice.addItem(title,key)
        self.pivot_choice.setToolTip('选择三维旋转基准；目标不可用时保留当前视角')
        self.pivot_choice.currentIndexChanged.connect(self._update_view_pivot)
        bar.addWidget(self.pivot_choice)
        self.ground_adjust=QDoubleSpinBox();self.ground_adjust.setAccessibleName('初始化相机离地高度')
        self.ground_adjust.setDecimals(1);self.ground_adjust.setRange(30.,210.);self.ground_adjust.setSingleStep(1.)
        self.ground_adjust.setValue(self.ground_reference.height_m*100.)
        self.ground_adjust.setFixedWidth(110)
        self.ground_adjust.setSuffix(' cm');self.ground_adjust.setToolTip('初始额头相机距地高度；默认按身高 175 cm 的直立 SMPL 模型估计。坐姿或下蹲开始时可调整。')
        self.ground_adjust.setKeyboardTracking(True)
        self.ground_adjust.valueChanged.connect(self._ground_adjusted)
        self.ground_slider=QSlider(Qt.Horizontal);self.ground_slider.setAccessibleName('初始化相机离地高度滑块')
        self.ground_slider.setRange(300,2100);self.ground_slider.setValue(round(self.ground_reference.height_m*1000))
        self.ground_slider.setTracking(True);self.ground_slider.setToolTip('调整初始相机离地高度，并重新求解人体姿态')
        self.ground_slider.valueChanged.connect(lambda value:self.ground_adjust.setValue(value/10.))
        self.world_meta=CompactLabel('等待定位');self.world_meta.setObjectName('compactMeta')
        self.world_meta.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Preferred)
        self.body_meta=QLabel('人体·等待头');self.body_meta.setObjectName('compactMeta')
        self.vio_panel=VioPanel()
        # 复用世界身份、质量门控与轨迹更新逻辑，只保留三维视图本身。
        for name in ('slider','details','raw_container','filtered_title','filtered_status','fit_comparison','coordinate_legend'):
            widget=getattr(self.vio_panel,name,None)
            if widget is not None:widget.hide()
        for name in ('live_enabled','live_profile','show_estimate','show_truth','compare_native','compare_map','update_map'):
            widget=getattr(self.vio_panel,name,None)
            if widget is not None:widget.hide()
        for widget in self.vio_panel.findChildren(QPushButton):widget.hide()
        bar.addWidget(self.vio_panel.history_button)
        self.vio_panel.history_button.show()
        inner=self.vio_panel.layout()
        inner.setContentsMargins(0,0,0,0);inner.setSpacing(0)
        self.vio_panel.comparison_split.setHandleWidth(0)
        self.preview=self.vio_panel.preview
        self.hand_overlay=WorldHandOverlay(self.preview)
        self.body_overlay=BodyPoseOverlay(self.preview)
        box.addWidget(self.vio_panel,1)
        self.ground_controls,ground_controls=self._control_group('初始离地')
        ground_controls.addWidget(self.ground_adjust)
        self.ground_slider.setFixedWidth(80);ground_controls.addWidget(self.ground_slider)
        self.world_meta.hide()
        self.track_legend=QLabel('<span style="color:#33DEBD">━ 定位</span>　'
            '<span style="color:#60A5FA">┄ 预测</span>　<span style="color:#F472B6">┄ 候选</span>')
        self.track_legend.setToolTip('定位：已准入轨迹；预测：短时 IMU 传播；候选：尚未验证的视觉结果。预测与候选均不代表可信定位。')
        panel.bar.addWidget(self.track_legend)
        panel.place_overlay()
        self.trajectory_toggle.setToolTip('显示轨迹：绿色为已准入定位，蓝色为 IMU 预测，粉色为未验证视觉候选。')
        self.view_controls.setToolTip('定位：绿色；IMU 预测：蓝色；视觉候选：粉色。预测和候选均未验证。')
        return panel

    def _middle_column(self):
        """中列（小）：原始鱼眼左、原始鱼眼右、深度图，纵向一列。"""
        column=QWidget();box=QVBoxLayout(column);box.setContentsMargins(0,0,0,0);box.setSpacing(0)
        self.raw_left_tile=ImageTile('等待原始左图');self.raw_left_view=self.raw_left_tile.view
        self.raw_right_tile=ImageTile('等待原始右图');self.raw_right_view=self.raw_right_tile.view
        self.raw_left_tile.set_caption('原始 · 左')
        self.raw_right_tile.set_caption('原始 · 右')
        self.depth_tile=ImageTile('等待双目深度');self.depth_view=self.depth_tile.view
        self.depth_tile.set_caption('深度 · 全部有效值')
        for tile in (self.raw_left_tile,self.raw_right_tile,self.depth_tile):box.addWidget(tile)
        box.addStretch(1)
        return column

    def _right_column(self):
        """右列（大）：校正后的左右图及其手部识别结果。"""
        column=QWidget();box=QVBoxLayout(column);box.setContentsMargins(0,0,0,0);box.setSpacing(0)
        self.left_tile=ImageTile('等待左相机');self.left_view=self.left_tile.view
        self.right_tile=ImageTile('等待右相机');self.right_view=self.right_tile.view
        self.left_tile.set_caption('校正 · 左')
        self.right_tile.set_caption('校正 · 右')
        box.addWidget(self.left_tile);box.addWidget(self.right_tile);box.addStretch(1)
        return column

    def _button(self,text,callback):
        b=QPushButton(text);b.setAccessibleName(text);b.setSizePolicy(QSizePolicy.Maximum,QSizePolicy.Fixed);b.clicked.connect(callback);return b

    def _toggle(self,text,checked,tip):
        b=QToolButton();b.setText(text);b.setObjectName('viewToggle');b.setCheckable(True);b.setChecked(checked)
        b.setToolTip(tip);b.toggled.connect(lambda _=False:self._apply_view_toggles())
        return b

    # ---------- 显示开关 ----------
    def _apply_view_toggles(self):
        """三个开关互相独立：轨迹只影响轨迹线，相机只影响坐标轴。"""
        set_items_visible(self.preview.trajectory_items,self.trajectory_toggle.isChecked())
        set_items_visible(self.preview.axis_items,self.camera_toggle.isChecked())
        self.hand_overlay.set_visible(self.hands_toggle.isChecked() and self.hand_skeleton_toggle.isChecked())
        self.body_overlay.set_visible(self.body_mesh_toggle.isChecked() or self.body_skeleton_toggle.isChecked())
        self.hand_overlay.set_mesh_visible(self.hand_mesh_toggle.isChecked())
        self.hand_overlay.set_skeleton_visible(self.hand_skeleton_toggle.isChecked())
        self.body_overlay.set_mesh_visible(self.body_mesh_toggle.isChecked())
        self.body_overlay.set_skeleton_visible(self.body_skeleton_toggle.isChecked())
        self.preview.update()

    def reset_world_view(self):
        points=[pose.T_world_from_camera_left[:3,3] for pose in self.vio_panel.poses]
        for item in self.hand_overlay.items:
            try:points.extend(np.asarray(item.pos))
            except (AttributeError,TypeError):pass
        for item in self.body_overlay.items:
            try:points.extend(np.asarray(item.pos))
            except (AttributeError,TypeError):pass
        self._pose_fitted=False
        self.preview.fit_geometry(points)
        self._update_view_pivot()

    def _update_view_pivot(self):
        key=self.pivot_choice.currentData()
        point=None
        if key=='world':point=np.zeros(3)
        elif key=='camera' and self._world_pose is not None:point=np.asarray(self._world_pose)[:3,3]
        elif key=='body' and self.body_pose is not None:
            joints=self.body_pose.joints
            values=[np.asarray(joints[k],dtype=float) for k in ('Pelvis','Chest') if k in joints]
            if values:point=np.mean(values,axis=0)
        elif key=='hands' and self._world_pose is not None:
            hands=self._displayed_hands or []
            values=[np.asarray(j.xyz_m,dtype=float) for h in hands for j in h.joints if j.valid and np.isfinite(j.xyz_m).all()]
            if values:
                transform=np.asarray(self._world_pose);point=transform[:3,:3]@np.mean(values,axis=0)+transform[:3,3]
        if point is not None and np.asarray(point).shape==(3,) and np.isfinite(point).all():
            display=DISPLAY_WORLD_ROTATION@point
            self.preview.opts['center']=QVector3D(*map(float,display));self.preview.update()

    # ---------- 状态与控制 ----------
    def set_status(self,text):
        if self.paused and not text.startswith('已暂停'):
            text='已暂停 · '+text if text.startswith('提示：') else '已暂停'
        self.last_status=text;self.state_label.setText(text);self.statusBar().showMessage(text)

    def show_error(self,text):
        self.last_error=text;self.set_status('提示：'+text)

    def _customer_vio_status(self,message):
        """Translate estimator states into short customer-facing guidance."""
        kind=message.get('type')
        if kind=='error':
            detail=str(message.get('message','定位异常'))
            return ('设备异常 · 请检查相机或 IMU 连接' if any(word in detail for word in ('VS030','相机','IMU','SDK'))
                    else '定位不可用 · '+detail)
        if kind in ('status','session_phase'):
            detail=str(message.get('message',''))
            if any(word in detail for word in ('等待双目','时序核验','初始化','加载 ORB')):
                return '定位初始化'
            return detail or '定位初始化'
        if kind=='vio_reset':
            return '定位恢复中'
        if kind!='pose':
            return None
        pre=message.get('recovery_preaccept') or {}
        if message.get('automatic_new_map'):
            return '新地图初始化'
        prediction=pre.get('loss_prediction') or {}
        if prediction.get('status')=='predicted':
            return 'IMU 预测 · 未验证'
        localization=message.get('localization') or {}
        state=localization.get('state')
        if pre.get('await_visual_requalification') or state in ('DEGRADED','VISUAL_INITIALIZING'):
            return '视觉失跟 · 恢复中'
        if localization.get('admission',{}).get('map_pose_available'):
            return '定位正常'
        if state in ('INERTIAL_INITIALIZING','INERTIAL_REFINING','WAITING_FOR_SENSORS'):
            return '定位初始化'
        return '定位不可用'

    def sync_controls(self):
        running=bool(self.session.worker and self.session.worker.isRunning())
        for w in (self.source,self.mode,self.open_button,self.results_button,self.output_button,self.start_button,self.compute_button,self.record_output_button,self.trim_button):w.setEnabled(not running)
        for w in (self.hand_filter_toggle,self.body_gate_toggle,self.ground_adjust,self.ground_slider,self.camera_forehead_offset):w.setEnabled(not running)
        if self.recovery_profile:
            self.source.setEnabled(False);self.mode.setEnabled(False)
        elif self.live_recovery_profile:
            self.mode.setEnabled(False)
        self.stop_button.setEnabled(running);self.record_button.setEnabled(running and self.source.currentData()!='recorded' and not self.pending_record)
        self.trim_button.setEnabled(not self.recording and not self.pending_record)
        self.pause_button.setEnabled(running and not self.session.worker.stop_event.is_set())
        self.pause_button.setText('恢复' if self.paused else '暂停')
        self.pause_button.setAccessibleName('恢复' if self.paused else '暂停')
        self.record_button.setText('结束录制' if self.recording else '录制')
        self.start_button.setEnabled(not running and self.source.currentData()!='recorded')

    def start_computation(self):
        if self.source.findData('recorded')<0:
            self.show_error('当前入口不支持离线计算');return
        self.source.setCurrentIndex(self.source.findData('recorded'))
        self.start_pipeline()

    def import_recording(self):
        path=QFileDialog.getExistingDirectory(self,'选择项目原生录制目录',str(DEFAULT_IMPORT_ROOT))
        if path:self.select_recording(path)

    def choose_result_output(self):
        (self.result_output_parent or DEFAULT_RESULT_ROOT).mkdir(parents=True,exist_ok=True)
        path=QFileDialog.getExistingDirectory(self,'选择计算结果的父目录',str(self.result_output_parent or DEFAULT_RESULT_ROOT))
        if path:
            self.result_output_parent=Path(path)
            self.output_label.setText('结果位置：'+str(self.result_output_parent)+'（每次计算创建新文件夹）')
            self.output_label.setToolTip(str(self.result_output_parent))
            self.output_button.setToolTip('结果目录：'+str(self.result_output_parent))
            self.statusBar().showMessage('计算结果将写入：'+str(self.result_output_parent)+' 下的新目录')

    def choose_record_output(self):
        self.record_output_parent.mkdir(parents=True,exist_ok=True)
        path=QFileDialog.getExistingDirectory(self,'选择录制的父目录',str(self.record_output_parent))
        if path:
            self.record_output_parent=Path(path)
            self.record_output_button.setToolTip('录制位置：'+str(self.record_output_parent))
            self.statusBar().showMessage('录制将写入：'+str(self.record_output_parent)+' 下的新目录')

    def trim_recording(self):
        initial=self.recording_path or self.recording_source or self.record_output_parent
        selected=QFileDialog.getExistingDirectory(self,'选择要截取的 VS030 录制',str(initial))
        if not selected:return
        path=Path(selected)
        from recording_trim import RecordingTrimDialog
        dialog=RecordingTrimDialog(path,self)
        if dialog.exec() and getattr(dialog,'saved_path',None):
            self.recording_source=dialog.saved_path
            self.statusBar().showMessage('截取录制已保存：'+str(dialog.saved_path))

    def _output_note(self,text):
        self.output_label.setText(text)
        self.output_button.setToolTip(text)

    def _source_mode_changed(self):
        if self.recovery_profile or self.live_recovery_profile:return
        current=self.mode.currentData()
        if current not in ('live_recovery_v4','offline_recovery_v4'):return
        selected='offline_recovery_v4' if self.source.currentData()=='recorded' else 'live_recovery_v4'
        self.mode.setCurrentIndex(self.mode.findData(selected))
        self.source_label.setText(('录制 · '+self.recording_source.name if self.recording_source else '打开录制后计算')
            if self.source.currentData()=='recorded' else '实时 · VS030')
        self.sync_controls()

    def enable_demo(self):
        """Explicit CLI/test entry only; synthetic data is absent from customer sources."""
        if self.source.findData('synthetic')<0:self.source.addItem('合成演示 · 非实测','synthetic')
        self.source.setCurrentIndex(self.source.findData('synthetic'))

    def select_recording(self,path):
        self.recording_source=Path(path);self.source.setCurrentIndex(self.source.findData('recorded'));self.source_label.setText('录制：'+str(path)+(' · 全分辨率恢复候选' if self.recovery_profile else ' · 按录制快照重新计算'))
        if not self.recovery_profile and not self.live_recovery_profile:
            self.mode.setCurrentIndex(self.mode.findData('offline_recovery_v4'))
            self.source_label.setText('录制 · '+Path(path).name)
            self.source_label.setToolTip(str(path))

    def start_pipeline(self):
        if not self.stop_pipeline():return
        try:
            kind=self.source.currentData();self.last_error=None
            if self.recovery_profile and (kind!='recorded' or self.mode.currentData()!='stereo_inertial'):
                raise ValueError('离线恢复候选仅支持双目＋IMU录制回放')
            if self.live_recovery_profile and kind!='vendor':
                raise ValueError('v4 实时恢复 · 交付 R1仅支持 VS030 设备；离线录制请使用离线候选入口')
            selected_mode=self.mode.currentData()
            if selected_mode=='offline_recovery_v4' and kind!='recorded':
                raise ValueError('v4 离线恢复 · 交付 R1只支持录制回放')
            if selected_mode=='live_recovery_v4' and kind not in ('vendor','synthetic'):
                raise ValueError('v4 实时恢复 · 交付 R1只支持 VS030 设备')
            if kind=='recorded' and not self.recording_source:raise ValueError('请先导入录制目录')
            compute_options={'strict_body':self.body_gate_toggle.isChecked(),
                'initial_camera_height_m':float(self.ground_adjust.value())/100.,
                'camera_forehead_offset_mm':self.camera_forehead_offset.value(),
                'hand_filter':self.hand_filter_toggle.isChecked()}
            output=(Path(self.replay_output_dir) if self.replay_output_dir and self.result_output_parent is None else
                    (self.result_output_parent or DEFAULT_RESULT_ROOT)/
                    ((self.recording_source.name+'_computed_' if self.recording_source else 'replay_')+str(time.time_ns())))
            options={'vio_worker':self.live_vio_worker}
            if self.recovery_profile:options=self.recovery_profile.worker_options(self.recording_source,output)
            elif self.live_recovery_profile:options={**options,**self.live_recovery_profile.worker_options()}
            elif selected_mode=='offline_recovery_v4':
                from offline_recovery import OfflineRecoveryProfile
                options=OfflineRecoveryProfile().worker_options(self.recording_source,output,isolated_environment=True)
            elif selected_mode=='live_recovery_v4' and kind=='vendor':
                options={**options,**LiveRecoveryProfile().worker_options()}
            if kind=='recorded':options['save_depth']=True
            source=create_source(SourceRequest(kind=kind,path=str(self.recording_source or ''),loop=False))
            detector=yaml.safe_load((ROOT/'configs/detector.yaml').read_text(encoding='utf8')) if kind!='synthetic' else None
            if detector:detector['model_path']=str(ROOT/detector['model_path'])
            config=yaml.safe_load((ROOT/'configs/stereo.yaml').read_text(encoding='utf8'))
            self.saved_timer.stop();self.saved_play.setText('播放');self.saved_controls.hide()
            self.generation+=1;self.smoother.reset();self._pose_fitted=False;self.latest=None;self.saved=[];self.last_frame_at=0
            self._world_pose=None;self._has_native=None;self._raw_cache=None;self._raw_shown=None
            self._hand_poses.clear();self._hand_pose_world=None
            self._live_hand_result=None;self._live_hand_pose=None;self._live_hand_age_ms=None
            self.body_estimator.reset();self.body_pose=None;self._body_status='WAITING_HEAD';self.body_overlay.clear()
            self._displayed_hand_stamp=None;self._displayed_hands=None
            clear_stereo(self.left_view,self.right_view);clear_stereo(self.depth_view)
            self.depth_tile.show()
            self.depth_view.title='等待双目深度';self.depth_tile.set_caption('深度 · 全部有效值')
            clear_stereo(self.raw_left_view,self.raw_right_view)
            self.left_tile.set_caption('校正 · 左');self.right_tile.set_caption('校正 · 右')
            self.raw_left_tile.set_caption('原始 · 左');self.raw_right_tile.set_caption('原始 · 右')
            self.hand_overlay.clear();self.body_overlay.clear();self.vio_panel.clear();self.vio_panel.begin_live()
            self.session.start(source,config=config,detector_config=detector,offline=kind=='recorded',live_vio=kind!='synthetic',vio_mode='stereo_inertial' if selected_mode in ('offline_recovery_v4','live_recovery_v4') else selected_mode,vio_output_dir=output,vio_worker=options.pop('vio_worker'),**options)
            self._computed_output=output if kind=='recorded' else None
            self._computed_options=compute_options if kind=='recorded' else None
            self.session.command('hand_filter',compute_options['hand_filter'])
            if kind=='recorded':
                self._output_note('本次结果：'+str(output));self.output_label.setToolTip(str(output))
                self.statusBar().showMessage('计算结果目录：'+str(output))
            self.world_meta.setText('合成演示不运行定位' if kind=='synthetic' else '等待定位初始化')
            self.set_status('正在准备');self.sync_controls()
        except Exception as exc:self.show_error(str(exc));self.sync_controls()

    def stop_pipeline(self):
        self._pending_saved_depth=None
        self._saved_depth_advance=None
        self._saved_body_advance=None;self.smpl_controller.reset();self._smpl_request_key=None
        self.ground_reference.reset()
        self.saved_timer.stop();self.saved_play.setText('播放')
        if not self.session.stop():self.show_error('后台仍在停止，请稍后重试');return False
        if self.saved_replay is not None:self.saved_replay.close();self.saved_replay=None
        self.generation+=1;self.recording=False;self.pending_record=False;self.paused=False;self._paused_vio_reset=None;self._paused_vio_world=None;self.sync_controls()
        self.body_estimator.reset();self.body_pose=None;self._body_status='WAITING_HEAD';self.body_overlay.clear();self.body_meta.setText('人体 · WAITING_HEAD')
        if not self._closing:self.set_status('已停止')
        return True

    def toggle_pause(self):
        worker=self.session.worker
        if not worker or not worker.isRunning() or worker.stop_event.is_set():return
        self.paused=not self.paused
        self.session.pause(self.paused)
        if self.paused:
            # In-flight depth work belongs to the pre-pause display generation.
            self.generation+=1
            self._paused_vio_world=self.vio_panel._live_map_epoch
            self._hand_poses.clear();self._displayed_hand_stamp=None
            self.set_status('已暂停')
        else:
            self.smoother.reset()
            if self._paused_vio_reset is not None:
                self.vio_panel.update_live(self._paused_vio_reset)
                self._paused_vio_reset=None
            elif self.source.currentData()=='vendor':
                # Tracking continued in the background; do not draw a line over
                # the poses intentionally omitted from the paused display.
                self.vio_panel._live_gap_pending=True
                self.vio_panel._raw_gap=True
            self._paused_vio_world=None
            self.last_frame_at=0
            self.set_status('运行中')
        self.sync_controls()

    def on_ready(self,info):
        self.calibration=info.get('calibration');meta=info.get('metadata');self.set_status('运行中')
        if not info.get('model_loaded') and not getattr(meta,'synthetic',False):
            self.set_status('手部模型未加载 · 相机预览仍可用')
        self.source_label.setText('合成演示 · 非实测' if getattr(meta,'synthetic',False) else
            ('录制 · '+self.recording_source.name if self.recording_source else '录制回放') if self.source.currentData()=='recorded' else '实时 · VS030')

    def _update_body_pose(self, result):
        """Consume only the exact result's pose and world hands; never mix frames."""
        cached=(result.diagnostics.get('offline_derived_pose') or {}).get('body')
        unified=bool((result.diagnostics.get('offline_derived_pose') or {}).get('unified_smpl'))
        self.ground_reference.observe(result)
        derived=result.diagnostics.get('offline_derived_pose') or {}
        current_mount=(derived.get('head_anchor_version')==HEAD_ANCHOR_VERSION or
            bool(cached and (cached.get('diagnostics') or {}).get('head_anchor_version')==HEAD_ANCHOR_VERSION))
        if not unified or self._ground_override or not current_mount:
            if unified and cached and not self._ground_override:
                ground=(cached.get('diagnostics') or {}).get('ground_y_m')
                p=result.diagnostics.get('vio_pose') or result.diagnostics.get('live_vio_pose') or {}
                origin=self.ground_reference.origins.get(repr(display_reference_key(p)))
                if ground is not None and origin is not None:self._show_ground_height(float(ground)-origin)
            self._request_smpl(result)
            return
        self.smpl_controller.reset();self._smpl_request_key=None
        if cached is None:
            self._clear_smpl('SMPL · 本帧不可用')
            return
        # Saved replay uses the serialized pose directly. This avoids rerunning
        # IK and gating on every seek/playback tick.
        if cached is not None:
            try:
                estimate=BodyPose(frame_id=int(cached['frame_id']),timestamp_ns=int(cached['timestamp_ns']),
                    reference_key=tuple(cached['reference_key']) if cached.get('reference_key') else None,
                    status=str(cached.get('status','estimated_preview')),
                    joints={k:np.asarray(v,dtype=float) for k,v in cached.get('joints',{}).items()},
                    measured_joints=frozenset(cached.get('measured_joints',[])),
                    bones=tuple(tuple(x) for x in cached.get('bones',[])),
                    diagnostics={**cached.get('diagnostics',{}),'body_mesh':
                        (result.diagnostics.get('offline_derived_pose') or {}).get('body_mesh'),
                        'body_mesh_index':(result.diagnostics.get('offline_derived_pose') or {}).get('body_mesh_index'),
                        'body_mesh_vertices_path':(result.diagnostics.get('offline_derived_pose') or {}).get('body_mesh_vertices_path') or cached.get('diagnostics',{}).get('body_mesh_vertices_path')})
                self.body_pose=estimate; self._body_status=str(estimate.status).upper(); self.body_overlay.update(estimate)
                if not self._ground_override:
                    ref=repr(display_reference_key(result.diagnostics.get('vio_pose') or result.diagnostics.get('live_vio_pose') or {}))
                    origin=self.ground_reference.origins.get(ref)
                    ground=estimate.diagnostics.get('ground_y_m')
                    if origin is not None and ground is not None:
                        self._show_ground_height(float(ground)-origin)
                self.body_meta.setText('SMPL · 估计')
                self._apply_view_toggles(); return
            except (TypeError,ValueError,KeyError):
                if unified:
                    self.body_pose=None;self.body_overlay.clear();self._body_status='UNAVAILABLE'
                    self.body_meta.setText('SMPL · 数据不可用')
                    return
                pass
        self._clear_smpl('SMPL · 数据不可用')

    def _show_ground_height(self,height_m):
        self.ground_reference.height_m=float(height_m)
        self.ground_adjust.blockSignals(True);self.ground_adjust.setValue(height_m*100.)
        self.ground_adjust.blockSignals(False)
        self.ground_slider.blockSignals(True);self.ground_slider.setValue(round(height_m*1000.))
        self.ground_slider.blockSignals(False)

    def _clear_smpl(self,text):
        self.body_pose=None;self.body_overlay.clear();self.body_meta.setText(text)
        self._body_status='UNAVAILABLE'

    def _request_smpl(self,result):
        floor=self.ground_reference.floor_for(result)
        payload=request_from_result(result,floor) if floor is not None else None
        if payload is None:
            self.smpl_controller.reset();self._smpl_request_key=None
            self._clear_smpl('SMPL · 等待定位');return
        if self.body_gate_toggle.isChecked() and len(payload['wrists'])<2:
            self.smpl_controller.reset();self._clear_smpl('SMPL · 等待双手');return
        live=getattr(self.session.worker,'live_vio',None) if self.session.worker else None
        if live is not None:
            payload['streaming']=True;payload['stream_session']=getattr(live,'source_session_id',None)
        key=(getattr(result,'session_id',None),result.frame_id,result.timestamp_ns,self.generation,
             round(floor,6),repr(payload['reference']))
        if key==self._smpl_request_key:return
        self._smpl_request_key=key
        if not payload.get('streaming'):
            self._clear_smpl('SMPL · 求解中');self.hand_overlay.update_mano_meshes([])
        self._smpl_request_id=self.smpl_controller.request(payload)

    def _on_smpl_ready(self,reply):
        if self._closing:return
        if reply.get('streaming'):
            live=getattr(self.session.worker,'live_vio',None) if self.session.worker else None
            if live is None or reply.get('stream_session')!=getattr(live,'source_session_id',None):return
            age_ms=(time.perf_counter_ns()-reply['timestamp_ns'])/1e6
            if not 0<=age_ms<=500:return
            reference=tuple(reply['reference']) if reply.get('reference') is not None else None
            if reference!=self._hand_pose_world:return
        else:
            if reply.get('request_id')!=self._smpl_request_id:return
            if self.latest is None or (reply['frame_id'],reply['timestamp_ns'])!=(self.latest.frame_id,self.latest.timestamp_ns):return
        if not reply.get('accepted'):
            self._clear_smpl('SMPL · 姿态未通过')
            self.body_meta.setToolTip('当前高度与观测尚未求得合格姿态：'+', '.join(reply.get('reasons',[])))
        else:
            self.body_pose=pose_from_reply(reply,self.smpl_controller.asset)
            self.body_overlay.update(self.body_pose);self._body_status='ESTIMATED_PREVIEW'
            self.body_meta.setText(f'SMPL · {age_ms:.0f} ms' if reply.get('streaming') else 'SMPL · 估计')
            self.body_meta.setToolTip(f"相机、双手和地面约束；身体为推测；本次求解 {reply['elapsed_ms']:.0f} ms")
            self.hand_overlay.update_mano_meshes(reply.get('mano',[]))
            self._apply_view_toggles()
            self._update_view_pivot()
        self._resume_saved_body()

    def _on_smpl_failed(self,text):
        if self._closing:return
        self._clear_smpl('SMPL · 计算失败');self.body_meta.setToolTip(text)
        self._resume_saved_body()

    def _resume_saved_body(self):
        waiting=self._saved_body_advance;self._saved_body_advance=None
        if waiting is not None and waiting[0]==self.result_slider.value() and self.saved_play.text()=='暂停':
            self._schedule_saved((time.perf_counter()-waiting[1])*1000.)

    def _ground_adjusted(self, value):
        self.ground_slider.blockSignals(True)
        self.ground_slider.setValue(round(value*10))
        self.ground_slider.blockSignals(False)
        self._ground_override=True;self.ground_reference.height_m=float(value)/100.
        # Changing the floor must invalidate an already fitted body even if
        # the frame has not changed. Results for earlier slider values expire.
        self._smpl_request_key=None
        if self.latest is not None:
            self._update_body_pose(self.latest)

    def on_vio(self,message):
        if self.paused:
            if message.get('type')=='vio_reset':self._paused_vio_reset=dict(message)
            elif message.get('type')=='pose':
                from egohand.vio.map_identity import world_identity
                world=display_reference_key(message)
                if world is not None and self._paused_vio_world is not None and world!=self._paused_vio_world:
                    self._paused_vio_reset={'type':'vio_reset','message':'暂停期间地图坐标发生变化'}
                if world is not None:self._paused_vio_world=world
            if message.get('type')=='error':self.set_status('提示：'+str(message.get('message','定位异常')))
            return
        if message.get('type')=='pose':
            self._last_vio_message=dict(message);self._vio_pose_times.append(time.monotonic())
            timing=(message.get('runtime_performance') or {}).get('timing_ms') or {}
            if isinstance(timing.get('submit_to_admission'),(int,float)):
                self._last_vio_latency_ms=float(timing['submit_to_admission'])
        # 视觉/VIO来源切换仅断开显示线段，不代表底层地图或世界坐标已重置。
        self.vio_panel.update_live(message)
        if message.get('type')=='pose':
            from egohand.vio.map_identity import world_identity
            world=display_reference_key(message)
            if world!=self._hand_pose_world:
                self._hand_poses.clear();self._hand_pose_world=world
                self._world_pose=None
                if self._displayed_hands is not None:
                    self._show_hand_overlay(self._displayed_hands,None,require_pose='local_motion' in message)
            stamp=message.get('timestamp_ns')
            if type(stamp) is int and stamp>=0:
                transform=consumer_map_pose(message,world_hands=True,expected_timestamp_ns=stamp)
                self._hand_poses[stamp]=transform
                self._hand_poses.move_to_end(stamp)
                while len(self._hand_poses)>64:self._hand_poses.popitem(last=False)
                # VIO may finish after hand processing. Refresh only that exact displayed frame.
                if stamp==self._displayed_hand_stamp and not self._displayed_hand_has_pose:
                    self._world_pose=transform
                    self._show_hand_overlay(self._displayed_hands or [],transform,require_pose='local_motion' in message)
        elif message.get('type') in ('vio_reset','status','error'):
            self._world_pose=None;self.hand_overlay.clear()
            self.body_estimator.reset();self.body_pose=None;self._body_status='WAITING_HEAD';self.body_overlay.clear()
            self.body_meta.setText('人体·等待头')
            self._hand_poses.clear();self._hand_pose_world=None;self._displayed_hand_stamp=None
        if message.get('type')=='pose' and message.get('mode')=='hybrid':
            source=('VIO · 惯性优化已确认' if message.get('pose_source')=='vio'
                    else '视觉引导 · '+str(message.get('fallback_reason') or '惯性位姿暂不可用'))
            self.world_meta.setText('混合 · '+source+(' · 跟踪中' if consumer_map_pose(message) is not None else ' · 等待稳定跟踪'))
            return
        if message.get('type')=='pose':
            best=getattr(self.vio_panel,'_display_pose',None)
            if best is not None and best.source=='imu_prediction':
                self.world_meta.setText(f'IMU 预测 · {best.prediction_duration_s or 0:.2f} s')
            elif best is not None and best.source=='visual_candidate':
                self.world_meta.setText('视觉候选 · 未验证')
            elif best is not None and best.source=='frozen':
                self.world_meta.setText('位姿冻结 · 等待恢复')
            else:
                self.world_meta.setText('世界轨迹 · '+('跟踪中' if consumer_map_pose(message) is not None else '等待稳定跟踪'))
            motion=message.get('local_motion') or {}
            if 'local_motion' in message and (best is None or best.source in ('trusted','origin')):
                self.world_meta.setText(f'局部段 {motion.get("segment_index","—")} · '
                    +('跟踪中' if consumer_map_pose(message) is not None else '定位不可用'))
        else:
            text=message.get('message')
            if text:self.world_meta.setText(str(text))
        customer_status=self._customer_vio_status(message)
        if customer_status:self.set_status(customer_status)

    def on_finished(self):
        self.paused=False;self._paused_vio_reset=None;self._paused_vio_world=None
        self.poll();self.recording=False;self.pending_record=False;self.sync_controls()
        output=self._computed_output;self._computed_output=None
        options=self._computed_options;self._computed_options=None
        self._finish_recording_mesh_cache()
        if output is not None and not self._closing:
            self._start_pose_derivation(output,options=options)
            try:
                report=json.loads((output/'reprocessing.json').read_text(encoding='utf8'))
                if report.get('status')=='completed' and report.get('result_count',0)>0:
                    self._start_replay_cache(output)
            except (OSError,ValueError) as exc:
                self.statusBar().showMessage('计算完成，但无法检查缓存状态：'+str(exc))
        if not self._closing:self.set_status('已停止 · '+str(self.last_error) if getattr(self,'last_error',None) else '已停止 · 保留最后结果')

    def _start_pose_derivation(self, output, options=None):
        """Persist body/MANO-ready sidecar so replay and seeking do no inference."""
        output=Path(output)
        if any(folder==output and kind=='pose' and proc.poll() is None for proc,folder,kind in self._cache_jobs):return
        if (output/'derived_pose.jsonl').is_file(): return
        env=os.environ.copy();env['PYTHONPATH']=str(ROOT/'src')+os.pathsep+env.get('PYTHONPATH','')
        log=(output/'derived_pose_build.log').open('w',encoding='utf8')
        try:
            cfg=yaml.safe_load((ROOT/'configs/mano.yaml').read_text(encoding='utf8'))
            options=options or {'strict_body':self.body_gate_toggle.isChecked(),'initial_camera_height_m':self.ground_reference.height_m}
            command=[str(cfg['python']),str(ROOT/'scripts/build_unified_smpl_replay.py'),str(output),str(output),
                '--in-place','--mano','--initial-camera-height-m',str(options['initial_camera_height_m']),
                '--camera-forehead-offset-mm',str(options.get('camera_forehead_offset_mm',self.camera_forehead_offset.value()))]
            if options.get('strict_body'):command.append('--strict-body')
            proc=subprocess.Popen(command,
                cwd=str(ROOT),env=env,stdout=log,stderr=subprocess.STDOUT,
                creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        finally: log.close()
        self._cache_jobs.append((proc,output,'pose'))
        self._output_note('正在预计算 SMPL / MANO 网格，完成后可直接回看：'+str(output))

    def _start_replay_cache(self,output):
        output=Path(output)
        if (output/'display_cache.sqlite3').exists():return
        env=os.environ.copy();env['PYTHONPATH']=str(ROOT/'src')+os.pathsep+env.get('PYTHONPATH','')
        log=(output/'display_cache_build.log').open('w',encoding='utf8')
        try:
            proc=subprocess.Popen([sys.executable,str(ROOT/'scripts/build_saved_replay_cache.py'),str(output)],
                cwd=str(ROOT),env=env,stdout=log,stderr=subprocess.STDOUT,
                creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        finally:log.close()
        self._cache_jobs.append((proc,output,'replay'))
        self._output_note('正在后台生成拖动缓存：'+str(output))

    def _poll_cache_jobs(self):
        active=[]
        for proc,output,kind in self._cache_jobs:
            code=proc.poll()
            if code is None:active.append((proc,output,kind));continue
            name='SMPL / MANO 网格缓存' if kind=='pose' else '拖动缓存'
            filename='unified_smpl_manifest.json' if kind=='pose' else 'display_cache.sqlite3'
            log='derived_pose_build.log' if kind=='pose' else 'display_cache_build.log'
            if code==0 and (output/filename).is_file():
                self._output_note(name+'已完成：'+str(output))
            else:
                self._output_note(name+'生成失败：'+str(output/log))
                self.set_status(name+'生成失败 · 原始结果仍可回看')
        self._cache_jobs=active

    # ---------- 每帧显示 ----------
    def poll(self):
        if self.paused:return
        item=self.session.take_latest()
        if item:
            frame,result,stats=item;self.current_frame=frame;self.latest=result;self.last_frame_at=time.monotonic()
            shown=result if (result.diagnostics.get('hand_filter') or {}).get('enabled') else self.smoother.update(result)
            # 中列：校正后的左右图 + 识别结果（与原来一致的数据与叠加策略）。
            display_stereo(frame,shown,self.left_view,self.right_view)
            self._show_native_frame(frame)
            pose=result.diagnostics.get('vio_pose')
            if pose:self.on_vio(pose)
            self._bind_displayed_hands(shown.hands,result)
            live_world=result.world_hands or {}
            require_pose=(isinstance(pose,dict) and 'local_motion' in pose) or 'live_vio_pose' in result.diagnostics or live_world.get('status')=='pending' or (live_world.get('join') or {}).get('source')=='live_localization'
            self._show_hand_overlay(shown.hands,self._world_pose,require_pose=require_pose)
            derived=(shown.diagnostics.get('offline_derived_pose') or {}).get('mano') or {}
            cached_body=(shown.diagnostics.get('offline_derived_pose') or {}).get('body') or {}
            cached_smpl=(cached_body.get('diagnostics') or {}).get('head_anchor_version')==HEAD_ANCHOR_VERSION and not self._ground_override
            if cached_smpl:self.hand_overlay.update_mano_meshes(derived.get('predicted',[]) if not require_pose or self._world_pose is not None else [],self._world_pose,shown.hands)
            if not (self.session.worker and getattr(self.session.worker,'live_vio',None) is not None):
                self._update_body_pose(shown)
            self._apply_view_toggles()
            if not self._pose_fitted:
                points=[p.T_world_from_camera_left[:3,3] for p in self.vio_panel.poses]
                camera=np.eye(4) if self._world_pose is None else self._world_pose
                joints=np.asarray([j.xyz_m for h in shown.hands for j in h.joints if j.valid])
                if len(joints):points.extend(joints@camera[:3,:3].T+camera[:3,3])
                if points:fit_view(self.preview,points);self._pose_fitted=True
            self._update_view_pivot()
            self.left_tile.set_caption(f'校正 · 左 · {frame.left_image.shape[1]}×{frame.left_image.shape[0]}')
            self.right_tile.set_caption(f'校正 · 右 · {frame.right_image.shape[1]}×{frame.right_image.shape[0]}')
            self._sync_aspect(frame)
            valid_count=sum(h.valid_joint_count for h in result.hands)
            hands_note=f'手部 {len(result.hands)} 只 · {valid_count} 关节'
            vio_fps=0.0
            if len(self._vio_pose_times)>1:
                span=self._vio_pose_times[-1]-self._vio_pose_times[0]
                if span>0:vio_fps=(len(self._vio_pose_times)-1)/span
            latency=self._last_vio_latency_ms
            vio_note=f"VIO {vio_fps:.1f} FPS" if vio_fps else 'VIO — FPS'
            if latency is not None:vio_note+=f' · VIO延迟 {latency:.1f} ms'
            self.footer.setText(f"帧 {frame.frame_id}   ·   采集 {stats.get('read_fps',0):.1f} FPS   ·   手部 {stats.get('processing_fps',0):.1f} FPS   ·   {vio_note.split(' · ')[0]}   ·   丢帧 {stats.get('dropped_frames',0)}")
            self.footer.setToolTip(hands_note+' · '+vio_note)
            if self.session.worker and getattr(self.session.worker,'live_vio',None) is not None:
                self.footer.setText(self.footer.text()+(' · 世界手部 '+f'{self._live_hand_age_ms:.0f} ms' if self._live_hand_age_ms is not None else ' · 世界手部等待定位'))
            if time.monotonic()-self.depth_at>.08 and (self.depth_task is None or not self.depth_task.isRunning()):
                self.depth_at=time.monotonic()
                c=getattr(self,'calibration',None)
                if c:
                    self._start_depth_task(self.generation,frame,c)
                else:self.depth_tile.set_caption('缺少有效双目标定，深度未启用')
        elif self.last_frame_at and time.monotonic()-self.last_frame_at>2 and self.session.worker and self.session.worker.isRunning():
            self.footer.setText('画面中断 · 超过 2 s')
        if self.session.worker and getattr(self.session.worker,'live_vio',None) is not None:
            self._show_hand_overlay([],None,require_pose=True)
            if self._live_hand_pose is not None and self._live_hand_result is not None:
                self._update_body_pose(self._live_hand_result)
            else:
                self.smpl_controller.reset();self._smpl_request_key=None
                self._clear_smpl('SMPL · 等待定位')
            if self.body_pose is not None and time.perf_counter_ns()-self.body_pose.timestamp_ns>500_000_000:
                self._clear_smpl('SMPL · 等待新姿态');self.hand_overlay.update_mano_meshes([])

    def on_localized_hands(self,result):
        if self.paused:return
        worker=self.session.worker
        session=getattr(getattr(worker,'live_vio',None),'source_session_id',None)
        pose=result.diagnostics.get('live_vio_pose') or {}
        if (pose.get('live_capture') or {}).get('source_session_id')!=session or session is None:return
        if self._live_hand_result is not None and result.timestamp_ns<=self._live_hand_result.timestamp_ns:return
        self._live_hand_result=result
        self._show_hand_overlay([],None,require_pose=True)
        self._update_body_pose(result)

    def _prepare_live_hand_paint(self):
        if not self._paint_latest_pair or self.paused:return
        worker=self.session.worker
        if worker is None or getattr(worker,'live_vio',None) is None:return
        previous_result=self._live_hand_result
        previous_pose=self._live_hand_pose
        result,_=worker.hand_presentation.read()
        if result is not None:
            pose=result.diagnostics.get('live_vio_pose') or {}
            session=worker.live_vio.source_session_id
            if ((pose.get('live_capture') or {}).get('source_session_id')==session and
                    (self._live_hand_result is None or result.timestamp_ns>self._live_hand_result.timestamp_ns)):
                self._live_hand_result=result
        # Recheck original expiry/session/reference gates immediately before GL.
        transform=self._live_hand_transform()
        self._live_hand_pose=transform
        if transform is None:
            if self.hand_overlay.has_geometry():self.hand_overlay.clear()
        elif (previous_result is not self._live_hand_result or previous_pose is None or
                not np.array_equal(previous_pose,transform)):
            self.hand_overlay.update(self._live_hand_result.hands,transform)
        # Report the actual selected geometry, rather than the queued UI result.
        if (previous_result is not self._live_hand_result or
                (previous_pose is None)!=(self._live_hand_pose is None)):
            base=self.footer.text().split(' · 世界手部')[0]
            self.footer.setText(base+(' · 世界手部 '+f'{self._live_hand_age_ms:.0f} ms' if self._live_hand_pose is not None else ' · 世界手部等待定位'))
        # Creating GL buffers can consume the last few milliseconds of lifetime.
        if self._live_hand_pose is not None and time.perf_counter_ns()-self._live_hand_result.timestamp_ns>250_000_000:
            self._live_hand_pose=None;self._live_hand_age_ms=None
            self.hand_overlay.clear()
            self.footer.setText(self.footer.text().split(' · 世界手部')[0]+' · 世界手部等待定位')

    def _live_hand_transform(self):
        self._live_hand_age_ms=None
        result=self._live_hand_result
        current=self._last_vio_message
        if result is None or not isinstance(current,dict):return None
        pose=result.diagnostics.get('live_vio_pose') or {}
        session=getattr(getattr(self.session.worker,'live_vio',None),'source_session_id',None)
        if (pose.get('live_capture') or {}).get('source_session_id')!=session or session is None:return None
        if consumer_map_pose(current,world_hands=True,expected_timestamp_ns=current.get('timestamp_ns')) is None:return None
        if display_reference_key(pose)!=display_reference_key(current):return None
        if getattr(self,'_paint_latest_pair',False):
            _,published=self.session.worker.hand_presentation.read()
            if not isinstance(published,dict) or published.get('type')!='pose':return None
            if (published.get('live_capture') or {}).get('source_session_id')!=session:return None
            if consumer_map_pose(published,world_hands=True,expected_timestamp_ns=published.get('timestamp_ns')) is None:return None
            if display_reference_key(published)!=display_reference_key(current):return None
        age_ms=(time.perf_counter_ns()-result.timestamp_ns)/1e6
        if not 0<=age_ms<=250:return None
        transform=consumer_result_display_pose(result)
        if transform is not None:self._live_hand_age_ms=age_ms
        return transform

    def _bind_displayed_hands(self,hands,result):
        """Exact timestamp association inside the current SessionController worker.

        This display join does not certify clock/calibration accuracy or world continuity.
        Saved/offline frame evidence is authoritative when present, including rejection.
        """
        self._displayed_hand_stamp=result.timestamp_ns;self._displayed_hands=hands
        self._displayed_hand_has_pose='vio_pose' in result.diagnostics or 'live_vio_pose' in result.diagnostics
        if type(result.timestamp_ns) is not int or result.timestamp_ns<0:
            self._world_pose=None;return
        if self._displayed_hand_has_pose:
            self._world_pose=consumer_result_display_pose(result)
        else:
            transform=self._hand_poses.get(result.timestamp_ns)
            self._world_pose=None if transform is None else transform.copy()

    def _show_hand_overlay(self,hands,transform,*,require_pose=False):
        if self.session.worker and getattr(self.session.worker,'live_vio',None) is not None:
            self._live_hand_pose=self._live_hand_transform()
            if self._live_hand_pose is None:self.hand_overlay.clear()
            else:self.hand_overlay.update(self._live_hand_result.hands,self._live_hand_pose)
            base=self.footer.text().split(' · 世界手部')[0]
            self.footer.setText(base+(' · 世界手部 '+f'{self._live_hand_age_ms:.0f} ms' if self._live_hand_age_ms is not None else ' · 世界手部等待定位'))
            return
        # A local segment view must not interpret camera-only points as segment points.
        if require_pose and transform is None:self.hand_overlay.clear()
        else:self.hand_overlay.update(hands,transform)

    def _show_native_frame(self,frame):
        """中列：未经校正的原始鱼眼图像，按设备输出原样显示。

        原始帧不一定与当前处理帧同号：处理线程可能领先或落后一帧。这里始终显示
        处理线程最新的那对原始帧，并在角标标出它的帧号，既不冒充同帧，也不会因为
        帧号对不上而长时间停在旧图上。
        """
        native=getattr(frame,'debug_native_images',None)
        raw_id=frame.frame_id
        if not native:
            raw=getattr(self.session.worker,'last_input_frame',None)
            if raw is not None and raw.image_geometry=='raw':
                native=(raw.left_image,raw.right_image);raw_id=raw.frame_id
        if not native:
            if self._has_native is not False:
                self._has_native=False;self._raw_cache=None;self._raw_shown=None
                clear_stereo(self.raw_left_view,self.raw_right_view)
                self.raw_left_tile.set_caption('原始 · 左 · 当前帧无原始图')
                self.raw_right_tile.set_caption('原始 · 右 · 当前帧无原始图')
            return
        self._has_native=True
        # 同一对原始帧只重绘一次：重复重绘只会拖慢界面。
        if self._raw_shown==raw_id:return
        self._raw_shown=raw_id;self._raw_cache=(raw_id,native[0],native[1])
        note='' if raw_id==frame.frame_id else f' · 第 {raw_id} 帧'
        for tile,view,image,side in ((self.raw_left_tile,self.raw_left_view,native[0],'左'),
                                     (self.raw_right_tile,self.raw_right_view,native[1],'右')):
            # 原生像素原样显示：不裁剪、不做圆形遮罩；只在显示副本上降采样。
            view.display(display_sized(image,max(240,tile.width())))
            tile.set_caption(f'原始 · {side} · {image.shape[1]}×{image.shape[0]}{note}')

    def _start_depth_task(self,generation,frame,calibration):
        if self.depth_task:self.depth_task.deleteLater()
        self.depth_task=DepthTask(generation,frame,calibration,self.depth_cache,self)
        self.depth_task.completed.connect(self.on_depth)
        task=self.depth_task
        task.finished.connect(lambda:self._depth_finished(task))
        self.depth_task.start()

    def _request_saved_depth(self,frame,calibration):
        # One active calculation and one latest request: seeks never queue a
        # backlog or run the shared OpenCV matcher concurrently.
        request=(self.generation,frame,calibration)
        if self.depth_task is not None and self.depth_task.isRunning():
            self._pending_saved_depth=request
        else:
            self._pending_saved_depth=None
            self._start_depth_task(*request)

    def _depth_finished(self,task):
        if task is not self.depth_task:return
        request=self._pending_saved_depth
        self._pending_saved_depth=None
        if request is not None and request[0]==self.generation and not self._closing:
            self._start_depth_task(*request)

    def on_depth(self,generation,image,text):
        if generation==self.generation:
            if image is None:
                clear_stereo(self.depth_view);self.depth_tile.set_caption(text)
            else:
                self.depth_view.display(image);self.depth_tile.set_caption(text)
            waiting=self._saved_depth_advance
            if waiting is not None and waiting[0]==self.result_slider.value():
                self._saved_depth_advance=None
                self._schedule_saved((time.perf_counter()-waiting[1])*1000.)

    # ---------- 录制 / 截图 / 回看 ----------
    def toggle_recording(self):
        if self.recording:self.pending_record=True;self.session.command('record_stop');self.sync_controls();return
        self.record_output_parent.mkdir(parents=True,exist_ok=True)
        self.recording_path=self.record_output_parent/('session_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns()%1000000000))
        self.pending_record=True;self.session.command('record_start',self.recording_path);self.sync_controls()

    def on_recording(self,active,paused,message):
        if active and self.recording_path is not None:self._recording_mesh_pending=Path(self.recording_path)
        self.recording=active;self.pending_record=False;self.set_status('录制中' if active else message);self.sync_controls()
        if not active and message=='录制已停止':self._finish_recording_mesh_cache()

    def _finish_recording_mesh_cache(self):
        path=self._recording_mesh_pending
        if path is None:return
        if (path/'results/frames.jsonl').is_file():
            self._recording_mesh_pending=None
            self._start_pose_derivation(path)

    def save_screenshot(self):
        folder=DELIVERY_ROOT/'screenshots';folder.mkdir(parents=True,exist_ok=True)
        path=folder/('client_'+time.strftime('%Y%m%d_%H%M%S')+'.png')
        if self.grab().save(str(path)):self.statusBar().showMessage('截图已保存：'+str(path))
        else:self.show_error('截图保存失败')

    def open_results(self):
        path=QFileDialog.getExistingDirectory(self,'选择含 results/frames.jsonl 的会话',str(DEFAULT_IMPORT_ROOT))
        if not path:return
        self.open_results_path(Path(path))

    def open_results_path(self,path):
        path=Path(path)
        try:
            if not self.stop_pipeline():return
            self.saved,truncated=load_recorded_results(path)
            # Seeking must use the segment's INITIAL camera origin, never
            # redefine the floor from the currently selected frame.
            for result in self.saved:self.ground_reference.observe(result)
            self.source.setCurrentIndex(self.source.findData('recorded'))
            self._saved_rectifier=None;self._saved_last_index=None
            if not self.saved:raise ValueError('没有可回看的结果')
            clear_stereo(self.left_view,self.right_view);clear_stereo(self.depth_view)
            clear_stereo(self.raw_left_view,self.raw_right_view)
            self.hand_overlay.clear();self.vio_panel.clear()
            self.body_estimator.reset();self.body_pose=None;self.body_overlay.clear();self.body_meta.setText('人体 · WAITING_HEAD')
            try:
                replay=SavedResultReplay(path,self.saved);replay.open();replay.start()
                self.saved_replay=replay
            except (OSError,ValueError,KeyError) as exc:
                if 'replay' in locals():replay.close()
                self.saved_replay=None
                self.source_label.setText('原始图像不可用：'+str(exc)+' · 可继续查看三维结果')
            self.result_slider.setRange(0,len(self.saved)-1);self.result_slider.setValue(0);self.saved_controls.show();self.display_saved(0)
            self.set_status('已有结果回看'+(' · 尾部不完整' if truncated else ''))
        except Exception as exc:self.show_error(str(exc))

    def toggle_saved_playback(self):
        if not self.saved:return
        if self.saved_timer.isActive() or self._saved_depth_advance is not None or self._saved_body_advance is not None:
            self._saved_depth_advance=None
            self._saved_body_advance=None
            self.saved_timer.stop();self.saved_play.setText('播放');return
        if self.result_slider.value() >= len(self.saved)-1:self.result_slider.setValue(0)
        self.saved_play.setText('暂停');self._schedule_saved()

    def _saved_seek_started(self):
        self._saved_resume_after_seek=self.saved_timer.isActive() or self._saved_depth_advance is not None or self._saved_body_advance is not None
        self._saved_depth_advance=None
        self._saved_body_advance=None
        self.saved_timer.stop()

    def _saved_seek_preview(self,index):
        if self.saved:
            self.source_label.setText(f'拖动至 {index+1} / {len(self.saved)} · 松开后加载')

    def _saved_seek_finished(self):
        # Some Qt accessibility and non-tracking slider paths update the
        # value without delivering the intermediate valueChanged callback.
        # Always commit the released position explicitly.
        if self.saved and hasattr(self.saved[0], 'diagnostics'):
            self.display_saved(int(self.result_slider.value()))
        if getattr(self,'_saved_resume_after_seek',False):
            self._schedule_saved()
        self._saved_resume_after_seek=False

    def _schedule_saved(self, processing_ms=0.):
        index=self.result_slider.value()
        if not self.saved or index>=len(self.saved)-1:
            self.saved_play.setText('播放');return
        if self.smpl_controller.pending:
            self._saved_body_advance=(index,time.perf_counter()-processing_ms/1000.)
            return
        delta=self.saved[index+1].timestamp_ns-self.saved[index].timestamp_ns
        speed=self._saved_speed_value()
        delay=0. if speed is None else (delta/1e6 if 0<delta<=2_000_000_000 else 33.333)/speed
        self.saved_timer.start(max(1,ceil(delay-processing_ms)))

    def _saved_speed_value(self):
        text=self.saved_speed.currentText().strip()
        if text=='最快':return None
        try:value=float(text.replace('×','').strip())
        except (TypeError,ValueError):value=1.
        return min(32.,max(.05,value))

    def _normalize_saved_speed(self):
        value=self._saved_speed_value();self.saved_speed.setEditText('最快' if value is None else f'{value:g}×')
        self._saved_speed_changed()

    def _saved_speed_changed(self,*_):
        if not self.saved_timer.isActive():return
        self.saved_timer.stop()
        self._schedule_saved()

    def _advance_saved(self):
        if self.result_slider.value()<len(self.saved)-1:
            started=time.perf_counter()
            self.result_slider.setValue(self.result_slider.value()+1)
            # Do not advance away before this frame's depth is ready. This
            # keeps all three image rows paired without showing old depth as
            # current, and playback slows honestly when display work is heavy.
            if (self.depth_task is not None and self.depth_task.isRunning()
                    and (self.depth_task.generation==self.generation or self._pending_saved_depth is not None)):
                self._saved_depth_advance=(self.result_slider.value(),started)
            else:self._schedule_saved((time.perf_counter()-started)*1000.)
        else:self.saved_play.setText('播放')

    def display_saved(self,index):
        if self.saved:
            # Every selected frame invalidates older asynchronous depth work.
            self.generation+=1;self._pending_saved_depth=None
            if self.depth_tile.isHidden():
                self.depth_tile.show()
                self._fit_columns()
            self.latest=self.saved[index]
            sequential=self._saved_last_index is not None and index==self._saved_last_index+1
            self.vio_panel.show_recorded(self.saved,index,show_hands=False,incremental=sequential)
            self._update_body_pose(self.latest)
            self._world_pose=None
            self._hand_poses.clear();self._displayed_hand_stamp=None;self._displayed_hands=None
            # The selected hand frame must not inherit an earlier camera pose.
            self._world_pose=consumer_result_display_pose(self.latest)
            self._displayed_hands=self.latest.hands
            saved_pose=self.latest.diagnostics.get('vio_pose') or self.latest.diagnostics.get('live_vio_pose') or {}
            self._show_hand_overlay(self.latest.hands,self._world_pose,require_pose='local_motion' in saved_pose or 'live_vio_pose' in self.latest.diagnostics)
            derived=(self.latest.diagnostics.get('offline_derived_pose') or {}).get('mano') or {}
            require_pose='local_motion' in saved_pose or 'live_vio_pose' in self.latest.diagnostics
            cached_body=(self.latest.diagnostics.get('offline_derived_pose') or {}).get('body') or {}
            cached_smpl=(cached_body.get('diagnostics') or {}).get('head_anchor_version')==HEAD_ANCHOR_VERSION and not self._ground_override
            if cached_smpl:self.hand_overlay.update_mano_meshes(derived.get('predicted',[]) if not require_pose or self._world_pose is not None else [],self._world_pose,self.latest.hands)
            self._apply_view_toggles()
            keep_depth=sequential and self.depth_view.pix is not None and self.saved_replay is not None
            for view in (self.left_view,self.right_view,self.raw_left_view,self.raw_right_view):clear_stereo(view)
            if not keep_depth:clear_stereo(self.depth_view)
            self.depth_view.title='深度不可用 · 缺少双目图像'
            self.depth_tile.set_caption('深度 · 不可用')
            image_status='原始图像不可用 · 三维结果可查看'
            self._saved_depth_loaded=False
            read_depth=getattr(self.saved_replay,'depth_for_result',None)
            if read_depth is not None:
                stored=read_depth(self.latest)
                if stored is not None and stored[0] is not None:
                    from egohand.visualization.stereo_depth import colorize_depth
                    depth,info=stored
                    image,valid=colorize_depth(depth,.15,3.,False)
                    self.on_depth(self.generation,image,f'深度 · 已保存 · {info.get("reference_camera","—")} · {valid.mean():.0%}')
                    self._saved_depth_loaded=True
            if self.saved_replay is not None:
                try:
                    from egohand.calibration.rectifier import StereoRectifier
                    self.saved_replay.seek(index);frame=self.saved_replay.read()
                    raw=(frame.left_image,frame.right_image)
                    if self.latest.diagnostics.get('output_geometry')=='rectified':
                        if self._saved_rectifier is None:self._saved_rectifier=StereoRectifier(self.saved_replay.calibration)
                        frame=self._saved_rectifier.rectify(frame)
                    frame.debug_native_images=raw
                    display_stereo(frame,self.latest,self.left_view,self.right_view)
                    self._show_native_frame(frame);self._sync_aspect(frame)
                    if not self._saved_depth_loaded:
                        self.depth_view.title='深度计算中'
                        self.depth_tile.set_caption('深度 · 上帧' if keep_depth else '深度 · 计算中')
                        self._request_saved_depth(frame,self.saved_replay.calibration)
                    image_status='原始双目与保存结果同帧'
                except (OSError,ValueError,KeyError) as exc:
                    if not self._saved_depth_loaded:
                        clear_stereo(self.depth_view);self.depth_tile.set_caption('深度 · 不可用')
                    image_status='图像未显示：'+str(exc)
            motion=getattr(self.latest,'local_motion',None) or {}
            segment_text=f' · 局部段 {motion.get("segment_index","—")}' if getattr(self.latest,'local_motion',None) is not None else ''
            self.source_label.setText('结果回看'+segment_text)
            self.source_label.setToolTip(image_status)
            available=self._world_pose is not None
            self.world_meta.setText(('局部段 '+str(motion.get('segment_index','—')) if motion else '保存轨迹')+
                (' · 位姿可用' if available else ' · 当前位姿不可用'))
            hands=sum(h.valid_joint_count>0 for h in self.latest.hands)
            source=motion.get('pose_source') or saved_pose.get('mode') or '来源未知'
            delta=(self.saved[index].timestamp_ns-self.saved[index-1].timestamp_ns)/1e9 if index else 0
            fps=f'{1/delta:.1f}' if delta>0 else '—'
            self.footer.setText(f'帧 {index+1} / {len(self.saved)}   ·   {fps} FPS（录制）   ·   手部 {hands}   ·   '+
                ('定位可用' if available else '定位不可用'))
            self.footer.setToolTip(f'原始帧 {getattr(self.latest,"frame_id","—")} · {source} · {image_status}')
            if not sequential:self.reset_world_view()
            self._update_view_pivot()
            self._saved_last_index=index

    def _sync_aspect(self,frame):
        """用实际图像宽高比排版，图像区块不会出现明显黑边。"""
        image=getattr(frame,'left_image',None)
        if image is None or not getattr(image,'shape',None) or not image.shape[0]:return
        aspect=image.shape[1]/image.shape[0]
        if abs(aspect-self.image_aspect)<1e-4:return
        self.image_aspect=aspect
        for tile in (self.depth_tile,self.left_tile,self.right_tile,self.raw_left_tile,self.raw_right_tile):
            tile.set_aspect(aspect)
        self._fit_columns()

    def _fit_columns(self):
        """按图像长宽比分配中/右列宽度，使每张图正好填满格子，不出现明显黑边。"""
        height=self.columns.height()
        if height<=0:return
        mid_h=height/3;right_h=height/2
        mid_w=int(round(mid_h*self.image_aspect));right_w=int(round(right_h*self.image_aspect))
        budget=int(self.columns.width()*0.76)
        if mid_w+right_w>budget and mid_w+right_w>0:
            scale=budget/(mid_w+right_w);mid_w=int(mid_w*scale);right_w=int(right_w*scale)
        self.middle_column.setFixedWidth(max(110,mid_w))
        self.right_column.setFixedWidth(max(110,right_w))
        for tile in (self.raw_left_tile,self.raw_right_tile,self.depth_tile):
            tile.setFixedHeight(max(1,round(max(110,mid_w)/self.image_aspect)))
        for tile in (self.left_tile,self.right_tile):
            tile.setFixedHeight(max(1,round(max(110,right_w)/self.image_aspect)))

    def showEvent(self,event):
        super().showEvent(event)
        if hasattr(self,'column_layout'):self._fit_columns()

    def resizeEvent(self,event):
        super().resizeEvent(event)
        if hasattr(self,'command_groups'):self._layout_toolbar()
        if hasattr(self,'column_layout'):self._fit_columns()

    def _layout_toolbar(self):
        margins=self.centralWidget().layout().contentsMargins()
        available=self.centralWidget().width()-margins.left()-margins.right()
        self._toolbar_wide=self.command_strip.minimumSizeHint().width()<=available
        scrollbar=self.toolbar_scroll.horizontalScrollBar()
        height=self.command_strip.sizeHint().height()
        if not self._toolbar_wide:height+=scrollbar.sizeHint().height()
        self.toolbar_scroll.setFixedHeight(height)

    def closeEvent(self,event):
        self._closing=True
        if not self.stop_pipeline():event.ignore();self._closing=False;return
        if self.depth_task and self.depth_task.isRunning():
            if not self.depth_task.wait(5000):event.ignore();self._closing=False;return
        # GUI 真正退出时才释放常驻 ORB 进程（及其已加载的词典）。
        self.live_vio_worker.close()
        self.smpl_controller.close()
        event.accept()
