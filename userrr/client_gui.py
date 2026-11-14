"""Customer delivery shell. Processing and presentation remain in egohand shared modules."""
import sys
from pathlib import Path
import time
from collections import OrderedDict,deque
import numpy as np
import yaml
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (QMainWindow,QWidget,QVBoxLayout,QHBoxLayout,QLabel,QPushButton,
    QComboBox,QFileDialog,QSlider,QSizePolicy,QBoxLayout,QToolButton)
from egohand.core.pipeline import ROOT
from egohand.gui.shared.session import SessionController
from egohand.gui.shared.sources import SourceRequest,create_source
from egohand.gui.shared.presentation import display_stereo,clear_stereo
from egohand.gui.theme import THEMES,stylesheet
from egohand.gui.vio_panel import VioPanel
from egohand.vio.live_runtime import LiveVioWorker
from egohand.vio.live_recovery import LiveRecoveryProfile
from egohand.vio.local_segments import consumer_display_pose as consumer_map_pose,display_reference_key
from egohand.storage.replay import load_recorded_results,SavedResultReplay
from egohand.visualization.display_smoothing import DisplaySmoother

sys.path.insert(0,str(Path(__file__).resolve().parent))
from delivery_views import (DepthCache,DepthTask,ImageTile,WorldHandOverlay,WorldPanel,
    display_sized,fit_view,set_items_visible)

DELIVERY_ROOT=Path(__file__).resolve().parent
STYLE_SHEET=stylesheet('dark')+'''
QLabel#tileCaption{color:#B9C6D6;background:rgba(13,17,24,150);border-radius:3px;padding:2px 6px;font-size:11px;}
QWidget#worldOverlay{background:rgba(13,17,24,135);border-radius:4px;}
QLabel#compactTitle{font-size:14px;font-weight:600;} QLabel#compactMeta{color:#8697AD;font-size:11px;}
QToolButton#viewToggle{background:transparent;border:1px solid transparent;color:#8697AD;padding:3px 8px;
    font-size:11px;border-radius:3px;} QToolButton#viewToggle:hover{color:#E8EDF5;}
QToolButton#viewToggle:checked{color:#5FE0C5;border-color:#20463F;}
QPushButton{min-height:26px;padding:0 10px;font-weight:400;} QComboBox{min-height:26px;}
QPushButton#start{border-color:#5FE0C5;color:#5FE0C5;font-weight:600;}'''


class ClientMainWindow(QMainWindow):
    def __init__(self, *, recovery_profile=None, live_recovery_profile=None, replay_output_dir=None):
        super().__init__();self.setWindowTitle('EgoHand DELIVERY · 客户交付版 · VS030')
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
        self._vio_pose_times=deque(maxlen=40);self._last_vio_message=None;self._last_vio_latency_ms=None
        self.saved_replay=None
        self.smoother=DisplaySmoother(yaml.safe_load((ROOT/'configs/app.yaml').read_text(encoding='utf8')).get('display_smoothing'))
        self.image_aspect=4/3
        self.depth_cache=DepthCache()
        self._raw_cache=None;self._raw_shown=None
        self._build()
        self.session.ready.connect(self.on_ready);self.session.error.connect(self.show_error)
        self.session.startup_status.connect(self.set_status);self.session.vio_status.connect(self.on_vio)
        self.session.recording_state_changed.connect(self.on_recording);self.session.finished.connect(self.on_finished)
        self.timer=QTimer(self);self.timer.timeout.connect(self.poll);self.timer.start(40)
        self.setStyleSheet(STYLE_SHEET)
        for view in (self.left_view,self.right_view,self.depth_view,self.raw_left_view,self.raw_right_view):
            view.set_theme(THEMES['dark'])
        self.vio_panel.set_theme(THEMES['dark'])
        self.sync_controls()

    # ---------- 布局 ----------
    def _build(self):
        root=QWidget();root.setObjectName('applicationRoot');self.setCentralWidget(root)
        layout=QVBoxLayout(root);layout.setContentsMargins(10,8,10,8);layout.setSpacing(6)
        head=QHBoxLayout();head.setSpacing(8)
        title=QLabel('EgoHand DELIVERY');title.setObjectName('compactTitle');head.addWidget(title)
        self.state_label=QLabel('未连接');self.state_label.setObjectName('compactMeta');self.state_label.setWordWrap(True);self.state_label.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Preferred);head.addWidget(self.state_label,1)
        head.addStretch();layout.addLayout(head)
        toolbar=QBoxLayout(QBoxLayout.LeftToRight);toolbar.setSpacing(6);self.command_layout=toolbar
        selection=QHBoxLayout();selection.setSpacing(6);actions=QHBoxLayout();actions.setSpacing(6)
        toolbar.addLayout(selection);toolbar.addStretch();toolbar.addLayout(actions)
        self.source=QComboBox();self.source.setAccessibleName('数据源');self.source.addItem('VS030 设备','vendor');self.source.addItem('录制回放','recorded');self.source.addItem('合成演示（非实测）','synthetic')
        self.mode=QComboBox();self.mode.setAccessibleName('定位算法');self.mode.addItem('双目＋IMU · 正式 DLL','stereo_inertial');self.mode.addItem('纯双目视觉 · 正式 DLL','stereo')
        self.mode.addItem('混合 · VIO优先 / 视觉回退','hybrid')
        self.mode.addItem('双目＋IMU · v4 离线恢复实验版','offline_recovery_v4')
        self.mode.addItem('双目＋IMU · v4 实时恢复实验版','live_recovery_v4')
        self.mode.setCurrentIndex(self.mode.findData('live_recovery_v4'))
        self.source.currentIndexChanged.connect(self._source_mode_changed)
        if self.recovery_profile:
            self.source.clear();self.source.addItem('录制回放 · 离线候选','recorded')
            self.mode.clear();self.mode.addItem('双目＋IMU · 恢复候选 v4','stereo_inertial')
        elif self.live_recovery_profile:
            self.mode.clear();self.mode.addItem('双目＋IMU · v4 实时恢复实验版','stereo_inertial')
        self.mode.setToolTip('混合模式只运行一个IMU_STEREO实例；允许视觉先建图，IMU初始化并通过门控后优先使用VIO。定位精度仍待验收，仅停止后切换。')
        self.source.setMinimumWidth(120);self.mode.setMinimumWidth(235)
        selection.addWidget(self.source);selection.addWidget(self.mode)
        self.open_button=self._button('导入录制',self.import_recording);selection.addWidget(self.open_button)
        self.results_button=self._button('查看已有结果',self.open_results);selection.addWidget(self.results_button)
        self.start_button=self._button('连接 / 启动',self.start_pipeline);self.start_button.setObjectName('start');actions.addWidget(self.start_button)
        self.pause_button=self._button('暂停',self.toggle_pause)
        self.pause_button.setToolTip('回放暂停在当前帧；实时模式暂停画面和手部处理，采集、定位和正在进行的原始录制在后台继续。')
        actions.addWidget(self.pause_button)
        self.stop_button=self._button('停止',self.stop_pipeline);actions.addWidget(self.stop_button)
        self.record_button=self._button('开始录制',self.toggle_recording);actions.addWidget(self.record_button)
        self.capture_button=self._button('截图',self.save_screenshot);actions.addWidget(self.capture_button);layout.addLayout(toolbar)
        self.source_label=QLabel('选择数据源后启动。');self.source_label.setObjectName('compactMeta');self.source_label.setWordWrap(True);layout.addWidget(self.source_label)
        if self.recovery_profile:
            self.profile_label=QLabel(self.recovery_profile.label+' · 仅处理离线录制')
            self.profile_label.setWordWrap(True);layout.addWidget(self.profile_label)
        elif self.live_recovery_profile:
            self.profile_label=QLabel(self.live_recovery_profile.label+' · 仅显式实验入口 · 不改变正式默认')
            self.profile_label.setWordWrap(True);layout.addWidget(self.profile_label)

        # 三列：左 3D（吸收多余空间）、中 深度/校正左/校正右（单列）、右 原始左右。
        self.columns=QWidget();self.column_layout=QHBoxLayout(self.columns)
        self.column_layout.setContentsMargins(0,0,0,0);self.column_layout.setSpacing(2)
        self.world_column=self._world_panel()
        self.middle_column=self._middle_column()
        self.right_column=self._right_column()
        self.column_layout.addWidget(self.world_column,1)
        self.column_layout.addWidget(self.middle_column,0)
        self.column_layout.addWidget(self.right_column,0)
        self.column_widgets=[self.world_column,self.middle_column,self.right_column]
        layout.addWidget(self.columns,1)

        self.result_slider=QSlider(Qt.Horizontal);self.result_slider.setAccessibleName('已有结果帧');self.result_slider.hide();self.result_slider.valueChanged.connect(self.display_saved);layout.addWidget(self.result_slider)
        self.footer=QLabel('相机 —   ·   定位 —   ·   手部 —   ·   FPS —');self.footer.setObjectName('compactMeta');self.footer.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Preferred);self.footer.setWordWrap(True);layout.addWidget(self.footer)
        self.statusBar().showMessage('就绪 · 定位精度尚待验收')

    def _world_panel(self):
        """左列：一个三维视图，同时显示相机位姿、轨迹与手部位姿。"""
        panel=WorldPanel();box=QVBoxLayout(panel);box.setContentsMargins(0,0,0,0);box.setSpacing(0)
        bar=panel.bar
        self.trajectory_toggle=self._toggle('轨迹',True,'显示/隐藏相机轨迹线')
        self.camera_toggle=self._toggle('相机',True,'显示/隐藏当前相机坐标轴')
        self.hands_toggle=self._toggle('手部',True,'显示/隐藏世界系手部骨架')
        self.fit_button=QToolButton();self.fit_button.setText('复位视角');self.fit_button.setObjectName('viewToggle')
        self.fit_button.setToolTip('把视角重新套到轨迹与手部的包围盒')
        self.fit_button.clicked.connect(self.reset_world_view)
        for widget in (self.trajectory_toggle,self.camera_toggle,self.hands_toggle,self.fit_button):bar.addWidget(widget)
        self.world_meta=QLabel('等待定位');self.world_meta.setObjectName('compactMeta')
        self.world_meta.setMaximumWidth(180);self.world_meta.setWordWrap(True);bar.addWidget(self.world_meta)
        self.vio_panel=VioPanel()
        # 复用世界身份、质量门控与轨迹更新逻辑，只保留三维视图本身。
        for name in ('slider','details','raw_container','filtered_title','filtered_status','fit_comparison'):
            widget=getattr(self.vio_panel,name,None)
            if widget is not None:widget.hide()
        for name in ('live_enabled','live_profile','show_estimate','show_truth','compare_native','compare_map','update_map'):
            widget=getattr(self.vio_panel,name,None)
            if widget is not None:widget.hide()
        for widget in self.vio_panel.findChildren(QPushButton):widget.hide()
        inner=self.vio_panel.layout()
        inner.setContentsMargins(0,0,0,0);inner.setSpacing(0)
        self.vio_panel.comparison_split.setHandleWidth(0)
        self.preview=self.vio_panel.preview
        self.hand_overlay=WorldHandOverlay(self.preview)
        box.addWidget(self.vio_panel,1)
        panel.overlay.show();panel.place_overlay()
        return panel

    def _middle_column(self):
        """中列（小）：原始鱼眼左、原始鱼眼右、深度图，纵向一列。"""
        column=QWidget();box=QVBoxLayout(column);box.setContentsMargins(0,0,0,0);box.setSpacing(2)
        self.raw_left_tile=ImageTile('等待原始左图');self.raw_left_view=self.raw_left_tile.view
        self.raw_right_tile=ImageTile('等待原始右图');self.raw_right_view=self.raw_right_tile.view
        self.raw_left_tile.set_caption('原始 · 左')
        self.raw_right_tile.set_caption('原始 · 右')
        self.depth_tile=ImageTile('等待双目深度');self.depth_view=self.depth_tile.view
        self.depth_tile.set_caption('深度 · 全部有效值')
        for tile in (self.raw_left_tile,self.raw_right_tile,self.depth_tile):box.addWidget(tile,1)
        return column

    def _right_column(self):
        """右列（大）：校正后的左右图及其手部识别结果。"""
        column=QWidget();box=QVBoxLayout(column);box.setContentsMargins(0,0,0,0);box.setSpacing(2)
        self.left_tile=ImageTile('等待左相机');self.left_view=self.left_tile.view
        self.right_tile=ImageTile('等待右相机');self.right_view=self.right_tile.view
        self.left_tile.set_caption('校正 · 左')
        self.right_tile.set_caption('校正 · 右')
        box.addWidget(self.left_tile,1);box.addWidget(self.right_tile,1)
        return column

    def _button(self,text,callback):
        b=QPushButton(text);b.setAccessibleName(text);b.clicked.connect(callback);return b

    def _toggle(self,text,checked,tip):
        b=QToolButton();b.setText(text);b.setObjectName('viewToggle');b.setCheckable(True);b.setChecked(checked)
        b.setToolTip(tip);b.toggled.connect(lambda _=False:self._apply_view_toggles())
        return b

    # ---------- 显示开关 ----------
    def _apply_view_toggles(self):
        """三个开关互相独立：轨迹只影响轨迹线，相机只影响坐标轴。"""
        set_items_visible(self.preview.trajectory_items,self.trajectory_toggle.isChecked())
        set_items_visible(self.preview.axis_items,self.camera_toggle.isChecked())
        self.hand_overlay.set_visible(self.hands_toggle.isChecked())
        self.preview.update()

    def reset_world_view(self):
        points=[pose.T_world_from_camera_left[:3,3] for pose in self.vio_panel.poses]
        for item in self.hand_overlay.items:
            try:points.extend(np.asarray(item.pos))
            except (AttributeError,TypeError):pass
        self._pose_fitted=False
        self.preview.fit_geometry(points)

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
                return '正在初始化定位 · 请缓慢移动相机，观察周围环境'
            return detail or '正在初始化定位'
        if kind=='vio_reset':
            return '自动重定位 · 请面向清晰、有纹理的环境'
        if kind!='pose':
            return None
        pre=message.get('recovery_preaccept') or {}
        if message.get('automatic_new_map'):
            return '新地图建立中 · 正在恢复局部定位'
        prediction=pre.get('loss_prediction') or {}
        if prediction.get('status')=='predicted':
            return 'IMU 临时预测 · 当前为空间坐标未确认的临时结果'
        localization=message.get('localization') or {}
        state=localization.get('state')
        if pre.get('await_visual_requalification') or state in ('DEGRADED','VISUAL_INITIALIZING'):
            return '视觉暂时丢失 · 请放慢移动，系统正在恢复'
        if localization.get('admission',{}).get('map_pose_available'):
            return '定位正常 · 可以正常行走和识别'
        if state in ('INERTIAL_INITIALIZING','INERTIAL_REFINING','WAITING_FOR_SENSORS'):
            return '正在初始化定位 · 请缓慢移动相机，观察周围环境'
        return '定位暂时不可用 · 请暂停需要连续空间坐标的任务'

    def sync_controls(self):
        running=bool(self.session.worker and self.session.worker.isRunning())
        for w in (self.source,self.mode,self.open_button,self.results_button,self.start_button):w.setEnabled(not running)
        if self.recovery_profile:
            self.source.setEnabled(False);self.mode.setEnabled(False)
        elif self.live_recovery_profile:
            self.mode.setEnabled(False)
        self.stop_button.setEnabled(running);self.record_button.setEnabled(running and self.source.currentData()!='recorded' and not self.pending_record)
        self.pause_button.setEnabled(running and not self.session.worker.stop_event.is_set())
        self.pause_button.setText('恢复' if self.paused else '暂停')
        self.pause_button.setAccessibleName('恢复' if self.paused else '暂停')
        self.record_button.setText('停止录制' if self.recording else '开始录制')

    def import_recording(self):
        path=QFileDialog.getExistingDirectory(self,'选择项目原生录制目录',str(ROOT/'recordings'))
        if path:self.select_recording(path)

    def _source_mode_changed(self):
        if self.recovery_profile or self.live_recovery_profile:return
        current=self.mode.currentData()
        if current not in ('live_recovery_v4','offline_recovery_v4'):return
        selected='offline_recovery_v4' if self.source.currentData()=='recorded' else 'live_recovery_v4'
        self.mode.setCurrentIndex(self.mode.findData(selected))

    def select_recording(self,path):
        self.recording_source=Path(path);self.source.setCurrentIndex(self.source.findData('recorded'));self.source_label.setText('录制：'+str(path)+(' · 全分辨率恢复候选' if self.recovery_profile else ' · 按录制快照重新计算'))
        if not self.recovery_profile and not self.live_recovery_profile:
            self.mode.setCurrentIndex(self.mode.findData('offline_recovery_v4'))
            self.source_label.setText('录制：'+str(path)+' · 已选择 v4 离线恢复实验版')

    def start_pipeline(self):
        if not self.stop_pipeline():return
        try:
            kind=self.source.currentData();self.last_error=None
            if self.recovery_profile and (kind!='recorded' or self.mode.currentData()!='stereo_inertial'):
                raise ValueError('离线恢复候选仅支持双目＋IMU录制回放')
            if self.live_recovery_profile and kind!='vendor':
                raise ValueError('v4 实时恢复实验版仅支持 VS030 设备；离线录制请使用离线候选入口')
            selected_mode=self.mode.currentData()
            if selected_mode=='offline_recovery_v4' and kind!='recorded':
                raise ValueError('v4 离线恢复实验版只支持录制回放')
            if selected_mode=='live_recovery_v4' and kind not in ('vendor','synthetic'):
                raise ValueError('v4 实时恢复实验版只支持 VS030 设备')
            if kind=='recorded' and not self.recording_source:raise ValueError('请先导入录制目录')
            output=Path(self.replay_output_dir) if self.replay_output_dir else DELIVERY_ROOT/'sessions'/('replay_'+str(time.time_ns()))
            options={'vio_worker':self.live_vio_worker}
            if self.recovery_profile:options=self.recovery_profile.worker_options(self.recording_source,output)
            elif self.live_recovery_profile:options={**options,**self.live_recovery_profile.worker_options()}
            elif selected_mode=='offline_recovery_v4':
                from offline_recovery import OfflineRecoveryProfile
                options=OfflineRecoveryProfile().worker_options(self.recording_source,output,isolated_environment=True)
            elif selected_mode=='live_recovery_v4' and kind=='vendor':
                options={**options,**LiveRecoveryProfile().worker_options()}
            source=create_source(SourceRequest(kind=kind,path=str(self.recording_source or ''),loop=False))
            detector=yaml.safe_load((ROOT/'configs/detector.yaml').read_text(encoding='utf8')) if kind!='synthetic' else None
            if detector:detector['model_path']=str(ROOT/detector['model_path'])
            config=yaml.safe_load((ROOT/'configs/stereo.yaml').read_text(encoding='utf8'))
            self.generation+=1;self.smoother.reset();self._pose_fitted=False;self.latest=None;self.saved=[];self.result_slider.hide();self.last_frame_at=0
            self._world_pose=None;self._has_native=None;self._raw_cache=None;self._raw_shown=None
            self._hand_poses.clear();self._hand_pose_world=None
            self._displayed_hand_stamp=None;self._displayed_hands=None
            clear_stereo(self.left_view,self.right_view);clear_stereo(self.depth_view)
            clear_stereo(self.raw_left_view,self.raw_right_view)
            self.left_tile.set_caption('校正 · 左');self.right_tile.set_caption('校正 · 右')
            self.raw_left_tile.set_caption('原始 · 左');self.raw_right_tile.set_caption('原始 · 右')
            self.hand_overlay.clear();self.vio_panel.clear();self.vio_panel.begin_live()
            self.session.start(source,config=config,detector_config=detector,offline=kind=='recorded',live_vio=kind!='synthetic',vio_mode='stereo_inertial' if selected_mode in ('offline_recovery_v4','live_recovery_v4') else selected_mode,vio_output_dir=output,vio_worker=options.pop('vio_worker'),**options)
            self.world_meta.setText('合成演示不运行定位' if kind=='synthetic' else '等待定位初始化')
            self.set_status('正在准备');self.sync_controls()
        except Exception as exc:self.show_error(str(exc));self.sync_controls()

    def stop_pipeline(self):
        if not self.session.stop():self.show_error('后台仍在停止，请稍后重试');return False
        if self.saved_replay is not None:self.saved_replay.close();self.saved_replay=None
        self.generation+=1;self.recording=False;self.pending_record=False;self.paused=False;self._paused_vio_reset=None;self._paused_vio_world=None;self.sync_controls()
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
        self.source_label.setText('合成演示 · 非设备实测' if getattr(meta,'synthetic',False) else ('录制回放 · v4 离线恢复实验版' if self.mode.currentData()=='offline_recovery_v4' or self.recovery_profile else '录制回放 · 按原始数据重新计算') if self.source.currentData()=='recorded' else 'VS030 已连接 · 左右通道 / '+('v4 恢复候选已启用' if self.live_recovery_profile else '当前算法已启用'))

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
            self._hand_poses.clear();self._hand_pose_world=None;self._displayed_hand_stamp=None
        if message.get('type')=='pose' and message.get('mode')=='hybrid':
            source=('VIO · 惯性优化已确认' if message.get('pose_source')=='vio'
                    else '视觉引导 · '+str(message.get('fallback_reason') or '惯性位姿暂不可用'))
            self.world_meta.setText('混合 · '+source+(' · 跟踪中' if consumer_map_pose(message) is not None else ' · 等待稳定跟踪'))
            return
        if message.get('type')=='pose':
            best=getattr(self.vio_panel,'_display_pose',None)
            if best is not None and best.source=='imu_prediction':
                self.world_meta.setText(f'IMU 临时预测 · 相机轴持续显示 · {best.prediction_duration_s or 0:.2f} 秒')
            elif best is not None and best.source=='visual_candidate':
                self.world_meta.setText('视觉候选，尚未验证 · 仅供参考')
            elif best is not None and best.source=='frozen':
                self.world_meta.setText(f'位置已冻结 · 数据年龄 {best.age_ms or 0:.1f} ms')
            else:
                self.world_meta.setText('世界轨迹 · '+('跟踪中' if consumer_map_pose(message) is not None else '等待稳定跟踪'))
            motion=message.get('local_motion') or {}
            if 'local_motion' in message and (best is None or best.source in ('trusted','origin')):
                self.world_meta.setText(f'局部段 {motion.get("segment_index","—")} · '
                    +('跟踪中' if consumer_map_pose(message) is not None else '定位不可用 · 手部保留相机坐标'))
        else:
            text=message.get('message')
            if text:self.world_meta.setText(str(text))
        customer_status=self._customer_vio_status(message)
        if customer_status:self.set_status(customer_status)

    def on_finished(self):
        self.paused=False;self._paused_vio_reset=None;self._paused_vio_world=None
        self.poll();self.recording=False;self.pending_record=False;self.sync_controls()
        if not self._closing:self.set_status('已停止 · '+str(self.last_error) if getattr(self,'last_error',None) else '已停止 · 保留最后结果')

    # ---------- 每帧显示 ----------
    def poll(self):
        if self.paused:return
        item=self.session.take_latest()
        if item:
            frame,result,stats=item;self.current_frame=frame;self.latest=result;self.last_frame_at=time.monotonic()
            shown=self.smoother.update(result)
            # 中列：校正后的左右图 + 识别结果（与原来一致的数据与叠加策略）。
            display_stereo(frame,shown,self.left_view,self.right_view)
            self._show_native_frame(frame)
            pose=result.diagnostics.get('vio_pose')
            if pose:self.on_vio(pose)
            self._bind_displayed_hands(shown.hands,result)
            self._show_hand_overlay(shown.hands,self._world_pose,require_pose=isinstance(pose,dict) and 'local_motion' in pose)
            self._apply_view_toggles()
            if not self._pose_fitted:
                points=[p.T_world_from_camera_left[:3,3] for p in self.vio_panel.poses]
                camera=np.eye(4) if self._world_pose is None else self._world_pose
                joints=np.asarray([j.xyz_m for h in shown.hands for j in h.joints if j.valid])
                if len(joints):points.extend(joints@camera[:3,:3].T+camera[:3,3])
                if points:fit_view(self.preview,points);self._pose_fitted=True
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
            self.footer.setText(f"采集 {stats.get('read_fps',0):.1f} · 手部 {stats.get('processing_fps',0):.1f} · {vio_note} · 丢帧 {stats.get('dropped_frames',0)} · {time.strftime('%H:%M:%S')}")
            if time.monotonic()-self.depth_at>.08 and (self.depth_task is None or not self.depth_task.isRunning()):
                self.depth_at=time.monotonic()
                c=getattr(self,'calibration',None)
                if c:
                    if self.depth_task:self.depth_task.deleteLater()
                    self.depth_task=DepthTask(self.generation,frame,c,self.depth_cache,self)
                    self.depth_task.completed.connect(self.on_depth);self.depth_task.start()
                else:self.depth_tile.set_caption('缺少有效双目标定，深度未启用')
        elif self.last_frame_at and time.monotonic()-self.last_frame_at>2 and self.session.worker and self.session.worker.isRunning():
            self.footer.setText('等待新画面 · 上一帧已超过 2 秒，当前相机状态待确认')

    def _bind_displayed_hands(self,hands,result):
        """Exact timestamp association inside the current SessionController worker.

        This display join does not certify clock/calibration accuracy or world continuity.
        Saved/offline frame evidence is authoritative when present, including rejection.
        """
        self._displayed_hand_stamp=result.timestamp_ns;self._displayed_hands=hands
        self._displayed_hand_has_pose='vio_pose' in result.diagnostics
        if type(result.timestamp_ns) is not int or result.timestamp_ns<0:
            self._world_pose=None;return
        if self._displayed_hand_has_pose:
            self._world_pose=consumer_map_pose(result.diagnostics['vio_pose'],
                world_hands=True,expected_timestamp_ns=result.timestamp_ns)
        else:
            transform=self._hand_poses.get(result.timestamp_ns)
            self._world_pose=None if transform is None else transform.copy()

    def _show_hand_overlay(self,hands,transform,*,require_pose=False):
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

    def on_depth(self,generation,image,text):
        if generation==self.generation:
            if image is None:
                clear_stereo(self.depth_view);self.depth_tile.set_caption(text)
            else:
                self.depth_view.display(image);self.depth_tile.set_caption(text)

    # ---------- 录制 / 截图 / 回看 ----------
    def toggle_recording(self):
        if self.recording:self.pending_record=True;self.session.command('record_stop');self.sync_controls();return
        self.recording_path=DELIVERY_ROOT/'recordings'/('session_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns()%1000000000))
        self.pending_record=True;self.session.command('record_start',self.recording_path);self.sync_controls()

    def on_recording(self,active,paused,message):
        self.recording=active;self.pending_record=False;self.set_status('录制中' if active else message);self.sync_controls()

    def save_screenshot(self):
        folder=DELIVERY_ROOT/'screenshots';folder.mkdir(parents=True,exist_ok=True)
        path=folder/('client_'+time.strftime('%Y%m%d_%H%M%S')+'.png')
        if self.grab().save(str(path)):self.statusBar().showMessage('截图已保存：'+str(path))
        else:self.show_error('截图保存失败')

    def open_results(self):
        path=QFileDialog.getExistingDirectory(self,'选择含 results/frames.jsonl 的会话',str(DELIVERY_ROOT))
        if not path:return
        try:
            if not self.stop_pipeline():return
            self.saved,truncated=load_recorded_results(path)
            if not self.saved:raise ValueError('没有可回看的结果')
            clear_stereo(self.left_view,self.right_view);clear_stereo(self.depth_view)
            clear_stereo(self.raw_left_view,self.raw_right_view)
            self.hand_overlay.clear();self.vio_panel.clear()
            try:
                replay=SavedResultReplay(path,self.saved);replay.open();replay.start()
                self.saved_replay=replay
            except (OSError,ValueError,KeyError) as exc:
                if 'replay' in locals():replay.close()
                self.saved_replay=None
                self.source_label.setText('原始图像不可用：'+str(exc)+' · 可继续查看三维结果')
            self.result_slider.setRange(0,len(self.saved)-1);self.result_slider.setValue(0);self.result_slider.show();self.display_saved(0)
            self.set_status('已有结果回看'+(' · 尾部不完整' if truncated else ''))
        except Exception as exc:self.show_error(str(exc))

    def display_saved(self,index):
        if self.saved:
            self.latest=self.saved[index]
            self.vio_panel.show_recorded(self.saved,index,show_hands=False)
            self._world_pose=None
            self._hand_poses.clear();self._displayed_hand_stamp=None;self._displayed_hands=None
            # The selected hand frame must not inherit an earlier camera pose.
            self._world_pose=consumer_map_pose(self.latest.diagnostics.get('vio_pose'),
                world_hands=True,expected_timestamp_ns=self.latest.timestamp_ns)
            saved_pose=self.latest.diagnostics.get('vio_pose') or {}
            self._show_hand_overlay(self.latest.hands,self._world_pose,require_pose='local_motion' in saved_pose)
            self._apply_view_toggles()
            for view in (self.left_view,self.right_view,self.depth_view,self.raw_left_view,self.raw_right_view):clear_stereo(view)
            image_status='原始图像不可用 · 三维结果可查看'
            if self.saved_replay is not None:
                try:
                    from egohand.calibration.rectifier import StereoRectifier
                    self.saved_replay.seek(index);frame=self.saved_replay.read()
                    raw=(frame.left_image,frame.right_image)
                    if self.latest.diagnostics.get('output_geometry')=='rectified':
                        frame=StereoRectifier(self.saved_replay.calibration).rectify(frame)
                    frame.debug_native_images=raw
                    display_stereo(frame,self.latest,self.left_view,self.right_view)
                    self._show_native_frame(frame);self._sync_aspect(frame)
                    image_status='原始双目与保存结果同帧'
                except (OSError,ValueError,KeyError) as exc:image_status='图像未显示：'+str(exc)
            motion=getattr(self.latest,'local_motion',None) or {}
            segment_text=f' · 局部段 {motion.get("segment_index","—")}' if getattr(self.latest,'local_motion',None) is not None else ''
            self.source_label.setText(f'已有结果回看 · {index+1} / {len(self.saved)}{segment_text} · {image_status}')
            available=self._world_pose is not None
            self.world_meta.setText(('局部段 '+str(motion.get('segment_index','—')) if motion else '保存轨迹')+
                (' · 位姿可用' if available else ' · 当前位姿不可用'))
            hands=sum(h.valid_joint_count>0 for h in self.latest.hands)
            source=motion.get('pose_source') or saved_pose.get('mode') or '来源未知'
            self.footer.setText(f'保存结果 · {source}   ·   相机系手部 {hands}   ·   '+
                ('局部坐标可用' if available else '局部坐标不可用')+f'   ·   帧 {getattr(self.latest,"frame_id","—")}')
            self.reset_world_view()

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
        mid_h=(height-4)/3;right_h=(height-2)/2
        mid_w=int(round(mid_h*self.image_aspect));right_w=int(round(right_h*self.image_aspect))
        budget=int((self.columns.width()-8)*0.76)
        if mid_w+right_w>budget and mid_w+right_w>0:
            scale=budget/(mid_w+right_w);mid_w=int(mid_w*scale);right_w=int(right_w*scale)
        self.middle_column.setFixedWidth(max(110,mid_w))
        self.right_column.setFixedWidth(max(110,right_w))

    def showEvent(self,event):
        super().showEvent(event)
        if hasattr(self,'column_layout'):self._fit_columns()

    def resizeEvent(self,event):
        super().resizeEvent(event)
        if hasattr(self,'command_layout'):
            self.command_layout.setDirection(QBoxLayout.TopToBottom if self.width()<1250 else QBoxLayout.LeftToRight)
        if hasattr(self,'column_layout'):self._fit_columns()

    def closeEvent(self,event):
        self._closing=True
        if not self.stop_pipeline():event.ignore();self._closing=False;return
        if self.depth_task and self.depth_task.isRunning():
            if not self.depth_task.wait(5000):event.ignore();self._closing=False;return
        # GUI 真正退出时才释放常驻 ORB 进程（及其已加载的词典）。
        self.live_vio_worker.close()
        event.accept()
