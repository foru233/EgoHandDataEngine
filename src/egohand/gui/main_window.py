import json
import logging
import time
from dataclasses import replace
from pathlib import Path
import cv2
import numpy as np
import psutil
import yaml
from PySide6.QtCore import Qt,QTimer,Slot,Signal,QThread
from PySide6.QtWidgets import QMainWindow,QFileDialog,QInputDialog
from .workers import PipelineWorker
from .reconstruction_panel import DETECTOR_KEYS
from ..sources.synthetic import SyntheticStereoSource
from ..sources.recorded import RecordedStereoSource
from ..sources.image import SingleImageSource
from ..sources.hot3d import HOT3DSource,HOT3DFrameProcessor
from ..sources.show3d import SHOW3DSource,SHOW3DFrameProcessor
from ..sources.dexycb import DexYCBSource
from ..sources.vendor_placeholder import VendorStereoSource,CALIBRATION_PATH,KALIBR_LEFT_CAMERA
from ..sources.vendor_xdsdk import XdSdkStereoSource
from ..calibration.loader import load_calibration
from ..calibration.rectifier import StereoRectifier
from ..storage.exporter import export_all
from ..storage.replay import load_recorded_results
from ..core.pipeline import ROOT

from .workspace import build_workspace, apply_theme, sync_controls, present_result
from ..visualization.skeleton_view import optical_to_display
from ..visualization.display_smoothing import DisplaySmoother

class ReplayThread(QThread):
    """后台执行离线 VIO 回放；延迟 import ReplayVio，模块未就绪时给中文错误而不是让 GUI 崩溃。"""
    status=Signal(str)
    finished_ok=Signal(dict)
    failed=Signal(str)
    def __init__(self,recording,timing=None,stride=1,parent=None):
        super().__init__(parent)
        self.recording=str(recording);self.timing=timing;self.stride=max(1,int(stride))
        self._runner=None;self._cancel_requested=False
    def _on_status(self,message):
        if not isinstance(message,dict):return
        text=message.get("message")
        if text:self.status.emit(str(text))
    def run(self):
        try:
            from ..vio.replay_runtime import ReplayVio
        except Exception as exc:
            self.failed.emit("离线回放模块尚未就绪：无法导入 egohand.vio.replay_runtime（"+str(exc)+"）")
            return
        if self._cancel_requested:
            self.failed.emit("离线回放已取消");return
        try:
            self._runner=ReplayVio(self.recording,timing=self.timing,stride=self.stride,on_status=self._on_status)
            if self._cancel_requested:self._runner.cancel()
            report=self._runner.run()
        except Exception as exc:
            self.failed.emit("离线回放失败："+str(exc));return
        finally:
            self._runner=None
        self.finished_ok.emit(report if isinstance(report,dict) else {"status":"completed"})
    def cancel(self):
        self._cancel_requested=True
        runner=self._runner
        if runner is not None:
            try:runner.cancel()
            except Exception:pass

class MainWindow(QMainWindow):
    source_switched=Signal()
    def __init__(self):
        super().__init__()
        self.setWindowTitle("EgoHand Studio · 自适应工作台")
        available=self.screen().availableGeometry()
        self.resize(min(1500,int(available.width()*.92)),min(960,int(available.height()*.90)))
        self.setMinimumSize(min(1100,available.width()-40),min(720,available.height()-40))
        self.worker=None;self.loaded_calibration=None;self.model_path=ROOT/"models/hand_landmarker.task"
        self.app_config=yaml.safe_load((ROOT/"configs/app.yaml").read_text(encoding="utf-8"))
        self.display_smoother=DisplaySmoother(self.app_config.get('display_smoothing'))
        self.detector_config=yaml.safe_load((ROOT/"configs/detector.yaml").read_text(encoding="utf-8"))
        self.stereo_config=yaml.safe_load((ROOT/"configs/stereo.yaml").read_text(encoding="utf-8"))
        self.model_path=ROOT/self.detector_config["model_path"]
        self.latest=None;self.saved_results=[];self.paused=False;self.recording=False
        self.recording_paused=False;self.recording_pending=False
        self.recording_path=None;self._recording_progress_at=0.0
        self.replay_thread=None;self.replay_report=None
        self.hot3d_replay_processor=HOT3DFrameProcessor()
        self.show3d_replay_processor=SHOW3DFrameProcessor()
        self._hot3d_fit_key=None
        self.replay_source=None;self.replay_index={};self.replay_rectifier=None
        self._build()
        from .mano_controller import ManoController
        self.mano_controller=ManoController(self)
        self.reconstruction_panel.use_epipolar.setChecked(self.stereo_config.get('use_epipolar_error',True))
        self.reconstruction_panel.use_reprojection.setChecked(self.stereo_config.get('use_reprojection_error',True))
        self.reconstruction_panel.use_3d_tracking.setChecked(self.stereo_config.get('enable_3d_tracking',True))
        self._active_source_index=self.source_panel.kind.currentIndex()
        self.reconstruction_panel.display_smoothing.toggled.connect(self.display_smoothing_changed)
        from .debug_page import install_debug_page
        install_debug_page(self)
        self.theme.setCurrentIndex(1 if self.app_config.get("theme")=="light" else 0)
        apply_theme(self,"light" if self.theme.currentIndex() else "dark")
        self.reconstruction_panel.num_hands.setValue(self.detector_config.get("num_hands",2))
        self.reconstruction_panel.algorithm.setCurrentIndex(max(0,self.reconstruction_panel.algorithm.findData(self.detector_config.get('algorithm','mediapipe'))))
        for key,spin in zip(('box_threshold','keypoint_threshold'),self.reconstruction_panel.rtm_confidence):spin.setValue(self.detector_config.get(key,spin.value()))
        for key,spin in zip(("min_hand_detection_confidence","min_hand_presence_confidence","min_tracking_confidence"),self.reconstruction_panel.confidence):spin.setValue(self.detector_config.get(key,.5))
        self.detector_config.update(self.reconstruction_panel.settings())
        self.reconstruction_panel.mark_applied(self.reconstruction_panel.settings(),"参数已就绪；修改后点击应用。合成投影不运行模型。")
        self.source_panel.scenario.setCurrentIndex(max(0,self.source_panel.scenario.findData(self.app_config.get("scenario","crossing"))))
        self.source_panel.offline.setChecked(self.app_config.get("mode")=="offline")
        # 默认来源由 app.yaml 决定；下拉框变更会初始化相应路径和状态。
        source_indices={"synthetic":0,"recorded":1,"image":2,"vendor":3,"hot3d":4,"show3d":5}
        self.source_panel.kind.setCurrentIndex(source_indices.get(self.app_config.get("source","show3d"),5))
        self.timer=QTimer(self);self.timer.timeout.connect(self.poll);self.timer.start(33)
    def _build(self):
        build_workspace(self)
    def display_smoothing_changed(self,enabled):
        self.display_smoother.reset()
        self.statusBar().showMessage('稳定显示已'+('启用' if enabled else '关闭')+'；检测和导出仍使用原始数据。')
    def show_joint(self,text):
        self.joint_info.setText(text)
        self.joint_info.setToolTip(text)
    def reset_scene_view(self):
        self.skeleton.reset_camera();self._hot3d_fit_key=None
        if self.latest:self.fit_hot3d_scene(self.latest)
    def ground_truth_toggled(self,enabled):
        frame=getattr(self,'_debug_frame',None)
        hands=frame.metadata.get('ground_truth_left_camera',[]) if frame else []
        self.skeleton.set_ground_truth(hands,enabled)
        self.statusBar().showMessage(('真值：紫色小球，左相机坐标' if hands else '当前帧无可显示的官方真值') if enabled else '真值显示已关闭')
    def mano_toggled(self,*_):
        controller=getattr(self,'mano_controller',None)
        if controller:
            controller.reset()
            if self.latest:controller.update(self.latest,getattr(self,'_debug_frame',None))
    def fit_hot3d_scene(self,result):
        key=result.diagnostics.get('hot3d_sequence_id') or result.diagnostics.get('show3d_sequence_id')
        if not key or key==self._hot3d_fit_key:return
        points=np.array([j.xyz_m for h in result.hands for j in h.joints if j.valid])
        if not len(points):return
        # 自动取景同时覆盖左相机原点和手部；不改变任何测量坐标。
        points=np.vstack((optical_to_display(points),np.zeros((1,3))))
        center=(points.min(axis=0)+points.max(axis=0))/2
        radius=float(np.linalg.norm(points-center,axis=1).max())
        self.skeleton.setCameraPosition(distance=max(.85,radius*4))
        self._hot3d_fit_key=key
    def source_changed(self,*_):
        # 停止失败时保留原来的来源和会话，避免界面与仍运行的后台线程不一致。
        if not self.stop_pipeline():
            self.source_panel.kind.blockSignals(True)
            self.source_panel.kind.setCurrentIndex(self._active_source_index)
            self.source_panel.kind.blockSignals(False)
            return
        if hasattr(self,'mano_controller'):self.mano_controller.reset()
        self._active_source_index=self.source_panel.kind.currentIndex()
        self.display_smoother.reset()
        for card,title in ((self.left_card,'左相机'),(self.right_card,'右相机')):
            card.header.itemAt(0).widget().setText(title)
        self.source_panel.update_kind(self.source_panel.kind.currentIndex())
        self.source_switched.emit()
        if self.source_panel.kind.currentIndex()==5:
            self.source_panel.path.setText(str(ROOT/"samples/date/SHOW3D"))
        if self.source_panel.kind.currentIndex()==6:
            self.source_panel.path.setText(str(ROOT/"samples/date/DexYCB"))
        self.latest=None;self.saved_results=[];self.loaded_calibration=None
        self.provenance_badge.setText('等待数据')
        self.provenance_badge.setToolTip('启动数据源后显示当前帧的来源')
        self.source_panel.seek_index.setMaximum(10_000_000)
        sync_controls(self)
        if self.source_panel.kind.currentIndex()==5:
            self.calibration_panel.label.setText("SHOW3D · 等待启动数据源\n相机参数和外参将在处理图像时逐帧读取")
        else:
            self.calibration_panel.label.setText("等待数据源启动后检查标定")
        self.banner.setText("Ready · 数据源已切换，时序状态已重置")
    def browse_source(self):
        kind=self.source_panel.kind.currentIndex()
        if kind==1:
            path,_=QFileDialog.getOpenFileName(self,"选择录制 manifest.json / 外部数据清单",str(ROOT),"JSON (*.json)")
        elif kind==2:
            path,_=QFileDialog.getOpenFileName(self,"选择真实手部图片",str(ROOT/"samples"),"Images (*.png *.jpg *.jpeg)")
        elif kind==4:
            path,_=QFileDialog.getOpenFileName(self,"选择 HOT3D tar（目录可在采集配置中选择）",str(ROOT/"samples/date/hot3d"),"HOT3D (*.tar)")
        elif kind in (5,6):
            self.browse_hot3d_directory();return
        else: return
        if path: self.source_panel.path.setText(path)
    def browse_hot3d_directory(self):
        if self.source_panel.kind.currentIndex()==6:
            path=QFileDialog.getExistingDirectory(self,"选择 DexYCB 数据集、受试者或序列目录",str(ROOT/"samples/date/DexYCB"))
            if path:self.source_panel.path.setText(path)
            return
        is_show3d=self.source_panel.kind.currentIndex()==5
        path=QFileDialog.getExistingDirectory(self,"选择 SHOW3D 根目录或单场景目录" if is_show3d else "选择 HOT3D 目录",str(ROOT/"samples/date/SHOW3D" if is_show3d else ROOT/"samples/date/hot3d"))
        if path:self.source_panel.path.setText(path)
    def hot3d_mode_changed(self,*_):
        if self.worker and self.worker.isRunning() and self.source_panel.kind.currentIndex() in (4,5):
            self.worker.command("hot3d_mode",{"mode":self.source_panel.hot3d_mode.currentData(),"frame_index":self.latest.frame_id if self.latest else 0})
            self.paused=True
    def _current_worker_signal(self):
        # wait() 不会清除已经排入 GUI 事件队列的旧线程信号。
        sender=self.sender()
        return not isinstance(sender,PipelineWorker) or sender is self.worker
    @Slot(str)
    def on_hot3d_mode(self,mode):
        if not self._current_worker_signal():return
        combo=self.source_panel.hot3d_mode
        combo.blockSignals(True);combo.setCurrentIndex(combo.findData(mode));combo.blockSignals(False)
    @Slot(str)
    def _error(self,text):
        if not self._current_worker_signal():return
        logging.error(text)
        self.statusBar().showMessage(text)
        self.banner.setText("Error · "+text)
        if (self.source_panel.kind.currentIndex()==5 and
                ('等待首帧' in self.calibration_panel.label.text() or
                 '等待启动数据源' in self.calibration_panel.label.text())):
            self.calibration_panel.label.setText('SHOW3D · 数据源或标定尚未就绪\n请查看状态栏错误并检查数据目录')
        self.banner.setToolTip(text)
        self.banner.setProperty("tone","error")
        self.banner.style().unpolish(self.banner);self.banner.style().polish(self.banner)
    def load_calibration(self):
        path,_=QFileDialog.getOpenFileName(self,"加载标定",str(ROOT/"configs"),"Calibration (*.yaml *.yml *.json)")
        if not path:return
        if not self.stop_pipeline():return
        try:
            data=yaml.safe_load(Path(path).read_text(encoding='utf-8-sig')) if Path(path).suffix.lower() in ('.yaml','.yml') else None
            left=None
            if isinstance(data,dict) and 'cam0' in data and 'cam1' in data:
                choice,ok=QInputDialog.getItem(self,'选择 Kalibr 通道对应','当前左图对应哪个标定相机？',['cam0','cam1'],0,False)
                if not ok:return
                left=choice
            self.loaded_calibration=load_calibration(path,kalibr_left_camera=left)
            c=self.loaded_calibration
            self.calibration_panel.label.setText(f"{c.calibration_id}\n{path}\n{'SYNTHETIC' if c.synthetic else c.calibration_source}")
        except Exception as e:
            self.calibration_panel.label.setText(("保留原标定 · " if self.loaded_calibration else "Missing Calibration · ")+str(e));self._error(str(e))
    def load_model(self):
        path,_=QFileDialog.getOpenFileName(self,"选择官方 Hand Landmarker 模型",str(ROOT/"models"),"Model (*.task)")
        if path:
            if not self.stop_pipeline():return
            self.model_path=Path(path)
            self.statusBar().showMessage("模型路径已设置，将在后台启动时验证加载")
    def make_source(self):
        kind=self.source_panel.kind.currentIndex()
        if kind==0: return SyntheticStereoSource(self.source_panel.scenario.currentData(),frames=18000,fps=self.app_config.get("preview_fps",30))
        path=self.source_panel.path.text()
        if kind==1: return RecordedStereoSource(path,self.source_panel.loop.isChecked(),self.source_panel.reuse.isChecked())
        if kind==2: return SingleImageSource(path or ROOT/"samples/public/hand.jpg")
        if kind==4:return HOT3DSource(path,loop=self.source_panel.loop.isChecked(),view_mode=self.source_panel.hot3d_mode.currentData())
        if kind==5:return SHOW3DSource(path or ROOT/"samples/date/SHOW3D",loop=self.source_panel.loop.isChecked(),view_mode=self.source_panel.hot3d_mode.currentData())
        if kind==6:return DexYCBSource(path or ROOT/"samples/date/DexYCB",loop=self.source_panel.loop.isChecked())
        return XdSdkStereoSource()
    def apply_detection_settings(self):
        settings=self.reconstruction_panel.settings()
        if self.worker and self.worker.isRunning():
            self.reconstruction_panel.set_busy(True)
            self.worker.command("detector_config",{"model_path":str(self.model_path),**settings})
        else:
            self._detector_settings_applied(settings,"已应用，下次开始检测时生效；仅当前程序会话保存。")
    def epipolar_setting_changed(self,enabled):
        self.stereo_config['use_epipolar_error']=bool(enabled)
        if self.worker and self.worker.isRunning():
            self.worker.command('use_epipolar_error',bool(enabled))
        self.statusBar().showMessage('极线误差约束已'+('启用' if enabled else '关闭')+'；运行中从后续处理帧生效。')
    def reprojection_setting_changed(self,enabled):
        self.stereo_config['use_reprojection_error']=bool(enabled)
        if self.worker and self.worker.isRunning():self.worker.command('use_reprojection_error',bool(enabled))
        self.statusBar().showMessage('重投影误差约束已'+('启用' if enabled else '关闭')+'；运行中从后续处理帧生效。')
    def tracking_3d_setting_changed(self,enabled):
        self.stereo_config['enable_3d_tracking']=bool(enabled)
        if self.worker and self.worker.isRunning():self.worker.command('enable_3d_tracking',bool(enabled))
        self.statusBar().showMessage('三维跨帧 ID 跟踪已'+('启用' if enabled else '关闭')+'；轨迹将重置。')
    @Slot(dict,str)
    def _detector_settings_applied(self,settings,message):
        if not self._current_worker_signal():return
        self.detector_config.update({key:settings[key] for key in DETECTOR_KEYS})
        self.reconstruction_panel.mark_applied(settings,message)
        self.statusBar().showMessage(message)
    @Slot(str)
    def _detector_settings_failed(self,message):
        if not self._current_worker_signal():return
        self.reconstruction_panel.set_busy(False)
        self.reconstruction_panel.apply_hint.setText("应用失败，旧参数继续使用："+message)
        self._error("检测参数应用失败："+message)
    def start_pipeline(self):
        if not self.stop_pipeline():return
        self._hot3d_fit_key=None
        try:
            source=self.make_source()
            detector=None
            if self.source_panel.kind.currentIndex() in (1,2,3,4,5,6):
                detector={"model_path":str(self.model_path),**{key:self.detector_config[key] for key in DETECTOR_KEYS}}
            self.worker=PipelineWorker(source,config={k:self.stereo_config.get(k,k!='vs030_user_confirmed_sync') for k in ('use_epipolar_error','use_reprojection_error','enable_3d_tracking','vs030_user_confirmed_sync')},detector_config=detector,calibration=self.loaded_calibration,offline=self.source_panel.offline.isChecked(),parent=self,rois=self.roi_panel.settings(),live_vio=(isinstance(source,XdSdkStereoSource) and self.vio_panel.live_enabled.isChecked()))
            if self.worker.live_vio_enabled:self.vio_panel.begin_live()
            self.worker.vio_status.connect(self.vio_panel.update_live)
            self.worker.error.connect(self._error);self.worker.ready.connect(self.on_ready)
            self.worker.startup_status.connect(self.on_startup_status)
            self.worker.hot3d_mode_applied.connect(self.on_hot3d_mode)
            self.worker.detector_settings_applied.connect(self._detector_settings_applied)
            self.worker.detector_settings_failed.connect(self._detector_settings_failed)
            self.worker.roi_settings_applied.connect(self.on_roi_settings_applied)
            self.worker.roi_settings_failed.connect(self.on_roi_settings_failed)
            self.worker.recording_state_changed.connect(self.on_recording_state)
            self.worker.finished.connect(self.on_worker_finished)
            self.worker.start();self.paused=False;self.latest=None;self.saved_results=[]
            self.banner.setText("Running · 正在初始化数据源")
            sync_controls(self)
        except Exception as e:self._error(str(e))
    @Slot(str)
    def on_startup_status(self,message):
        if self._current_worker_signal() and self.worker and not self.worker.stop_event.is_set():
            self.banner.setText(f'Running · {message}')
    @Slot(dict,str)
    def on_roi_settings_applied(self,settings,message):
        if not self._current_worker_signal():return
        selected=self.roi_panel.side.currentIndex()
        for side,view in (('left',self.left_view),('right',self.right_view)):
            self.roi_panel.set_roi(side,settings[side],emit=False);view.set_roi(settings[side])
        self.roi_panel.side.setCurrentIndex(selected)
        self.roi_panel.hint.setText(message)
    @Slot(str)
    def on_roi_settings_failed(self,message):
        if self._current_worker_signal():self.roi_panel.hint.setText('ROI应用失败：'+message)
    @Slot()
    def on_worker_finished(self):
        if not self._current_worker_signal() or self.worker is None:return
        # 先呈现最后一帧，再标记结束；保留已退出线程的缓存供导出和单帧查看。
        self.poll()
        self.saved_results=list(self.worker.history) or self.saved_results
        self.paused=False;self.recording=False;self.recording_paused=False;self.recording_pending=False
        if self.recording_path is not None:
            self._finish_recording_summary(self.recording_path);self.recording_path=None
        self.reconstruction_panel.set_busy(False)
        if self.worker.last_error:self._error(self.worker.last_error)
        else:self.banner.setText('Stopped · 数据处理已结束')
        if self.worker.live_vio_enabled and not self.vio_panel.details.text().startswith('实时 VIO 已停止'):
            self.vio_panel.details.setText('实时 VIO 已停止 · 保留最后一段轨迹供查看')
        sync_controls(self)
    @Slot(dict)
    def on_ready(self,info):
        if not self._current_worker_signal() or (self.worker and self.worker.stop_event.is_set()):return
        c=info["calibration"]
        if info['metadata'].source_type=='vendor_vs030':
            self.left_card.header.itemAt(0).widget().setText('左相机 · 通道 4')
            self.right_card.header.itemAt(0).widget().setText('右相机 · 通道 5')
            self.statusBar().showMessage('VS030：同步由用户确认；左右曝光时间差尚未测量')
            # 已有轨迹标定不能被隐式覆盖；否则预载该设备的相机—IMU 外参。
            if c and c.calibration_id.startswith('kalibr-data3-camchain-imucam-') and not self.vio_panel.calibration_path:
                try:self.vio_panel.set_calibration(CALIBRATION_PATH,kalibr_camera=KALIBR_LEFT_CAMERA)
                except Exception as exc:self.statusBar().showMessage('相机—IMU 标定未加载：'+str(exc))
        if info['metadata'].source_type=='dexycb':
            self.source_panel.seek_index.setMaximum(info['metadata'].extra['frame_count']-1)
        if info['metadata'].source_type=='show3d':
            self.source_panel.seek_index.setMaximum(info['metadata'].extra['frame_count']-1)
            self.calibration_panel.label.setText('SHOW3D · 数据源已打开\n等待首帧读取相机参数和外参')
            self.statusBar().showMessage('SHOW3D 已打开；GT 真值与算法预测分开查看；非本地相机标定')
            sync_controls(self)
            return
        if info['metadata'].source_type=='hot3d_clips':
            self.source_panel.seek_index.setMaximum(info['metadata'].extra['frame_count']-1)
            self.calibration_panel.label.setText('HOT3D · 官方 FISHEYE624\n逐帧标定 · 1201-1 / 1201-2\n同步依据：数据集记录')
            self.statusBar().showMessage('HOT3D 已打开；官方真值与算法预测分开查看')
            sync_controls(self)
            return
        if c:self.calibration_panel.label.setText(f"{c.calibration_id}\n{'SYNTHETIC' if c.synthetic else c.calibration_source}\n已建立校正映射")
        else:self.calibration_panel.label.setText("Missing Calibration · 2D Only")
        self.statusBar().showMessage("Running · "+("Model Loaded" if info["model_loaded"] else "Synthetic Geometry / Model Not Loaded"))
        sync_controls(self)
    def toggle_pause(self):
        if self.worker and self.worker.isRunning():
            self.paused=not self.paused;self.worker.pause(self.paused)
            self.banner.setText("Paused · 画面已暂停" if self.paused else "Running · 数据流运行中")
            sync_controls(self)
    def request_pipeline_stop(self):
        if not self.worker or not self.worker.isRunning():
            self.stop_pipeline()
            return
        self.worker.stop()
        self.banner.setText('正在停止采集…')
        sync_controls(self)

    def stop_pipeline(self):
        if self.worker:
            self.worker.stop()
            if not self.worker.wait(15000):
                self._error("后台线程尚未退出，请检查数据源读取超时")
                return False
            self.saved_results=list(self.worker.history) or self.saved_results
            self.worker.deleteLater();self.worker=None
        if self.recording_path is not None:
            # 录制随数据源一起停止时也要展示该会话的 manifest/replay_check 摘要。
            self._finish_recording_summary(self.recording_path);self.recording_path=None
        if self.replay_source:
            self.replay_source.close();self.replay_source=None
        self.recording=False;self.recording_paused=False;self.recording_pending=False;self.paused=False
        self.reconstruction_panel.set_busy(False)
        if hasattr(self,"banner"):self.banner.setText("Stopped")
        if hasattr(self,"state_badge"):sync_controls(self)
        return True
    def step_frame(self):
        if self.saved_results and not (self.worker and self.worker.isRunning()):
            i=min(self.source_panel.seek_index.value(),len(self.saved_results)-1)
            self.display_saved(i);self.source_panel.seek_index.setValue(min(i+1,len(self.saved_results)-1))
        elif self.worker:self.worker.step();self.paused=True
    def seek_frame(self):
        if self.worker and self.worker.isRunning():self.worker.seek(self.source_panel.seek_index.value());self.paused=True
        elif self.saved_results:self.display_saved(self.source_panel.seek_index.value())
    def start_recording(self):
        if not self.worker or not self.worker.isRunning():self._error("请先启动数据源");return
        if self.recording or self.recording_pending:return
        # 优先使用面板里已选好的保存位置；只有为空时才弹目录对话框。
        directory=self.recording_panel.get_directory()
        if not directory:
            directory=QFileDialog.getExistingDirectory(self,"选择录制保存目录",str(self.recording_panel.get_directory() or ROOT/"recordings"))
            if not directory:return
            self.recording_panel.set_directory(directory)
        # 同一秒内连续开始/停止不能复用目录；实际录制状态由后台确认。
        path=Path(directory)/(time.strftime("session_%Y%m%d_%H%M%S")+f'_{time.time_ns()%1_000_000_000:09d}')
        self.recording_pending=True;self.recording_path=path
        self._recording_progress_at=0.0
        self.worker.command("record_start",path)
        self.statusBar().showMessage("正在启动录制："+str(path))
        sync_controls(self)
    def toggle_recording(self):
        if self.recording:self.pause_recording(not self.recording_paused)
        else:self.start_recording()
    def pause_recording(self,value):
        if not self.recording or self.recording_pending or not self.worker:return
        self.recording_pending=True
        self.worker.command("record_pause",bool(value))
        sync_controls(self)
    def stop_recording(self):
        if not self.worker or self.recording_pending:return
        self.recording_pending=True
        self.worker.command("record_stop")
        sync_controls(self)
    @Slot(bool,bool,str)
    def on_recording_state(self,recording,paused,message):
        if not self._current_worker_signal():return
        self.recording=recording;self.recording_paused=paused if recording else False;self.recording_pending=False
        self.statusBar().showMessage(message)
        if recording:
            self._recording_progress_at=0.0
            self._poll_recording_progress()
        else:
            # 停止或启动失败：有会话路径就按 manifest.json 给出回放校验摘要。
            session=self.recording_path;self.recording_path=None
            if session is not None:self._finish_recording_summary(session)
        sync_controls(self)
    def _finish_recording_summary(self,session):
        """停止后读该会话的 manifest.json（replay_check）并展示到面板。"""
        session=Path(session)
        manifest=None
        try:manifest=json.loads((session/"manifest.json").read_text(encoding="utf-8"))
        except (OSError,ValueError):manifest=None
        check=manifest.get("replay_check") if isinstance(manifest,dict) else None
        if not isinstance(check,dict):
            # 兼容只落盘 replay_check.json、尚未同步进 manifest 的旧录制。
            try:check=json.loads((session/"replay_check.json").read_text(encoding="utf-8"))
            except (OSError,ValueError):check=None
        if not isinstance(manifest,dict) and not isinstance(check,dict):
            self.recording_panel.set_replay_status("未找到 manifest.json，录制可能未开始："+str(session))
            return
        self.recording_panel.set_last_session(session,check)
        status=str(manifest.get("recording_status","")) if isinstance(manifest,dict) else ""
        if status=="incomplete":
            self.recording_panel.set_replay_status("录制已停止，但记录不完整："+str(manifest.get("incomplete_reason") or "未给出原因"))
    def _poll_recording_progress(self):
        """录制期间 ≤2 Hz 刷新面板计数；只读 writer.manifest，不干预写盘线程。"""
        if not self.recording:return
        now=time.perf_counter()
        if now-self._recording_progress_at<0.5:return
        self._recording_progress_at=now
        recorder=getattr(self.worker,"recorder",None) if self.worker else None
        writer=getattr(recorder,"writer",None) if recorder is not None else None
        manifest=getattr(writer,"manifest",None) if writer is not None else None
        if not isinstance(manifest,dict):return
        by_sink=manifest.get("sdk_count_by_sink") or {}
        def count(sink):
            try:return int(by_sink.get(str(sink),0) or 0)
            except (TypeError,ValueError):return 0
        frames=manifest.get("frame_index_count") or 0
        if not frames:frames=manifest.get("raw_count") or 0
        progress={"frames":int(frames)}
        # 只有 SDK 录制才有“视频包 / IMU 包”概念；普通 PNG 录制不显示 0 值以免误导。
        if by_sink:
            left=count(4);right=count(5);imu=count(7)
            try:payload=float(manifest.get("sdk_payload_bytes") or 0)
            except (TypeError,ValueError):payload=0.0
            progress.update({"video_packets":left+right,"left_packets":left,"right_packets":right,
                "imu_packets":imu,"mb":payload/1e6})
        started=getattr(recorder,"started",None)
        progress["elapsed_s"]=max(0.0,now-started) if isinstance(started,(int,float)) else None
        self.recording_panel.set_progress(progress)
    @Slot(dict)
    def replay_recording(self,request):
        """面板“离线 VIO 回放…”→ 后台线程执行，不阻塞界面；模块缺失时给中文提示。"""
        if not isinstance(request,dict):return
        recording=request.get("recording")
        if not recording:return
        if self.recording or self.recording_pending:
            self.recording_panel.set_replay_status("录制进行中，停止录制后再回放。");return
        if self.replay_thread is not None and self.replay_thread.isRunning():
            self.recording_panel.set_replay_status("已有离线回放正在运行，请等待其完成。");return
        timing={}
        try:
            data=yaml.safe_load((ROOT/"configs/vio/live_timing.yaml").read_text(encoding="utf-8")) or {}
            if isinstance(data,dict):timing=dict(data)
        except (OSError,yaml.YAMLError):timing={}
        try:
            if request.get("lag_ms") is not None:timing["camera_to_imu_lag_ms"]=float(request["lag_ms"])
        except (TypeError,ValueError):pass
        if request.get("alignment") in ("lattice","timestamp"):timing["imu_alignment"]=request["alignment"]
        try:stride=max(1,int(request.get("stride") or 1))
        except (TypeError,ValueError):stride=1
        self.recording_panel.set_replay_running(True)
        self.recording_panel.set_replay_status("离线 VIO 回放启动中…（后台线程，界面可继续操作）")
        self.statusBar().showMessage("离线 VIO 回放启动中："+str(recording))
        thread=ReplayThread(recording,timing=timing,stride=stride,parent=self)
        thread.status.connect(self._on_replay_status)
        thread.finished_ok.connect(self._on_replay_finished)
        thread.failed.connect(self._on_replay_failed)
        thread.finished.connect(thread.deleteLater)
        self.replay_thread=thread
        thread.start()
        sync_controls(self)
    @Slot(str)
    def _on_replay_status(self,text):
        self.recording_panel.set_replay_status(str(text),tooltip=str(text))
        self.statusBar().showMessage(str(text))
    @Slot(dict)
    def _on_replay_finished(self,report):
        self.replay_thread=None;self.replay_report=report
        self.recording_panel.set_replay_running(False)
        status=str(report.get("status") or "completed")
        summary=self._replay_summary(report)
        output=report.get("output") or {}
        output_dir=output.get("dir") if isinstance(output,dict) else None
        if status=="completed":
            self.recording_panel.set_replay_status("离线 VIO 回放完成\n"+summary,tooltip=str(output_dir or summary))
            self.statusBar().showMessage("离线 VIO 回放完成 · "+self._replay_brief(report))
        else:
            self.recording_panel.set_replay_status(f"离线 VIO 回放未完成（{status}）\n"+summary,tooltip=str(output_dir or summary))
            self.statusBar().showMessage("离线 VIO 回放未完成 · "+str(report.get("error") or status))
        self.recording_panel.set_replay_output(output_dir)
        sync_controls(self)
    @Slot(str)
    def _on_replay_failed(self,message):
        self.replay_thread=None
        self.recording_panel.set_replay_running(False)
        text="离线 VIO 回放失败："+str(message)
        self.recording_panel.set_replay_status(text,tooltip=text)
        self._error(text)
        sync_controls(self)
    @staticmethod
    def _replay_summary(report):
        def number(source,key):
            if not isinstance(source,dict):return None
            value=source.get(key)
            return int(value) if isinstance(value,(int,float)) and not isinstance(value,bool) else None
        frames=report.get("frames") or {}
        lines=[]
        processed=number(frames,"processed")
        if processed is not None:
            parts=[f"处理 {processed} 帧"]
            for key,title in (("ok","跟踪 OK"),("valid_poses","有效位姿")):
                value=number(frames,key)
                if value is not None:parts.append(f"{title} {value}")
            lines.append(" · ".join(parts))
        imu=report.get("imu") or {}
        rows=number(imu,"rows")
        if rows is not None:
            text=f"IMU {rows} 条"
            rate=imu.get("rate_hz") if isinstance(imu,dict) else None
            if isinstance(rate,(int,float)) and not isinstance(rate,bool):text+=f" · {float(rate):.1f} Hz"
            lines.append(text)
        epochs=report.get("map_epochs")
        if isinstance(epochs,list):lines.append(f"地图 epoch {len(epochs)} 段")
        elapsed=report.get("elapsed_s")
        if isinstance(elapsed,(int,float)) and not isinstance(elapsed,bool):lines.append(f"耗时 {float(elapsed):.1f} s")
        if report.get("error"):lines.append("原因："+str(report["error"]))
        output=report.get("output") or {}
        replay_path=output.get("replay") if isinstance(output,dict) else None
        if replay_path:lines.append("报告 replay.json（完整路径见“打开结果”/悬停）")
        return "\n".join(lines) or "（无摘要）"
    @staticmethod
    def _replay_brief(report):
        frames=report.get("frames") or {}
        def number(source,key):
            value=source.get(key) if isinstance(source,dict) else None
            return int(value) if isinstance(value,(int,float)) and not isinstance(value,bool) else None
        parts=[]
        processed=number(frames,"processed")
        if processed is not None:parts.append(f"处理 {processed} 帧")
        valid=number(frames,"valid_poses")
        if valid is not None:parts.append(f"有效位姿 {valid}")
        output=report.get("output") or {}
        if isinstance(output,dict) and output.get("dir"):parts.append(str(output["dir"]))
        return " · ".join(parts)
    def open_results(self):
        path=QFileDialog.getExistingDirectory(self,"选择原生录制目录",str(ROOT/"recordings"))
        if not path:return
        if not self.stop_pipeline():return
        replay=None
        try:
            results,truncated=load_recorded_results(path)
            if not results:raise ValueError('录制目录没有可回放的处理结果')
            replay=RecordedStereoSource(path,reuse_detections=True)
            replay.open();replay.start()
            index={(int(row["frame_id"]),int(row["timestamp_ns"])):i for i,row in enumerate(replay.rows)}
            rectifier=StereoRectifier(replay.calibration) if replay.calibration else None
            self.saved_results=results;self.replay_source=replay
            self.replay_index=index;self.replay_rectifier=rectifier
            self.source_panel.seek_index.setRange(0,len(results)-1)
            self.source_panel.seek_index.setValue(0)
            self.display_saved(0)
            if self._debug_frame is not None:self.statusBar().showMessage(f"已有结果 {len(self.saved_results)} 帧；尾行恢复={truncated}")
        except Exception as e:
            if replay is not None and replay is not self.replay_source:replay.close()
            self._error(str(e))
    def _clear_replay_images(self):
        self._debug_frame=None
        for view,card in ((self.left_view,self.left_card),(self.right_view,self.right_card)):
            view.set_roi_selection(False)
            view.pix=None;view.last_bgr=None;view._image_size=None
            view.clear();view.update();card.meta.setText('本帧图像不可用')
        self.roi_panel.sizes.clear();self.roi_panel.draw.setChecked(False);self.roi_panel._sync_fields()
    def _prepare_replay_frame(self,frame):
        if frame.metadata.get('hot3d'):
            self.replay_rectifier,frame=self.hot3d_replay_processor.prepare(frame)
        elif frame.metadata.get('show3d'):
            from show3d.dataset import CameraCalibration
            # JSON 录制会把官方 dataclass/ndarray 转成字典和列表，按原字段还原。
            show=dict(frame.metadata['show3d'])
            show['calibration']=[CameraCalibration(**{**c,'t_world_from_camera':
                np.asarray(c['t_world_from_camera'],dtype=float) if c['t_world_from_camera'] is not None else None})
                if isinstance(c,dict) else c for c in show['calibration']]
            show['ground_truth']={side:{**entry,'world_mm':np.asarray(entry['world_mm'],dtype=float)}
                for side,entry in show['ground_truth'].items()}
            frame=replace(frame,metadata={**frame.metadata,'show3d':show})
            self.replay_rectifier,frame=self.show3d_replay_processor.prepare(frame)
        elif self.latest.diagnostics.get('output_geometry')=='rectified' and self.replay_rectifier:
            frame=self.replay_rectifier.rectify(frame)
        if self.latest.diagnostics.get('output_geometry')=='rectified' and frame.image_geometry!='rectified':
            raise ValueError('缺少匹配的标定，不能将校正关键点叠加到原始图像')
        return frame
    def display_saved(self,index):
        if not self.saved_results:return
        r=self.saved_results[min(max(index,0),len(self.saved_results)-1)]
        self.latest=r
        self.fit_hot3d_scene(r)
        self._clear_replay_images()
        frame=None;image_error='本帧原始图像不可用；仅显示已有三维结果'
        key=(r.frame_id,r.timestamp_ns)
        if self.replay_source and key in self.replay_index:
            try:
                self.replay_source.seek(self.replay_index[key])
                frame=self.replay_source.read()
                frame=self._prepare_replay_frame(frame)
                self._debug_frame=frame
                p=self.reconstruction_panel
                for side,view,img,ds in (("left",self.left_view,frame.left_image,r.left_detections),("right",self.right_view,frame.right_image,r.right_detections)):
                    pairs=None if r.diagnostics.get('result_kind') in ('official_ground_truth','ground_truth_unavailable') or r.diagnostics.get('synthetic') else r.diagnostics.get('association_quality',{}).get('accepted_pairs',[])
                    view.display(img,ds,r.hands,p.numbers.isChecked(),p.epilines.isChecked(),side,self.replay_rectifier.epipolar_axis if self.replay_rectifier else 1,accepted_pairs=pairs)
                image_error=None
            except Exception as e:
                frame=None;self._clear_replay_images()
                image_error=f"已有结果的原始图像读取失败：{e}"
        self.banner.setText("已有处理结果回放 · "+r.reconstruction_status+f" · frame {r.frame_id}")
        self.metrics.setText(f"frame {r.frame_id}\n原处理会话 {r.session_id}\n{r.diagnostics}")
        present_result(self,r,frame=frame)
        self.mano_controller.update(r,frame)
        self.skeleton.display(r)
        if image_error:self._error(image_error)
    def export_current(self):
        self._export([self.latest] if self.latest else [])
    def export_results(self):
        results=list(self.worker.history) if self.worker else self.saved_results
        if not results and self.latest:results=[self.latest]
        self._export(results)
    def _export(self,results):
        if not results:self._error("暂无结果可导出");return
        directory=QFileDialog.getExistingDirectory(self,f"导出 {len(results)} 帧（运行缓存最多 600；完整会话请先打开已有结果）",str(ROOT/"exports"))
        if not directory:return
        try:
            path=export_all(results,Path(directory)/time.strftime("export_%Y%m%d_%H%M%S"))
            if self.left_view.last_bgr is not None:cv2.imwrite(str(path/"left_display.png"),self.left_view.last_bgr)
            if self.right_view.last_bgr is not None:cv2.imwrite(str(path/"right_display.png"),self.right_view.last_bgr)
            self.skeleton.grabFramebuffer().save(str(path/"skeleton.png"))
            self.statusBar().showMessage(f"已导出 {len(results)} 帧：{path}")
        except Exception as e:self._error(str(e))
    def poll(self):
        self._poll_recording_progress()
        if not self.worker or not self.worker.output:return
        frame,r,stats=self.worker.output.pop()
        self._debug_frame=frame
        self.latest=r
        shown=self.display_smoother.update(r) if self.reconstruction_panel.display_smoothing.isChecked() else r
        if r.source_type=='show3d':
            if frame.image_geometry=='rectified' and r.calibration_id:
                self.calibration_panel.label.setText('SHOW3D · 本帧标定可用\n官方 PinholePlane · 左右相机逐帧外参\n已建立双目校正')
            else:
                self.calibration_panel.label.setText('SHOW3D · 本帧标定不可用\n无法完成双目校正和三维重建')
        p=self.reconstruction_panel
        epi_axis=self.worker.engine.rectifier.epipolar_axis if self.worker.engine.rectifier else 1
        for side,view,image,ds in (("left",self.left_view,frame.left_image,shown.left_detections),("right",self.right_view,frame.right_image,shown.right_detections)):
            pairs=None if r.diagnostics.get('result_kind') in ('official_ground_truth','ground_truth_unavailable') or r.diagnostics.get('synthetic') else r.diagnostics.get('association_quality',{}).get('accepted_pairs',[])
            view.display(image,ds,shown.hands,p.numbers.isChecked(),p.epilines.isChecked(),side,epi_axis,accepted_pairs=pairs)
        self.skeleton.set_ground_truth(frame.metadata.get('ground_truth_left_camera',[]),self.show_ground_truth.isChecked(),redraw=False)
        self.mano_controller.update(r,frame)
        self.skeleton.display(shown)
        self.fit_hot3d_scene(r)
        if not self.worker.stop_event.is_set():
            self.banner.setText(("Paused · " if self.paused else "Running · ")+r.reconstruction_status+(" · "+r.diagnostics["reason"] if r.diagnostics.get("reason") else ""))
        d=r.diagnostics
        lines=[f"来源：{r.source_type} · frame {r.frame_id}",f"标定：{r.calibration_id}",f"图像几何：{frame.image_geometry}",f"模型：{'已加载' if self.worker.engine.detectors else 'Model Not Loaded / Geometry'}",f"左时间戳：{frame.left_timestamp_ns}",f"右时间戳：{frame.right_timestamp_ns}",f"同步误差：{d.get('sync_error_ms','N/A')} ms",f"检测手数：L {len(r.left_detections)} / R {len(r.right_detections)}",f"有效关节：{sum(h.valid_joint_count for h in r.hands)}",f"平均重投影：{d.get('mean_reprojection_error_px','N/A')} px",f"检测 FPS：L {1000/max(d.get('left_detection_ms',0),.001):.1f} / R {1000/max(d.get('right_detection_ms',0),.001):.1f}",f"匹配 / 三角化：{d.get('matching_and_triangulation_ms',0):.2f} ms",f"处理：{r.processing_time_ms:.2f} ms",f"读取 / 重建 FPS：{stats['read_fps']:.1f} / {stats['processing_fps']:.1f}",f"端到端延迟：{stats['latency_ms']:.1f} ms",f"队列：{stats['processing_queue']} · 丢帧：{stats['dropped_frames']}",f"录制队列：{stats['recording_queue']} · 写入：{stats['recording_MB_s']:.2f} MB/s (raw)",f"进程内存：{psutil.Process().memory_info().rss/1024**2:.1f} MiB"]
        if r.source_type == "vendor_vs030" and self.worker and hasattr(self.worker.source, "get_capture_stats"):
            capture = self.worker.source.get_capture_stats()
            by_sink = capture.get("sdk_packets_by_sink", {})
            lines.extend([
                f"SDK 原始包：左 {by_sink.get('4', 0)} / 右 {by_sink.get('5', 0)} / IMU {by_sink.get('7', 0)}",
                f"预览解码丢包：{capture.get('preview_decode_packets_dropped', 0)} · 输出丢帧：{capture.get('preview_output_frames_dropped', 0)}",
                f"预览解码错误：{capture.get('preview_decode_errors', 0)} · IMU VIO 队列丢样：{capture.get('imu_vio_queue_drops', 0)}",
            ])
        if d.get("synthetic"):
            lines[10]="二维输入：Synthetic Geometry（未运行模型）"
        elif d.get("left_detection_origin")=="precomputed":
            lines[10]="二维输入：保存 / 外部预计算关键点"
        self.metrics.setText("\n".join(lines))
        self.banner.setProperty("tone",None)
        present_result(self,r,stats,frame)
    def closeEvent(self,event):
        if not self.stop_pipeline():event.ignore();return
        if self.replay_thread is not None and self.replay_thread.isRunning():
            # 离线回放是独立子进程/线程；关窗前取消并等待，避免遗留进程与 socket。
            self.replay_thread.cancel()
            if not self.replay_thread.wait(15000):self.replay_thread.terminate();self.replay_thread.wait(3000)
        self.replay_thread=None
        self.mano_controller.close();self.timer.stop();event.accept()
