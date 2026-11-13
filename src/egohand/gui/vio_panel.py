"""三维离线轨迹查看：估计与真值均明确为同一标定的左相机坐标。"""
from pathlib import Path
from dataclasses import replace
from threading import Event
import cv2
import numpy as np
import yaml
import pyqtgraph.opengl as gl
from PySide6.QtCore import Qt,Signal,QEvent,QTimer
from PySide6.QtGui import QColor,QVector3D,QVector4D
from PySide6.QtWidgets import (QCheckBox,QComboBox,QFileDialog,QHBoxLayout,QLabel,QPushButton,
                               QSlider,QVBoxLayout,QWidget,QInputDialog,QMessageBox,QSplitter)
from ..core.pipeline import ROOT
from ..vio.trajectory import load_euroc_trajectory,load_tum_body_ground_truth,align_positions_to_ground_truth,rigid_transform
from ..vio.localization_state import consumer_map_pose
from ..vio.local_segments import consumer_display_pose,display_reference_key

NAVIGATION_ROOT=ROOT/'samples'/'date'/'VIO'
WORLD_AXIS_LENGTH_M=.25
CAMERA_AXIS_LENGTH_M=.16
AXIS_COLORS=((1.,.32,.32,1.),(.30,.90,.48,1.),(.32,.62,1.,1.))

def pose_axes(transform,length=CAMERA_AXIS_LENGTH_M):
    """返回世界坐标中的相机原点与三轴端点，轴长度只用于可视化。"""
    pose=rigid_transform(transform)
    origin=pose[:3,3]
    return np.vstack((origin,origin+length*pose[:3,:3].T))

def live_camera_pose_text(message):
    """显示当前有效的左相机位姿；不将失效帧或历史位姿冒充实时结果。"""
    T=consumer_display_pose(message)
    if T is None:
        return '相机 6D 位姿 · 当前不可用'
    try:
        from scipy.spatial.transform import Rotation
        xyz=T[:3,3]
        # 小写 xyz 表示绕固定坐标轴依次旋转，等价于 R = Rz(yaw) Ry(pitch) Rx(roll)。
        import warnings
        with warnings.catch_warnings():
            warnings.filterwarnings('ignore',message='Gimbal lock detected.*',category=UserWarning)
            roll,pitch,yaw=Rotation.from_matrix(T[:3,:3]).as_euler('xyz',degrees=True)
    except (TypeError,ValueError,IndexError):
        return '相机 6D 位姿 · 数据无效'
    motion=message.get('local_motion') or {}
    frame=('局部段 '+str(motion.get('segment_index')) if motion.get('segment_id') else
           '显示重锚坐标' if message.get('display_map_coordinates_preserved') is False else '世界坐标 W')
    # 双目基线提供度量尺度；惯性优化未完成仍不能保证尺度和位姿准确。
    scale_note='' if message.get('inertial_scale_finalized',True) else ' · 惯性优化未完成，尺度待验证'
    return (f'左相机 6D · {frame} · XYZ {xyz[0]:+.3f}, {xyz[1]:+.3f}, {xyz[2]:+.3f} m{scale_note}\n'
            f'姿态 RPY（固定轴 XYZ） {roll:+.1f}°, {pitch:+.1f}°, {yaw:+.1f}°')

class TrajectoryView(gl.GLViewWidget):
    def __init__(self,parent=None):
        super().__init__(parent)
        self.setMinimumHeight(220);self.setAccessibleName('相机三维轨迹')
        self.trajectory_items=[];self.setBackgroundColor('#0E141D')
        self.axis_items=[];self.hand_items=[]
        self.grid=gl.GLGridItem();self.grid.setSize(2,2);self.grid.setSpacing(.25,.25);self.addItem(self.grid)
        self.reset_camera()
    def reset_camera(self):
        self.setCameraPosition(distance=max(.5,self.opts.get('distance',3.)),elevation=35,azimuth=-55)
    def zoom_by(self,factor):
        self.setCameraPosition(distance=max(.1,min(1000.,self.opts['distance']*factor)))
    def set_theme(self,tokens):
        self.setBackgroundColor(tokens['canvas']);self.grid.setColor(QColor(tokens['border']));self.update()
    def contains_position(self,xyz):
        if not self.deviceWidth() or not self.deviceHeight():
            return True
        point=QVector4D(*map(float,xyz),1.)
        clip=self.projectionMatrix().map(self.viewMatrix().map(point))
        return clip.w()>0 and max(abs(clip.x()),abs(clip.y()),abs(clip.z()))<=clip.w()

    def set_pose_axes(self,pose,*,show_world=True):
        for item in self.axis_items:self.removeItem(item)
        self.axis_items=[]
        for transform,length,prefix,width in ((np.eye(4),WORLD_AXIS_LENGTH_M,'W',2),
                                               (pose,CAMERA_AXIS_LENGTH_M,'C',3)):
            if transform is None or (prefix=='W' and not show_world):continue
            points=pose_axes(transform,length)
            for axis,color in enumerate(AXIS_COLORS):
                item=gl.GLLinePlotItem(pos=points[[0,axis+1]],color=color,width=width,antialias=True,glOptions='translucent')
                self.addItem(item);self.axis_items.append(item)
                label=gl.GLTextItem(pos=points[axis+1],text=prefix+' '+('X','Y','Z')[axis],color=QColor.fromRgbF(*color))
                self.addItem(label);self.axis_items.append(label)
        self.update()

    def draw(self,estimated,truth,estimate_on,truth_on,index,fit=False,*,estimate_breaks=()):
        for item in self.trajectory_items:self.removeItem(item)
        self.trajectory_items=[];all_points=[]
        for points,enabled,color,breaks in (
            (estimated,estimate_on,(.20,.87,.74,1.),estimate_breaks),
            (truth,truth_on,(.85,.42,1.,1.),()),
        ):
            if not enabled or not len(points):continue
            points=np.asarray(points,dtype=float);all_points.append(points)
            line_points=points
            mode='line_strip'
            if breaks:
                connected=np.ones(len(points)-1,dtype=bool)
                connected[np.asarray(breaks)-1]=False
                line_points=np.stack((points[:-1][connected],points[1:][connected]),axis=1).reshape(-1,3)
                mode='lines'
                isolated=~(np.r_[False,connected] | np.r_[connected,False])
                if index is not None and index<len(points):
                    isolated[index]=False
                if isolated.any():
                    item=gl.GLScatterPlotItem(pos=points[isolated],color=color,size=5,pxMode=True,glOptions='translucent')
                    self.addItem(item);self.trajectory_items.append(item)
            if len(line_points)>1:
                item=gl.GLLinePlotItem(pos=line_points,mode=mode,color=color,width=2,antialias=True,glOptions='translucent')
                self.addItem(item);self.trajectory_items.append(item)
            if index is not None and index < len(points) and color != (.20,.87,.74,1.):
                item=gl.GLScatterPlotItem(pos=points[[index]],color=color,size=9,pxMode=True,glOptions='translucent')
                self.addItem(item);self.trajectory_items.append(item)
        if fit and all_points:
            values=np.concatenate(all_points);center=(values.min(0)+values.max(0))/2
            radius=max(float(np.linalg.norm(values-center,axis=1).max()),.2)
            self.opts['center']=QVector3D(*map(float,center));self.setCameraPosition(distance=radius*3.4,elevation=35,azimuth=-55)
        self.update()

    def set_hands(self,hands,transform):
        from ..detection.landmarks import BONES
        for item in self.hand_items:self.removeItem(item)
        self.hand_items=[]
        if transform is None:return
        for hand in hands:
            points=np.asarray([j.xyz_m for j in hand.joints],dtype=float)
            valid=np.asarray([j.valid for j in hand.joints]) & np.isfinite(points).all(axis=1)
            if not valid.any():continue
            points=points@transform[:3,:3].T+transform[:3,3]
            color=(.37,.88,.77,1.) if hand.handedness=='Left' else (.94,.73,.46,1.)
            edges=[points[[a,b]] for a,b in BONES if valid[a] and valid[b]]
            if edges:
                line=gl.GLLinePlotItem(pos=np.concatenate(edges),mode='lines',color=color,width=2,antialias=True)
                self.addItem(line);self.hand_items.append(line)
            dots=gl.GLScatterPlotItem(pos=points[valid],color=color,size=7,pxMode=True)
            self.addItem(dots);self.hand_items.append(dots)

    def fit_geometry(self,extra_points=()):
        """Frame the trajectory, current skeleton and coordinate axes together."""
        points=[]
        if len(extra_points):points.append(np.asarray(extra_points,dtype=float).reshape(-1,3))
        for item in self.trajectory_items+self.hand_items+self.axis_items:
            position=getattr(item,'pos',None)
            if position is None:continue
            values=np.asarray(position,dtype=float).reshape(-1,3)
            values=values[np.isfinite(values).all(axis=1)]
            if len(values):points.append(values)
        if not points:return
        xyz=np.concatenate(points);center=(xyz.min(0)+xyz.max(0))/2
        radius=max(float(np.linalg.norm(xyz-center,axis=1).max()),.2)
        self.opts['center']=QVector3D(*map(float,center))
        aspect=max(1.,self.width()/max(1,self.height()))
        self.setCameraPosition(distance=radius*3.4*aspect,elevation=35,azimuth=-55)

class VioPanel(QWidget):
    calibration_changed=Signal(str)
    live_pose_readout=Signal(str)
    def __init__(self,parent=None):
        super().__init__(parent)
        self.poses=[];self.truth=[];self.calibration=None;self.calibration_path=None;self.alignment=None
        self._live_previous=None
        self.map_directory=None
        self.map_manifest=None
        self.map_cloud=None
        self.map_layers={}
        self.map_updates=Event()
        self.map_comparison=Event();self.map_comparison.set()
        self.map_revision=None
        self._reset_live_path()
        self.last_directory=NAVIGATION_ROOT
        layout=QVBoxLayout(self);layout.setContentsMargins(10,8,10,8);layout.setSpacing(6)
        controls=QHBoxLayout();controls.setSpacing(8)
        load=QPushButton('导入估计');load.setToolTip('导入 ORB-SLAM3 双目惯性体轨迹；需先在标定区加载相机—IMU 标定。');load.clicked.connect(self.import_trajectory)
        gt=QPushButton('导入真值');gt.setToolTip('导入 TUM 格式、秒时间戳的 IMU 体坐标真值轨迹。');gt.clicked.connect(self.import_truth)
        clear=QPushButton('清除');clear.clicked.connect(self.clear)
        self.live_enabled=QCheckBox('实时 VIO（实验）');self.live_enabled.setChecked(True)
        self.live_enabled.setToolTip('采集前设置；实时结果使用设备时钟和待验证的 IMU 数据布局。')
        self.live_profile=QComboBox();self.live_profile.setAccessibleName('实时 VIO Profile')
        self.live_profile.addItem('正式稳定版','stable')
        self.live_profile.addItem('v4 实时恢复实验版','v4')
        self.live_profile.setToolTip('v4 只在显式选择后加载隔离候选 DLL 和恢复配置，不改变正式默认。')
        self.show_estimate=QCheckBox('拟合 / 估计');self.show_estimate.setChecked(True)
        self.show_truth=QCheckBox('真值');self.show_truth.setChecked(True)
        for widget in (load,gt,clear,self.show_estimate,self.show_truth,self.live_enabled,self.live_profile):controls.addWidget(widget)
        controls.addStretch();layout.addLayout(controls)
        comparison_controls=QHBoxLayout()
        self.compare_native=QCheckBox('原生 / 界面对比')
        self.compare_native.setToolTip('左侧显示同一 GUI 左相机坐标下、未经 Python 质量门控的原始估计；右侧保留门控后结果。仅改变显示。')
        self.fit_comparison=QPushButton('显示两侧全部轨迹');self.fit_comparison.hide()
        comparison_controls.addWidget(self.compare_native);comparison_controls.addWidget(self.fit_comparison)
        comparison_controls.addStretch();layout.addLayout(comparison_controls)
        self.preview=TrajectoryView()
        self.raw_preview=TrajectoryView();self.raw_preview.setAccessibleName('未经质量门控的原生相机轨迹')
        self.comparison_split=QSplitter(Qt.Horizontal)
        self.raw_container=QWidget();raw_layout=QVBoxLayout(self.raw_container);raw_layout.setContentsMargins(0,0,0,0)
        self.raw_title=QLabel('原生估计 · 未经质量门控');raw_layout.addWidget(self.raw_title)
        raw_layout.addWidget(self.raw_preview,1)
        self.raw_status=QLabel('等待实时原始位姿');self.raw_status.setWordWrap(True);raw_layout.addWidget(self.raw_status)
        filtered_container=QWidget();filtered_layout=QVBoxLayout(filtered_container);filtered_layout.setContentsMargins(0,0,0,0)
        self.filtered_title=QLabel('界面结果 · 质量门控后');self.filtered_title.hide();filtered_layout.addWidget(self.filtered_title)
        filtered_layout.addWidget(self.preview,1)
        self.filtered_status=QLabel();self.filtered_status.setWordWrap(True);self.filtered_status.hide();filtered_layout.addWidget(self.filtered_status)
        self.comparison_split.addWidget(self.raw_container);self.comparison_split.addWidget(filtered_container)
        self.comparison_split.setChildrenCollapsible(False);self.raw_container.hide()
        layout.addWidget(self.comparison_split,1)
        self._raw_poses=[];self._raw_breaks=[];self._raw_epoch=None;self._raw_gap=False;self._raw_current=None
        self.preview.installEventFilter(self);self.raw_preview.installEventFilter(self)
        self.compare_native.toggled.connect(self._toggle_native_comparison)
        self.fit_comparison.clicked.connect(self._fit_native_comparison)
        legend=QLabel('参考坐标 W（回放时为当前局部段） · 相机坐标 C（左相机光学系） · X 红 / Y 绿 / Z 蓝')
        legend.setWordWrap(True);layout.addWidget(legend)
        map_controls=QHBoxLayout()
        self.load_map=QPushButton('导入预扫地图');self.load_map.clicked.connect(self.import_map)
        self.close_map=QPushButton('关闭预扫地图');self.close_map.clicked.connect(self.unload_map)
        map_controls.addWidget(self.load_map);map_controls.addWidget(self.close_map);map_controls.addStretch()
        layout.insertLayout(1,map_controls)
        revision_controls=QHBoxLayout()
        self.compare_map=QCheckBox('检测地图差异');self.compare_map.setChecked(True)
        self.compare_map.toggled.connect(self._toggle_map_comparison)
        self.update_map=QCheckBox('记录地图更新')
        self.update_map.setEnabled(False)
        self.update_map.toggled.connect(self._toggle_map_updates)
        self.save_map=QPushButton('另存为新版');self.save_map.setEnabled(False)
        self.save_map.clicked.connect(self.save_map_revision)
        self.discard_map=QPushButton('丢弃本次更新');self.discard_map.setEnabled(False)
        self.discard_map.clicked.connect(self.discard_map_revision)
        for control in (self.compare_map,self.update_map,self.save_map,self.discard_map):revision_controls.addWidget(control)
        revision_controls.addStretch();layout.insertLayout(2,revision_controls)
        self.slider=QSlider(Qt.Horizontal);self.slider.setAccessibleName('选择轨迹位姿');self.slider.setEnabled(False)
        self.slider.valueChanged.connect(self.show_pose);layout.addWidget(self.slider)
        self.details=QLabel('未加载轨迹 · 绿色：估计  紫色：真值  ·  手部同步：不可用')
        self.details.setWordWrap(True);self.details.setTextInteractionFlags(Qt.TextSelectableByMouse);layout.addWidget(self.details)
        self.show_estimate.toggled.connect(self._update_view);self.show_truth.toggled.connect(self._update_view)

    def eventFilter(self,watched,event):
        if (hasattr(self,'raw_preview') and watched in (self.preview,self.raw_preview) and self.compare_native.isChecked()
                and event.type() in (QEvent.Type.MouseMove,QEvent.Type.Wheel,QEvent.Type.MouseButtonRelease)):
            QTimer.singleShot(0,lambda source=watched:self._sync_comparison_camera(source))
        return super().eventFilter(watched,event)

    def _sync_comparison_camera(self,source=None):
        if not self.compare_native.isChecked():return
        source=self.preview if source is None else source
        target=self.raw_preview if source is self.preview else self.preview
        for key in ('center','distance','elevation','azimuth','fov','rotation'):
            target.opts[key]=source.opts[key]
        target.update()

    def _toggle_native_comparison(self,enabled):
        self.raw_container.setVisible(enabled);self.filtered_title.setVisible(enabled)
        self.filtered_status.setVisible(enabled);self.fit_comparison.setVisible(enabled)
        if enabled:
            self.comparison_split.setSizes([1,1]);self._draw_raw_comparison();self._sync_comparison_camera()

    def _fit_native_comparison(self):
        points=[pose[:3,3] for pose in self._raw_poses]
        points.extend(p.T_world_from_camera_left[:3,3] for p in self.poses)
        if not points:return
        points=np.asarray(points);center=(points.min(0)+points.max(0))/2
        self.preview.opts['center']=QVector3D(*map(float,center))
        self.preview.setCameraPosition(distance=max(.68,float(np.linalg.norm(points-center,axis=1).max())*3.4))
        self._sync_comparison_camera()

    def _reset_raw_comparison(self):
        self._raw_poses=[];self._raw_breaks=[];self._raw_epoch=None;self._raw_gap=False;self._raw_current=None
        self.raw_preview.draw([],[],False,False,None);self.raw_preview.set_pose_axes(None)
        self.raw_status.setText('等待实时原始位姿');self.filtered_status.setText('等待实时门控结果')

    def _draw_raw_comparison(self):
        points=np.asarray([p[:3,3] for p in self._raw_poses]).reshape(-1,3)
        self.raw_preview.draw(points,[],True,False,None,estimate_breaks=self._raw_breaks)
        self.raw_preview.set_pose_axes(self._raw_current,show_world=bool(self._raw_poses))

    def _update_raw_comparison(self,message):
        from ..vio.map_identity import world_identity
        epoch=world_identity(message)
        if self._raw_epoch is not None and epoch!=self._raw_epoch:self._reset_raw_comparison()
        self._raw_epoch=epoch
        raw=message.get('raw_matrix',message.get('matrix'))
        valid=message.get('raw_pose_valid',message.get('pose_valid',False))
        try:pose=rigid_transform(raw) if valid and raw is not None else None
        except (TypeError,ValueError,IndexError):pose=None
        self._raw_current=pose
        if pose is not None:
            if self._raw_gap and self._raw_poses:self._raw_breaks.append(len(self._raw_poses))
            self._raw_poses.append(pose.copy());self._raw_gap=False
            if len(self._raw_poses)>10000:
                self._raw_poses=self._raw_poses[-10000:];self._raw_breaks=[b-1 for b in self._raw_breaks if b>1]
        else:self._raw_gap=True
        stamp=message.get('timestamp_ns','?')
        self.raw_status.setText(f'帧时间 {stamp} ns · 地图 {epoch}\n'+
            (f'XYZ {pose[0,3]:+.3f}, {pose[1,3]:+.3f}, {pose[2,3]:+.3f} m · 未经质量门控' if pose is not None else '本帧原生位姿不可用 · 保留历史'))
        accepted=consumer_map_pose(message) is not None
        self.filtered_status.setText(f'帧时间 {stamp} ns · '+('已显示' if accepted else '暂停追加')+
            '\n'+str(message.get('quality_status','')))
        if self.compare_native.isChecked():self._draw_raw_comparison()

    def import_map(self):
        from ..vio.map_package import read_package
        directory=QFileDialog.getExistingDirectory(self,'选择包含 map.json 的地图包')
        if not directory:return
        try:
            manifest,points=read_package(directory)
        except (ValueError,OSError,KeyError,TypeError) as exc:
            QMessageBox.warning(self,'无法导入地图',str(exc))
            return
        self.unload_map()
        self.clear()
        self.map_directory=Path(directory)
        self.map_manifest=manifest
        self.map_cloud=gl.GLScatterPlotItem(pos=points,color=(.64,.66,.70,.85),
                                           size=3,pxMode=True,glOptions='translucent')
        self.preview.addItem(self.map_cloud)
        self.preview.set_pose_axes(None)
        if len(points):
            center=(points.min(axis=0)+points.max(axis=0))/2
            radius=np.linalg.norm(points-center,axis=1).max()
            self.preview.opts['center']=QVector3D(*map(float,center))
            self.preview.setCameraPosition(distance=max(1.,float(radius)*3.4))
        self.details.setText(f"预扫地图预览 · {manifest['map_id']} · 版本 {manifest['version']}")

    def unload_map(self):
        self.discard_map_revision()
        self._clear_map_layers()
        if self.map_cloud is not None:
            self.preview.removeItem(self.map_cloud)
        self.map_directory=None
        self.map_manifest=None
        self.map_cloud=None

    def _toggle_map_updates(self,enabled):
        if enabled:self.map_updates.set()
        else:self.map_updates.clear()

    def _toggle_map_comparison(self,enabled):
        if enabled:self.map_comparison.set()
        else:
            self.map_comparison.clear()
            self._clear_map_layers()

    def _clear_map_layers(self):
        for item in self.map_layers.values():self.preview.removeItem(item)
        self.map_layers.clear()

    def discard_map_revision(self):
        self.map_revision=None
        self.save_map.setEnabled(False);self.discard_map.setEnabled(False)

    def save_map_revision(self):
        if self.map_revision is None:return
        directory=QFileDialog.getExistingDirectory(self,'选择新版地图的父目录')
        if not directory:return
        comparison,candidates=self.map_revision
        destination=Path(directory)/(self.map_directory.name+f"-v{self.map_manifest['version']+1}")
        try:comparison.save_revision(destination,candidates)
        except (OSError,ValueError) as exc:QMessageBox.warning(self,'地图另存失败',str(exc))
        else:
            self.discard_map_revision()
            self.details.setText(f'地图新版已另存至 {destination}')

    def update_live(self,message):
        kind=message.get('type')
        if kind in ('map_difference','map_updates_ready'):
            if not self.map_manifest or message['preset_map_id']!=self.map_manifest['map_id']:return
            if kind=='map_updates_ready':
                self.map_revision=(message['comparison'],message['candidates'])
                self.save_map.setEnabled(True);self.discard_map.setEnabled(True)
                return
            if not self.map_comparison.is_set():return
            self._clear_map_layers()
            colors={'dynamic':(1.,.8,.14,.95),'added':(.16,.54,1.,1.),
                    'new_area':(.34,.8,1.,.58),'removed':(1.,.22,.22,.58)}
            for name,points in message['layers'].items():
                if len(points):
                    item=gl.GLScatterPlotItem(pos=points,color=colors[name],size=5,pxMode=True,
                                              glOptions='translucent')
                    self.preview.addItem(item);self.map_layers[name]=item
            return
        if kind=='vio_reset':
            self._reset_raw_comparison()
            # 进程重启后使用新的地图坐标系，不能与旧轨迹直接拼接。
            self.poses=[];self.alignment=None;self._live_map_epoch=None;self._live_display_segment=None
            self._reset_live_path()
            self.slider.setEnabled(False);self.slider.setRange(0,0)
            self.preview.draw([],[],False,False,None)
            self.live_pose_readout.emit('相机 6D 位姿 · 当前不可用')
            self.details.setText('实时 VIO · '+message.get('message',''))
            return
        if kind in ('status','error','session_phase'):
            self._raw_current=None;self._raw_gap=True
            if self.compare_native.isChecked():self._draw_raw_comparison()
            self.raw_status.setText('当前无实时位姿 · '+message.get('message',''))
            self.filtered_status.setText('当前无实时位姿 · '+message.get('message',''))
            self.live_pose_readout.emit('相机 6D 位姿 · 当前不可用')
            identity=message.get('vio_identity') or {}
            identity_text=''
            if identity:
                identity_text=(f" · PID {identity.get('pid','—')} · DLL SHA256 {str(identity.get('library_sha256',''))[:12]}"
                               f" · 恢复配置 SHA256 {str(identity.get('recovery_config_sha256',''))[:12]}")
            self.details.setText(('实时 VIO 已停止 · ' if kind=='error' else '实时 VIO · ')+message.get('message','')+identity_text)
            return
        if kind!='pose':return
        check=message.get('recovery_check')
        if check is not None:
            status=check.get('status','unavailable')
            self._last_recovery_status={'consistent':'未超限','inconsistent':'不一致','unavailable':'无法判断'}.get(status,status)
        pre=message.get('recovery_preaccept') or {}
        attempts=pre.get('candidates') or []
        if attempts:
            status=attempts[-1].get('status','unavailable')
            self._last_recovery_status={'consistent':'未超限','inconsistent':'已拒绝','unavailable':'未验证'}.get(status,status)
        direct=pre.get('direct_recovery')
        if direct is not None:
            status=direct.get('status','unavailable')
            self._last_recovery_status={'consistent':'未超限（事后检查）','inconsistent':'不一致（事后检查）','unavailable':'未验证（事后检查）'}.get(status,status)
        preset=message.get('preset_map_id')
        if preset is not None and (self.map_manifest is None or preset!=self.map_manifest['map_id']):
            return
        self.live_pose_readout.emit(live_camera_pose_text(message))
        admitted=consumer_display_pose(message)
        if preset is not None and admitted is None:
            self._update_raw_comparison(message)
            self._clear_map_layers()
            self.preview.set_pose_axes(None)
            self._live_gap_pending=True
            self.details.setText('正在寻找地图位置 · '+message.get('quality_status','跟踪未稳定'))
            return
        state=message.get('tracking_state','UNKNOWN');initialized=bool(message.get('imu_initialized'))
        from ..vio.map_identity import world_identity
        epoch=display_reference_key(message)
        segment=message.get('display_segment')
        if epoch is not None and self._live_map_epoch is not None and epoch!=self._live_map_epoch:
            self.poses=[];self.alignment=None
            self._reset_live_path()
            self.slider.setEnabled(False);self.slider.setRange(0,0)
            self.preview.draw([],[],False,False,None)
        if epoch is not None:self._live_map_epoch=epoch
        if 'local_motion' in message and admitted is None:
            self.poses=[];self._live_breaks=[]
            self.preview.draw([],[],False,False,None)
            self.preview.set_hands([],None)
        if segment is not None:
            if self._live_display_segment is not None and segment!=self._live_display_segment:
                self._live_gap_pending=True
            self._live_display_segment=segment
        self._update_raw_comparison(message)
        if admitted is not None:
            from ..vio.trajectory import OfflinePose
            pose=OfflinePose(int(message['timestamp_ns']),admitted,self.calibration_path or 'vs030',
                             pose_source='orb_slam3_live_experimental',tracking_status=state)
            fit=((not self.poses or self._live_gap_pending) and self.show_estimate.isChecked()
                 and not self.preview.contains_position(pose.T_world_from_camera_left[:3,3]))
            if self._live_gap_pending and self.poses:
                self._live_breaks.append(len(self.poses))
            self._live_gap_pending=False
            self.poses.append(pose)
            overflow=len(self.poses)-10000
            if overflow>0:
                self.poses=self.poses[overflow:]
                self._live_breaks=[i-overflow for i in self._live_breaks if i>overflow]
            self.slider.blockSignals(True);self.slider.setRange(0,len(self.poses)-1);self.slider.setValue(len(self.poses)-1);self.slider.setEnabled(True);self.slider.blockSignals(False)
            self._update_view(fit=fit)
        else:
            self._live_gap_pending=True
            self.preview.set_pose_axes(None)
        lag=message.get('camera_to_imu_lag_ms')
        timing=f' · 时差 {lag:+.0f} ms（实验）' if isinstance(lag,(int,float)) else ''
        retained=' · 保留历史轨迹' if self._live_gap_pending and self.poses else ''
        if message.get('automatic_new_map'):
            retained+=' · 恢复超时，正在建立新地图'
        elif pre.get('await_visual_requalification'):
            elapsed=pre.get('quarantine_elapsed_s',0)
            if pre.get('timeout_action') == 'report_only':
                retained+=(f' · 验证中 {elapsed:.1f} 秒，'
                           + ('超时后继续隔离' if pre.get('quarantine_timed_out') else '超时后等待复核'))
            else:
                # Older workers did not publish timeout_action and requested a new map.
                retained+=f' · 验证中 {elapsed:.1f} 秒，超时自动新建地图'
        if message.get('display_map_coordinates_preserved') is False:
            retained += ' · 显示已重锚（非原地图坐标）'
        if preset is not None:
            self.details.setText(f'已在预扫地图中定位 · {len(self.poses)} 帧\n'
                                 '黄：临时前景 · 蓝：已知空处新增 · 浅蓝：未扫描区域 · 红：已移除')
            return
        recovery=getattr(self,'_last_recovery_status',None)
        recovery_text=f' · 恢复预测{recovery}（实验门限）' if recovery else ''
        extrapolation=pre.get('loss_prediction') or {}
        prediction_text=''
        if extrapolation.get('status')=='predicted':
            T=extrapolation.get('T_world_from_body')
            try:
                xyz=np.asarray(T,dtype=float)[:3,3]
                if xyz.shape==(3,) and np.isfinite(xyz).all():
                    prediction_text=(f'\n纯 IMU 外推（机体原点，非视觉位姿）：'
                                     f'{xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f} m · '
                                     f'{extrapolation.get("duration_s",0):.2f} s')
            except (TypeError,ValueError,IndexError):
                pass
        elif extrapolation.get('status')=='unavailable':
            prediction_text='\n纯 IMU 外推不可用：'+str(extrapolation.get('reason','缺少有效数据'))
        self.details.setText(f'实时 VIO（实验） · {state} · IMU'+('已初始化' if initialized else '初始化中')+
            f' · 有效位姿 {len(self.poses)} 帧{timing}{retained}{recovery_text}\n{message.get("quality_status", "时间戳、IMU 字段及标定精度仍需实测验收")}{prediction_text}')
        motion=message.get('local_motion') or {}
        if motion.get('segment_id'):
            self.details.setText(f'局部轨迹段 {motion.get("segment_index")} · {motion.get("pose_source")} · {state}\n'
                f'当前段 {len(self.poses)} 帧 · 新旧段独立 · 位姿精度待验证')

    def set_hand_result(self,result):
        pose=result.diagnostics.get('vio_pose')
        transform=consumer_display_pose(pose,world_hands=True,expected_timestamp_ns=result.timestamp_ns)
        self.preview.set_hands(result.hands,transform)

    def show_recorded(self,results,index,*,show_hands=True):
        """Build one segment once per seek, without inference or per-frame redraw."""
        from ..vio.trajectory import OfflinePose
        self.clear();self.begin_live()
        current=results[index].diagnostics.get('vio_pose')
        if not isinstance(current,dict):
            self.preview.set_hands([],None);return
        key=display_reference_key(current)
        selected=[];breaks=[];gap=False
        for result in results[:index+1]:
            pose=result.diagnostics.get('vio_pose')
            if not isinstance(pose,dict) or display_reference_key(pose)!=key:continue
            transform=consumer_display_pose(pose,expected_timestamp_ns=result.timestamp_ns if 'local_motion' in pose else None)
            if transform is None:gap=True;continue
            if gap and selected:breaks.append(len(selected))
            gap=False
            selected.append(OfflinePose(pose['timestamp_ns'],transform,getattr(result,'calibration_id',None) or 'vs030',
                pose_source='recorded_admitted_local',tracking_status=pose.get('tracking_state')))
        self.poses=selected;self._live_breaks=breaks
        self.slider.blockSignals(True);self.slider.setRange(0,max(0,len(selected)-1))
        self.slider.setValue(max(0,len(selected)-1));self.slider.blockSignals(False)
        self._update_view(fit=True)
        if consumer_display_pose(current,expected_timestamp_ns=results[index].timestamp_ns) is None:
            self.preview.set_pose_axes(None)
        self.live_pose_readout.emit(live_camera_pose_text(current))
        motion=current.get('local_motion') or {}
        self._recorded_details=(f'已有结果 · {motion.get("pose_source") or current.get("mode") or "来源未知"} · 局部段 {motion.get("segment_index","未知")} · {len(selected)} 位姿\n'
            '按已保存的分段身份显示；不连接不兼容坐标')
        self.details.setText(self._recorded_details)
        if show_hands:self.set_hand_result(results[index])
        self.preview.fit_geometry()
        if show_hands:QTimer.singleShot(0,self.preview.fit_geometry)

    def _reset_live_path(self):
        self._recorded_details=None
        if hasattr(self,'raw_preview'):self._reset_raw_comparison()
        self._last_recovery_status=None
        self._live_map_epoch=None
        self._live_display_segment=None
        self._live_gap_pending=False
        self._live_breaks=[]

    def set_map_controls_enabled(self,enabled):
        self.load_map.setEnabled(enabled);self.close_map.setEnabled(enabled)
        if enabled:
            self.update_map.setChecked(False);self.update_map.setEnabled(False)

    def begin_live(self,*,preset=False):
        self.live_pose_readout.emit('相机 6D 位姿 · 等待实时 VIO')
        # 新会话不和先前导入/采集的位姿或真值拼成一条轨迹。
        self._live_previous=(self.poses,self.truth,self.alignment)
        if not preset:self.unload_map()
        self.set_map_controls_enabled(False)
        self.discard_map_revision()
        self._clear_map_layers()
        self.update_map.setChecked(False);self.update_map.setEnabled(preset)
        self.poses=[];self.truth=[];self.alignment=None
        self._reset_live_path()
        self.slider.setEnabled(False);self.slider.setRange(0,0)
        self.preview.draw([],[],False,False,None)
        self.preview.set_pose_axes(None,show_world=False)
        self.details.setText('实时 VIO（实验） · 等待启动')

    def set_theme(self,tokens):
        self.preview.set_theme(tokens);self.raw_preview.set_theme(tokens)
    def zoom_by(self,factor):
        self.preview.zoom_by(factor);self._sync_comparison_camera()
    def reset_camera(self):self._update_view(fit=True)
    def clear(self):
        self.live_pose_readout.emit('相机 6D 位姿 · 当前不可用')
        self.poses=[];self.truth=[];self.alignment=None
        self._reset_live_path()
        self.slider.setEnabled(False);self.slider.setRange(0,0)
        self.details.setText('未加载轨迹 · 绿色：估计  紫色：真值  ·  手部同步：不可用')
        self.preview.draw([],[],False,False,None)
        self.preview.set_pose_axes(None,show_world=self.map_manifest is not None)

    def select_calibration(self):
        path,_=QFileDialog.getOpenFileName(self,'选择轨迹对应的相机 IMU 标定 YAML',str(NAVIGATION_ROOT),'标定 (*.yaml *.yml)')
        if not path:return
        try:
            data=yaml.safe_load(Path(path).read_text(encoding='utf-8-sig')) if not Path(path).read_text(encoding='utf-8-sig').lstrip().startswith('%YAML') else None
            left=None
            if isinstance(data,dict) and 'cam0' in data and 'cam1' in data:
                left,ok=QInputDialog.getItem(self,'选择 Kalibr 相机','轨迹使用的相机对应哪个标定相机？',['cam0','cam1'],0,False)
                if not ok:return
            self.set_calibration(path,kalibr_camera=left)
        except Exception as exc:
            self.calibration_changed.emit('加载失败：'+str(exc)+('\n保留当前标定：'+Path(self.calibration_path).name if self.calibration_path else ''))

    def set_calibration(self,path,*,kalibr_camera=None):
        path=str(Path(path).resolve())
        if (self.poses or self.truth) and self.calibration_path and path!=self.calibration_path:
            raise ValueError('当前已加载轨迹，请先在位姿视图清除轨迹后更换标定')
        if kalibr_camera is not None:
            if kalibr_camera not in ('cam0','cam1'):raise ValueError('请选择 cam0 或 cam1')
            data=yaml.safe_load(Path(path).read_text(encoding='utf-8-sig'))
            # Kalibr T_cam_imu: IMU -> 相机；轨迹模块需要相机 -> IMU。
            extrinsic=np.linalg.inv(rigid_transform(data[kalibr_camera]['T_cam_imu']))
            name=f'Kalibr {kalibr_camera} · T_imu_cam'
        else:
            storage=cv2.FileStorage(path,cv2.FILE_STORAGE_READ)
            if not storage.isOpened():raise ValueError('无法读取标定文件')
            try:extrinsic=storage.getNode('IMU.T_b_c1').mat()
            finally:storage.release()
            name='IMU.T_b_c1'
        if extrinsic is None:raise ValueError('标定缺少 IMU.T_b_c1')
        extrinsic=rigid_transform(extrinsic)
        if (self.poses or self.truth) and self.calibration is not None and not np.allclose(extrinsic,self.calibration,atol=1e-8):
            raise ValueError('标定内容与已加载轨迹不同，请先清除轨迹')
        self.calibration=extrinsic;self.calibration_path=path
        self.calibration_changed.emit('已加载 · '+name+'\n'+Path(path).name)

    def _calibration_ready(self):
        if self.calibration is not None and self.calibration_path:return True
        self.details.setText('请先在“标定 → 相机—IMU 标定”中加载轨迹对应的 YAML。')
        return False

    def import_trajectory(self):
        if not self._calibration_ready():return
        path,_=QFileDialog.getOpenFileName(self,'选择 ORB-SLAM3 双目惯性体轨迹',str(self.last_directory),'轨迹 (*.txt *.tum);;所有文件 (*)')
        if not path:return
        try:
            cal,ext=self.calibration_path,self.calibration
            if Path(path).suffix.lower()=='.tum':
                poses=load_tum_body_ground_truth(path,calibration_id=str(Path(cal).resolve()),T_body_from_camera=ext)
                poses=[replace(p,pose_source='orb_slam3_offline_optimized',tracking_status='trajectory_available_tracking_state_unknown') for p in poses]
            else:
                poses=load_euroc_trajectory(path,output_frame='body',calibration_id=str(Path(cal).resolve()),T_body_from_camera=ext)
            self.poses=poses
            self._reset_live_path()
            self._loaded()
        except Exception as exc:self.details.setText('估计轨迹导入失败：'+str(exc))

    def import_truth(self):
        if not self._calibration_ready():return
        path,_=QFileDialog.getOpenFileName(self,'选择真值体轨迹（TUM 格式、时间单位秒）',str(self.last_directory),'轨迹 (*.tum *.txt);;所有文件 (*)')
        if not path:return
        try:
            cal,ext=self.calibration_path,self.calibration
            truth=load_tum_body_ground_truth(path,calibration_id=str(Path(cal).resolve()),T_body_from_camera=ext)
            self.truth=truth
            self._loaded()
        except Exception as exc:self.details.setText('真值轨迹导入失败：'+str(exc))

    def _loaded(self):
        self.alignment=align_positions_to_ground_truth(self.poses,self.truth)
        timeline=self.poses or self.truth
        self.slider.blockSignals(True);self.slider.setRange(0,len(timeline)-1);self.slider.setValue(0);self.slider.setEnabled(True);self.slider.blockSignals(False)
        self._update_view(fit=True)

    def show_pose(self,index):self._update_view()
    def _update_view(self,*_,fit=False):
        estimated=np.asarray([p.T_world_from_camera_left[:3,3] for p in self.poses],dtype=float).reshape(-1,3)
        truth=np.asarray([p.T_world_from_camera_left[:3,3] for p in self.truth],dtype=float).reshape(-1,3)
        if self.alignment and len(estimated):
            rotation,translation,_=self.alignment;estimated=estimated@rotation.T+translation
        # 滑块以估计轨迹为准；真值标记只在 5 ms 内找到同一时间时显示。
        index=self.slider.value();truth_index=index if not self.poses else None
        if self.poses and self.truth:
            from bisect import bisect_left
            stamps=[p.timestamp_ns for p in self.truth];target=self.poses[index].timestamp_ns;i=bisect_left(stamps,target)
            nearby=[j for j in (i-1,i) if 0<=j<len(stamps)]
            if nearby:
                nearest=min(nearby,key=lambda j:abs(stamps[j]-target))
                if abs(stamps[nearest]-target)<=5_000_000:truth_index=nearest
        self.preview.draw(estimated,[],self.show_estimate.isChecked(),False,index if self.poses else None,
                          estimate_breaks=self._live_breaks)
        camera_pose=None
        if self.poses and self.show_estimate.isChecked() and index<len(self.poses):
            camera_pose=self.poses[index].T_world_from_camera_left.copy()
            if self.alignment:
                camera_pose[:3,:3]=self.alignment[0]@camera_pose[:3,:3]
                camera_pose[:3,3]=estimated[index]
        elif self.truth and self.show_truth.isChecked() and truth_index is not None:
            camera_pose=self.truth[truth_index].T_world_from_camera_left
        self.preview.set_pose_axes(camera_pose,show_world=bool(self.poses or self.truth or self.map_manifest))
        self._sync_comparison_camera()
        # 单独添加真值，保留两条轨迹独立的当前点索引。
        if self.show_truth.isChecked() and len(truth):
            color=(.85,.42,1.,1.)
            if len(truth)>1:
                item=gl.GLLinePlotItem(pos=truth,color=color,width=2,antialias=True,glOptions='translucent');self.preview.addItem(item);self.preview.trajectory_items.append(item)
            if truth_index is not None:
                item=gl.GLScatterPlotItem(pos=truth[[truth_index]],color=color,size=9,pxMode=True,glOptions='translucent');self.preview.addItem(item);self.preview.trajectory_items.append(item)
        if fit:
            points=[]
            if self.show_estimate.isChecked() and len(estimated):points.append(estimated)
            if self.show_truth.isChecked() and len(truth):points.append(truth)
            if points:
                xyz=np.concatenate(points);center=(xyz.min(0)+xyz.max(0))/2;radius=max(float(np.linalg.norm(xyz-center,axis=1).max()),.2)
                self.preview.opts['center']=QVector3D(*map(float,center));self.preview.setCameraPosition(distance=radius*3.4,elevation=35,azimuth=-55)
            self.preview.fit_geometry()
        self.preview.update()
        self._sync_comparison_camera()
        timeline=self.poses or self.truth
        if timeline:
            pos=estimated[index] if self.poses else truth[index]
            rotation=timeline[index].T_world_from_camera_left[:3,:3]
            if self.poses and self.alignment:rotation=self.alignment[0]@rotation
            from scipy.spatial.transform import Rotation
            q=Rotation.from_matrix(rotation).as_quat()
            alignment=f'SE(3) 对齐 {self.alignment[2]} 个时间匹配点，无尺度调整' if self.alignment else '两轨迹未对齐（时间匹配不足或运动退化）' if self.poses and self.truth else '等待另一条轨迹'
            self.details.setText(f'当前 {index+1}/{len(timeline)} · XYZ {pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f} m · {alignment}\n'
                f'时间 {timeline[index].timestamp_ns} ns · 姿态 xyzw {q[0]:+.3f}, {q[1]:+.3f}, {q[2]:+.3f}, {q[3]:+.3f} · '
                f'绿色：估计 {len(self.poses)} 点  紫色：真值 {len(self.truth)} 点 · 手部同步：不可用')
        if self._recorded_details:self.details.setText(self._recorded_details)
