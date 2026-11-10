"""三维离线轨迹查看：估计与真值均明确为同一标定的左相机坐标。"""
from pathlib import Path
from dataclasses import replace
import cv2
import numpy as np
import yaml
import pyqtgraph.opengl as gl
from PySide6.QtCore import Qt,Signal
from PySide6.QtGui import QColor,QVector3D
from PySide6.QtWidgets import (QCheckBox,QFileDialog,QHBoxLayout,QLabel,QPushButton,
                               QSlider,QVBoxLayout,QWidget,QInputDialog)
from ..core.pipeline import ROOT
from ..vio.trajectory import load_euroc_trajectory,load_tum_body_ground_truth,align_positions_to_ground_truth,rigid_transform

NAVIGATION_ROOT=ROOT/'samples'/'date'/'VIO'

class TrajectoryView(gl.GLViewWidget):
    def __init__(self,parent=None):
        super().__init__(parent)
        self.setMinimumHeight(220);self.setAccessibleName('相机三维轨迹')
        self.trajectory_items=[];self.setBackgroundColor('#0E141D')
        self.grid=gl.GLGridItem();self.grid.setSize(2,2);self.grid.setSpacing(.25,.25);self.addItem(self.grid)
        self.reset_camera()
    def reset_camera(self):
        self.setCameraPosition(distance=max(.5,self.opts.get('distance',3.)),elevation=35,azimuth=-55)
    def zoom_by(self,factor):
        self.setCameraPosition(distance=max(.1,min(1000.,self.opts['distance']*factor)))
    def set_theme(self,tokens):
        self.setBackgroundColor(tokens['canvas']);self.grid.setColor(QColor(tokens['border']));self.update()
    def draw(self,estimated,truth,estimate_on,truth_on,index,fit=False):
        for item in self.trajectory_items:self.removeItem(item)
        self.trajectory_items=[];all_points=[]
        for points,enabled,color in ((estimated,estimate_on,(.20,.87,.74,1.)),(truth,truth_on,(.85,.42,1.,1.))):
            if not enabled or not len(points):continue
            points=np.asarray(points,dtype=float);all_points.append(points)
            if len(points)>1:
                item=gl.GLLinePlotItem(pos=points,color=color,width=2,antialias=True,glOptions='translucent')
                self.addItem(item);self.trajectory_items.append(item)
            if index is not None and index < len(points):
                item=gl.GLScatterPlotItem(pos=points[[index]],color=color,size=9,pxMode=True,glOptions='translucent')
                self.addItem(item);self.trajectory_items.append(item)
        if fit and all_points:
            values=np.concatenate(all_points);center=(values.min(0)+values.max(0))/2
            radius=max(float(np.linalg.norm(values-center,axis=1).max()),.2)
            self.opts['center']=QVector3D(*map(float,center));self.setCameraPosition(distance=radius*3.4,elevation=35,azimuth=-55)
        self.update()

class VioPanel(QWidget):
    calibration_changed=Signal(str)
    def __init__(self,parent=None):
        super().__init__(parent)
        self.poses=[];self.truth=[];self.calibration=None;self.calibration_path=None;self.alignment=None
        self._live_previous=None
        self._live_map_epoch=None
        self._live_display_segment=None
        self.last_directory=NAVIGATION_ROOT
        layout=QVBoxLayout(self);layout.setContentsMargins(10,8,10,8);layout.setSpacing(6)
        controls=QHBoxLayout();controls.setSpacing(8)
        load=QPushButton('导入估计');load.setToolTip('导入 ORB-SLAM3 双目惯性体轨迹；需先在标定区加载相机—IMU 标定。');load.clicked.connect(self.import_trajectory)
        gt=QPushButton('导入真值');gt.setToolTip('导入 TUM 格式、秒时间戳的 IMU 体坐标真值轨迹。');gt.clicked.connect(self.import_truth)
        clear=QPushButton('清除');clear.clicked.connect(self.clear)
        self.live_enabled=QCheckBox('实时 VIO（实验）');self.live_enabled.setChecked(True)
        self.live_enabled.setToolTip('采集前设置；实时结果使用设备时钟和待验证的 IMU 数据布局。')
        self.show_estimate=QCheckBox('拟合 / 估计');self.show_estimate.setChecked(True)
        self.show_truth=QCheckBox('真值');self.show_truth.setChecked(True)
        for widget in (load,gt,clear,self.show_estimate,self.show_truth,self.live_enabled):controls.addWidget(widget)
        controls.addStretch();layout.addLayout(controls)
        self.preview=TrajectoryView();layout.addWidget(self.preview,1)
        self.slider=QSlider(Qt.Horizontal);self.slider.setAccessibleName('选择轨迹位姿');self.slider.setEnabled(False)
        self.slider.valueChanged.connect(self.show_pose);layout.addWidget(self.slider)
        self.details=QLabel('未加载轨迹 · 绿色：估计  紫色：真值  ·  手部同步：不可用')
        self.details.setWordWrap(True);self.details.setTextInteractionFlags(Qt.TextSelectableByMouse);layout.addWidget(self.details)
        self.show_estimate.toggled.connect(self._update_view);self.show_truth.toggled.connect(self._update_view)

    def update_live(self,message):
        kind=message.get('type')
        if kind in ('status','error'):
            self.details.setText(('实时 VIO · ' if kind=='status' else '实时 VIO 已停止 · ')+message.get('message',''))
            return
        if kind!='pose':return
        state=message.get('tracking_state','UNKNOWN');initialized=bool(message.get('imu_initialized'))
        epoch=message.get('map_epoch')
        segment=message.get('display_segment')
        if ((epoch is not None and self._live_map_epoch is not None and epoch!=self._live_map_epoch) or
            (segment is not None and self._live_display_segment is not None and segment!=self._live_display_segment)):
            self.poses=[];self.alignment=None
            self.slider.setEnabled(False);self.slider.setRange(0,0)
            self.preview.draw([],[],False,False,None)
        if epoch is not None:self._live_map_epoch=epoch
        if segment is not None:self._live_display_segment=segment
        matrix=message.get('matrix')
        if message.get('pose_valid') and matrix is not None:
            from ..vio.trajectory import OfflinePose
            pose=OfflinePose(int(message['timestamp_ns']),rigid_transform(matrix),self.calibration_path or 'vs030',
                             pose_source='orb_slam3_live_experimental',tracking_status=state)
            self.poses.append(pose)
            if len(self.poses)>10000:self.poses=self.poses[-10000:]
            self.slider.blockSignals(True);self.slider.setRange(0,len(self.poses)-1);self.slider.setValue(len(self.poses)-1);self.slider.setEnabled(True);self.slider.blockSignals(False)
            self._update_view()
        lag=message.get('camera_to_imu_lag_ms')
        timing=f' · 时差 {lag:+.0f} ms（实验）' if isinstance(lag,(int,float)) else ''
        self.details.setText(f'实时 VIO（实验） · {state} · IMU'+('已初始化' if initialized else '初始化中')+
            f' · 有效位姿 {len(self.poses)} 帧{timing}\n{message.get("quality_status", "时间戳、IMU 字段及标定精度仍需实测验收")}')

    def begin_live(self):
        # 新会话不和先前导入/采集的位姿或真值拼成一条轨迹。
        self._live_previous=(self.poses,self.truth,self.alignment)
        self.poses=[];self.truth=[];self.alignment=None
        self._live_map_epoch=None
        self._live_display_segment=None
        self.slider.setEnabled(False);self.slider.setRange(0,0)
        self.preview.draw([],[],False,False,None)
        self.details.setText('实时 VIO（实验） · 等待启动')

    def set_theme(self,tokens):self.preview.set_theme(tokens)
    def zoom_by(self,factor):self.preview.zoom_by(factor)
    def reset_camera(self):self._update_view(fit=True)
    def clear(self):
        self.poses=[];self.truth=[];self.alignment=None
        self.slider.setEnabled(False);self.slider.setRange(0,0)
        self.details.setText('未加载轨迹 · 绿色：估计  紫色：真值  ·  手部同步：不可用')
        self.preview.draw([],[],False,False,None)

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
        self.preview.draw(estimated,[],self.show_estimate.isChecked(),False,index if self.poses else None,fit=False)
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
        self.preview.update()
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
