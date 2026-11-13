"""客户交付界面的显示组件：无边框图像块、世界系手部叠加、深度后台任务。

本模块只做显示：不改变引擎、检测、三角化或 VIO 的任何行为。
"""
import time

import cv2
import numpy as np
import pyqtgraph.opengl as gl
from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QVector3D
from PySide6.QtWidgets import QLabel, QWidget

from egohand.detection.landmarks import BONES
from egohand.gui.theme import THEMES
from egohand.vio.trajectory import rigid_transform
from egohand.visualization.image_view import ImageView

HAND_COLORS={'Left':(0.37,0.88,0.77,1.0),'Right':(0.94,0.73,0.46,1.0),'Unknown':(0.83,0.86,0.90,1.0)}


class ImageTile(QWidget):
    """图像块：内部视图按图像长宽比铺满格子，标题以角标浮在图像上，不占用图像面积。"""

    def __init__(self,placeholder,parent=None,aspect=4/3):
        super().__init__(parent)
        self.aspect=float(aspect)
        self.view=ImageView(placeholder)
        self.view.set_theme(THEMES['dark'])
        self.view.setMinimumSize(1,1)
        self.view.setParent(self)
        self.caption=QLabel('',self.view)
        self.caption.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.caption.hide()

    def set_aspect(self,aspect):
        """用实际图像宽高比排版，避免出现大面积黑边。"""
        try:aspect=float(aspect)
        except (TypeError,ValueError):return
        if aspect<=0 or not np.isfinite(aspect):return
        if abs(aspect-self.aspect)<1e-4:return
        self.aspect=aspect
        self._place()

    def _place(self):
        """图像按原生长宽比铺满格子；比例几乎一致时直接铺满，避免出现细黑边。"""
        width,height=self.width(),self.height()
        if width<=0 or height<=0:return
        cell_aspect=width/height
        if abs(cell_aspect-self.aspect)<=0.02:
            self.view.setGeometry(0,0,width,height)
            return
        if cell_aspect>self.aspect:
            target_h=height;target_w=max(1,int(round(height*self.aspect)))
        else:
            target_w=width;target_h=max(1,int(round(width/self.aspect)))
        self.view.setGeometry((width-target_w)//2,(height-target_h)//2,target_w,target_h)

    def resizeEvent(self,event):
        super().resizeEvent(event)
        self._place()

    def showEvent(self,event):
        super().showEvent(event)
        self._place()

    def set_caption(self,text):
        if not text:
            self.caption.hide();return
        self.caption.setText(text)
        self.caption.adjustSize()
        self.caption.move(8,6)
        self.caption.show()
        self.caption.raise_()

    def clear(self):
        self.view.set_roi_selection(False)
        self.view.pix=None;self.view.last_bgr=None;self.view._image_size=None
        self.view.clear();self.view.update()


def transform_points(points,transform=None):
    """左相机光学系关节 -> VIO 世界系；没有有效位姿时按相机原点（单位变换）显示。"""
    values=np.asarray(points,dtype=float).reshape(-1,3)
    matrix=np.eye(4) if transform is None else rigid_transform(transform)
    return values@matrix[:3,:3].T+matrix[:3,3]


class WorldHandOverlay:
    """把左相机光学系关节变换到 VIO 世界系，画在与相机轨迹同一个三维视图里。"""

    def __init__(self,view):
        self.view=view
        self.items=[]
        self.visible=True

    def set_visible(self,visible):
        self.visible=bool(visible)
        for item in self.items:item.setVisible(self.visible)

    def clear(self):
        for item in self.items:self.view.removeItem(item)
        self.items=[]

    def has_geometry(self):
        return bool(self.items)

    def update(self,hands,transform=None):
        """hands 的关节在左相机光学系；transform 为 T_world_camera（缺失时按相机原点显示）。"""
        self.clear()
        if not hands:return
        pending=[]
        for index,hand in enumerate(hands):
            joints=np.asarray([j.xyz_m for j in hand.joints],dtype=float)
            valid=np.array([j.valid for j in hand.joints])&np.isfinite(joints).all(axis=1)
            if not valid.any():continue
            world=transform_points(joints,transform)
            color=HAND_COLORS.get(hand.handedness,HAND_COLORS['Unknown'])
            if hand.track_id>0:
                color=(color[0],color[1],color[2],1.0)
            segments=[world[[a,b]] for a,b in BONES if valid[a] and valid[b]]
            if segments:
                pending.append(('lines',np.concatenate(segments),color))
            pending.append(('points',world[valid],color))
        for kind,pos,color in pending:
            if kind=='lines':
                item=gl.GLLinePlotItem(pos=pos,mode='lines',color=color,width=2,antialias=True,
                                       glOptions='translucent')
            else:
                item=gl.GLScatterPlotItem(pos=pos,color=color,size=7,pxMode=True,
                                          glOptions='translucent')
            item.setVisible(self.visible)
            self.view.addItem(item)
            self.items.append(item)


def fit_view(view,points,*,elevation=32,azimuth=-52):
    """把相机拉到能同时看见轨迹与手部的位置。"""
    if view is None or not len(points):return
    values=np.asarray(points,dtype=float).reshape(-1,3)
    values=values[np.isfinite(values).all(axis=1)]
    if not len(values):return
    center=(values.min(axis=0)+values.max(axis=0))/2
    radius=max(float(np.linalg.norm(values-center,axis=1).max()),0.25)
    view.opts['center']=QVector3D(*map(float,center))
    view.setCameraPosition(distance=radius*3.2,elevation=elevation,azimuth=azimuth)
    view.update()


def set_items_visible(items,visible):
    for item in items:
        try:item.setVisible(bool(visible))
        except (AttributeError,RuntimeError):pass


class WorldPanel(QWidget):
    """三维视图容器：开关条浮在视图左上角，不占用图像高度，三列顶部因此齐平。"""

    def __init__(self,parent=None):
        super().__init__(parent)
        from PySide6.QtWidgets import QHBoxLayout
        self.overlay=QWidget(self)
        self.overlay.setObjectName('worldOverlay')
        self.overlay.setAttribute(Qt.WA_StyledBackground,True)
        self.bar=QHBoxLayout(self.overlay)
        self.bar.setContentsMargins(4,2,6,2);self.bar.setSpacing(2)
        self.overlay.hide()

    def place_overlay(self):
        self.overlay.adjustSize()
        self.overlay.move(6,6)
        self.overlay.show()
        self.overlay.raise_()

    def resizeEvent(self,event):
        super().resizeEvent(event)
        self.place_overlay()


class DepthTask(QThread):
    """与原有深度显示一致的计算：同一 StereoRectifier 与 StereoDepth。

    校正器与匹配器跨帧复用（构造一次要做两张全分辨率映射表），否则显示节奏会被
    重复初始化吃掉。只影响显示刷新率，不改变深度算法本身。
    """

    completed=Signal(int,object,str)

    def __init__(self,generation,frame,calibration,cache,parent=None,*,near_m=.15,far_m=3.,filter_range=True):
        super().__init__(parent)
        self.generation=generation;self.frame=frame;self.calibration=calibration;self.cache=cache
        self.near_m=float(near_m);self.far_m=float(far_m);self.filter_range=bool(filter_range)

    def run(self):
        try:
            import cv2
            started=time.perf_counter()
            rectifier,depth_engine,scale,swapped,vertical=self.cache.acquire(self.calibration,self.frame)
            f=rectifier.rectify(self.frame)
            images=[cv2.resize(cv2.cvtColor(i,cv2.COLOR_BGR2GRAY),None,fx=scale,fy=scale)
                    for i in (f.left_image,f.right_image)]
            if vertical:images=[np.ascontiguousarray(i.T) for i in images]
            if swapped:images=images[::-1]
            depth=depth_engine.estimate(*images,min_depth_m=0.01,max_depth_m=float('inf'))
            if vertical:depth=depth.T
            color,valid=colorize_depth(depth,self.near_m,self.far_m,self.filter_range)
            color[~valid]=0
            ms=(time.perf_counter()-started)*1000
            text=(f'深度 · '+('右目参考' if swapped else '左目参考')+
                  f' · {self.near_m:g}–{self.far_m:g} m'+(' · 范围内' if self.filter_range else ' · 全部有效值')+
                  f' · 有效 {valid.mean():.0%} · {ms:.0f} ms')
            self.completed.emit(self.generation,color,text)
        except Exception as exc:
            self.completed.emit(self.generation,None,'深度暂不可用：'+str(exc))


def colorize_depth(depth,near_m,far_m,filter_range=True):
    """Colorize display data; all-values mode saturates outside the color scale."""
    if not 0 < near_m < far_m:raise ValueError('深度范围的近端必须小于远端')
    valid=np.isfinite(depth)&(depth>0)
    if filter_range:valid &= (depth>=near_m)&(depth<=far_m)
    scaled=np.nan_to_num((far_m-depth)/(far_m-near_m),nan=0.,posinf=0.,neginf=0.)
    color=cv2.applyColorMap(np.uint8(np.clip(scaled,0,1)*255),cv2.COLORMAP_TURBO)
    color[~valid]=0
    return color,valid


class DepthCache:
    """按标定复用校正器与 SGBM 匹配器，避免每次刷新重建映射表。"""

    # 与 StereoDepth.estimate 的有效门限一致的近端距离，用来反推需要的视差搜索宽度。
    NEAR_M=.10

    def __init__(self):
        self._key=None;self._rectifier=None;self._depth=None
        self._scale=.25;self._swapped=False;self._vertical=False
        self._disparities=96

    def acquire(self,calibration,frame):
        key=(getattr(calibration,'calibration_id',None),getattr(frame,'calibration_id',None),
             int(getattr(frame.left_image,'shape',(0,0))[1] or 0))
        if key!=self._key:self._build(calibration,frame)
        return self._rectifier,self._depth,self._scale,self._swapped,self._vertical

    @staticmethod
    def required_disparities(fx, baseline, near_m=NEAR_M):
        """近端处需要的视差宽度；SGBM 要求是 16 的倍数。"""
        import math
        needed=fx*baseline/near_m
        return int(min(96,max(16,16*math.ceil(needed/16))))

    def _build(self,calibration,frame):
        import cv2
        from egohand.calibration.rectifier import StereoRectifier
        from egohand.vio.map_depth import StereoDepth
        rectifier=StereoRectifier(calibration)
        width=int(getattr(frame.left_image,'shape',(0,0))[1] or 0) or 1600
        scale=min(1.,360/width)
        projection=rectifier.P1.copy()
        vertical=rectifier.disparity_axis==1
        if vertical:projection[0,0]=rectifier.P1[1,1]
        swapped=rectifier.P2[rectifier.disparity_axis,3]>0
        baseline=float(np.linalg.norm(calibration.t_right_left))
        depth_engine=StereoDepth(projection,baseline,scale)
        # 只改显示用匹配器的搜索窗口：原厂默认 96 对应最近 0.06 m，远超 0.25 m 的近端门限，
        # 多出来的搜索宽度只会加宽左右两侧搜索死区（黑边）并拖慢计算。其余参数与
        # StereoDepth.__init__ 保持一致，深度换算和有效门限完全不变。
        disparities=self.required_disparities(depth_engine.fx,baseline)
        if disparities!=96:
            depth_engine.matcher=cv2.StereoSGBM_create(
                minDisparity=0,numDisparities=disparities,blockSize=5,
                P1=200,P2=800,uniquenessRatio=12,speckleWindowSize=60,speckleRange=2)
        self._rectifier=rectifier;self._depth=depth_engine;self._disparities=disparities
        self._scale=scale;self._swapped=swapped;self._vertical=vertical
        self._key=(getattr(calibration,'calibration_id',None),getattr(frame,'calibration_id',None),
                   int(getattr(frame.left_image,'shape',(0,0))[1] or 0))


def fisheye_native(image):
    """原始鱼眼按设备输出原样显示：不裁剪、不加圆形遮罩、不改像素。"""
    return image


def display_sized(image,target_width):
    """仅用于显示：按目标宽度做一次面积平均降采样，减少 Qt 侧平滑缩放开销。

    只缩小显示副本，不改变原始帧、检测像素或任何计算输入。
    """
    if image is None:return None
    height,width=image.shape[:2]
    if target_width<=0 or width<=target_width*1.2:return image
    scale=target_width/width
    size=(max(1,int(round(width*scale))),max(1,int(round(height*scale))))
    return cv2.resize(image,size,interpolation=cv2.INTER_AREA)
