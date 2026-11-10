import numpy as np
import pyqtgraph.opengl as gl
from PySide6.QtCore import Qt,Signal,QPointF,QEvent
from PySide6.QtGui import QVector3D,QVector4D,QFont,QColor
from PySide6.QtWidgets import QLabel,QApplication
from ..detection.landmarks import BONES
from .image_view import pair_color

def optical_to_display(points):
    """左相机光学坐标 (右、下、前) 转为显示坐标 (右、前、上)。"""
    return np.asarray(points,dtype=float)[...,[0,2,1]]*[1,1,-1]

class SkeletonView(gl.GLViewWidget):
    joint_selected=Signal(str)
    def __init__(self,parent=None):
        super().__init__(parent)
        self._clear_drag()
        self.setMinimumHeight(220)
        self.setBackgroundColor("#111820")
        self.theme={"left_hand":"#5FE0C5","right_hand":"#F0BB76"}
        self.last_result=None
        # 默认围绕固定左相机原点；手移动不会带着观察视角和网格一起移动。
        self.rotation_center_mode='camera'
        self._pivot_target=None
        self._pivot_result=None
        self.ground_truth=[];self.show_ground_truth=False
        self.mano_prediction=[];self.mano_truth=[]
        # 合并为一个网格绘制所有小球，避免重叠点精度测试造成外层遮住中心。
        # 真值点半径（米）：缩小以便同时看清预测点和骨架。
        self.truth_sphere=gl.MeshData.sphere(rows=8,cols=12,radius=0.002)
        self.dynamic=[];self.points=[]
        self.grid=gl.GLGridItem(antialias=False,glOptions="opaque");self.grid.setSize(.8,.8);self.grid.setSpacing(.1,.1);self.addItem(self.grid)
        self._place_grid()
        # 光学坐标 X 向右、Y 向下、Z 向前；显示为 (X,Z,-Y)，使 GL 的 Z 轴向上。
        self.camera_axis=gl.GLAxisItem(size=QVector3D(.10,.10,.10));self.camera_axis.rotate(-90,1,0,0,local=True);self.addItem(self.camera_axis)
        self.camera_origin=gl.GLScatterPlotItem(pos=np.array([[0.,0.,0.]]),color=(1.,1.,1.,1.),size=9,pxMode=True,glOptions="translucent");self.addItem(self.camera_origin)
        # Widget overlays avoid mixing QPainter into PyQtGraph's GL item pass.
        # GLTextItem can leave driver state behind during light-theme redraws.
        self.legend=QLabel("白点：左相机原点  ·  固定参考网格：Y = 0.35 m，间距 0.10 m",self)
        self.legend.setGeometry(12,10,480,25)
        self.legend.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.reset_camera()
    def reset_camera(self):
        self._clear_drag()
        self.opts["center"]=QVector3D(0,.375,0)
        self._pivot_target=None
        self._update_rotation_center(force=True)
        self.setCameraPosition(distance=1.35,elevation=60,azimuth=-90)
    def set_rotation_center_mode(self,mode):
        if mode not in ('hand','camera','range'):raise ValueError('未知旋转中心')
        self.rotation_center_mode=mode
        self._pivot_target=None
        self._update_rotation_center(force=True)
        self.update()
    def _update_rotation_center(self,force=False):
        # GL 旋转和平移共用 center：跟踪手部位移时保留用户手动平移的偏移。
        if self._drag_button is not None and not force:return
        points=[]
        if self.last_result is not None:
            points=[j.xyz_m for h in self.last_result.hands for j in h.joints
                    if j.valid and np.isfinite(j.xyz_m).all()]
        if not points and self.show_ground_truth:
            for hand in self.ground_truth:
                xyz=np.asarray(hand['xyz_m'],float)
                valid=np.asarray(hand['valid'],bool)&np.isfinite(xyz).all(axis=1)
                points.extend(xyz[valid])
        if self.rotation_center_mode=='camera':target=np.zeros(3)
        elif len(points):
            xyz=optical_to_display(points)
            if self.rotation_center_mode=='range':xyz=np.vstack((xyz,np.zeros((1,3))))
            target=(xyz.min(axis=0)+xyz.max(axis=0))/2
        else:
            # 没有可用手部时保留上次中心，避免丢失检测时视角跳动。
            return
        if force or self._pivot_target is None:center=target
        else:center=np.array(self.opts['center'].toTuple())+target-self._pivot_target
        self.opts['center']=QVector3D(*map(float,center))
        self._pivot_target=target
    def zoom_by(self,factor):
        self.setCameraPosition(distance=max(.2,min(5.,self.opts["distance"]*factor)))
    def set_theme(self,tokens):
        self.theme=tokens;self.setBackgroundColor(tokens["canvas"])
        self.grid.setColor(QColor(tokens["border"]))
        self.legend.setStyleSheet(f"color:{tokens['muted']};background:{tokens['canvas']};font-size:12px;")
        if self.last_result is not None:self.display(self.last_result)
        self.update()
    def display(self,result):
        self.last_result=result
        if result is not self._pivot_result:
            self._pivot_result=result
            self._update_rotation_center()
        for item in self.dynamic: self.removeItem(item)
        self.dynamic=[];self.points=[]
        # 仅变换显示坐标，不改变输出：光学 (x,y,z) -> GL (x,z,-y)。
        for index,h in enumerate(result.hands):
            is_gt=result.diagnostics.get('result_kind')=='official_ground_truth' or result.diagnostics.get('synthetic')
            col=QColor((self.theme["left_hand"] if h.handedness=="Left" else self.theme["right_hand"] if h.handedness=="Right" else '#D4DAE3') if is_gt or (h.track_id<=0 and h.tracking_status!='frame_only') else pair_color(self.theme,h.track_id if h.track_id>0 else index+1)).getRgbF()
            xyz=optical_to_display([j.xyz_m for j in h.joints])
            for a,b in BONES:
                if h.joints[a].valid and h.joints[b].valid:
                    item=gl.GLLinePlotItem(pos=xyz[[a,b]],color=col,width=2,antialias=False,glOptions="opaque")
                    self.addItem(item);self.dynamic.append(item)
            valid=np.array([j.valid for j in h.joints]) & np.isfinite(xyz).all(axis=1)
            if valid.any():
                item=gl.GLScatterPlotItem(pos=xyz[valid],color=col,size=8,pxMode=True,glOptions="translucent")
                self.addItem(item);self.dynamic.append(item)
                for i in np.flatnonzero(valid): self.points.append((xyz[i],h,h.joints[i]))
        if self.show_ground_truth:
            for hand in self.ground_truth:
                xyz=optical_to_display(hand['xyz_m'])
                valid=np.asarray(hand['valid'],bool)&np.isfinite(xyz).all(axis=1)
                if valid.any():
                    # 真值使用单层紫色小球；有官方语义映射时才连接骨骼。
                    indices=hand.get('mediapipe_indices')
                    if indices is not None and len(indices)==len(xyz) and len(set(indices))==len(indices):
                        lookup={joint:i for i,joint in enumerate(indices)}
                        pairs=[(lookup[a],lookup[b]) for a,b in BONES if a in lookup and b in lookup]
                        if 1 not in lookup and 0 in lookup and 2 in lookup:pairs.append((lookup[0],lookup[2]))
                        segments=[xyz[[a,b]] for a,b in pairs if valid[a] and valid[b]]
                        if segments:
                            item=gl.GLLinePlotItem(pos=np.concatenate(segments),mode='lines',color=(.82,.35,1.,1.),width=2,antialias=False,glOptions='opaque')
                            self.addItem(item);self.dynamic.append(item)
                    centres=xyz[valid];vertices=self.truth_sphere.vertexes();faces=self.truth_sphere.faces()
                    combined=(centres[:,None,:]+vertices[None,:,:]).reshape(-1,3)
                    triangles=(faces[None,:,:]+np.arange(len(centres))[:,None,None]*len(vertices)).reshape(-1,3)
                    item=gl.GLMeshItem(vertexes=combined,faces=triangles,color=(.82,.35,1.,1.),smooth=True,drawEdges=False,glOptions='opaque')
                    self.addItem(item);self.dynamic.append(item)
        # 预测表面和官方真值线框使用同一左相机坐标，不改变骨架数据。
        for meshes,color,wire in ((self.mano_prediction,(1.,.57,.18,.45),False),
                                  (self.mano_truth,(.82,.35,1.,.9),True)):
            for mesh in meshes:
                vertices=optical_to_display(mesh['vertices'])
                faces=np.asarray(mesh['faces'],dtype=np.int32)
                if vertices.shape!=(778,3) or faces.ndim!=2 or faces.shape[1]!=3 or not np.isfinite(vertices).all():continue
                item=gl.GLMeshItem(vertexes=vertices,faces=faces,smooth=True,drawFaces=not wire,
                    drawEdges=wire,edgeColor=color,color=color,glOptions='translucent')
                self.addItem(item);self.dynamic.append(item)
    def set_mano_meshes(self,predicted,truth,redraw=True):
        self.mano_prediction=predicted or [];self.mano_truth=truth or []
        if redraw and self.last_result is not None:self.display(self.last_result)
    def _place_grid(self):
        # 固定在左相机坐标系 Y=0.35 m，平行光学 XZ 平面，不随手部运动。
        self.grid.resetTransform()
        self.grid.translate(0.,.75,-.35)
    def set_ground_truth(self,hands,enabled,redraw=True):
        self.ground_truth=hands or [];self.show_ground_truth=bool(enabled)
        self._update_rotation_center()
        self.legend.setText('紫色：官方真值  ·  彩色骨架：算法预测' if enabled and self.ground_truth else '当前帧无可显示真值' if enabled else '白点：左相机原点  ·  固定参考网格：Y = 0.35 m，间距 0.10 m')
        if redraw and self.last_result is not None:self.display(self.last_result)
    def mousePressEvent(self,event):
        # Initialize the inherited anchor before any optional picking/UI work.
        # A failed pick must never leave the previous gesture's mousePos active.
        super().mousePressEvent(event)
        self._clear_drag()
        if event.button() in (Qt.LeftButton,Qt.MiddleButton):
            self._drag_button=event.button()
            self._press_global=QPointF(event.globalPosition())
            self._last_global=QPointF(self._press_global)
            self._press_modifiers=event.modifiers()
        event.accept()

    def mouseMoveEvent(self,event):
        if self._drag_button is None or not event.buttons() & self._drag_button:
            self._clear_drag()
            self.mousePos=QPointF(event.position())
            event.accept()
            return
        current=QPointF(event.globalPosition())
        delta=current-self._last_global
        self._last_global=current
        if (current-self._press_global).manhattanLength()>=QApplication.startDragDistance():
            self._dragged=True
        # Keep PyQtGraph's orbit/pan semantics but supply an offset in stable
        # screen coordinates. Widget reflow is not physical pointer movement.
        self.mousePos=event.position()-delta
        super().mouseMoveEvent(event)
        event.accept()

    def mouseReleaseEvent(self,event):
        click=(self._drag_button==Qt.LeftButton and event.button()==Qt.LeftButton
               and not self._dragged and self._press_global is not None
               and (event.globalPosition()-self._press_global).manhattanLength()<QApplication.startDragDistance()
               and not (self._press_modifiers | event.modifiers()) & Qt.ControlModifier)
        self._clear_drag()
        super().mouseReleaseEvent(event)
        self._update_rotation_center()
        if click:self._pick_joint(event.position())
        event.accept()

    def _clear_drag(self):
        self._drag_button=None
        self._press_global=None
        self._last_global=None
        self._press_modifiers=Qt.NoModifier
        self._dragged=False

    def event(self,event):
        if event.type() in (QEvent.UngrabMouse,QEvent.WindowDeactivate,QEvent.FocusOut,QEvent.Hide):
            self._clear_drag()
        return super().event(event)

    def _pick_joint(self,position):
        if self.points:
            matrix=self.projectionMatrix()*self.viewMatrix()
            candidates=[]
            for p,h,j in self.points:
                # map() avoids PySide6's ambiguous matrix/vector operator
                # dispatch, and explicit floats avoid NumPy scalar overloads.
                clip=matrix.map(QVector4D(float(p[0]),float(p[1]),float(p[2]),1.0))
                if clip.w()<=0: continue
                screen=np.array([(clip.x()/clip.w()+1)*self.width()/2,(1-clip.y()/clip.w())*self.height()/2])
                distance=np.linalg.norm(screen-[position.x(),position.y()])
                candidates.append((distance,h,j))
            if candidates:
                distance,h,j=min(candidates,key=lambda c:c[0])
                if distance<18:
                    x,y,z=j.xyz_m
                    smoothed=' · 显示平滑' if self.last_result.diagnostics.get('display_smoothed') else ''
                    self.joint_selected.emit(f"track {h.track_id} · {j.joint_id} {j.joint_name} · X {x:.5f}  Y {y:.5f}  Z {z:.5f} m · {j.status}{smoothed}")
