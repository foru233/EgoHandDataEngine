"""客户交付界面的显示组件：无边框图像块、世界系手部叠加、深度后台任务。

本模块只做显示：不改变引擎、检测、三角化或 VIO 的任何行为。
"""
import time
import hashlib
from pathlib import Path

import cv2
import numpy as np
import pyqtgraph.opengl as gl
from egohand.core.pipeline import ROOT
from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QVector3D
from PySide6.QtGui import QColor,QPainter
from PySide6.QtWidgets import QLabel, QWidget

from egohand.detection.landmarks import BONES
from egohand.gui.theme import THEMES
from egohand.vio.trajectory import rigid_transform
from egohand.visualization.image_view import ImageView
from egohand.visualization.stereo_depth import DepthCache,colorize_depth,compute_depth

HAND_COLORS={'Left':(0.37,0.88,0.77,1.0),'Right':(0.94,0.73,0.46,1.0),'Unknown':(0.83,0.86,0.90,1.0)}
# Display-only rigid rotation: legacy world Y-up/down convention is rotated
# into the customer view's vertical Z direction.  det(R)=+1, so handedness
# is preserved and VIO, hands, and body share exactly the same presentation.
DISPLAY_WORLD_ROTATION=np.array([[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]])


class DeliveryImageView(ImageView):
    """Keep empty customer tiles quiet; research instructions stay in ImageView."""
    def paintEvent(self,event):
        if self.pix is not None:
            super().paintEvent(event)
            return
        painter=QPainter(self)
        painter.fillRect(self.rect(),QColor(self.theme['canvas']))
        painter.setPen(QColor(self.theme['muted']))
        painter.drawText(self.rect(),Qt.AlignCenter,self.title)


class ImageTile(QWidget):
    """图像块：内部视图按图像长宽比铺满格子，标题以角标浮在图像上，不占用图像面积。"""

    def __init__(self,placeholder,parent=None,aspect=4/3):
        super().__init__(parent)
        self.aspect=float(aspect)
        self.view=DeliveryImageView(placeholder)
        self.view.set_theme(THEMES['dark'])
        self.view.setMinimumSize(1,1)
        self.view.setParent(self)
        self.caption=QLabel('',self.view);self.caption.setObjectName('tileCaption')
        self.caption.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.caption.setWordWrap(True)
        self.caption.setMaximumHeight(36)
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
        self._place_caption()

    def showEvent(self,event):
        super().showEvent(event)
        self._place()

    def set_caption(self,text):
        if not text:
            self.caption.hide();return
        self.caption.setToolTip(text)
        # Dimensions and diagnostic prose belong in the tooltip, not over pixels.
        parts=text.split(' · ')
        short=' · '.join(parts[:2]) if parts[0] in ('原始','校正') else '深度'
        if parts[0]=='深度' and len(parts)>1 and parts[1] in ('上帧','计算中'):short+=' · '+parts[1]
        if '无原始图' in text:short+=' · 无图'
        elif '不可用' in text or '未启用' in text:short+=' · 不可用'
        elif parts[0]=='原始' and len(parts)>3:short+=' · 异帧'
        self.caption.setText(short)
        self._place_caption()
        self.caption.show()
        self.caption.raise_()

    def _place_caption(self):
        width=max(1,min(self.caption.sizeHint().width(),self.view.width()-16))
        self.caption.setGeometry(8,6,width,min(36,max(16,self.caption.heightForWidth(width))))

    def clear(self):
        self.view.set_roi_selection(False)
        self.view.pix=None;self.view.last_bgr=None;self.view._image_size=None
        self.view.clear();self.view.update()


def transform_points(points,transform=None):
    """左相机光学系关节 -> VIO 世界系；没有有效位姿时按相机原点（单位变换）显示。"""
    values=np.asarray(points,dtype=float).reshape(-1,3)
    matrix=np.eye(4) if transform is None else rigid_transform(transform)
    world=values@matrix[:3,:3].T+matrix[:3,3]
    return world@DISPLAY_WORLD_ROTATION.T

def _tube_mesh(a,b,r0,r1,sections=10):
    """Low-poly tapered bone surface, without joint spheres."""
    a=np.asarray(a,float); b=np.asarray(b,float); axis=b-a; length=float(np.linalg.norm(axis))
    if length<1e-6:return None
    w=axis/length; ref=np.array([0.,0.,1.]) if abs(w[2])<.9 else np.array([0.,1.,0.])
    u=np.cross(w,ref);u/=max(np.linalg.norm(u),1e-9);v=np.cross(w,u)
    rings=[]
    for center,r in ((a,r0),(b,r1)):
        rings.append([center+r*(np.cos(t)*u+np.sin(t)*v) for t in np.linspace(0,2*np.pi,sections,endpoint=False)])
    vertices=np.asarray(rings).reshape(-1,3)
    faces=[]
    for i in range(sections):
        j=(i+1)%sections;faces.extend(((i,j,sections+j),(i,sections+j,sections+i)))
    # Triangulated caps keep the face array rectangular for GLMeshItem.
    for i in range(1,sections-1):
        faces.append((0,i+1,i));faces.append((sections,sections+i,sections+i+1))
    return vertices,np.asarray(faces,dtype=np.int32)

def _align_rotation(a,b):
    a=np.asarray(a,float);b=np.asarray(b,float)
    na=np.linalg.norm(a);nb=np.linalg.norm(b)
    if na<1e-8 or nb<1e-8:return np.eye(3)
    a=a/na;b=b/nb;v=np.cross(a,b);c=float(np.dot(a,b));s=np.linalg.norm(v)
    if s<1e-8:return np.eye(3) if c>0 else np.diag([1.,-1.,-1.])
    K=np.array([[0.,-v[2],v[1]],[v[2],0.,-v[0]],[-v[1],v[0],0.]])/s
    return np.eye(3)+K*s+K@K*(1-c)


class WorldHandOverlay:
    """把左相机光学系关节变换到 VIO 世界系，画在与相机轨迹同一个三维视图里。"""

    def __init__(self,view):
        self.view=view
        self.items=[]
        self.visible=True
        self._stable_by_track={}
        self._last_wrist={}
        from egohand.vio.hand_identity_stabilizer import HandIdentityStabilizer
        self._identity=HandIdentityStabilizer()
        self.mesh_visible=True
        self.skeleton_visible=True
        self.mesh_items=[]
        self._mano_signature=None

    def set_visible(self,visible):
        self.visible=bool(visible)
        for item in self.items:item.setVisible(self.visible)

    def clear(self):
        for item in self.items:self.view.removeItem(item)
        self.items=[]
        for item in self.mesh_items:self.view.removeItem(item)
        self.mesh_items=[]

    def set_mesh_visible(self, visible):
        self.mesh_visible=bool(visible)
        for item in self.mesh_items:item.setVisible(self.visible and self.mesh_visible)

    def set_skeleton_visible(self, visible):
        self.skeleton_visible=bool(visible)
        for item in self.items:item.setVisible(self.visible and self.skeleton_visible)

    def update_mano_meshes(self, meshes, transform=None, hands=None):
        bound=bool(meshes) and all(m.get('coordinate_frame')=='body_world' and m.get('smpl_parent_joint') in (20,21) for m in meshes)
        if bound:
            # The unified cache stores MANO geometry relative to the very
            # same SMPL wrist transform, already in the body's world frame.
            # Never apply the camera pose a second time or reanchor to the
            # detector wrist after this explicit parent binding.
            transform=None;hands=None
            resolved=[]
            for m in meshes:
                try:
                    T=rigid_transform(m['T_world_from_smpl_wrist'])
                    local=np.asarray(m['vertices_wrist_local'],float)
                    resolved.append({**m,'vertices':(local@T[:3,:3].T+T[:3,3]).tolist()})
                except (KeyError,ValueError,TypeError):continue
            meshes=resolved
        matrix=np.eye(4) if transform is None else rigid_transform(transform)
        # Fitting can leave a residual at the MANO wrist.  Anchor the rendered
        # mesh by a rigid translation to the matching, same-frame track; the
        # stored fit and the detector output remain untouched.  Ambiguous or
        # missing IDs deliberately receive no correction.
        wrists={}
        for hand in hands or ():
            track=getattr(hand,'track_id',None)
            if not isinstance(track,(int,np.integer)) or track<=0:
                continue
            joints=getattr(hand,'joints',())
            if not joints or not getattr(joints[0],'valid',False):
                continue
            xyz=np.asarray(joints[0].xyz_m,dtype=float)
            if xyz.shape!=(3,) or not np.isfinite(xyz).all():
                continue
            wrists.setdefault(int(track),[]).append(xyz)
        offsets=[]
        for mesh in meshes or []:
            track=mesh.get('track_id')
            candidates=wrists.get(track,[])
            joints=np.asarray(mesh.get('joints20',[]),dtype=float)
            offset=np.zeros(3)
            if len(candidates)==1 and joints.ndim==2 and joints.shape[1]==3 and len(joints) and np.isfinite(joints[0]).all():
                offset=candidates[0]-joints[0]
            offsets.append(offset)
        signature=(len(meshes or []), tuple((len(m.get('vertices',[])),len(m.get('faces',[])),
            hashlib.blake2b(np.asarray(m.get('vertices',[]),dtype=np.float64).tobytes(),digest_size=16).digest(),
            hashlib.blake2b(np.asarray(m.get('faces',[]),dtype=np.int32).tobytes(),digest_size=16).digest()) for m in (meshes or [])),
            tuple(matrix.reshape(-1)),tuple(tuple(x) for x in offsets))
        if signature==self._mano_signature and self.mesh_items:
            for item in self.mesh_items:item.setVisible(self.mesh_visible)
            return
        for item in self.mesh_items:self.view.removeItem(item)
        self.mesh_items=[]
        self._mano_signature=signature
        if not self.mesh_visible:return
        for mesh,offset in zip(meshes or [],offsets):
            vertices=np.asarray(mesh.get('vertices'),dtype=float)
            faces=np.asarray(mesh.get('faces'),dtype=np.int32)
            if vertices.ndim!=2 or vertices.shape[1]!=3 or faces.ndim!=2 or faces.shape[1]!=3:continue
            if not np.isfinite(vertices).all():continue
            vertices=transform_points(vertices+offset,matrix)
            item=gl.GLMeshItem(vertexes=vertices,faces=faces,color=(1.,.57,.18,.55),smooth=True,drawEdges=False,glOptions='translucent')
            item.setVisible(self.mesh_visible);self.view.addItem(item);self.mesh_items.append(item)

    def has_geometry(self):
        return bool(self.items)

    def update(self,hands,transform=None):
        """hands 的关节在左相机光学系；transform 为 T_world_camera（缺失时按相机原点显示）。"""
        self.clear()
        if not hands:return
        hands,_=self._identity.update(hands)
        pending=[]
        current_wrist={}
        used=set()
        for index,hand in enumerate(hands):
            joints=np.asarray([j.xyz_m for j in hand.joints],dtype=float)
            valid=np.array([j.valid for j in hand.joints])&np.isfinite(joints).all(axis=1)
            if not valid.any():continue
            world=transform_points(joints,transform)
            track=int(getattr(hand,'track_id',-1) or -1)
            stable=self._stable_by_track.get(track) if track>0 else None
            if stable is None:
                wrist=world[0] if valid[0] else world[valid][0]
                candidates=[(float(np.linalg.norm(wrist-prev)),side) for side,prev in self._last_wrist.items() if side not in used]
                if candidates and min(candidates)[0] < .18:
                    stable=min(candidates)[1]
                else:
                    stable=hand.handedness if hand.handedness in ('Left','Right') else 'Unknown'
            if track>0:self._stable_by_track[track]=stable
            used.add(stable)
            current_wrist[stable]=world[0] if valid[0] else world[valid][0]
            from egohand.visualization.image_view import pair_color
            color=QColor(pair_color(THEMES['dark'], track if track>0 else index+1)).getRgbF()
            if hand.track_id>0:
                color=(color[0],color[1],color[2],1.0)
            segments=[world[[a,b]] for a,b in BONES if valid[a] and valid[b]]
            if segments:
                pending.append(('lines',np.concatenate(segments),color))
            pending.append(('points',world[valid],color))
        self._last_wrist=current_wrist
        for kind,pos,color in pending:
            if kind=='lines':
                item=gl.GLLinePlotItem(pos=pos,mode='lines',color=color,width=2,antialias=True,
                                       glOptions='translucent')
            else:
                item=gl.GLScatterPlotItem(pos=pos,color=color,size=.012,pxMode=False,
                                          glOptions='translucent')
            item.setVisible(self.visible and self.skeleton_visible)
            self.view.addItem(item)
            self.items.append(item)


class BodyPoseOverlay:
    """Display-only body skeleton; measured wrists and inferred joints differ visually."""

    def __init__(self, view):
        self.view = view
        self.items = []
        self.visible = False
        self.pose = None
        self.mesh_visible = True
        self._mesh_item = None
        self._mesh_frame_valid = False
        self._smpl_asset = None
        self._fit_vertices = None
        self._fit_path = None
        self._fit_valid = None
        self.skeleton_visible = True

    def has_geometry(self):
        return bool(self.items)

    def set_visible(self, visible):
        self.visible = bool(visible)
        for item in self.items:
            if getattr(item,'_body_mesh',False):
                item.setVisible(self.visible and self.mesh_visible and self._mesh_frame_valid)
            else:
                item.setVisible(self.visible and self.skeleton_visible)

    def set_mesh_visible(self, visible):
        self.mesh_visible=bool(visible)
        for item in self.items:
            if getattr(item,'_body_mesh',False): item.setVisible(self.visible and self.mesh_visible and self._mesh_frame_valid)

    def set_skeleton_visible(self, visible):
        self.skeleton_visible=bool(visible)
        for item in self.items:
            if not getattr(item,'_body_mesh',False): item.setVisible(self.visible and self.skeleton_visible)

    def clear(self, preserve_mesh=False):
        for item in self.items:
            if preserve_mesh and item is self._mesh_item: continue
            self.view.removeItem(item)
        if not preserve_mesh:self._mesh_item=None
        self.items = [self._mesh_item] if preserve_mesh and self._mesh_item is not None else []
        self.pose = None

    def _update_unified(self,pose):
        """Official joints and surface already share one exported SMPL pose."""
        from pyqtgraph.opengl.shaders import ShaderProgram,VertexShader,FragmentShader
        if 'smplPreview' not in ShaderProgram.names:
            ShaderProgram('smplPreview',[
                VertexShader('''varying vec3 n;
                    void main(){n=normalize(gl_NormalMatrix*gl_Normal);
                    gl_FrontColor=gl_Color;gl_BackColor=gl_Color;gl_Position=ftransform();}'''),
                FragmentShader('''varying vec3 n;
                    void main(){float light=0.48+0.52*max(0.0,dot(normalize(n),normalize(vec3(0.3,0.5,1.0))));
                    gl_FragColor=vec4(gl_Color.rgb*light,1.0);}''')])
        diag=pose.diagnostics
        path=Path(diag.get('body_mesh_vertices_path') or '')
        index=diag.get('body_mesh_index');cached=diag.get('body_mesh') or {}
        inline=diag.get('body_mesh_vertices')
        if (inline is not None or (path.is_file() and type(index) is int)) and cached.get('faces'):
            try:
                if inline is not None:vertices=np.asarray(inline,float)
                else:
                    if self._fit_path!=path:
                        self._fit_vertices=np.load(path,mmap_mode='r',allow_pickle=False)
                        self._fit_path=path
                    vertices=np.asarray(self._fit_vertices[index],float)
                faces=np.asarray(cached['faces'],np.int32)
                hidden=[]
                for side in diag.get('mano_attached_sides',[]):
                    hidden.extend((cached.get('mano_hand_face_indices') or {}).get(side,[]))
                if hidden:
                    mask=np.ones(len(faces),bool);mask[np.asarray(hidden,dtype=int)]=False
                    faces=faces[mask]
                if vertices.shape!=(6890,3) or not np.isfinite(vertices).all():raise ValueError('Invalid SMPL vertices')
                # body_world is already the exact joint coordinate system.
                # Apply only the same proper display rotation used by hands.
                vertices=vertices@DISPLAY_WORLD_ROTATION.T
                self._mesh_frame_valid=True
                if self._mesh_item is None:
                    item=gl.GLMeshItem(vertexes=vertices,faces=faces,color=(.40,.66,.92,1.),shader='smplPreview',smooth=True,drawEdges=False,glOptions='opaque')
                    item._body_mesh=True;self.view.addItem(item);self._mesh_item=item;self.items.append(item)
                else:self._mesh_item.setMeshData(vertexes=vertices,faces=faces)
                self._mesh_item.setVisible(self.visible and self.mesh_visible)
            except (ValueError,IndexError,OSError):self._mesh_frame_valid=False
        # The 24 points and every edge come directly from SMPL, never legacy
        # lengths, IK targets or a separately skinned handcrafted skeleton.
        points=pose.joints
        segments=[np.array([points[a],points[b]])@DISPLAY_WORLD_ROTATION.T for a,b in pose.bones if a in points and b in points]
        if segments:
            item=gl.GLLinePlotItem(pos=np.concatenate(segments),mode='lines',color=(.75,.85,1.,1.),width=2,antialias=True)
            item.setVisible(self.visible and self.skeleton_visible);self.view.addItem(item);self.items.append(item)
        ground=diag.get('ground_y_m')
        if isinstance(ground,(int,float)) and hasattr(self.view,'grid'):
            self.view.grid.resetTransform();self.view.grid.translate(0.,0.,-ground)

    def update(self, pose):
        self.clear(preserve_mesh=True)
        self.pose = pose
        self._mesh_frame_valid=False
        # A previously displayed mesh belongs to a different frame until a
        # valid mesh is explicitly installed below.
        if self._mesh_item is not None:
            self._mesh_item.setVisible(False)
        if pose is None:
            return
        if pose.diagnostics.get('unified_smpl'):
            self._update_unified(pose)
            return
        strict = pose.status == 'strict_valid'
        inferred = (0.42, 0.68, 1.0, 0.72) if strict else (1.0, 0.68, 0.18, 0.72)
        measured_colors = {'LeftWrist': (0.37, 0.88, 0.77, 1.0),
                           'RightWrist': (0.94, 0.73, 0.46, 1.0)}
        points = pose.joints
        rotation = pose.diagnostics.get('head_rotation_world')
        if rotation is not None and 'Head' in points:
            rotation = np.asarray(rotation, dtype=float)
            if rotation.shape == (3, 3) and np.isfinite(rotation).all():
                # A light wire ellipsoid conveys head shape/orientation.  It is
                # illustrative geometry, not a scanned or inferred face.
                head = np.asarray(points['Head'], dtype=float)
                segments = head_wireframe(head, rotation)
                item = gl.GLLinePlotItem(pos=segments @ DISPLAY_WORLD_ROTATION.T,
                                         mode='lines', color=(1., .9, .5, 1.), width=3,
                                         antialias=True, glOptions='translucent')
                item.setVisible(self.visible and self.skeleton_visible)
                self.view.addItem(item); self.items.append(item)
        foot_points = []
        for name in ("LeftAnkle", "RightAnkle"):
            if name in points:
                foot_points.append(points[name] @ DISPLAY_WORLD_ROTATION.T)
        if self.mesh_visible:
            cached=pose.diagnostics.get('body_mesh')
            if isinstance(cached,dict) and cached.get('vertices') and cached.get('faces'):
                vertices=np.asarray(cached['vertices'],dtype=float)
                using_fit=False
                fit_path=Path(pose.diagnostics.get('body_mesh_vertices_path',''))
                fit_index=pose.diagnostics.get('body_mesh_index')
                if fit_index is not None and fit_path.is_file():
                    try:
                        if self._fit_path != fit_path:
                            self._fit_vertices=np.load(fit_path,mmap_mode='r',allow_pickle=False)
                            valid_path=fit_path.with_name('body_vertices_valid.npy')
                            self._fit_valid=np.load(valid_path,mmap_mode='r',allow_pickle=False) if valid_path.is_file() else None
                            self._fit_path=fit_path
                        index=int(fit_index)
                        valid=0<=index<len(self._fit_vertices) and (self._fit_valid is None or (len(self._fit_valid)==len(self._fit_vertices) and bool(self._fit_valid[index])))
                        if valid:
                            vertices=np.asarray(self._fit_vertices[int(fit_index)],dtype=float)
                            using_fit=True
                            cached={**cached,'faces':self._smpl_asset['faces'] if self._smpl_asset is not None else cached['faces']}
                    except (OSError,ValueError,IndexError):pass
                asset_path=ROOT/'models'/'smpl'/'body_mesh_smpl.npz'
                if self._smpl_asset is None and asset_path.is_file():
                    try:self._smpl_asset=dict(np.load(asset_path,allow_pickle=False))
                    except Exception:self._smpl_asset=None
                if using_fit:
                    # The fitter stores SMPL coordinates with Y up; BodyPose
                    # uses Y down before the shared world-to-display rotation.
                    vertices=vertices*np.array([1.,-1.,1.])
                vertices=vertices@DISPLAY_WORLD_ROTATION.T
                faces=np.asarray(cached['faces'],dtype=np.int32)
                if using_fit and vertices.ndim==2 and vertices.shape[1]==3 and faces.ndim==2 and faces.shape[1]==3 and np.isfinite(vertices).all():
                    self._mesh_frame_valid=True
                    if self._mesh_item is None:
                        item=gl.GLMeshItem(vertexes=vertices,faces=faces,color=(.18,.55,.95,.32),smooth=False,drawEdges=False,glOptions='translucent')
                        item._body_mesh=True;self.view.addItem(item);self._mesh_item=item;self.items.append(item)
                    else:
                        self._mesh_item.setMeshData(vertexes=vertices,faces=faces)
                    self._mesh_item.setVisible(self.visible and self.mesh_visible)
            else:
                verts=[];faces=[]
                radii={'Head':.105,'Neck':.075,'CervicalBase':.13,'Chest':.18,'Pelvis':.16,
                   'Shoulder':.075,'Elbow':.055,'Wrist':.045,'Hip':.085,'Knee':.065,'Ankle':.045}
                for parent,child in pose.bones:
                    if parent not in points or child not in points:continue
                    mesh=_tube_mesh(np.asarray(points[parent])@DISPLAY_WORLD_ROTATION.T,
                                np.asarray(points[child])@DISPLAY_WORLD_ROTATION.T,
                                next((r for k,r in radii.items() if parent.endswith(k)),.05),
                                next((r for k,r in radii.items() if child.endswith(k)),.04),sections=6)
                    if mesh is None:continue
                    v,f=mesh; base=len(verts);verts.extend(v.tolist());faces.extend((f+base).tolist())
                if verts:
                    item=gl.GLMeshItem(vertexes=np.asarray(verts),faces=np.asarray(faces,dtype=np.int32),color=(.18,.55,.95,.28),smooth=False,drawEdges=False,glOptions='translucent')
                    item._body_mesh=True;item.setVisible(self.visible and self.mesh_visible);self.view.addItem(item);self._mesh_item=item;self.items.append(item)
        if foot_points and hasattr(self.view, 'grid'):
            # Display-only floor follows the estimated foot soles, never the
            # head.  The world origin/axes remain unchanged.
            floor_z = float(np.mean(np.asarray(foot_points)[:, 2])) - .025
            ground_y = pose.diagnostics.get('ground_y_m')
            if isinstance(ground_y,(int,float)) and np.isfinite(ground_y):
                floor_z = float((np.array([0.,ground_y,0.]) @ DISPLAY_WORLD_ROTATION.T)[2])
            self.view.grid.resetTransform()
            self.view.grid.translate(0., 0., floor_z)
        # Batch same-style bones and joints: creating one GL item per joint
        # and bone on every replay frame made seeking and playback expensive.
        segments=[np.asarray([points[a],points[b]],dtype=float)@DISPLAY_WORLD_ROTATION.T
                  for a,b in pose.bones if a in points and b in points]
        if segments:
            item=gl.GLLinePlotItem(pos=np.concatenate(segments),mode='lines',color=inferred,
                                   width=3,antialias=True,glOptions='translucent')
            item.setVisible(self.visible and self.skeleton_visible)
            self.view.addItem(item);self.items.append(item)
        groups={}
        for name,point in points.items():
            measured=name in pose.measured_joints
            color=measured_colors.get(name,inferred) if measured else inferred
            groups.setdefault((color,9 if measured else 6),[]).append(point@DISPLAY_WORLD_ROTATION.T)
        for (color,size),positions in groups.items():
            item=gl.GLScatterPlotItem(pos=np.asarray(positions,dtype=float),color=color,
                                      size=size,pxMode=True,glOptions='translucent')
            item.setVisible(self.visible and self.skeleton_visible)
            self.view.addItem(item);self.items.append(item)


def head_wireframe(center, rotation, samples=24):
    """Coarse head ellipsoid in head-local x-right/y-down/z-forward metres."""
    center=np.asarray(center,dtype=float);rotation=np.asarray(rotation,dtype=float)
    if center.shape!=(3,) or rotation.shape!=(3,3) or not np.isfinite(center).all() or not np.isfinite(rotation).all():
        raise ValueError('invalid head pose')
    angles=np.linspace(0,2*np.pi,samples,endpoint=False)
    circles=(np.column_stack((.075*np.cos(angles),.105*np.sin(angles),np.zeros(samples))),
             np.column_stack((.075*np.cos(angles),np.zeros(samples),.085*np.sin(angles))),
             np.column_stack((np.zeros(samples),.105*np.cos(angles),.085*np.sin(angles))))
    pairs=[np.stack((c,np.roll(c,-1,axis=0)),axis=1).reshape(-1,3) for c in circles]
    # A 3 cm in-head direction mark is shorter than the default 8 cm
    # head-to-camera offset and rotates with the same head pose.
    pairs.append(np.array([[0.,0.,0.],[0.,0.,.03]]))
    return np.concatenate(pairs)@rotation.T+center


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
        # Customer controls now live in a dedicated row above all three views.
        if self.bar.count():
            self.overlay.adjustSize()
            self.overlay.move(6,6)
            self.overlay.show()
            self.overlay.raise_()
        else:self.overlay.hide()

    def resizeEvent(self,event):
        super().resizeEvent(event)
        self.place_overlay()


class DepthTask(QThread):
    """与原有深度显示一致的计算：同一 StereoRectifier 与 StereoDepth。

    校正器与匹配器跨帧复用（构造一次要做两张全分辨率映射表），否则显示节奏会被
    重复初始化吃掉。只影响显示刷新率，不改变深度算法本身。
    """

    completed=Signal(int,object,str)

    def __init__(self,generation,frame,calibration,cache,parent=None):
        super().__init__(parent)
        self.generation=generation;self.frame=frame;self.calibration=calibration;self.cache=cache

    def run(self):
        try:
            import cv2
            started=time.perf_counter()
            depth,info=compute_depth(self.frame,self.calibration,self.cache)
            swapped=info["reference_camera"]=="right"
            color,valid=colorize_depth(depth,.15,3.,False)
            color[~valid]=0
            ms=(time.perf_counter()-started)*1000
            text=(f'深度·'+('右目' if swapped else '左目')+
                  f'·全部·{valid.mean():.0%}·{ms:.0f}ms')
            self.completed.emit(self.generation,color,text)
        except Exception as exc:
            self.completed.emit(self.generation,None,'深度暂不可用：'+str(exc))


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
