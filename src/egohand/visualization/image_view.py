import cv2
import numpy as np
from PySide6.QtCore import Qt,Signal,QPointF
from PySide6.QtGui import QImage,QPixmap,QPainter,QColor,QPen,QFont
from PySide6.QtCore import QRectF
from PySide6.QtWidgets import QLabel,QSizePolicy
from ..detection.landmarks import BONES
from ..detection.roi import validate_roi,roi_pixel_box

def pair_color(theme,track_id):
    """两侧同一已确认轨迹使用同色；颜色不代表生理左右手。"""
    palette=(theme['left_hand'],theme['right_hand'],'#88B8FF','#E6A6D8','#B2D982','#FF9E83','#C5AAFF','#7ED6DC')
    return palette[(track_id-1)%len(palette)]

class ImageView(QLabel):
    roi_changed=Signal(object)
    image_size_changed=Signal(int,int)
    roi_selection_finished=Signal()
    def __init__(self,title,parent=None):
        super().__init__(title,parent)
        self.title=title;self.pix=None;self.last_bgr=None
        self.roi=None;self.roi_selecting=False;self._roi_start=None;self._roi_end=None;self._image_size=None
        self.setFocusPolicy(Qt.StrongFocus)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(260,160)
        self.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored)
        self.theme={"canvas":"#0E141D","muted":"#A2B0C2","divider":"#273140","left_hand":"#5FE0C5","right_hand":"#F0BB76"}
        self.setText("")
    def set_theme(self,tokens):
        self.theme=tokens
        self.setStyleSheet(f"background:{tokens['canvas']};border:0;")
        self.update()
    def paintEvent(self,event):
        super().paintEvent(event)
        if self.pix is not None:
            self._paint_roi()
            return
        painter=QPainter(self);painter.setRenderHint(QPainter.Antialiasing)
        cx,cy=self.width()/2,self.height()/2
        painter.setPen(QPen(QColor(self.theme["divider"]),1))
        for x in range(24,self.width(),24):
            for y in range(24,self.height(),24):painter.drawPoint(x,y)
        painter.setPen(QPen(QColor(self.theme["muted"]),1.5))
        painter.drawRoundedRect(QRectF(cx-19,cy-42,38,28),5,5)
        painter.drawEllipse(QRectF(cx-7,cy-35,14,14))
        painter.setFont(QFont("Microsoft YaHei UI",10))
        painter.drawText(QRectF(0,cy,self.width(),24),Qt.AlignCenter,self.title)
        painter.setFont(QFont("Microsoft YaHei UI",9))
        painter.drawText(QRectF(0,cy+28,self.width(),22),Qt.AlignCenter,"选择数据源后点击开始")

    def image_rect(self):
        """实际等比图像区域；框选不能把 QLabel 两侧留白当作图像像素。"""
        if self.pix is None:return QRectF()
        area=self.contentsRect();size=self._display_pix().size().scaled(area.size(),Qt.KeepAspectRatio)
        return QRectF(area.x()+(area.width()-size.width())/2,area.y()+(area.height()-size.height())/2,size.width(),size.height())

    def set_roi(self,roi):
        self.roi=validate_roi(roi);self._scale();self.update()

    def set_roi_selection(self,enabled):
        self.roi_selecting=bool(enabled);self._roi_start=None;self._roi_end=None
        self.setCursor(Qt.CrossCursor if enabled else Qt.ArrowCursor)
        if enabled:self.setFocus()
        self._scale()
        self.update()

    def _roi_point(self,point,clamp=False):
        rect=self.image_rect()
        if rect.isEmpty() or (not clamp and not rect.contains(point)):return None
        return QPointF(min(1.,max(0.,(point.x()-rect.x())/rect.width())),min(1.,max(0.,(point.y()-rect.y())/rect.height())))

    def mousePressEvent(self,event):
        if self.roi_selecting and event.button()==Qt.RightButton:
            self.set_roi_selection(False);self.roi_selection_finished.emit();event.accept();return
        if self.roi_selecting and event.button()==Qt.LeftButton:
            self._roi_start=self._roi_point(event.position());self._roi_end=self._roi_start
            self.update();event.accept();return
        super().mousePressEvent(event)

    def mouseMoveEvent(self,event):
        if self.roi_selecting and self._roi_start is not None:
            self._roi_end=self._roi_point(event.position(),True);self.update();event.accept();return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self,event):
        if self.roi_selecting and event.button()==Qt.LeftButton and self._roi_start is not None:
            end=self._roi_point(event.position(),True);start=self._roi_start
            x,y=min(start.x(),end.x()),min(start.y(),end.y());w,h=abs(end.x()-start.x()),abs(end.y()-start.y())
            if w*self.pix.width()>=8 and h*self.pix.height()>=8:
                self.set_roi((x,y,w,h));self.roi_changed.emit(self.roi)
            self.set_roi_selection(False);self.roi_selection_finished.emit();event.accept();return
        super().mouseReleaseEvent(event)

    def keyPressEvent(self,event):
        if event.key()==Qt.Key_Escape and self.roi_selecting:
            self.set_roi_selection(False);self.roi_selection_finished.emit();event.accept();return
        super().keyPressEvent(event)

    def _paint_roi(self):
        if not self.roi_selecting:return
        roi=self.roi
        if self._roi_start is not None and self._roi_end is not None:
            a,b=self._roi_start,self._roi_end
            roi=(min(a.x(),b.x()),min(a.y(),b.y()),abs(a.x()-b.x()),abs(a.y()-b.y()))
        if roi is None:return
        rect=self.image_rect();x,y,w,h=roi
        box=QRectF(rect.x()+x*rect.width(),rect.y()+y*rect.height(),w*rect.width(),h*rect.height())
        painter=QPainter(self);painter.setClipRect(rect);painter.setRenderHint(QPainter.Antialiasing)
        shade=QColor(self.theme['left_hand']);shade.setAlpha(25);painter.fillRect(box,shade)
        painter.setPen(QPen(QColor(self.theme['left_hand']),2,Qt.DashLine));painter.drawRect(box.adjusted(1,1,-1,-1))
        painter.setFont(QFont('Microsoft YaHei UI',9));painter.drawText(box.adjusted(5,3,-3,-3),Qt.AlignTop|Qt.AlignLeft,'ROI')
        painter.end()
    def display(self,image,detections=(),hands=(),numbers=False,epilines=False,side="left",epi_axis=1,accepted_pairs=None):
        img=image.copy()
        if epilines:
            if epi_axis==1:
                for y in range(0,img.shape[0],40): cv2.line(img,(0,y),(img.shape[1]-1,y),(85,65,55),1)
            else:
                for x in range(0,img.shape[1],40): cv2.line(img,(x,0),(x,img.shape[0]-1),(85,65,55),1)
        for detection_index,d in enumerate(detections):
            uv=d.pixel_landmarks
            mask=d.valid & np.isfinite(uv).all(axis=1)
            matched=None
            if accepted_pairs is not None:
                for pair_index,pair in enumerate(accepted_pairs):
                    if pair[0 if side=='left' else 1]==detection_index and pair_index<len(hands):
                        h=hands[pair_index]
                        if h.valid_joint_count>0:matched=h
                        break
                qcolor=QColor(pair_color(self.theme,matched.track_id if matched.track_id>0 else pair_index+1) if matched else '#FFDA3D')
            else:
                qcolor=QColor(self.theme["left_hand"] if d.handedness=="Left" else self.theme["right_hand"] if d.handedness=="Right" else '#D4DAE3')
            col=(qcolor.blue(),qcolor.green(),qcolor.red())
            for a,b in BONES:
                if mask[a] and mask[b]: cv2.line(img,tuple(uv[a].astype(int)),tuple(uv[b].astype(int)),col,2,cv2.LINE_AA)
            for j in np.flatnonzero(mask):
                p=tuple(uv[j].astype(int));cv2.circle(img,p,3,col,-1,cv2.LINE_AA)
                if numbers: cv2.putText(img,str(j),p,cv2.FONT_HERSHEY_SIMPLEX,.35,(235,235,235),1,cv2.LINE_AA)
            anchor=uv[0] if mask[0] else (10,45)
            label=d.handedness
            if accepted_pairs is not None:
                label+=(f"  #{matched.track_id:02d} | {matched.valid_joint_count}/21" if matched.track_id>0 else f"  pair | {matched.valid_joint_count}/21") if matched else "  unmatched"
            for h in (() if accepted_pairs is not None else hands):
                point=h.joints[0].left_uv_px if side=="left" else h.joints[0].right_uv_px
                if point is not None and np.linalg.norm(np.array(point)-anchor)<2:
                    label+=f"  #{h.track_id:02d}  |  {h.valid_joint_count}/21" if h.track_id>=0 else "  unmatched"
                    # Red rings show geometry-rejected observations.
                    for j in h.joints:
                        p=j.left_uv_px if side=="left" else j.right_uv_px
                        if p is not None and not j.valid: cv2.circle(img,tuple(np.array(p).astype(int)),6,(50,50,245),1)
                    break
            if matched:
                for j in matched.joints:
                    p=j.left_uv_px if side=="left" else j.right_uv_px
                    if p is not None and not j.valid and np.isfinite(p).all():cv2.circle(img,tuple(np.array(p).astype(int)),6,(50,50,245),1)
            box=uv[mask].min(axis=0) if mask.any() else np.array([12,38])
            text_xy=(max(8,min(img.shape[1]-185,int(box[0]))),max(45,int(box[1])-12))
            cv2.putText(img,label,text_xy,cv2.FONT_HERSHEY_SIMPLEX,.43,col,1,cv2.LINE_AA)
        self.last_bgr=img
        rgb=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        self.pix=QPixmap.fromImage(QImage(rgb.data,rgb.shape[1],rgb.shape[0],rgb.strides[0],QImage.Format_RGB888).copy())
        size=(img.shape[1],img.shape[0])
        if size!=self._image_size:self._image_size=size;self.image_size_changed.emit(*size)
        self._scale()
    def _scale(self):
        if self.pix: self.setPixmap(self._display_pix().scaled(self.size(),Qt.KeepAspectRatio,Qt.SmoothTransformation))

    def _display_pix(self):
        # 只裁剪显示副本；原始图像、检测像素和三维坐标保持完整图坐标。
        if self.roi is None or self.roi_selecting:return self.pix
        x,y,w,h=roi_pixel_box(self.roi,(self.pix.height(),self.pix.width()))
        return self.pix.copy(x,y,w,h)
    def resizeEvent(self,event):
        self._scale();super().resizeEvent(event)
