"""Interactive, non-destructive trimming of VS030 SDK recordings."""
from pathlib import Path
import time
import cv2
import numpy as np
from PySide6.QtCore import Qt,QThread,Signal,QTimer
from PySide6.QtGui import QImage,QPixmap
from PySide6.QtWidgets import (QDialog,QVBoxLayout,QHBoxLayout,QLabel,QSlider,
                               QPushButton,QFileDialog,QMessageBox)
from egohand.storage.sdk_recording import iter_paired_frames,iter_index_rows
from egohand.storage.recording_clip import save_sdk_clip_time_range
from egohand.gui.theme import THEMES,stylesheet


class IndexWorker(QThread):
    ready=Signal(object)
    failed=Signal(str)
    def __init__(self,path,parent=None):
        super().__init__(parent);self.path=path
    def run(self):
        try:
            pending={};times=[]
            for row in iter_index_rows(self.path):
                if self.isInterruptionRequested():return
                sink=row.get('sink')
                if sink not in (4,5):continue
                sequence=row.get('sequence');entry=pending.setdefault(sequence,{})
                entry[sink]=int(row.get('device_timestamp_raw',0))
                if len(entry)==2:
                    if entry[4]==entry[5] and entry[4]>0:times.append(entry[4])
                    pending.pop(sequence)
            if not times:raise ValueError('没有可用的双目索引')
            if any(b<=a for a,b in zip(times,times[1:])):raise ValueError('双目索引时间戳不递增')
            self.ready.emit(times)
        except Exception as exc:self.failed.emit(str(exc))


class PreviewWorker(QThread):
    frame=Signal(object,object,bytes)
    failed=Signal(str)
    def __init__(self,path,target,parent=None):
        super().__init__(parent);self.path=path;self.target=int(target)
    def run(self):
        try:
            # HEVC requires decoder references from earlier packets. Decode in
            # the background, converting only frames near the requested time.
            best=None
            for frame in iter_paired_frames(self.path,min_device_ts=max(0,self.target-250_000),
                                            cancelled=self.isInterruptionRequested):
                if self.isInterruptionRequested():return
                if best is None or abs(frame.device_ts-self.target)<abs(best.device_ts-self.target):best=frame
                if frame.device_ts>=self.target:break
            if self.isInterruptionRequested():return
            if best is None:raise ValueError('该位置没有可解码的双目画面')
            left=cv2.resize(best.left_bgr,(320,240),interpolation=cv2.INTER_AREA)
            right=cv2.resize(best.right_bgr,(320,240),interpolation=cv2.INTER_AREA)
            ok,jpeg=cv2.imencode('.jpg',np.hstack((left,right)),[cv2.IMWRITE_JPEG_QUALITY,75])
            if not ok:raise ValueError('预览图编码失败')
            self.frame.emit(best.index,best.device_ts,jpeg.tobytes())
        except Exception as exc:self.failed.emit(str(exc))


class SaveWorker(QThread):
    completed=Signal(str)
    failed=Signal(str)
    def __init__(self,source,dest,start,end,parent=None):
        super().__init__(parent);self.args=(source,dest,start,end)
    def run(self):
        try:self.completed.emit(str(save_sdk_clip_time_range(*self.args)))
        except Exception as exc:self.failed.emit(str(exc))


class RecordingTrimDialog(QDialog):
    def __init__(self,source,parent=None):
        super().__init__(parent);self.source=Path(source);self.times=[];self.save_worker=None
        self.preview=None;self._pending_preview=None;self._active_preview=None;self._selection={}
        self._preview_cache={};self._closing=False
        self.setObjectName('recordingTrim')
        theme=THEMES['dark']
        self.setStyleSheet(stylesheet('dark')+'''
            QDialog#recordingTrim { background: %(background)s; }
            QLabel#trimPreview { background: %(canvas)s; color: %(muted)s; border: 1px solid %(divider)s; border-radius: 6px; }
            QLabel#trimInfo { color: %(muted)s; }
            QSlider::groove:horizontal { height: 4px; background: %(border)s; border-radius: 2px; }
            QSlider::sub-page:horizontal { background: %(accent)s; border-radius: 2px; }
            QSlider::handle:horizontal { background: %(text)s; border: 1px solid %(border)s; width: 14px; margin: -6px 0; border-radius: 7px; }
            QSlider::handle:horizontal:disabled { background: %(faint)s; }
            QPushButton { min-height: 28px; padding: 0 12px; font-weight: 400; border-radius: 4px; }
        ''' % theme)
        self.setWindowTitle('截取录制');self.resize(760,430)
        layout=QVBoxLayout(self)
        self.image=QLabel('正在读取双目索引，请稍候');self.image.setAlignment(Qt.AlignCenter)
        self.image.setObjectName('trimPreview')
        self.image.setMinimumHeight(210);layout.addWidget(self.image,1)
        self.info=QLabel('正在读取录制');layout.addWidget(self.info)
        self.info.setObjectName('trimInfo')
        self.start_slider=QSlider(Qt.Horizontal);self.start_slider.setAccessibleName('截取起点')
        self.end_slider=QSlider(Qt.Horizontal);self.end_slider.setAccessibleName('截取终点')
        for label,slider in (('起点',self.start_slider),('终点',self.end_slider)):
            row=QHBoxLayout();row.addWidget(QLabel(label));row.addWidget(slider,1);layout.addLayout(row)
            slider.setEnabled(False)
        self.start_slider.valueChanged.connect(lambda v:self._change(True,v))
        self.end_slider.valueChanged.connect(lambda v:self._change(False,v))
        buttons=QHBoxLayout();buttons.addStretch();self.save=QPushButton('保存截取');self.save.setEnabled(False)
        self.save.setProperty('role','primary')
        self.save.clicked.connect(self._save);buttons.addWidget(self.save)
        cancel=QPushButton('取消');cancel.clicked.connect(self.reject);buttons.addWidget(cancel);layout.addLayout(buttons)
        self.seek_timer=QTimer(self);self.seek_timer.setSingleShot(True);self.seek_timer.setInterval(100)
        self.seek_timer.timeout.connect(self._start_preview)
        self.index_worker=IndexWorker(self.source,self);self.index_worker.ready.connect(self._ready)
        self.index_worker.failed.connect(self._failed);self.index_worker.start()

    def _ready(self,times):
        self.times=times;maximum=len(times)-1
        for slider in (self.start_slider,self.end_slider):slider.setRange(0,maximum);slider.setEnabled(True)
        self.end_slider.blockSignals(True);self.end_slider.setValue(maximum);self.end_slider.blockSignals(False)
        self._selection={'start':times[0],'end':times[-1]}
        self._change(True,0)

    def _change(self,is_start,value):
        if not self.times:return
        if self.start_slider.value()>self.end_slider.value():
            other=self.end_slider if is_start else self.start_slider
            other.blockSignals(True);other.setValue(value);other.blockSignals(False)
            self._selection['end' if is_start else 'start']=self.times[value]
        a=self.start_slider.value();b=self.end_slider.value()
        key='start' if is_start else 'end';self._selection[key]=self.times[value]
        self._pending_preview=(key,value,self.times[value])
        self.save.setEnabled(False)
        self.info.setText(f'正在加载{"起点" if is_start else "终点"}预览 · 选中约 {(self.times[b]-self.times[a])/1e6:.2f} 秒')
        if self.preview is not None and self.preview.isRunning():self.preview.requestInterruption()
        self.seek_timer.start()

    def _start_preview(self):
        if self._closing or self._pending_preview is None:return
        if self.preview is not None and self.preview.isRunning():return
        request=self._pending_preview;self._pending_preview=None;self._active_preview=request
        cached=self._preview_cache.get(request[2])
        if cached is not None:
            self._show_preview(*cached);return
        self.preview=PreviewWorker(self.source,request[2],self)
        self.preview.frame.connect(self._show_preview);self.preview.failed.connect(self._preview_failed)
        self.preview.finished.connect(self._preview_finished);self.preview.start()

    def _show_preview(self,index,timestamp,jpeg):
        request=self._active_preview
        if request is None or self._pending_preview is not None:return
        key,slider_value,target=request
        slider=self.start_slider if key=='start' else self.end_slider
        if slider.value()!=slider_value:return
        self._preview_cache[target]=(index,timestamp,jpeg)
        if len(self._preview_cache)>8:self._preview_cache.pop(next(iter(self._preview_cache)))
        self._selection[key]=int(timestamp)
        image=QImage.fromData(jpeg,'JPG')
        self.image.setPixmap(QPixmap.fromImage(image).scaled(self.image.size(),Qt.KeepAspectRatio,Qt.SmoothTransformation))
        self.info.setText(f'{"起点" if key=="start" else "终点"}预览 · 原始第 {index+1} 帧 · 选中约 {(self._selection["end"]-self._selection["start"])/1e6:.2f} 秒')
        self.save.setEnabled(self._selection['start']<=self._selection['end'])

    def _preview_finished(self):
        if not self._closing and self._pending_preview is not None:self.seek_timer.start(0)

    def _preview_failed(self,text):
        if self._pending_preview is None:self._failed(text)

    def _save(self):
        parent=QFileDialog.getExistingDirectory(self,'选择截取录制的保存位置',str(self.source.parent))
        if not parent:return
        destination=Path(parent)/(self.source.name+'_clip_'+str(time.time_ns()))
        self.save.setEnabled(False);self.start_slider.setEnabled(False);self.end_slider.setEnabled(False)
        self.info.setText('正在保存并核验双目帧，请稍候')
        self.save_worker=SaveWorker(self.source,destination,self._selection['start'],self._selection['end'],self)
        self.save_worker.completed.connect(self._saved);self.save_worker.failed.connect(self._save_failed);self.save_worker.start()

    def _saved(self,path):
        self.save_worker.wait();self.saved_path=Path(path);QMessageBox.information(self,'截取完成','已保存：'+path);self.accept()

    def _save_failed(self,text):
        self._failed(text);self.save.setEnabled(True);self.start_slider.setEnabled(True);self.end_slider.setEnabled(True)

    def _failed(self,text):
        self.info.setText('失败：'+text)

    def reject(self):
        if self.save_worker is not None and self.save_worker.isRunning():
            self.info.setText('正在保存，请等待完成');return
        self._stop_preview();super().reject()

    def closeEvent(self,event):
        if self.save_worker is not None and self.save_worker.isRunning():event.ignore();return
        self._stop_preview();super().closeEvent(event)

    def _stop_preview(self):
        self._closing=True;self.seek_timer.stop()
        self.index_worker.requestInterruption();self.index_worker.wait()
        if self.preview is not None:self.preview.requestInterruption();self.preview.wait()
