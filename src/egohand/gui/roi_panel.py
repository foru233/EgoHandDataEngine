"""左右独立手动 ROI；图上框选与数值输入共用同一归一化坐标契约。"""
from pathlib import Path
import yaml
from PySide6.QtCore import Signal
from PySide6.QtGui import QAction,QKeySequence
from PySide6.QtWidgets import QGroupBox,QVBoxLayout,QHBoxLayout,QLabel,QPushButton,QComboBox,QSpinBox,QGridLayout
from ..detection.roi import validate_roi,roi_pixel_box


class RoiPanel(QGroupBox):
    changed=Signal(dict)
    selection_requested=Signal(str)
    def __init__(self,config_path=None,parent=None):
        super().__init__('手动检测 ROI',parent)
        self.config_path=Path(config_path) if config_path else None
        self.rois={'left':None,'right':None};self.sizes={};self._syncing=False
        layout=QVBoxLayout(self);layout.setContentsMargins(0,10,0,0);layout.setSpacing(6)
        row=QHBoxLayout();self.side=QComboBox();self.side.addItems(['左图','右图']);self.side.setAccessibleName('ROI相机选择')
        row.addWidget(self.side)
        self.draw=QPushButton('在图上框选');self.draw.setCheckable(True);self.draw.setMinimumHeight(34)
        self.draw.setToolTip('建议先暂停在目标帧；拖动重新框选，Esc或右键取消；数值也可调整。')
        row.addWidget(self.draw);layout.addLayout(row)
        grid=QGridLayout();self.fields=[]
        for i,name in enumerate(('X','Y','宽','高')):
            spin=QSpinBox();spin.setRange(0,16384);spin.setAccessibleName('ROI '+name+' 像素')
            grid.addWidget(QLabel(name),i//2,(i%2)*2);grid.addWidget(spin,i//2,(i%2)*2+1);self.fields.append(spin)
        layout.addLayout(grid)
        row=QHBoxLayout();self.apply=QPushButton('应用坐标');self.clear=QPushButton('当前全图')
        for b in (self.apply,self.clear):b.setMinimumHeight(34);row.addWidget(b)
        layout.addLayout(row)
        row=QHBoxLayout();self.clear_all=QPushButton('清除两侧');self.save=QPushButton('保存配置')
        for b in (self.clear_all,self.save):b.setMinimumHeight(34);row.addWidget(b)
        layout.addLayout(row)
        self.summary=QLabel();self.summary.setWordWrap(True);layout.addWidget(self.summary)
        self.hint=QLabel('按显示图像选区，坐标单位px；GT与已有结果不重算。')
        self.hint.setWordWrap(True);self.hint.setProperty('role','hint');layout.addWidget(self.hint)
        self.side.currentIndexChanged.connect(self._sync_fields)
        self.side.currentIndexChanged.connect(lambda *_:self.selection_requested.emit(self.current_side()) if self.draw.isChecked() else None)
        self.draw.clicked.connect(lambda enabled:self.selection_requested.emit(self.current_side() if enabled else ''))
        self.apply.clicked.connect(self.apply_coordinates)
        self.clear.clicked.connect(lambda:self.set_roi(self.current_side(),None))
        self.clear_all.clicked.connect(self.reset)
        self.save.clicked.connect(self.save_settings)
        if self.config_path and self.config_path.exists():
            try:
                data=yaml.safe_load(self.config_path.read_text(encoding='utf8')) or {}
                if not isinstance(data,dict):raise ValueError('ROI配置应包含left/right两个键')
                self.rois={s:validate_roi(data.get(s)) for s in self.rois}
            except (ValueError,OSError,yaml.YAMLError) as e:self.hint.setText('ROI配置未加载：'+str(e))
        self._sync_fields()

    def current_side(self):return ('left','right')[self.side.currentIndex()]
    def settings(self):return dict(self.rois)
    def set_image_size(self,side,width,height):
        self.sizes[side]=(width,height)
        if side==self.current_side():self._sync_fields()

    def _sync_fields(self,*_):
        side=self.current_side();size=self.sizes.get(side)
        self.apply.setEnabled(size is not None)
        self.draw.setEnabled(size is not None)
        if size:
            width,height=size;box=roi_pixel_box(self.rois[side],(height,width))
            for field,maximum,value in zip(self.fields,(width-1,height-1,width,height),box):field.setRange(0,maximum);field.setValue(value)
        self.summary.setText('左图：'+('ROI已指定' if self.rois['left'] else '全图')+'  |  右图：'+('ROI已指定' if self.rois['right'] else '全图'))

    def set_roi(self,side,roi,emit=True):
        self.rois[side]=validate_roi(roi)
        self.side.setCurrentIndex(0 if side=='left' else 1);self._sync_fields()
        if emit:self.changed.emit(self.settings())

    def apply_coordinates(self):
        side=self.current_side()
        if side not in self.sizes:self.hint.setText('请先显示图像再输入像素坐标。');return
        width,height=self.sizes[side];x,y,w,h=(f.value() for f in self.fields)
        if w<8 or h<8 or x+w>width or y+h>height:
            self.hint.setText('ROI需在图像内，宽高至少8px；请检查坐标。');return
        self.set_roi(side,(x/width,y/height,w/width,h/height))
        self.hint.setText('ROI已提交；输出关键点仍使用完整图像坐标。')

    def reset(self):
        self.rois={'left':None,'right':None};self._sync_fields();self.changed.emit(self.settings())

    def save_settings(self):
        if not self.config_path:return
        try:
            self.config_path.parent.mkdir(parents=True,exist_ok=True)
            data={s:list(r) if r else None for s,r in self.rois.items()}
            self.config_path.write_text('# 完整显示图的归一化 [x,y,w,h]；null表示全图。\n'+yaml.safe_dump(data,allow_unicode=True),encoding='utf8')
            self.hint.setText('已保存ROI；下次启动自动载入。')
        except OSError as e:self.hint.setText('保存失败：'+str(e))


def install_roi_controls(w,detection_layout,config_path):
    panel=RoiPanel(config_path);w.roi_panel=panel;detection_layout.addWidget(panel)
    views={'left':w.left_view,'right':w.right_view}
    def select(side):
        for name,view in views.items():view.set_roi_selection(name==side)
        if side:w.statusBar().showMessage('请在'+('左' if side=='left' else '右')+'图拖动框选ROI；Esc取消。')
    def apply(settings):
        select('');panel.draw.setChecked(False)
        for side,view in views.items():view.set_roi(settings[side])
        worker=getattr(w,'worker',None)
        if worker and worker.isRunning():
            worker.command('roi_config',settings);panel.hint.setText('ROI已提交，后台正在应用。')
        elif worker and getattr(w,'latest',None) and w.latest.source_type=='real_single_image':
            w.start_pipeline()
        else:panel.hint.setText('ROI已设置，下次模型推理生效；已有结果不重算。')
    for side,view in views.items():
        view.roi_changed.connect(lambda roi,side=side:panel.set_roi(side,roi))
        view.image_size_changed.connect(lambda width,height,side=side:panel.set_image_size(side,width,height))
        view.roi_selection_finished.connect(lambda:panel.draw.setChecked(False))
        view.set_roi(panel.rois[side])
    panel.selection_requested.connect(select);panel.changed.connect(apply)
    escape=QAction(w);escape.setShortcut(QKeySequence('Esc'))
    def cancel():select('');panel.draw.setChecked(False)
    escape.triggered.connect(cancel);w.addAction(escape)
    def source_changed(*_):
        select('');panel.draw.setChecked(False)
        panel.hint.setText('数据源已切换；ROI按图像比例保留，请确认适用范围或清除。')
    w.source_switched.connect(source_changed)
