"""Read-only frame diagnostics, independent of capture controls and GT inference."""
import json
import cv2
import numpy as np
from PySide6.QtCore import Qt,QTimer
from PySide6.QtWidgets import (QDialog,QVBoxLayout,QHBoxLayout,QLabel,QPushButton,
    QCheckBox,QTabWidget,QTableWidget,QTableWidgetItem,QHeaderView,QAbstractItemView,QPlainTextEdit,QWidget,QGridLayout)
from ..core.types import serializable
from ..detection.landmarks import JOINT_NAMES,BONES

GATE_REASONS={'INVALID':'1 · 二维点不可用 / 三角化无效','EPIPOLAR_REJECTED':'2 · 极线误差超限',
 'INVALID_DISPARITY':'3 · 视差过小','NEGATIVE_DEPTH':'4 · 左右深度不为正',
 'REPROJECTION_REJECTED':'5 · 重投影误差超限','DEPTH_RANGE_REJECTED':'6 · 深度超范围',
 'HIGH_UNCERTAINTY':'7 · 不确定性超限','BONE_LENGTH_REJECTED':'8 · 骨长超范围',
 'TEMPORAL_OUTLIER':'11 · 跨帧位移过大','IDENTITY_AMBIGUOUS':'10 · 轨迹身份歧义',
 'UNMATCHED':'9 · 左右手未匹配','STEREO_OBSERVED':'通过全部门控'}

def render_truth_comparison(result,frame):
    """One composite: left/right processed pixels, model GT, and MP offsets."""
    from ..benchmarks.metrics import match_2d
    truth=getattr(frame,'debug_ground_truth_uv',[{},{}]);panels=[]
    gt_mode=result.diagnostics.get('result_kind') in ('official_ground_truth','ground_truth_unavailable')
    for side,(base,ds) in enumerate(zip((frame.left_image,frame.right_image),(result.left_detections,result.right_detections))):
        image=base.copy();h,w=image.shape[:2];observed=[] if gt_mode else ds
        def inside(p):return np.isfinite(p).all() and 0<=p[0]<w and 0<=p[1]<h
        def points(uv,mask,col,radius):
            for p,m in zip(uv,mask):
                if m and inside(p):cv2.circle(image,tuple(np.rint(p).astype(int)),radius,col,1,cv2.LINE_AA)
        for uv,mask in truth[side].values():points(uv,mask,(255,220,30),5)
        for det in observed:points(det.pixel_landmarks,det.valid,(0,220,255),2)
        matches,_=match_2d(observed,truth[side]);errors=[]
        for index,hand_side in matches.items():
            uv,mask=truth[side][hand_side];det=observed[index]
            for p,q,m in zip(det.pixel_landmarks,uv,mask&det.valid):
                if m and inside(p) and inside(q):
                    cv2.line(image,tuple(np.rint(p).astype(int)),tuple(np.rint(q).astype(int)),(220,70,220),1,cv2.LINE_AA)
                    errors.append(float(np.linalg.norm(p-q)))
        label=('L' if side==0 else 'R')+' | '+('GT unavailable' if not truth[side] else 'Detector not run' if gt_mode else f'matched {len(matches)} | mean offset {np.mean(errors):.2f}px' if errors else 'No unambiguous detector / GT match')
        cv2.rectangle(image,(0,0),(w,30),(25,25,25),-1)
        cv2.putText(image,label,(8,21),cv2.FONT_HERSHEY_SIMPLEX,.47,(255,255,255),1,cv2.LINE_AA)
        panels.append(image)
    return np.vstack(panels)


def render_debug_views(result,frame):
    """Four images: actual 2D output and final accepted stereo observations."""
    r=serializable(result);d=r['diagnostics']
    native=getattr(frame,'debug_native_images',None)
    raw=d.get('debug_native_detections')
    use_native=native is not None and isinstance(raw,list) and len(raw)==2
    is_gt=d.get('result_kind') in ('official_ground_truth','ground_truth_unavailable')
    views=[r['left_detections'],r['right_detections']]
    output=[]
    def draw(img,uv,mask,color,rings=False):
        uv=np.asarray(uv,dtype=float);mask=np.asarray(mask,bool)&np.isfinite(uv).all(axis=1)
        mask&=(uv>=0).all(axis=1)&(uv<[img.shape[1],img.shape[0]]).all(axis=1)
        if not rings:
            for a,b in BONES:
                if mask[a] and mask[b]:cv2.line(img,tuple(uv[a].astype(int)),tuple(uv[b].astype(int)),color,2,cv2.LINE_AA)
        for j in np.flatnonzero(mask):cv2.circle(img,tuple(uv[j].astype(int)),5 if rings else 3,color,1 if rings else -1,cv2.LINE_AA)
        return int(mask.sum())
    for side,base in enumerate((frame.left_image,frame.right_image)):
        inputs=d.get('detector_input_by_view',[])
        side_native=use_native and not (side<len(inputs) and inputs[side]=='virtual_rectified_manual_roi')
        image=(native[side] if side_native else base).copy();count=0
        if not is_gt:
            for det in (raw[side] if side_native else views[side]):
                count+=draw(image,det['pixel_landmarks'],det['valid'],(0,220,255))
        if side_native:image=np.ascontiguousarray(np.rot90(image,3))
        text='Detector NOT RUN (GT mode)' if is_gt else f'2D points: {count} | '+('native upright' if side_native else 'processed pixels')
        cv2.putText(image,text,(10,23),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1,cv2.LINE_AA)
        output.append(image)
    for side,base in enumerate((frame.left_image,frame.right_image)):
        image=base.copy();count=0
        if not is_gt:
            # Gray = observed; red rings = common inputs from an unaccepted candidate.
            for det in views[side]:draw(image,det['pixel_landmarks'],det['valid'],(135,135,135))
            for candidate in d.get('association_quality',{}).get('candidates',[]):
                index=candidate['left_index'] if side==0 else candidate['right_index']
                if index>=len(views[side]):continue
                mask=np.zeros(21,bool)
                for j in candidate.get('joint_diagnostics',[]):mask[j['joint_id']]=j['left_available'] and j['right_available']
                for hand in r['hands']:
                    for j in hand['joints']:
                        p=j['left_uv_px'] if side==0 else j['right_uv_px']
                        if j['valid'] and p is not None:
                            point=views[side][index]['pixel_landmarks'][j['joint_id']]
                            if np.isfinite(np.asarray(point,dtype=float)).all() and np.linalg.norm(np.asarray(point)-p)<1:
                                mask[j['joint_id']]=False
                draw(image,views[side][index]['pixel_landmarks'],mask,(50,60,245),True)
            # Final output, including post-association/temporal/identity rejection.
            for hand in r['hands']:
                uv=[];mask=[]
                for j in hand['joints']:
                    p=j['left_uv_px'] if side==0 else j['right_uv_px']
                    uv.append(p if p is not None else [np.nan,np.nan]);mask.append(j['valid'])
                count+=draw(image,uv,mask,(80,230,80))
        cv2.putText(image,'GT mode: acceptance N/A' if is_gt else f'Final accepted: {count}',
                    (10,23),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1,cv2.LINE_AA)
        output.append(image)
    return output


def debug_snapshot(result):
    r=serializable(result);d=r['diagnostics'];views=[r['left_detections'],r['right_detections']]
    raw=d.get('debug_native_detections',d.get('native_mediapipe_detections'))
    # Benchmark versions may store native detections as a two-view list.
    if not isinstance(raw,list) or len(raw)!=2:raw=None
    detection_rows=[]
    for side,ds in enumerate(views):
        for i,det in enumerate(ds):
            original=raw[side][i] if raw is not None and i<len(raw[side]) else None
            for j,name in enumerate(JOINT_NAMES):
                detection_rows.append([('左','右')[side],i,det['handedness'],det.get('handedness_score'),name,
                    original['pixel_landmarks'][j] if original else None,det['pixel_landmarks'][j],
                    bool(det['valid'][j]),det.get('origin','unknown')])
    association=d.get('association_quality',{});pairs=[tuple(p) for p in association.get('accepted_pairs',[])]
    joint_rows=[];common=passed=final=0
    for c in association.get('candidates',[]):
        pair=(c['left_index'],c['right_index']);selected=pair in pairs
        hand=r['hands'][pairs.index(pair)] if selected and pairs.index(pair)<len(r['hands']) else None
        for item in c.get('joint_diagnostics',[]):
            j=item['joint_id'];both=item['left_available'] and item['right_available']
            output=hand['joints'][j] if hand else None
            accepted=bool(output and output['valid'])
            common+=both;passed+=item['geometry_valid'];final+=accepted
            reason=output['status'] if output and not accepted else item['status'] if not item['geometry_valid'] else '未选中' if not selected else 'STEREO_OBSERVED'
            explanation=GATE_REASONS.get(str(reason),'9 · 候选未选中 / 匹配歧义' if reason=='未选中' else str(reason))
            joint_rows.append([f'L{pair[0]} ↔ R{pair[1]}',JOINT_NAMES[j],item['left_available'],item['right_available'],both,
                item['geometry_valid'],item['status'],item.get('epipolar_error_px'),item.get('reprojection_error_px'),
                accepted,output['status'] if output else ('身份歧义 / 未选中' if association.get('ambiguous') else '候选未选中'),explanation])
    if not association.get('candidates'):
        for side,ds in enumerate(views):
            for i,det in enumerate(ds):
                for j,name in enumerate(JOINT_NAMES):
                    joint_rows.append([('L','R')[side]+str(i),name,bool(det['valid'][j]) if side==0 else False,bool(det['valid'][j]) if side==1 else False,False,False,'UNMATCHED',None,None,False,'UNMATCHED','0 · 缺少另一侧检测，未形成候选'])
    gt=d.get('result_kind') in ('official_ground_truth','ground_truth_unavailable')
    counts=f'输出手数 L {len(views[0])} / R {len(views[1])}'
    if raw is not None:
        modes=d.get('detector_input_by_view',[])
        detected=[len(views[i]) if i<len(modes) and modes[i]=='virtual_rectified_manual_roi' else len(raw[i]) for i in (0,1)]
        counts=f'模型检出 L {detected[0]} / R {detected[1]}；'+counts
    origin='官方真值模式：本帧未运行检测模型；不计入算法接受率。' if gt else (
        '合成投影 / 非模型推理。' if d.get('synthetic') else '输入来源：'+str(d.get('detection_origin',d.get('left_detection_origin','unknown'))))
    summary=f'Frame {r["frame_id"]} · {counts}\n{origin}\n'
    summary+=f'候选对 {len(association.get("candidates",[]))}；选中对 {len(pairs)}；共同输入 {common}；几何通过 {passed}；最终接受 {final}（候选关节观测计数，非唯一GT关节率）'
    if not association.get('candidates'):summary+='\n无逐关节候选证据：可能为单侧漏检、真值模式、几何未启用或旧记录未保存；不把未知填成接受。'
    return summary,detection_rows,joint_rows,r


def display_value(value):
    if value is None:return '—'
    if isinstance(value,bool):return '是' if value else '否'
    if isinstance(value,(float,int)):return f'{value:.4f}' if isinstance(value,float) else str(value)
    if isinstance(value,list):return ', '.join('—' if v is None else f'{v:.2f}' for v in value)
    return str(value)


class DebugPage(QDialog):
    def __init__(self,owner):
        super().__init__(owner);self.owner=owner;self.last_result=None
        self.setWindowTitle('双目调试 · 检测、接受与真值对比')
        # 使用普通可缩放窗口，并提供最大化按钮与右下角尺寸拖拽柄。
        self.setWindowFlags(Qt.Window | Qt.WindowSystemMenuHint | Qt.WindowMinMaxButtonsHint | Qt.WindowCloseButtonHint)
        self.setMinimumSize(720,420);self.setSizeGripEnabled(True);self.resize(1280,820)
        layout=QVBoxLayout(self);layout.setContentsMargins(8,8,8,8);layout.setSpacing(4)
        bar=QHBoxLayout();self.freeze=QCheckBox('冻结调试帧（不暂停采集）')
        refresh=QPushButton('刷新当前帧');refresh.clicked.connect(lambda:self.refresh(force=True))
        copy=QPushButton('复制本帧 JSON');copy.clicked.connect(self.copy_json)
        fit=QPushButton('自适应列宽');fit.setToolTip('按当前表头和单元格内容调整列宽；也可以拖动表头边界。')
        fit.clicked.connect(self.fit_columns)
        bar.addWidget(self.freeze);bar.addStretch();bar.addWidget(fit);bar.addWidget(refresh);bar.addWidget(copy);layout.addLayout(bar)
        self.summary=QLabel('尚无结果，请先启动采集或回放。');self.summary.setWordWrap(True);layout.addWidget(self.summary)
        tabs=QTabWidget();layout.addWidget(tabs,1)
        self.detections=self.table(['视图','检测序号','手侧标签','手侧分数¹','关节','原生 UV px','处理 UV px','映射可用²','来源'])
        self.joints=self.table(['候选对','关节','左可用','右可用','共同输入','几何通过','几何状态','极线 px','重投影 px','最终接受','最终状态','卡住的步骤'])
        tabs.addTab(self.detections,'MediaPipe / 二维输出');tabs.addTab(self.joints,'共同关节 / 接受原因')
        self.json=QPlainTextEdit();self.json.setReadOnly(True);tabs.addTab(self.json,'完整帧 JSON')
        note=QLabel('门控流程：输入与标定 → ①二维有效 → ②极线 → ③视差 → ④三角化/正深度 → ⑤重投影 → ⑥深度范围 → ⑦不确定性（默认仅诊断）→ ⑧骨长 → ⑨左右匹配 → ⑩身份 → ⑪时序。显示最先记录的拒绝原因；关闭的门控跳过。');note.setWordWrap(True);layout.addWidget(note)
        self.timer=QTimer(self);self.timer.timeout.connect(self.refresh);self.timer.start(250)

    def table(self,headers):
        table=QTableWidget(0,len(headers));table.setHorizontalHeaderLabels(headers)
        table.setEditTriggers(QAbstractItemView.NoEditTriggers);table.setAlternatingRowColors(True)
        table.setSelectionBehavior(QAbstractItemView.SelectRows);table.verticalHeader().hide()
        header=table.horizontalHeader();header.setSectionResizeMode(QHeaderView.Interactive)
        header.setStretchLastSection(False)
        table._auto_resizing=False;table._columns_initialized=False;table._user_sized_columns=set()
        def remember_user_width(index,old_size,new_size):
            if not table._auto_resizing:table._user_sized_columns.add(index)
        header.sectionResized.connect(remember_user_width)
        return table

    def _fit_table_columns(self,table,reset=False):
        # 测量表头和当前全部单元格；自动宽度只增不减，避免实时刷新闪动。
        # 用户拖动过的列不再被实时刷新覆盖；手动点“自适应列宽”可恢复自动管理。
        if reset:table._user_sized_columns.clear()
        header=table.horizontalHeader();metrics=table.fontMetrics();header_metrics=header.fontMetrics()
        table._auto_resizing=True
        try:
            for column in range(table.columnCount()):
                if column in table._user_sized_columns:continue
                title=table.horizontalHeaderItem(column)
                width=header_metrics.horizontalAdvance(title.text() if title else '')+28
                for row in range(table.rowCount()):
                    item=table.item(row,column)
                    if item:width=max(width,metrics.horizontalAdvance(item.text())+24)
                if table._columns_initialized and not reset:width=max(width,table.columnWidth(column))
                table.setColumnWidth(column,width)
            table._columns_initialized=True
        finally:table._auto_resizing=False

    def fit_columns(self):
        for table in (self.detections,self.joints):self._fit_table_columns(table,reset=True)

    def refresh(self,force=False):
        if not self.isVisible():return
        name=getattr(self.owner,'theme_name','dark')
        if getattr(self,'_theme',None)!=name:
            from .theme import THEMES,stylesheet
            t=THEMES[name];self._theme=name
            self.setStyleSheet(stylesheet(name)+f'''
                QDialog {{ background:{t['background']}; }}
                QTableWidget,QPlainTextEdit {{ background:{t['surface']}; alternate-background-color:{t['elevated']}; color:{t['text']}; gridline-color:{t['divider']}; border:1px solid {t['border']}; selection-background-color:{t['accent_soft']}; selection-color:{t['text']}; }}
                QHeaderView::section {{ background:{t['elevated']}; color:{t['text']}; padding:6px; border:0; }}
            ''')
        if self.freeze.isChecked() and not force:return
        result=self.owner.latest
        if result is self.last_result and not force:return
        self.last_result=result
        if result is None:
            self.summary.setText('尚无结果，请先启动采集或回放。');self.detections.setRowCount(0);self.joints.setRowCount(0);self.json.clear()
            return
        summary,ds,js,record=debug_snapshot(result)
        self.summary.setToolTip(summary)
        self.summary.setText(summary.split('\n')[0]+f'  |  最终有效关节 {sum(h.valid_joint_count for h in result.hands)}\n'+summary.split('\n')[1])
        for table,rows in [(self.detections,ds),(self.joints,js)]:
            table.setUpdatesEnabled(False);table.setRowCount(len(rows))
            for i,row in enumerate(rows):
                for j,value in enumerate(row):
                    cell=QTableWidgetItem(display_value(value));cell.setToolTip(cell.text());table.setItem(i,j,cell)
            self._fit_table_columns(table)
            table.setUpdatesEnabled(True)
        self.json.setPlainText(json.dumps(record,ensure_ascii=False,indent=2,allow_nan=False))

    def copy_json(self):
        from PySide6.QtWidgets import QApplication
        QApplication.clipboard().setText(self.json.toPlainText())


def install_debug_page(window):
    from PySide6.QtGui import QAction
    window.debug_page=DebugPage(window)
    action=QAction('MediaPipe / 共同关节调试…',window);action.setShortcut('Ctrl+D')
    def show():window.debug_page.show();window.debug_page.raise_();window.debug_page.refresh(force=True)
    action.triggered.connect(show);window.menuBar().addMenu('调试').addAction(action)
