"""Isolated native GUI extension: choose legacy or new offline joint algorithm."""
from pathlib import Path
import argparse,json,os,sqlite3,subprocess,sys,time
from joint_work_paths import JOINT_WORK, PROJECT, SCRIPT_DIR
sys.dont_write_bytecode=True;sys.path[:0]=[str(PROJECT/'src'),str(PROJECT/'userrr')]
from PySide6.QtCore import Qt,QTimer,QCoreApplication,QEvent
from PySide6.QtWidgets import QApplication,QComboBox,QStyle,QStyleOptionSlider
from client_gui import ClientMainWindow as OriginalWindow
import client_gui
from shared_replay_cache import SharedSavedResultReplay
from recorded_trajectory import RecordedTrajectory, recorded_message
from joint_preview_views import show_preview
from indexed_joint_pose import IndexedResults, load_joint_results
from egohand.vio.local_segments import display_reference_key
client_gui.SavedResultReplay=SharedSavedResultReplay
client_gui.load_recorded_results=load_joint_results
DEFAULT_RESULT_PARENT=PROJECT/'data'
REGISTRY=JOINT_WORK/'gui_algorithm_registry.json'

def configure_view(window):
    indexed=isinstance(window.saved,IndexedResults)
    candidates=[i for i,r in enumerate(window.saved)
        if (window.saved.valid_body[i] if indexed else (r.diagnostics.get('offline_derived_pose') or {}).get('body'))
        and sum(j.valid and str(j.status)=='observed_stereo' for h in r.hands for j in h.joints)>=8]
    if candidates:
        window.result_slider.setValue(candidates[0])
        window.pivot_choice.setCurrentIndex(window.pivot_choice.findData('body'))
        window.preview.setCameraPosition(distance=3.2)

class JointClientWindow(OriginalWindow):
    def __init__(self,*args,**kwargs):
        self._joint_job=None;self._joint_output=None;self._joint_log=None;self._joint_loaded=False;self._pending_old=None
        super().__init__(*args,**kwargs)
        self._trajectory_cache=RecordedTrajectory(self.vio_panel)
        self.vio_panel.show_recorded=self._trajectory_cache.show
        self._preserve_replay_view=False
        self._slider_handle_press=False
        self.result_slider.installEventFilter(self)
        self.setWindowTitle('EgoHand · 离线算法对比（旧 / ACE联合）')
        self.result_output_parent=DEFAULT_RESULT_PARENT
        self._output_note('结果位置：'+str(self.result_output_parent)+'（每次计算创建新文件夹）')
        row=self.compute_controls.layout()
        algorithm_index=row.indexOf(self.compute_button)
        self.reconstruction_algorithm=QComboBox();self.reconstruction_algorithm.setAccessibleName('离线重建算法')
        self.reconstruction_algorithm.addItem('旧算法 · 双目 + MANO','legacy')
        self.reconstruction_algorithm.addItem('新算法 · ACE联合','joint_smpl_mano_v2')
        self.reconstruction_algorithm.setItemData(1,'实验算法：ACE、双目几何与人体手部联合拟合',Qt.ToolTipRole)
        self.reconstruction_algorithm.setToolTip('选择影响离线计算与“查看所选结果”；已有结果按其实际算法显示。')
        self.reconstruction_algorithm.setCurrentIndex(1);self.reconstruction_algorithm.setFixedHeight(32)
        row.insertWidget(algorithm_index,self.reconstruction_algorithm)
        self.view_algorithm_button=self._button('查看所选结果',self.open_selected_algorithm)
        self.view_algorithm_button.setFixedHeight(32)
        row.insertWidget(row.indexOf(self.camera_mount_controls),self.view_algorithm_button)
        self.algorithm_note=client_gui.CompactLabel('离线处理 · 人体未观测部分由先验推断')
        self.algorithm_note.setObjectName('compactMeta')
        self.header_layout.insertWidget(self.header_layout.indexOf(self.state_label),self.algorithm_note,1)
        self._layout_toolbar()
        self._joint_timer=QTimer(self);self._joint_timer.timeout.connect(self._poll_joint_job);self._joint_timer.start(500)
        self.sync_controls()

    def _registry(self):
        try:return json.loads(REGISTRY.read_text(encoding='utf8'))
        except (OSError,ValueError):return {}

    def _register(self,algorithm,path,recording):
        REGISTRY.parent.mkdir(parents=True,exist_ok=True)
        registry=self._registry();key=str(Path(recording).resolve()).casefold()
        registry.setdefault(key,{})[algorithm]=str(path)
        temporary=REGISTRY.with_suffix('.tmp');temporary.write_text(json.dumps(registry,ensure_ascii=False,indent=2),encoding='utf8');temporary.replace(REGISTRY)

    def open_selected_algorithm(self):
        if self._joint_job is not None:return
        recording=self.recording_source
        if not recording:
            self.show_error('请先打开录制目录，或使用“查看计算结果”直接打开结果。');return
        path=self._registry().get(str(Path(recording).resolve()).casefold(),{}).get(self.reconstruction_algorithm.currentData())
        if not path or not Path(path).exists():
            self.show_error('所选算法尚无完成结果，请点击“计算”。');return
        self.open_results_path(Path(path));configure_view(self)

    def open_results_path(self,path):
        try:
            m=json.loads((Path(path)/'unified_smpl_manifest.json').read_text(encoding='utf8'))
            self._joint_loaded=m.get('algorithm')=='joint_smpl_mano_v2'
        except (OSError,ValueError):self._joint_loaded=False
        self._ground_override=False
        super().open_results_path(path)
        if hasattr(self,'reconstruction_algorithm'):
            self.algorithm_note.setText('当前结果：人体手部联合 · 无显示前腕部平移' if self._joint_loaded else '当前结果：旧算法 · 双目骨架与 MANO')
        self.sync_controls()

    def _request_smpl(self,result):
        if self._joint_loaded:
            self._clear_smpl('联合人体 · 本帧不可用');self.hand_overlay.update_mano_meshes([]);return
        return super()._request_smpl(result)

    def _saved_seek_finished(self):
        self._slider_handle_press=False
        if self.saved:
            index=self.result_slider.sliderPosition()
            if self.result_slider.value()!=index:
                self.result_slider.setValue(index)
            elif self._saved_last_index!=index:
                self.display_saved(index)
            else:
                self.source_label.setText('结果回看')
        if getattr(self,'_saved_resume_after_seek',False):
            self._schedule_saved()
        self._saved_resume_after_seek=False

    def eventFilter(self,widget,event):
        if widget is getattr(self,'result_slider',None) and event.type()==QEvent.MouseButtonPress:
            option=QStyleOptionSlider()
            widget.initStyleOption(option)
            handle=widget.style().subControlRect(QStyle.CC_Slider,option,QStyle.SC_SliderHandle,widget)
            if event.button()==Qt.LeftButton and handle.contains(event.position().toPoint()):
                self._slider_handle_press=True
                QTimer.singleShot(0,self._end_slider_press)
        return super().eventFilter(widget,event)

    def _end_slider_press(self):
        if self._slider_handle_press:
            self._slider_handle_press=False
            if not self.result_slider.isSliderDown() and self._saved_last_index!=self.result_slider.value():
                self.display_saved(self.result_slider.value())

    def display_saved(self,index):
        if getattr(self,'_slider_handle_press',False) or self.result_slider.isSliderDown():
            return
        if isinstance(self.saved,IndexedResults):
            self.saved.attach_pose(index)
        previous=self._saved_last_index
        self._preserve_replay_view=False
        if self.saved and previous is not None and 0<=previous<len(self.saved):
            before=recorded_message(self.saved[previous])
            after=recorded_message(self.saved[index])
            self._preserve_replay_view=(isinstance(before,dict) and isinstance(after,dict)
                and display_reference_key(before)==display_reference_key(after))
        replay=self.saved_replay
        try:
            if replay is not None and getattr(replay,'fast_preview',False):
                try:
                    if self._trajectory_cache.sequence is not self.saved:
                        self._trajectory_cache.prepare(self.saved,replay.preview_poses())
                    images,caption=replay.read_preview(index)
                except (OSError,ValueError,sqlite3.Error) as exc:
                    replay.fast_preview=False
                    self.statusBar().showMessage('快载缓存不可用：'+str(exc))
                else:
                    self.saved_replay=None
                    try:super().display_saved(index)
                    finally:self.saved_replay=replay
                    return show_preview(self,replay,images,caption)
            return super().display_saved(index)
        finally:
            self._preserve_replay_view=False

    def reset_world_view(self):
        if not getattr(self,'_preserve_replay_view',False):
            return super().reset_world_view()

    def _update_body_pose(self,result):
        if self._joint_loaded:
            derived=result.diagnostics.get('offline_derived_pose') or {}
            if not derived.get('body'):
                self._clear_smpl('联合人体 · 本帧不可用');self.hand_overlay.update_mano_meshes([]);return
            self._ground_override=False
        value=super()._update_body_pose(result)
        if self._joint_loaded and self.body_pose is not None:self.body_meta.setText('联合 SMPL · 估计')
        return value

    def _ground_adjusted(self,value):
        if self._joint_loaded:
            self.ground_slider.blockSignals(True);self.ground_slider.setValue(round(value*10.))
            self.ground_slider.blockSignals(False)
            return
        return super()._ground_adjusted(value)

    def sync_controls(self):
        super().sync_controls()
        busy=self._joint_job is not None
        if hasattr(self,'reconstruction_algorithm'):
            running=bool(self.session.worker and self.session.worker.isRunning())
            self.reconstruction_algorithm.setEnabled(not busy and not running)
            self.view_algorithm_button.setEnabled(not busy and not running)
        if busy:
            for w in (self.source,self.open_button,self.results_button,self.output_button,self.start_button,self.compute_button,self.trim_button,self.ground_adjust,self.ground_slider):w.setEnabled(False)
            self.camera_forehead_offset.setEnabled(False)
            self.stop_button.setEnabled(True);self.pause_button.setEnabled(False)
        if self._joint_loaded:
            self.ground_adjust.setToolTip('此数值用于下一次计算，当前回放保持已保存的拟合结果。')

    def start_computation(self):
        if not hasattr(self,'reconstruction_algorithm') or self.reconstruction_algorithm.currentData()=='legacy':
            self._joint_loaded=False;return super().start_computation()
        if self._joint_job is not None:return
        if not self.recording_source:self.show_error('请先打开录制目录。');return
        if not super().stop_pipeline():return
        recording=Path(self.recording_source)
        output=self.result_output_parent/('joint_'+str(time.time_ns()));self._joint_output=output
        JOINT_WORK.mkdir(parents=True,exist_ok=True)
        log=JOINT_WORK/('gui_joint_'+str(time.time_ns())+'.log');self._joint_log=log
        command=[sys.executable,str(SCRIPT_DIR/'joint_offline_pipeline.py'),'--recording',str(recording),'--output',str(output),
                 '--camera-forehead-offset-mm',str(self.camera_forehead_offset.value()),
                 '--initial-camera-height-m',str(self.ground_adjust.value()/100.)]
        env=os.environ.copy();env.update(PYTHONIOENCODING='utf-8',PYTHONDONTWRITEBYTECODE='1')
        try:
            with log.open('w',encoding='utf8') as stream:
                self._joint_job=subprocess.Popen(command,cwd=PROJECT,env=env,stdout=stream,stderr=subprocess.STDOUT,
                    creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0)|getattr(subprocess,'CREATE_NEW_PROCESS_GROUP',0))
            self.last_error=None;self.set_status('新算法离线联合计算中');self.algorithm_note.setText('联合计算中 · 可停止 · 日志：'+log.name)
            self.sync_controls()
        except Exception as exc:self._joint_job=None;self.show_error(str(exc));self.sync_controls()

    def _poll_joint_job(self):
        if self._pending_old is not None:
            path=self._pending_old
            if (path/'unified_smpl_manifest.json').exists() and (path/'display_cache.sqlite3').exists() and not any(folder==path and proc.poll() is None for proc,folder,_ in self._cache_jobs):
                self._register('legacy',path,self.recording_source);self._pending_old=None
        if self._joint_job is None:return
        code=self._joint_job.poll()
        if code is None:
            try:
                text=self._joint_log.read_text(encoding='utf8',errors='replace')
                stages={'extract_joint_measurements.py':'从录制提取相机轨迹和双目观测',
                    'prepare_neutral_joint_seed.py':'初始化中立人体','export_recording.py':'准备双目图像',
                    'prepare_recording_observations.py':'准备二维观测','encode_windows.py':'ACE 视频编码',
                    'infer_latents.py':'ACE 姿态推理','initialize_joint_hands.py':'准备手部初值',
                    'joint_body_mano.py':'人体与手部联合拟合','package_joint_replay.py':'导出回放数据',
                    'build_saved_replay_cache.py':'准备原始图像缓存','build_joint_fast_replay.py':'预计算回放缓存'}
                for line in reversed(text.splitlines()):
                    if line.startswith('CACHE '):
                        self.algorithm_note.setText('预计算回放缓存 '+line.split()[1]+' · 可停止');break
                    if line.startswith(('LATENT COMPLETE ','ACE COMPLETE ')):
                        fields=line.split()
                        direction='左目' if fields[2]=='left' else '右目'
                        stage='ACE 视频编码' if fields[0]=='LATENT' else 'ACE 姿态推理'
                        self.algorithm_note.setText(f'{stage} · {direction}已完成至第 {fields[4]} 帧 · 可停止');break
                    if line.startswith('JOINT ') and '/' in line:
                        self.algorithm_note.setText('联合拟合进度 '+line.split()[1]+' · 可停止');break
                    if line.startswith('STAGE '):
                        self.algorithm_note.setText(stages.get(line[6:],'准备重建数据')+' · 可停止');break
            except OSError:pass
            return
        self._joint_job=None;output=self._joint_output;self.sync_controls()
        try:
            if code:
                lines=self._joint_log.read_text(encoding='utf8',errors='replace').splitlines()
                detail=next((line.strip() for line in reversed(lines)
                    if line.startswith(('ValueError:', 'RuntimeError:', 'OSError:', 'FileNotFoundError:'))),
                    lines[-1].strip() if lines else '后台进程退出')
                raise RuntimeError('新算法计算失败：'+detail[:180]+'；日志：'+str(self._joint_log))
            m=json.loads((output/'unified_smpl_manifest.json').read_text(encoding='utf8'))
            if m.get('status')!='completed' or m.get('algorithm')!='joint_smpl_mano_v2':raise ValueError('联合结果未完整发布')
            pipeline=json.loads((output/'pipeline.json').read_text(encoding='utf8'))
            if pipeline.get('complete_reconstruction_passes')!=1 or not pipeline.get('standalone_fast_replay') or not all((output/name).is_file() for name in
                    ('display_cache.sqlite3','depth_results.sqlite3','joint_pose_index.npz')):
                raise ValueError('快载缓存未完整发布')
            self.open_results_path(output)
            error=getattr(self,'last_error',None)
            if (error or self.saved_replay is None or not self.saved_replay.fast_preview
                    or self.saved_replay.depth_results is None or not isinstance(self.saved,IndexedResults)):
                raise ValueError(error or '快载结果未能正常打开')
            configure_view(self)
            self._register('joint_smpl_mano_v2',output,self.recording_source)
            self._output_note('本次结果：'+str(output))
            self.set_status('新算法计算完成 · 已打开预计算结果')
        except Exception as exc:self.show_error(str(exc))

    def _start_pose_derivation(self,output,options=None):
        self._pending_old=Path(output)
        return super()._start_pose_derivation(output,options)

    def stop_pipeline(self):
        if self._joint_job is not None:
            pid=self._joint_job.pid
            subprocess.run(['taskkill','/PID',str(pid),'/T','/F'],capture_output=True,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0),timeout=10)
            self._joint_job=None;self.set_status('联合计算已停止 · 未完成目录保留')
        return super().stop_pipeline()

def main():
    p=argparse.ArgumentParser();p.add_argument('--open-results',type=Path);p.add_argument('--quit-after',type=float);a=p.parse_args()
    os.chdir(PROJECT);QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app=QApplication(sys.argv[:1]);app.setStyle('Fusion');window=JointClientWindow();window.show()
    if a.open_results and a.open_results.exists():window.open_results_path(a.open_results);QTimer.singleShot(500,lambda:configure_view(window))
    if a.quit_after:QTimer.singleShot(int(a.quit_after*1000),window.close)
    return app.exec() or int(bool(getattr(window,'last_error',None)))
if __name__=='__main__':raise SystemExit(main())
