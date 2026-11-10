# 线程调度层：主线程绘图，采集/重建/写盘独立；本轮只补注释，不改界面布局或交互。
# 队列参数优先调 configs/app.yaml；模型默认参数由 GUI 读取 detector.yaml 后传入。

import logging
import queue
import threading
import time
import yaml
from dataclasses import replace
from collections import deque
from pathlib import Path
from PySide6.QtCore import QThread,Signal
from ..core.pipeline import EgoHandEngine
from ..core.pipeline import ROOT
from ..storage.recorder import AsyncRecorder
from ..sources.hot3d import HOT3DFrameProcessor
from ..sources.show3d import SHOW3DFrameProcessor
from ..detection.mediapipe_detector import MediaPipeHandDetector
from ..detection.roi import validate_roi

def _publish_frame(source_type, rectifier, frame):
    """实时相机已启用校正时，失败帧不能让预览在原图和校正图之间闪动。"""
    return not (source_type=='vendor_vs030' and rectifier is not None and frame.image_geometry!='rectified')

# 工作线程向 GUI 发信号，不直接操作界面控件；命令队列串行应用模型/录制设置。
class PipelineWorker(QThread):
    vio_status=Signal(dict)
    hot3d_mode_applied=Signal(str)
    error=Signal(str)
    startup_status=Signal(str)
    ready=Signal(dict)
    detector_settings_applied=Signal(dict,str)
    detector_settings_failed=Signal(str)
    roi_settings_applied=Signal(dict,str)
    roi_settings_failed=Signal(str)
    recording_state_changed=Signal(bool,bool,str)
    def __init__(self,source,config=None,detector_config=None,calibration=None,offline=False,parent=None,rois=None,live_vio=False):
        super().__init__(parent)
        self.source=source;self.config=config or {};self.detector_config=detector_config;self.calibration=calibration
        self.live_vio_enabled=live_vio;self.live_vio=None
        self.hot3d_mode=getattr(source,"view_mode","prediction");self.hot3d_processor=HOT3DFrameProcessor();self.show3d_processor=SHOW3DFrameProcessor()
        self.offline=offline
        self.runtime=yaml.safe_load((ROOT/"configs/app.yaml").read_text(encoding="utf-8"))
        self.queue=queue.Queue(max(1,int(self.runtime["processing_queue"])))
        self.commands=queue.Queue()
        self.stop_event=threading.Event()
        self.pause_event=threading.Event()
        self.step_event=threading.Event()
        self.acquisition_done=threading.Event()
        self.output=deque(maxlen=1)
        self.history=deque(maxlen=600)
        self.dropped=0;self.read_count=0;self.processed=0
        self.recorder=None;self.seek_request=None;self.eof=False
        self.capture_thread=None;self.engine=None;self.last_error=None
        self.stats={}
        self.rois={side:validate_roi((rois or {}).get(side)) for side in ('left','right')}
        self.last_input_frame=None;self.roi_reprocess_pending=False;self.roi_hold_paused=False
    def stop(self):
        self.stop_event.set();self.pause_event.clear()
        interrupt=getattr(self.source,'interrupt_read',None)
        if interrupt:interrupt()
    def pause(self,value=True):
        self.pause_event.set() if value else self.pause_event.clear()
        if not value:self.roi_hold_paused=False
    def step(self):
        self.roi_hold_paused=False
        self.pause_event.set();self.step_event.set()
    def seek(self,index):
        self.seek_request=index;self.step()
    def command(self,name,value=None): self.commands.put((name,value))
    def _recording_state(self,message):
        recorder=self.recorder
        active=bool(recorder and recorder.accepting)
        self.recording_state_changed.emit(active,bool(active and recorder.paused),message)
    def _new_detectors(self):
        replacement=self._build_detectors(self.detector_config)
        previous=self.engine.detectors
        self.engine.detectors=replacement
        if previous:
            for d in previous:d.close()
    # 先完整创建两个新实例，再替换旧实例；部分创建失败要清理，不能留下半套检测器。
    def _build_detectors(self,config):
        if config and not self.source.get_metadata().synthetic:
            mode="IMAGE" if self.source.get_metadata().source_type in ("real_single_image","hot3d_clips","show3d","dexycb") else "VIDEO"
            pair=[]
            try:
                algorithm=config.get('algorithm','mediapipe')
                if algorithm=='rtmpose':
                    from ..detection.rtmpose_detector import RTMPoseHandDetector
                    options={k:config[k] for k in ('num_hands','box_threshold','keypoint_threshold') if k in config}
                    cls=RTMPoseHandDetector
                elif algorithm=='mediapipe':
                    options={k:config[k] for k in ('model_path','num_hands','min_hand_detection_confidence','min_hand_presence_confidence','min_tracking_confidence') if k in config}
                    if self.source.get_metadata().source_type=='vendor_vs030':
                        options['rescue_config']={'max_crops_per_frame':int(self.runtime.get('vs030_rescue_crops_per_frame',1))}
                    cls=MediaPipeHandDetector
                else:raise ValueError('未知检测算法：'+str(algorithm))
                for side in ('left','right'):pair.append(cls(mode=mode,**{**options,'roi':self.rois[side]}))
            except Exception:
                for d in pair: d.close()
                raise
            return pair
        return None
    # 只负责采集、原始录制与入处理队列；实时允许丢待处理帧，原始录制另有队列。
    def _capture(self):
        previous=None
        try:
            while not self.stop_event.is_set():
                if self.pause_event.is_set() and not self.step_event.is_set() and self.seek_request is None:
                    if self.live_vio and self.live_vio.thread.is_alive():
                        # 暂停手部处理时继续喂实时定位，避免 IMU 队列积压和时间断裂。
                        frame=self.source.read()
                        if frame is None:
                            if self.source.get_metadata().source_type == "vendor_vs030" and getattr(self.source, "running", False):
                                self.stop_event.wait(.02)
                                continue
                            break
                        self.live_vio.submit(frame)
                        continue
                    self.stop_event.wait(.02);continue
                reset=False
                if self.seek_request is not None:
                    index=self.seek_request;self.seek_request=None
                    if not hasattr(self.source,"seek"): raise ValueError("当前数据源不支持定位")
                    self.source.seek(index)
                    while True:
                        try: self.queue.get_nowait()
                        except queue.Empty: break
                    reset=True;previous=None
                tick=time.perf_counter()
                frame=self.source.read()
                if frame is None:
                    self.eof=True
                    # VS030 的预览解码可短时停顿/重置；原始 SDK 与 IMU 录制仍继续。
                    if self.source.get_metadata().source_type == "vendor_vs030" and getattr(self.source, "running", False):
                        self.stop_event.wait(.02)
                        continue
                    if hasattr(self.source,"seek"):
                        self.pause_event.set();self.step_event.clear();self.stop_event.wait(.02);continue
                    break
                self.eof=False
                if self.live_vio:self.live_vio.submit(frame)
                if self.calibration is not None and (frame.calibration_id is None or frame.source_type=='vendor_vs030'):
                    frame.calibration_id=self.calibration.calibration_id
                if previous is not None and frame.timestamp_ns<=previous and getattr(self.source,"loop",False): reset=True
                previous=frame.timestamp_ns
                frame.metadata={**frame.metadata,"processing_mode":"offline" if self.offline else "realtime","acquired_perf_counter_ns":time.perf_counter_ns(),"reset_tracking":reset}
                recorder=self.recorder
                if recorder:
                    if self.source.get_metadata().source_type == "vendor_vs030":
                        recorder.submit("frame_index", frame, False)
                    else:
                        recorder.submit("raw", frame, self.offline)
                self.read_count+=1
                if self.offline:
                    while not self.stop_event.is_set():
                        try: self.queue.put(frame,timeout=.05);break
                        except queue.Full: pass
                else:
                    try: self.queue.put_nowait(frame)
                    except queue.Full:
                        try: self.queue.get_nowait();self.dropped+=1
                        except queue.Empty: pass
                        self.queue.put_nowait(frame)
                self.step_event.clear()
                if not self.offline and not hasattr(self.source,'wait_for_preview'):
                    fps=getattr(self.source,"fps",30)
                    self.stop_event.wait(max(0,1/fps-(time.perf_counter()-tick)))
        except Exception as e:
            self.last_error=str(e);self.error.emit(str(e));logging.exception("capture failed")
        finally: self.acquisition_done.set()
    # 在工作线程应用控制命令，避免 GUI 同时改模型状态或写盘资源。
    def _commands(self):
        while True:
            try: name,value=self.commands.get_nowait()
            except queue.Empty: break
            try:
                if name in ('use_epipolar_error','use_reprojection_error','enable_3d_tracking'):
                    self.engine.config[name]=bool(value)
                    self.engine.reset_tracking()
                    if self.pause_event.is_set() and self.last_input_frame is not None:
                        self.roi_reprocess_pending=True;self.roi_hold_paused=True
                    if self.recorder:self.recorder.submit('event',{'event':name+'_changed','enabled':bool(value),'time_ns':time.time_ns()},self.offline)
                elif name=='roi_config':
                    requested={s:validate_roi(value.get(s)) for s in ('left','right')}
                    if requested==self.rois:
                        self.roi_settings_applied.emit(dict(self.rois),'ROI未改变。');continue
                    previous_rois=self.rois;self.rois=requested
                    is_gt=self.source.get_metadata().source_type in ('hot3d_clips','show3d') and self.hot3d_mode=='ground_truth'
                    try:replacement=None if is_gt else self._build_detectors(self.detector_config)
                    except Exception:
                        self.rois=previous_rois
                        self.roi_settings_applied.emit(dict(previous_rois),'保留原ROI。');raise
                    previous=self.engine.detectors;self.engine.detectors=replacement
                    self.engine.reset_tracking()
                    if previous:
                        for d in previous:d.close()
                    if self.pause_event.is_set() and self.last_input_frame is not None:
                        # 重算缓存的原始输入，但保留尚未处理的队列帧，不能破坏离线完整处理。
                        self.roi_reprocess_pending=True;self.roi_hold_paused=True
                    if self.recorder:self.recorder.submit('event',{'event':'roi_changed','roi':self.rois,'time_ns':time.time_ns()},self.offline)
                    message='ROI已保存，切到算法预测后生效。' if is_gt else 'ROI已应用；暂停时重算当前帧，运行时从后续帧生效。'
                    if self.source.get_metadata().synthetic:message='合成投影不运行模型；ROI仅在真实图像检测时生效。'
                    self.roi_settings_applied.emit(dict(self.rois),message)
                elif name=="hot3d_mode":
                    if value["mode"]=="prediction" and not self.engine.detectors:self._new_detectors()
                    self.hot3d_mode=value["mode"];self.engine.reset_tracking();self.seek(value["frame_index"])
                    self.hot3d_mode_applied.emit(self.hot3d_mode)
                elif name=="record_start":
                    if self.recorder:
                        old=self.recorder
                        stats=self.source.get_capture_stats() if hasattr(self.source,"get_capture_stats") else None
                        if hasattr(self.source,"sdk_record_callback"):self.source.sdk_record_callback=None
                        old.close(capture_stats=stats);self.recorder=None
                    md=self.source.get_metadata()
                    model={"sha256":[d.sha256 for d in self.engine.detectors],"detector_config":self.detector_config,"api":self.detector_config.get('algorithm','mediapipe'),"delegate":"CPU"} if self.engine.detectors else {"mode":"synthetic_geometry" if md.synthetic else "not_loaded"}
                    if self.engine.detectors:model['detector_rescue_config']=[dict(getattr(d,'rescue_config',{})) for d in self.engine.detectors]
                    model['manual_rois']=dict(self.rois)
                    self.recorder=AsyncRecorder(value,md,None if md.source_type in ("hot3d_clips","show3d") else self.engine.calibration,self.engine.config,model,max_queue=(2048 if md.source_type=='vendor_vs030' else max(1,int(self.runtime["recording_queue"]))))
                    if md.source_type=='vendor_vs030':
                        recorder=self.recorder
                        self.source.sdk_record_callback=lambda packet: recorder.submit('sdk',packet,False)
                    self.recorder.submit('event',{'event':'recording_started','time_ns':time.time_ns(),'directory':str(value)},False)
                    self._recording_state(('正在录制 SDK 双目 H.265 + 原始 IMU（预览独立；同步/传输语义待核验）：' if md.source_type=='vendor_vs030' else '正在录制：')+str(value))
                elif name=="record_pause" and self.recorder:self.recorder.set_paused(value)
                elif name=="record_stop" and self.recorder:
                    recorder=self.recorder
                    if hasattr(self.source,'sdk_record_callback'):self.source.sdk_record_callback=None
                    recorder.submit('event',{'event':'recording_stopped','time_ns':time.time_ns()},False)
                    stats=self.source.get_capture_stats() if hasattr(self.source,"get_capture_stats") else None
                    recorder.close(capture_stats=stats);self.recorder=None
                elif name=="detector_config":
                    replacement=self._build_detectors(value)
                    previous=self.engine.detectors
                    self.engine.detectors=replacement
                    self.detector_config=dict(value)
                    if replacement:self.engine.reset_tracking()
                    if previous:
                        for d in previous:d.close()
                    message="已应用；当前几何模拟不运行检测模型，投影与手数不受这些参数影响。" if self.source.get_metadata().synthetic else "已应用，后续处理帧使用新参数；检测器与跟踪会话已重置，采集和录制继续。"
                    self.detector_settings_applied.emit(dict(value),message)
                if name=='record_pause':self._recording_state('录制暂停' if value else '录制继续')
                elif name=='record_stop':self._recording_state('录制已停止')
            except Exception as e:
                if name in ('record_start','record_pause','record_stop'):
                    self._recording_state('录制操作失败：'+str(e))
                if name=='roi_config':self.roi_settings_failed.emit(str(e))
                if name=="hot3d_mode":self.hot3d_mode_applied.emit(self.hot3d_mode)
                if name=="detector_config":self.detector_settings_failed.emit(str(e))
                else:self.error.emit(str(e))
    # 统一启动/重建/退出与资源回收；跳转或循环后必须重置时间相关的引擎与模型状态。
    def _start_source(self):
        wait_for_preview=getattr(self.source,'wait_for_preview',None)
        if wait_for_preview is None:
            self.engine.start(self.source)
            return not self.stop_event.is_set()
        for attempt in range(2):
            if self.stop_event.is_set():return False
            try:
                self.startup_status.emit('正在连接 VS030')
                self.engine.start(self.source,cancelled=self.stop_event)
                if self.stop_event.is_set():return False
                self.startup_status.emit('正在等待 VS030 首对画面')
                return wait_for_preview(self.stop_event)
            except TimeoutError as exc:
                if self.stop_event.is_set():return False
                if attempt:raise
                self.startup_status.emit(f'{exc}，正在重新连接 VS030')
                if self.stop_event.wait(.25):return False

    def run(self):
        started=time.perf_counter()
        try:
            self.engine=EgoHandEngine(self.config)
            if not self._start_source():return
            if self.source.get_metadata().source_type=='vendor_vs030':
                if self.live_vio_enabled and self.calibration and self.source.get_calibration() and self.calibration.calibration_id!=self.source.get_calibration().calibration_id:
                    raise ValueError('实时 VIO 使用设备原标定，请先清除手动加载的其他标定')
                if self.live_vio_enabled and not self.stop_event.is_set():
                    from ..vio.live_runtime import LiveVio
                    self.live_vio=LiveVio(self.source,self.vio_status.emit)
                    self.live_vio.start(defer_imu=True)
                # 该设备近距离按独立配置调整，其他数据集继续使用通用门限。
                self.engine.config['max_epipolar_px']=self.engine.config.get('vs030_max_epipolar_px',self.engine.config['max_epipolar_px'])
                self.engine.config['parallel_stereo_detection']=not self.offline
            if self.stop_event.is_set():return
            if self.calibration: self.engine.set_calibration(self.calibration)
            if self.detector_config and not (self.source.get_metadata().source_type in ("hot3d_clips","show3d") and self.hot3d_mode=="ground_truth"):
                try: self._new_detectors()
                except Exception as e: self.error.emit(f"Model Not Loaded: {e}")
            self.ready.emit({"metadata":self.source.get_metadata(),"model_loaded":bool(self.engine.detectors),"calibration":self.engine.calibration})
            self._commands()
            self.capture_thread=threading.Thread(target=self._capture,name="EgoHandCapture",daemon=False)
            self.capture_thread.start()
            preview_error=None
            while not self.stop_event.is_set():
                self._commands()
                # 即使暂停且没有新帧，写盘失败也必须及时反馈到录制按钮。
                if self.recorder and self.recorder.error:
                    message=self.recorder.error
                    recorder=self.recorder
                    if hasattr(self.source,'sdk_record_callback'):self.source.sdk_record_callback=None
                    stats=self.source.get_capture_stats() if hasattr(self.source,"get_capture_stats") else None
                    recorder.close(capture_stats=stats);self.recorder=None
                    self._recording_state(message);self.error.emit(message)
                if self.roi_hold_paused and self.pause_event.is_set() and not self.roi_reprocess_pending:
                    self.stop_event.wait(.02);continue
                if self.roi_reprocess_pending and self.last_input_frame is not None:
                    frame=replace(self.last_input_frame,metadata={**self.last_input_frame.metadata,
                        'processing_mode':'roi_reprocess','acquired_perf_counter_ns':time.perf_counter_ns()})
                    self.roi_reprocess_pending=False
                else:
                    try: frame=self.queue.get(timeout=.03)
                    except queue.Empty:
                        if self.acquisition_done.is_set(): break
                        continue
                self.last_input_frame=frame
                if frame.metadata.get("reset_tracking") or (getattr(self.source,"loop",False) and self.engine.last_timestamp is not None and frame.timestamp_ns<=self.engine.last_timestamp):
                    self.engine.reset_tracking()
                    if self.detector_config and not (self.source.get_metadata().source_type in ("hot3d_clips","show3d") and self.hot3d_mode=="ground_truth"): self._new_detectors()
                if self.source.get_metadata().source_type=='hot3d_clips':
                    result=self.hot3d_processor.process(self.engine,frame,self.hot3d_mode)
                elif self.source.get_metadata().source_type=='show3d':
                    result=self.show3d_processor.process(self.engine,frame,self.hot3d_mode)
                else:
                    # 手动ROI要求实际重检；对应侧不复用录制文件里旧的二维观测。
                    if self.engine.detectors:
                        frame=replace(frame,left_detections=None if self.rois['left'] else frame.left_detections,
                            right_detections=None if self.rois['right'] else frame.right_detections)
                    pairing=None
                    if self.source.get_metadata().source_type=='dexycb':
                        pairing={'dataset_id':'dexycb','sync_quality':'DATASET_DOCUMENTED','pair_id':frame.metadata['dataset_pair_id'],'source':frame.metadata['synchronization_evidence']}
                    result=self.engine.process(frame,dataset_pairing=pairing)
                result.diagnostics['manual_rois']={'coordinate_space':'displayed_detector_input_normalized','left':self.rois['left'],'right':self.rois['right'],
                    'applied_to_prediction':bool(self.engine.detectors and result.diagnostics.get('detector_rectification_ready') and any(self.rois.values()) and not (self.source.get_metadata().source_type in ('hot3d_clips','show3d') and self.hot3d_mode=='ground_truth'))}
                if self.detector_config:result.diagnostics["detector_config"]=dict(self.detector_config)
                self.processed+=1
                if self.recorder:
                    self.recorder.submit("result",result,self.offline)
                    if self.recorder.error:
                        message=self.recorder.error
                        recorder=self.recorder
                        if hasattr(self.source,'sdk_record_callback'):self.source.sdk_record_callback=None
                        stats=self.source.get_capture_stats() if hasattr(self.source,"get_capture_stats") else None
                        recorder.close(capture_stats=stats);self.recorder=None
                        self._recording_state(message);self.error.emit(message)
                elapsed=max(time.perf_counter()-started,1e-6)
                self.stats={"read_fps":self.read_count/elapsed,"processing_fps":self.processed/elapsed,"processing_queue":self.queue.qsize(),"dropped_frames":self.dropped,"latency_ms":(time.perf_counter_ns()-frame.metadata["acquired_perf_counter_ns"])/1e6,"recording_queue":self.recorder.queue.qsize() if self.recorder else 0,"recording_MB_s":self.recorder.bytes_written/max(time.perf_counter()-self.recorder.started,1e-6)/1e6 if self.recorder else 0}
                self.history.append(result)
                if not _publish_frame(self.source.get_metadata().source_type,self.engine.rectifier,self.engine.last_frame):
                    message='相机帧未完成校正，保留上一张校正图：'+str(result.diagnostics.get('reason','未知错误'))
                    if message!=preview_error:self.error.emit(message);preview_error=message
                    continue
                preview_error=None
                self.output.append((self.engine.last_frame,result,dict(self.stats)))
        except Exception as e:
            self.last_error=str(e);self.error.emit(str(e));logging.exception("pipeline failed")
        finally:
            self.stop()
            if self.live_vio:self.live_vio.close()
            if self.capture_thread: self.capture_thread.join(timeout=10)
            if self.capture_thread and self.capture_thread.is_alive():
                self.error.emit("采集线程未退出：数据源必须实现有超时的 read()")
            # 先停止 SDK/相机数据源并排空原始 IPC，再关闭写盘线程，避免停止时尾包丢失。
            if self.engine:
                try:self.engine.close()
                except Exception as e:
                    if not self.last_error:self.last_error='数据源关闭异常：'+str(e)
                    self.error.emit('数据源关闭异常：'+str(e))
            if hasattr(self.source,'sdk_record_callback'):self.source.sdk_record_callback=None
            if self.recorder:
                if self.last_error and not self.recorder.error:self.recorder.error='采集异常：'+self.last_error
                self.recorder.submit('event',{'event':'recording_closed_by_pipeline_stop','time_ns':time.time_ns()},False)
                stats=self.source.get_capture_stats() if hasattr(self.source,"get_capture_stats") else None
                self.recorder.close(capture_stats=stats)
                self.recorder=None
