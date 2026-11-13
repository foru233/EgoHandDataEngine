# 主处理引擎：检查来源/同步/标定，再执行二维检测、左右关联、三角化及轨迹维护。
# 常用调参入口是 configs/stereo.yaml；本文件不读取 GUI 的 detector.yaml。

import time
import uuid
from concurrent.futures import ThreadPoolExecutor, wait
from dataclasses import replace
from itertools import product
from pathlib import Path
import numpy as np
import yaml
from .types import StereoHandResult, Hand3D, StereoFrame, HandDetection
from .status import Status
from ..calibration.rectifier import StereoRectifier
from ..calibration.validator import CalibrationError
from ..stereo.triangulation import JointTriangulator
from ..stereo.hand_matching import StereoHandAssociator, HandTrackManager
from ..stereo.uncertainty import DEFAULT_UNCERTAINTY

ROOT=Path(__file__).resolve().parents[3]

# 读取双目默认配置；调用方传入的 config 会在引擎创建时覆盖同名顶层项。
def default_config():
    return yaml.safe_load((ROOT/"configs/stereo.yaml").read_text(encoding="utf-8"))

# 面向 GUI 与无界面调用的统一引擎；三维输出约定为原始左相机光学坐标、单位米。
class EgoHandEngine:
    def __init__(self,config=None,calibration=None,detectors=None):
        self.config=default_config()
        if config: self.config.update(config)
        self.calibration=None
        self.rectifier=None
        self.calibration_error=None
        self.detectors=detectors
        self._detection_executor=None
        self.source=None
        self.last_timestamp=None
        self.session_id=str(uuid.uuid4())
        self.tracks=HandTrackManager(self.config)
        self.last_frame=None
        self.vio_binding=None
        if calibration is not None:
            self.set_calibration(calibration)

    # 更换标定时同时重建校正器/关联器并清空跟踪，防止沿用旧几何或旧 ID。
    def set_calibration(self,c):
        # Never retain a previously valid geometry after a failed load.
        self.calibration=None
        self.rectifier=None
        self.calibration_error=None
        try:
            r=StereoRectifier(c)
        except (CalibrationError,ValueError) as e:
            self.calibration_error=str(e)
            raise
        self.calibration=c
        self.rectifier=r
        self.associator=StereoHandAssociator(JointTriangulator(r,self.config),self.config)
        self.reset_tracking()

    # 回放跳转、循环、标定或来源改变时重置时间轴；track_id 仅在 session_id 内唯一。
    def reset_tracking(self):
        self.last_timestamp=None
        self.tracks=HandTrackManager(self.config)
        self.session_id=str(uuid.uuid4())
        # 会话切换后禁止沿用旧会话的位姿。
        self.vio_binding=None

    def bind_offline_pose_provider(self, provider, *, clock_domain, pose_session_id):
        """显式绑定当前会话；相机帧仍须声明相同的时钟域和标定。"""
        if not clock_domain or pose_session_id != self.session_id:
            raise ValueError('VIO 时钟域或采集会话不匹配')
        self.vio_binding=(provider, clock_domain, pose_session_id)

    # 按数据源接口打开/启动；没有可用标定时仍允许二维检测，不能猜测米制坐标。
    def start(self,source,*,cancelled=None):
        if self.source: self.stop()
        self.source=source
        source.open()
        if cancelled is not None and cancelled.is_set():return
        c=source.get_calibration()
        self.calibration=None
        self.rectifier=None
        self.calibration_error=None
        if c is not None:
            try: self.set_calibration(c)
            except CalibrationError: pass
        self.reset_tracking()
        if cancelled is None or not cancelled.is_set():source.start()

    def process_next(self):
        if self.source is None: raise RuntimeError("尚未启动数据源")
        f=self.source.read()
        return None if f is None else self.process(f)

    def _timed_detection(self, side, image, timestamp_ns):
        started = time.perf_counter()
        detections = self.detectors[side].detect(image, timestamp_ns)
        return detections, (time.perf_counter() - started) * 1000

    def _associate_detection_variants(self,detections,diag,tracks,timestamp_ns):
        """Select whole-view measured variants using the unchanged stereo gates."""
        options=[]
        for side,current in zip(('left','right'),detections):
            original=(diag.get(side+'_detection_details') or {}).get('full_frame_observations')
            valid=(isinstance(original,list) and len(original)==len(current)
                and all(isinstance(d,HandDetection) for d in original)
                and [d.handedness for d in original]==[d.handedness for d in current])
            # The original view is evaluated first and wins exact ties.
            options.append([('full_frame',original),('roi',current)] if valid else [('current',current)])
        alternatives=[];best=None;tri_ms=0.
        for left,right in product(*options):
            matches,ambiguous=self.associator.associate(left[1],right[1],tracks,timestamp_ns)
            tri_ms+=self.associator.tri.elapsed_ms
            support=sum(sum(j.valid for j in match.joints) for match in matches)
            # Prefer complete, unambiguous stereo hands, then valid joint support;
            # total assignment cost breaks support ties. No GT or pose is used.
            score=(len(matches),support,-self.associator.diagnostics.get('total_cost',0.))
            alternatives.append({'left':left[0],'right':right[0],'accepted_pairs':len(matches),
                'valid_joints':support,'assignment_cost':-score[2],'ambiguous':ambiguous})
            if best is None or score>best[0]:
                best=(score,[left[1],right[1]],matches,ambiguous,self.associator.diagnostics,left[0],right[0])
        self.associator.diagnostics=best[4]
        self.associator.tri.elapsed_ms=tri_ms
        diag['stereo_detection_selection']={'method':'whole_view_original_or_roi_existing_stereo_gates',
            'selected_left':best[5],'selected_right':best[6],'alternatives':alternatives,
            'ground_truth_used':False,'geometry_thresholds_changed':False,
            'absolute_accuracy_or_identity_verified':False}
        return best[1],best[2],best[3]

    # 外部图像调用入口；左右时间戳、同步声明、图像是否校正都必须由调用方明确提供。
    def process_stereo(self,left_image,right_image,*,frame_id,timestamp_ns,left_timestamp_ns=None,right_timestamp_ns=None,synchronized=False,source_type="external",metadata=None,left_detections=None,right_detections=None,image_geometry="raw",calibration=None):
        """Headless external-image API. Metadata must declare provenance/evidence.

        Precomputed pixels must correspond to image_geometry. Synchronization is
        never inferred from equal processing timestamps or matching image sizes.
        """
        if calibration is not None and calibration is not self.calibration:
            self.set_calibration(calibration)
        frame=StereoFrame(frame_id,timestamp_ns,left_image,right_image,left_timestamp_ns,right_timestamp_ns,self.calibration.calibration_id if self.calibration else None,source_type,synchronized,image_geometry,metadata or {},left_detections,right_detections)
        return self.process(frame)

    # 单帧主流程；dataset_pairing 是离线基准的显式配对依据，不等于硬件同步验证。
    def process(self,frame,calibration=None,*,dataset_pairing=None):
        start=time.perf_counter()
        if calibration is not None:
            if calibration is not self.calibration:self.set_calibration(calibration)
            if frame.calibration_id is None:frame=replace(frame,calibration_id=calibration.calibration_id)
        out=StereoHandResult(frame.frame_id,frame.timestamp_ns,frame.calibration_id,frame.source_type,session_id=self.session_id)
        diag=out.diagnostics
        ucfg={**DEFAULT_UNCERTAINTY,**self.config.get('uncertainty',{})}
        diag['uncertainty_assumptions']={**ucfg,'pixel_coordinate_space':'rectified_detector_pixels','conditional_on_fixed_calibration_and_correct_pairing':True,'systematic_bias_and_timing_modelled':False}
        diag['uncertainty_status']='diagnostic_assumed_noise' if ucfg['enabled'] else 'disabled'
        # 只有数据集名、来源、配对编号和 source_type 一致时才接受离线配对上下文。
        documented_pair=bool(dataset_pairing and dataset_pairing.get('dataset_id') in ('stb','hot3d','show3d','dexycb') and dataset_pairing.get('sync_quality')=='DATASET_DOCUMENTED' and dataset_pairing.get('pair_id') and dataset_pairing.get('source') and frame.source_type=='benchmark:'+dataset_pairing.get('dataset_id'))
        user_camera_pair=bool(self.config.get('vs030_user_confirmed_sync',False) and frame.source_type=='vendor_vs030'
            and frame.metadata.get('real_stereo') and frame.metadata.get('transport_pairing') in ('same_udp_bundle_not_sensor_sync','sdk_sequence_and_equal_device_timestamp'))
        if dataset_pairing and not documented_pair:raise ValueError('Invalid explicit public-dataset pairing context')
        diag['sync_quality']='DATASET_DOCUMENTED' if documented_pair else 'USER_CONFIRMED_SAME_BUNDLE' if user_camera_pair else frame.metadata.get('sync_quality','USER_ASSERTED' if frame.synchronized else 'UNKNOWN')
        if user_camera_pair:diag['sync_evidence_detail']='用户确认设备同步；依据设备数据流分组配对，尚未测量左右曝光差'
        diag['hardware_sync_precision_verified']=False
        if documented_pair:diag['dataset_pairing']=dataset_pairing
        use_epi=self.config.get('use_epipolar_error',True)
        use_reproj=self.config.get('use_reprojection_error',True)
        diag.update({"synthetic":bool(frame.metadata.get("synthetic",False)),"input_geometry":frame.image_geometry,"timestamp_kind":frame.metadata.get("timestamp_kind","unknown"),"synchronization_evidence":frame.metadata.get("synchronization_evidence"),"calibration_error":self.calibration_error,"processing_mode":frame.metadata.get("processing_mode","api"),"use_epipolar_error":use_epi})
        if user_camera_pair:diag['synchronization_evidence']='user_confirmed_hardware_sync_'+frame.metadata['transport_pairing']
        diag['use_reprojection_error']=use_reproj
        diag['quality_score_formula']='exp(-0.5*('+(' + '.join(x for x,on in [('(epipolar/max_epipolar)^2',use_epi),('(reprojection/max_reprojection)^2',use_reproj)] if on) or 'no enabled terms; score unavailable')+')); derived geometry, not model confidence'
        self.last_frame=frame
        def finish(status,reason=None):
            out.reconstruction_status=str(status)
            if reason: diag["reason"]=reason
            if self.vio_binding is not None:
                from ..vio.hand_world import world_hands_for_result
                provider, clock_domain, pose_session_id = self.vio_binding
                out.world_hands = world_hands_for_result(
                    out, provider, left_image_timestamp_ns=frame.left_timestamp_ns,
                    result_clock_domain=frame.metadata.get('clock_domain'),
                    pose_clock_domain=clock_domain, pose_session_id=pose_session_id)
            out.processing_time_ms=(time.perf_counter()-start)*1000
            return out
        # 先检查处理时间单调性；拒绝倒退时间，避免 VIDEO/跨帧跟踪沿用错误历史。
        if not isinstance(frame.timestamp_ns,int) or frame.timestamp_ns<0 or (self.last_timestamp is not None and frame.timestamp_ns<=self.last_timestamp):
            return finish(Status.ERROR,"TIMESTAMP_NON_MONOTONIC")
        self.last_timestamp=frame.timestamp_ns
        # 只有来源、同步、标定与帧几何全部满足要求才进入三维；失败时保留二维结果。
        geometric=self.rectifier is not None
        reason=None
        if geometric:
            if self.calibration.synthetic and not diag["synthetic"]:
                geometric=False; reason="SYNTHETIC_CALIBRATION_FOR_REAL_SOURCE"
            elif not self.calibration.synthetic and diag["synthetic"]:
                geometric=False; reason="REAL_CALIBRATION_FOR_SYNTHETIC_SOURCE"
            elif not diag["synthetic"] and not frame.metadata.get("real_stereo",False):
                geometric=False; reason="REAL_STEREO_PROVENANCE_MISSING"
            elif not documented_pair and not user_camera_pair and (not frame.synchronized or not frame.metadata.get("synchronization_evidence")):
                geometric=False; reason="Synchronization Unverified"
            elif not documented_pair and not user_camera_pair and (frame.left_timestamp_ns is None or frame.right_timestamp_ns is None):
                geometric=False; reason="MISSING_STEREO_TIMESTAMPS"
            elif frame.left_timestamp_ns is not None and frame.right_timestamp_ns is not None:
                error=abs(frame.left_timestamp_ns-frame.right_timestamp_ns)/1e6
                diag["sync_error_ms"]=error
                if error>self.config["max_sync_error_ms"]:
                    geometric=False; reason="SYNC_ERROR_EXCEEDED"
        # 图像校正与时间同步门控分开：同步失败仍可在已校正图上做二维检测，不能做三维。
        rectification_ready=False
        if self.rectifier is not None and reason not in ('SYNTHETIC_CALIBRATION_FOR_REAL_SOURCE','REAL_CALIBRATION_FOR_SYNTHETIC_SOURCE'):
            try:
                frame=self.rectifier.rectify(frame)
                self.last_frame=frame
                rectification_ready=True
            except (CalibrationError,ValueError) as e:
                geometric=False; reason=str(e)
        diag["output_geometry"]=frame.image_geometry
        diag['detector_rectification_ready']=rectification_ready
        # 优先使用明确提供的二维观测；否则调用左右独立模型实例，绝不把模型 world 点当三角化输入。
        detections=[]
        parallel = None
        if (self.config.get('parallel_stereo_detection') and self.detectors and rectification_ready
                and frame.left_detections is None and frame.right_detections is None):
            if self._detection_executor is None:
                self._detection_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='EgoHandRight')
            right = self._detection_executor.submit(self._timed_detection, 1, frame.right_image, frame.timestamp_ns)
            try:
                left = self._timed_detection(0, frame.left_image, frame.timestamp_ns)
                parallel = (left, right.result())
            except Exception as e:
                wait((right,))
                return finish(Status.ERROR,f"DETECTOR_ERROR: {e}")
        for side,image,precomputed in ((0,frame.left_image,frame.left_detections),(1,frame.right_image,frame.right_detections)):
            tick=time.perf_counter()
            elapsed_ms=None
            if precomputed is not None:
                diag[("left","right")[side]+"_detection_origin"]="precomputed"
                if any(d.origin=="synthetic_projection" for d in precomputed) and not diag["synthetic"]:
                    return finish(Status.ERROR,"SYNTHETIC_LANDMARKS_FOR_REAL_SOURCE")
                ds=precomputed
            elif self.detectors and rectification_ready:
                diag[("left","right")[side]+"_detection_origin"]="rtmpose" if type(self.detectors[side]).__name__=='RTMPoseHandDetector' else "mediapipe"
                try:
                    # 显示、ROI、检测和输出均使用同一侧完整校正图的像素坐标。
                    ds,elapsed_ms=parallel[side] if parallel is not None else self._timed_detection(side,image,frame.timestamp_ns)
                except Exception as e: return finish(Status.ERROR,f"DETECTOR_ERROR: {e}")
                diag[("left","right")[side]+"_detection_details"]=getattr(self.detectors[side],'last_detection_diagnostics',{})
                diag['detector_input']='stereo_rectified_image'
            else:
                diag[("left","right")[side]+"_detection_origin"]="none"
                ds=[]
                if self.detectors:
                    diag['model_status']='RECTIFICATION_REQUIRED'
                else:diag["model_status"]=str(Status.MODEL_NOT_LOADED)
            diag[("left","right")[side]+"_detection_ms"]=elapsed_ms if elapsed_ms is not None else (time.perf_counter()-tick)*1000
            detections.append(ds)
        out.left_detections,out.right_detections=detections
        diag["detected_hands_left"],diag["detected_hands_right"]=map(len,detections)
        if not geometric:
            return finish(Status.TWO_D,('缺少有效双目标定或图像校正失败，未运行关键点检测：'+str(reason or self.calibration_error or '未标定')) if self.detectors and not rectification_ready else reason or self.calibration_error or "Uncalibrated")
        tick=time.perf_counter()
        # 先做左右手配对，再做跨帧 ID 分配；两种歧义分别记录，不能强行拼成连续轨迹。
        tracking_enabled=bool(self.config.get('enable_3d_tracking',True))
        diag['tracking_3d_enabled']=tracking_enabled
        tracks=self.tracks.active(frame.timestamp_ns) if tracking_enabled else ()
        if self.config.get('stereo_refinement_selection',False):
            detections,matches,ambiguous=self._associate_detection_variants(detections,diag,tracks,frame.timestamp_ns)
            out.left_detections,out.right_detections=detections
        else:
            matches,ambiguous=self.associator.associate(*detections,tracks,frame.timestamp_ns)
        diag['association_quality']=self.associator.diagnostics
        diag["ambiguous"]=ambiguous
        diag["matching_and_triangulation_ms"]=(time.perf_counter()-tick)*1000
        diag["triangulation_ms"]=self.associator.tri.elapsed_ms
        matched_left=set();matched_right=set()
        for match in matches:
            a,b=detections[0][match.left_index],detections[1][match.right_index]
            matched_left.add(match.left_index)
            matched_right.add(match.right_index)
            out.hands.append(Hand3D(a.handedness,a.handedness_score,-1,match.joints,a.world_landmarks,sum(j.valid for j in match.joints),"observed",b.world_landmarks))
        # 未匹配的单侧检测仍保存二维像素，但其三维关节保持无效，不用零坐标补齐。
        for i,a in enumerate(detections[0]):
            if i not in matched_left:
                h=Hand3D(a.handedness,a.handedness_score,-1,mediapipe_world_landmarks=a.world_landmarks,tracking_status="ambiguous" if ambiguous else "unmatched")
                for j,uv in zip(h.joints,a.pixel_landmarks):
                    if np.isfinite(uv).all(): j.left_uv_px=tuple(uv)
                out.hands.append(h)
        for i,b in enumerate(detections[1]):
            if i not in matched_right:
                h=Hand3D(b.handedness,b.handedness_score,-1,right_mediapipe_world_landmarks=b.world_landmarks,tracking_status="ambiguous" if ambiguous else "unmatched_right")
                for j,uv in zip(h.joints,b.pixel_landmarks):
                    if np.isfinite(uv).all(): j.right_uv_px=tuple(uv)
                out.hands.append(h)
        if tracking_enabled:self.tracks.assign(out.hands,frame.timestamp_ns)
        else:
            for h in out.hands:
                if h.valid_joint_count:h.tracking_status='frame_only'
        diag["identity_ambiguous"]=any(h.tracking_status=="identity_ambiguous" for h in out.hands)
        ambiguous=ambiguous or diag["identity_ambiguous"]
        diag["ambiguous"]=ambiguous
        # 仅汇总当前有效点的几何一致性；低重投影误差不证明真实三维位置准确。
        errors=[j.reprojection_error_px for h in out.hands for j in h.joints if j.valid]
        diag["mean_reprojection_error_px"]=float(np.mean(errors)) if errors else None
        diag["valid_joints"]=sum(h.valid_joint_count for h in out.hands)
        stds=[j.depth_std_m for h in out.hands for j in h.joints if j.depth_std_m is not None]
        diag['depth_uncertainty']={'available_joints':len(stds),'mean_depth_std_m':float(np.mean(stds)) if stds else None,'calibrated_probability':False}
        diag['geometry_consistency']={'mean_reprojection_error_px':diag['mean_reprojection_error_px'],'not_a_measure_of_absolute_accuracy':True}
        diag['temporal_consistency']={'identity_ambiguous':diag['identity_ambiguous'],'method':'bounded_motion_and_pose_gating' if tracking_enabled else 'disabled'}
        if documented_pair:diag['metric_evidence_scope']='public_dataset_only_not_local_device'
        status=Status.SYNTHETIC if diag["synthetic"] else Status.METRIC
        if not errors: status=Status.INVALID_MATCH
        elif documented_pair and not diag['synthetic']:status='Dataset Stereo Reconstruction'
        elif user_camera_pair and errors:status='User-confirmed Stereo 3D'
        return finish(status,"ambiguous" if ambiguous else None)

    def stop(self):
        if self.source:
            self.source.stop(); self.source.close(); self.source=None
    # 停止数据源并释放两个模型实例；关闭资源不能只退出 GUI 绘制循环。
    def close(self):
        try:
            self.stop()
        finally:
            if self._detection_executor is not None:
                self._detection_executor.shutdown()
                self._detection_executor=None
            if self.detectors:
                for d in self.detectors: d.close()
