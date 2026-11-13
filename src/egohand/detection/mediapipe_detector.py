# MediaPipe CPU 二维检测封装；GUI 默认参数见 configs/detector.yaml。
# 左右视图使用独立实例；模型局部 world 坐标另存，不能替代双目米制坐标。

import hashlib
from pathlib import Path
import cv2
import numpy as np
import yaml
from dataclasses import replace
from ..core.types import HandDetection
from .roi import validate_roi,roi_pixel_box

# 默认仅在全图零检出时重试；离线可显式补检缺失的另一只手。
DEFAULT_RESCUE={"enabled":True,"crop_fraction":.65,"duplicate_distance_fraction":.25,
                "min_inside_joints":16,"min_image_side_px":128,"max_crops_per_frame":4,
                "retry_partial":False,"partial_retry_interval_frames":1,
                "refine_full_frame":False,"refinement_crop_scale":1.8}


def restore_crop_detection(detection,box,image_shape):
    """将裁剪局部的关键点还原到完整输入图，保持原图像素/归一化坐标一致。"""
    x,y,w,h=box;height,width=image_shape[:2]
    uv=detection.pixel_landmarks+np.array([x,y])
    normalized=detection.normalized_landmarks.copy()
    normalized[:,:2]=uv/[width,height]
    if normalized.shape[1]==3:normalized[:,2]*=w/width
    valid=detection.valid&np.isfinite(uv).all(axis=1)&(uv>=0).all(axis=1)&(uv<[width,height]).all(axis=1)
    return replace(detection,pixel_landmarks=uv,normalized_landmarks=normalized,valid=valid,
        bounding_box=(*uv.min(axis=0),*uv.max(axis=0)),origin='mediapipe_tiled_rescue')


def same_hand_pixels(a,b,fraction=.25):
    """相邻重叠裁剪可能报同一只手；按同索引关键点距离去重，不信任手侧标签来去重。"""
    common=a.valid&b.valid&np.isfinite(a.pixel_landmarks).all(axis=1)&np.isfinite(b.pixel_landmarks).all(axis=1)
    if common.sum()<8:return False
    size=max(np.linalg.norm(np.ptp(d.pixel_landmarks[common],axis=0)) for d in (a,b))
    distance=np.median(np.linalg.norm(a.pixel_landmarks[common]-b.pixel_landmarks[common],axis=1))
    return bool(distance<max(8.,fraction*size))

# OpenCV 输入为 BGR，MediaPipe 输入为 RGB；要求 uint8 三通道并保证连续内存。
def bgr_to_rgb(image):
    if image is None or image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3:
        raise ValueError("MediaPipe 输入必须为 uint8 BGR 三通道图像")
    return np.ascontiguousarray(cv2.cvtColor(image,cv2.COLOR_BGR2RGB))

# 构造参数集中在 __init__ 开头，含手数、三个模型阈值、IMAGE/VIDEO 和权重摘要校验。
class MediaPipeHandDetector:
    def __init__(self,model_path,mode="IMAGE",num_hands=2,min_hand_detection_confidence=.5,min_hand_presence_confidence=.5,min_tracking_confidence=.5,expected_sha256=None,rescue_config=None,roi=None):
        path=Path(model_path)
        if not path.is_file():
            raise FileNotFoundError(f"模型未加载：{path} 不存在，请运行 scripts/download_models.py")
        self.sha256=hashlib.sha256(path.read_bytes()).hexdigest()
        if expected_sha256 and self.sha256!=expected_sha256:
            raise ValueError("模型 SHA256 校验失败")
        if mode not in ("IMAGE","VIDEO"):
            raise ValueError("本阶段采用 IMAGE / VIDEO 同步推理；实时模式在后台 worker 使用 VIDEO")
        import mediapipe as mp
        self.mp=mp
        self.mode=mode
        self.last_ms=None
        self.closed=False
        self.num_hands=num_hands
        self._image_landmarker=None
        self.last_detection_diagnostics={}
        self.rescue_config=dict(DEFAULT_RESCUE)
        config_path=Path(__file__).resolve().parents[3]/'configs/detector.yaml'
        if config_path.is_file():
            self.rescue_config.update((yaml.safe_load(config_path.read_text(encoding='utf8')) or {}).get('rescue',{}))
        if rescue_config is not None:self.rescue_config.update(rescue_config)
        if not .5<=self.rescue_config['crop_fraction']<1:raise ValueError('rescue.crop_fraction 必须在 [0.5,1) 内')
        if not 0<self.rescue_config['duplicate_distance_fraction']<=1:raise ValueError('rescue.duplicate_distance_fraction 必须在 (0,1] 内')
        if not 1<=self.rescue_config['min_inside_joints']<=21 or self.rescue_config['min_image_side_px']<1:raise ValueError('rescue 关节计数/最小尺寸配置无效')
        if not 1<=self.rescue_config['max_crops_per_frame']<=4:raise ValueError('max_crops_per_frame 必须在 1 到 4 之间')
        interval=self.rescue_config['partial_retry_interval_frames']
        if isinstance(interval,bool) or not isinstance(interval,int) or interval<1:
            raise ValueError('partial_retry_interval_frames 必须为正整数')
        if not isinstance(self.rescue_config['retry_partial'],bool):raise ValueError('retry_partial 必须为布尔值')
        if not isinstance(self.rescue_config['refine_full_frame'],bool):raise ValueError('refine_full_frame 必须为布尔值')
        scale=self.rescue_config['refinement_crop_scale']
        if isinstance(scale,bool) or not np.isfinite(scale) or not 1<scale<=3:
            raise ValueError('refinement_crop_scale 必须在 (1,3] 内')
        self._rescue_cursor=0
        self._rescue_frame_number=0
        self._task_options=dict(base_options=mp.tasks.BaseOptions(model_asset_path=str(path.resolve()),delegate=mp.tasks.BaseOptions.Delegate.CPU),
            num_hands=num_hands,min_hand_detection_confidence=min_hand_detection_confidence,
            min_hand_presence_confidence=min_hand_presence_confidence,min_tracking_confidence=min_tracking_confidence)
        # 只使用官方 Tasks CPU 接口；检测/存在/跟踪分数作用于不同阶段，均不是三维精度。
        options=mp.tasks.vision.HandLandmarkerOptions(**self._task_options,running_mode=getattr(mp.tasks.vision.RunningMode,mode))
        self.landmarker=mp.tasks.vision.HandLandmarker.create_from_options(options)
        self.roi=None
        try:self.set_roi(roi)
        except Exception:self.close();raise

    def set_roi(self,roi):
        value=validate_roi(roi)
        if value==self.roi:return
        # 改变视频裁剪坐标后不能沿用旧视频跟踪状态；先创建成功再替换，失败保留旧实例。
        if self.mode=='VIDEO':
            options=self.mp.tasks.vision.HandLandmarkerOptions(**self._task_options,running_mode=self.mp.tasks.vision.RunningMode.VIDEO)
            replacement=self.mp.tasks.vision.HandLandmarker.create_from_options(options)
            previous=self.landmarker;self.landmarker=replacement;previous.close();self.last_ms=None
        self.roi=value

    # IMAGE 逐帧独立推理；VIDEO 要求严格递增的毫秒时间，回放跳转后须重建实例。
    def detect(self,image,timestamp_ns=0):
        if getattr(self,'roi',None) is None:return self._detect_with_rescue(image,timestamp_ns)
        box=roi_pixel_box(self.roi,image.shape);x,y,w,h=box
        detections=self._detect_with_rescue(np.ascontiguousarray(image[y:y+h,x:x+w]),timestamp_ns)
        result=[]
        for d in detections:
            restored=restore_crop_detection(d,box,image.shape)
            restored.valid&=(d.pixel_landmarks>=0).all(axis=1)&(d.pixel_landmarks<[w,h]).all(axis=1)
            restored.origin=d.origin+'_manual_roi'
            result.append(restored)
        self.last_detection_diagnostics.update(manual_roi=list(self.roi),pixel_roi=list(box),
            roi_coordinate_space='full_detector_input_image',method='manual_roi/'+self.last_detection_diagnostics['method'])
        return result

    def _detect_with_rescue(self,image,timestamp_ns=0):
        hands=self._detect_single(image,timestamp_ns)
        cfg=self.rescue_config
        full_count=len(hands)
        self._rescue_frame_number=getattr(self,'_rescue_frame_number',0)+1
        detail={'method':'full_frame','full_frame_hands':len(hands),'crop_attempts':0,
                'rescue_hands':0,'rescue_config':dict(cfg),'used_ground_truth':False}
        self.last_detection_diagnostics=detail
        if cfg.get('refine_full_frame',False) and hands:
            hands=self._refine_full_frame(image,hands,timestamp_ns,detail)
        # 完整保留本帧全图观测，只追加当前真实裁剪的新观测。补检不生成缺失点，
        # VIDEO 主图的时间轴不重交；跨裁剪去重不依赖可能翻转的手侧标签。
        if (not cfg['enabled'] or len(hands)>=self.num_hands
                or min(image.shape[:2])<cfg['min_image_side_px']):return hands
        if hands:
            if not cfg.get('retry_partial',False):return hands
            interval=cfg.get('partial_retry_interval_frames',1)
            if (self._rescue_frame_number-1)%interval:
                detail['partial_retry_deferred']=True
                return hands
        height,width=image.shape[:2];cw=int(round(width*cfg['crop_fraction']));ch=int(round(height*cfg['crop_fraction']))
        tiles=[(x,y) for y in (0,height-ch) for x in (0,width-cw)]
        count=cfg['max_crops_per_frame'];start=getattr(self,'_rescue_cursor',0)
        self._rescue_cursor=(start+count)%len(tiles)
        for tile in range(count):
            x,y=tiles[(start+tile)%len(tiles)]
            crop=np.ascontiguousarray(image[y:y+ch,x:x+cw]);detail['crop_attempts']+=1
            for d in self._detect_single(crop,timestamp_ns,image_mode=True):
                inside=d.valid&np.isfinite(d.pixel_landmarks).all(axis=1)&(d.pixel_landmarks>=0).all(axis=1)&(d.pixel_landmarks<[cw,ch]).all(axis=1)
                if inside.sum()<cfg['min_inside_joints']:continue
                candidate=restore_crop_detection(d,(x,y,cw,ch),image.shape)
                candidate.valid&=inside
                if not any(same_hand_pixels(candidate,old,cfg['duplicate_distance_fraction']) for old in hands):hands.append(candidate)
                if len(hands)>=self.num_hands:break
            if len(hands)>=self.num_hands:break
        detail.update(method='full_frame_then_overlapping_tiles',rescue_hands=len(hands)-full_count)
        return hands

    def _refine_full_frame(self,image,hands,timestamp_ns,detail):
        """Current-frame crop inference; retain originals and reject ambiguous replacements."""
        cfg=self.rescue_config;height,width=image.shape[:2]
        detail.update(method='full_frame_then_hand_roi',full_frame_observations=list(hands),
            refinement_attempts=0,refined_hands=0,refinement=[])
        output=[]
        for index,original in enumerate(hands):
            points=original.pixel_landmarks
            inside=original.valid&np.isfinite(points).all(axis=1)&(points>=0).all(axis=1)&(points<[width,height]).all(axis=1)
            note={'index':index,'status':'insufficient_inside_original_points'}
            detail['refinement'].append(note)
            if inside.sum()<cfg['min_inside_joints']:
                output.append(original);continue
            low=points[inside].min(axis=0);high=points[inside].max(axis=0)
            side=max(cfg['min_image_side_px'],int(np.ceil(max(high-low)*cfg['refinement_crop_scale'])))
            side=min(side,height,width)
            x=int(np.clip(np.floor((low[0]+high[0]-side)/2),0,width-side))
            y=int(np.clip(np.floor((low[1]+high[1]-side)/2),0,height-side))
            note.update(box=[x,y,side,side],status='no_unique_same_hand_crop_observation')
            detail['refinement_attempts']+=1
            matches=[]
            for detected in self._detect_single(np.ascontiguousarray(image[y:y+side,x:x+side]),timestamp_ns,image_mode=True):
                valid=detected.valid&np.isfinite(detected.pixel_landmarks).all(axis=1)&(detected.pixel_landmarks>=0).all(axis=1)&(detected.pixel_landmarks<[side,side]).all(axis=1)
                if valid.sum()<cfg['min_inside_joints']:continue
                restored=restore_crop_detection(detected,(x,y,side,side),image.shape)
                restored.valid&=valid
                # Contextual full-image identity stays fixed. Never choose a
                # replacement if it could refer to another visible full-image hand.
                owners=[i for i,hand in enumerate(hands) if same_hand_pixels(restored,hand,cfg['duplicate_distance_fraction'])]
                if owners==[index]:matches.append(restored)
            if len(matches)==1:
                refined=matches[0]
                note.update(status='refined',crop_handedness=refined.handedness,
                    crop_handedness_score=refined.handedness_score,output_label_source='original_full_frame_context')
                output.append(replace(refined,handedness=original.handedness,
                    handedness_score=original.handedness_score,origin='mediapipe_roi_refinement'))
                detail['refined_hands']+=1
            else:output.append(original)
        return output

    def _detect_single(self,image,timestamp_ns=0,image_mode=False):
        # VIDEO 主图只提交一次时间戳；裁剪补救使用独立 IMAGE 实例，不破坏视频时间轴。
        if self.closed: raise RuntimeError("检测器已关闭")
        rgb=bgr_to_rgb(image)
        mp_image=self.mp.Image(image_format=self.mp.ImageFormat.SRGB,data=rgb)
        if self.mode=="IMAGE" or image_mode:
            task=self.landmarker
            if self.mode!='IMAGE':
                if self._image_landmarker is None:
                    options=self.mp.tasks.vision.HandLandmarkerOptions(**self._task_options,running_mode=self.mp.tasks.vision.RunningMode.IMAGE)
                    self._image_landmarker=self.mp.tasks.vision.HandLandmarker.create_from_options(options)
                task=self._image_landmarker
            result=task.detect(mp_image)
        else:
            # 原始 API 时间为 ns，Tasks VIDEO 接口用 ms；同一毫秒不能重复提交。
            timestamp_ms=timestamp_ns//1_000_000
            if self.last_ms is not None and timestamp_ms<=self.last_ms:
                raise ValueError("VIDEO 模式需要严格递增的毫秒时间戳；跳转回放后须重建检测器")
            self.last_ms=timestamp_ms
            result=self.landmarker.detect_for_video(mp_image,timestamp_ms)
        h,w=image.shape[:2]
        hands=[]
        for i,points in enumerate(result.hand_landmarks):
            xyz=np.array([[p.x,p.y,p.z] for p in points],dtype=np.float64)
            # 归一化 x/y 乘真实图像宽高还原像素；归一化 z 不参与双目三角化。
            uv=xyz[:,:2]*[w,h]
            label=result.handedness[i][0]
            world=np.array([[p.x,p.y,p.z] for p in result.hand_world_landmarks[i]],dtype=np.float64)
            hands.append(HandDetection(label.category_name,label.score,xyz,uv,world,bounding_box=(*uv.min(axis=0),*uv.max(axis=0)),timestamp_ns=timestamp_ns,valid=np.isfinite(uv).all(axis=1)))
        return hands

    # 释放原生模型资源；用 closed 保证重复关闭安全。
    def close(self):
        if not self.closed:
            self.landmarker.close()
            if self._image_landmarker is not None:self._image_landmarker.close()
            self.closed=True
