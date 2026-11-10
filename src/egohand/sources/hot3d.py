# HOT3D 数据源/GUI 运行桥接；原始录制保留原生 FISHEYE624 灰度图，重建视图另行转换。
# GT 显示和算法预测是独立模式，禁止把官方姿态点交给预测引擎补全。

"""Read-only HOT3D Clips source and runtime bridge for the existing GUI.

Raw frames retain native FISHEYE624 pixels. The processor prepares virtual
stereo views, keeps official GT outside the prediction engine, and returns XYZ
in the original 1201-1 optical frame. No Torch import or asset conversion here.
"""
from dataclasses import replace
import json
from pathlib import Path, PurePosixPath
import tarfile
import time
import cv2
import numpy as np
from .base import StereoDataSource
from ..core.types import SourceMetadata,StereoFrame,StereoHandResult,HandDetection,Hand3D,serializable
from ..benchmarks.hot3d import HOT3DStereoGeometry,CANONICAL_MP,MAPPING_VERSION
from ..benchmarks.provenance import sha256

FORMAT_SOURCE='https://github.com/facebookresearch/hot3d/blob/main/hot3d/clips/README.md'


# 只读 tar 或官方扁平解包目录；拒绝重复文件、路径穿越和不完整灰度配对。
class _ClipReader:
    def __init__(self,path):
        self.path=Path(path);self.archive=tarfile.open(path) if self.path.is_file() else None
        self.names=self.archive.getnames() if self.archive else [p.name for p in self.path.iterdir() if p.is_file()]
        if len(self.names)!=len(set(self.names)):self.close();raise ValueError('HOT3D 包含重复文件名')
        for name in self.names:
            p=PurePosixPath(name)
            if p.is_absolute() or '..' in p.parts or len(p.parts)!=1:
                self.close();raise ValueError('HOT3D 需要官方扁平文件结构：'+name)
        self.frames=sorted(n.removesuffix('.info.json') for n in self.names if n.endswith('.info.json'))
        if not self.frames:self.close();raise ValueError('没有找到 HOT3D *.info.json')
        for frame in self.frames:
            for suffix in ('.cameras.json','.image_1201-1.jpg','.image_1201-2.jpg'):
                if frame+suffix not in self.names:self.close();raise ValueError('HOT3D 双目文件缺失：'+frame+suffix)
    def read(self,name):
        if name not in self.names:raise ValueError('HOT3D 文件缺失：'+name)
        if self.archive:
            member=self.archive.getmember(name)
            if not member.isfile():raise ValueError('不支持非普通文件：'+name)
            with self.archive.extractfile(member) as stream:return stream.read()
        target=(self.path/name).resolve()
        if not target.is_relative_to(self.path.resolve()):raise ValueError('HOT3D 文件超出所选目录')
        return target.read_bytes()
    def json(self,name):return json.loads(self.read(name))
    def close(self):
        if self.archive:self.archive.close();self.archive=None


# 对外提供逐帧原生图像/时间/标注来源，选用 1201-1 与 1201-2，排除 RGB214-1。
class HOT3DSource(StereoDataSource):
    def __init__(self,path,loop=False,view_mode='ground_truth',toolkit_root=None):
        self.path=Path(path);self.loop=loop;self.view_mode=view_mode
        self.toolkit_root=Path(toolkit_root) if toolkit_root else None
        self.index=0;self.running=False;self.reader=None;self.reader_index=None;self.fps=30
        self.rows=[];self.clips=[];self.metadata=None

    def open(self):
        self.close();self.rows=[];self.clips=[]
        if not self.path.exists():raise FileNotFoundError('HOT3D 路径不存在：'+str(self.path))
        candidates=(self.path.parent,*self.path.parents) if self.path.is_file() else (self.path,*self.path.parents)
        self.root=next((p for p in candidates if (p/'official_file_metadata.json').is_file()),self.path.parent if self.path.is_file() else self.path)
        fallback=Path(__file__).resolve().parents[3]/'samples/date/hot3d/official-hand-toolkit'
        self.toolkit_root=self.toolkit_root or (self.root/'official-hand-toolkit' if (self.root/'official-hand-toolkit').is_dir() else fallback)
        metadata=self.root/'official_file_metadata.json'
        official=json.loads(metadata.read_text(encoding='utf8')) if metadata.exists() else []
        if self.path.is_file():
            if self.path.suffix.lower()!='.tar':raise ValueError('请选择 HOT3D clip-XXXXXX.tar 或片段目录')
            targets=[self.path]
        elif list(self.path.glob('*.cameras.json')):
            sibling=self.path.with_suffix('.tar');targets=[sibling if sibling.is_file() else self.path]
        else:
            targets=sorted(self.path.rglob('clip-*.tar'))
            for directory in sorted(self.path.rglob('clip-*')):
                if directory.is_dir() and list(directory.glob('*.cameras.json')) and not directory.with_suffix('.tar').is_file():targets.append(directory)
        if not targets:raise ValueError('目录内没有 HOT3D Clips tar 或已解包片段')
        for clip_index,target in enumerate(targets):
            digest=sha256(target) if target.is_file() else None
            known=[r for r in official if Path(r['path']).name==target.name]
            record=next((r for r in official if r.get('lfs',{}).get('oid')==digest),None) if digest else None
            if known and not record:raise ValueError('HOT3D 文件哈希与官方记录不符：'+target.name)
            clip={'path':target,'sha256':digest,'official':record,'gt':{},'gt_reason':'没有经过验证的官方 UmeTrack 转换缓存'}
            with_reader=_ClipReader(target)
            try:
                clip['frames']=with_reader.frames
                self._load_ground_truth(clip)
                self.rows.extend({'clip':clip_index,'key':fid} for fid in clip['frames'])
            finally:with_reader.close()
            self.clips.append(clip)
        self.metadata=SourceMetadata('hot3d_clips',real_stereo=True,synchronized=False,
            real_calibration=True,metric_capable=True,ground_truth=any(c['gt'] for c in self.clips),
            synchronization_evidence=FORMAT_SOURCE,timestamp_kind='dataset_sensor',
            extra={'dataset_id':'hot3d','frame_count':len(self.rows),'native_camera_model':'FISHEYE624',
                'streams':['1201-1','1201-2'],'sync_quality':'DATASET_DOCUMENTED',
                'hardware_sync_precision_verified':False,'source_path':str(self.path.resolve()),
                'clips':[{'name':c['path'].stem,'frames':len(c['frames']),'gt_available':bool(c['gt']),
                    'gt_reason':c['gt_reason'],'official_path':c['official']['path'] if c['official'] else None} for c in self.clips]})
        self.index=0

    # 真值来自已审计的官方手模型转换缓存；缺失时保留不可用状态，不从参数向量猜 XYZ。
    def _load_ground_truth(self,clip):
        record=clip['official']
        if record and record['path'].startswith('test_'):
            clip['gt_reason']='官方测试集没有公开 hands.json；不提供三维真值';return
        if not record:
            clip['gt_reason']='缺少可核验的原始 tar / 官方哈希；仅支持图像与预测';return
        cache=self.root/'prepared'/clip['path'].stem/'umetrack_world.json'
        if not cache.is_file():return
        try:
            data=json.loads(cache.read_text(encoding='utf8'))
            if data.get('archive_sha256')!=clip['sha256'] or data.get('model')!='official_umetrack' or data.get('unit')!='m' or data.get('coordinate_frame')!='world':raise ValueError('转换缓存与原始数据、单位或坐标不一致')
            hashes=data.get('official_code_sha256',{})
            if len(hashes)<2 or any(sha256(self.toolkit_root/name)!=value for name,value in hashes.items()):raise ValueError('官方模型代码摘要不一致，请重新转换')
            for fid,hands in data['frames'].items():
                if fid not in clip['frames']:raise ValueError('真值帧号超出片段')
                for side,points in hands.items():
                    a=np.asarray(points,dtype=float)
                    if side not in ('left','right') or a.shape!=(20,3) or not np.isfinite(a).all():raise ValueError('真值必须为官方 canonical20 有限三维点')
            clip['gt']=data['frames'];clip['gt_reason']=None;clip['gt_cache_sha256']=sha256(cache)
        except (ValueError,KeyError,OSError) as error:clip['gt_reason']='真值缓存不可用：'+str(error)

    def get_calibration(self):
        # Native, per-frame FISHEYE624 cannot be described by StereoCalibration.
        return None
    def get_metadata(self):return self.metadata
    def start(self):self.running=True
    def stop(self):self.running=False
    def close(self):
        self.stop()
        if self.reader:self.reader.close()
        self.reader=None;self.reader_index=None
    def seek(self,index):
        if not 0<=index<len(self.rows):raise IndexError(index)
        self.index=index
    def read(self):
        if not self.running:return None
        if self.index>=len(self.rows):
            if not self.loop:return None
            self.index=0
        ordinal=self.index;row=self.rows[ordinal];self.index+=1;clip=self.clips[row['clip']]
        if self.reader_index!=row['clip']:
            if self.reader:self.reader.close()
            self.reader=_ClipReader(clip['path']);self.reader_index=row['clip']
        key=row['key'];info=self.reader.json(key+'.info.json');cameras=self.reader.json(key+'.cameras.json')
        images=[]
        for stream in ('1201-1','1201-2'):
            if cameras[stream]['calibration']['projection_model_type']!='CameraModelType.FISHEYE624':raise ValueError('当前 HOT3D GUI 适配只支持官方 FISHEYE624')
            img=cv2.imdecode(np.frombuffer(self.reader.read(key+'.image_'+stream+'.jpg'),np.uint8),cv2.IMREAD_UNCHANGED)
            c=cameras[stream]['calibration']
            if img is None or img.ndim!=2 or img.shape!=(c['image_height'],c['image_width']):raise ValueError('HOT3D 灰度图尺寸/通道不符合标定：'+stream)
            images.append(cv2.cvtColor(img,cv2.COLOR_GRAY2BGR))
        timestamps=info['image_timestamps_ns'];left_ts=int(timestamps['1201-1']);right_ts=int(timestamps['1201-2'])
        hot={'sequence_id':clip['path'].stem,'frame_key':key,'source_path':str(clip['path'].resolve()),
            'archive_sha256':clip['sha256'],'official_hash_verified':bool(clip['official']),
            'official_path':clip['official']['path'] if clip['official'] else None,
            'toolkit_root':str(self.toolkit_root.resolve()),'cameras':cameras,'info':info,
            'gt_world_m':clip['gt'].get(key,{}),'gt_model':'official_umetrack' if clip['gt'].get(key) else None,
            'gt_cache_sha256':clip.get('gt_cache_sha256'),'gt_unavailable_reason':clip['gt_reason'],
            'joint_mapping_version':MAPPING_VERSION,'streams':['1201-1','1201-2']}
        return StereoFrame(ordinal,left_ts,*images,left_ts,right_ts,source_type='hot3d_clips',
            image_geometry='raw_fisheye624',metadata={'real_stereo':True,'synthetic':False,
                'timestamp_kind':'dataset_sensor','sync_quality':'DATASET_DOCUMENTED',
                'synchronization_evidence':FORMAT_SOURCE,'hot3d':hot})


# 源负责读原生数据，此桥接负责官方模型视图转换、预测/GT 分流与坐标回转。
class HOT3DFrameProcessor:
    def __init__(self):self.geometry=None;self.geometry_key=None;self.sequence=None

    # 以当前帧完整标定作为缓存键；Aria 片段内标定更新必须重建几何映射。
    def prepare(self,frame):
        hot=frame.metadata['hot3d']
        key=json.dumps([hot['cameras'][s]['calibration'] for s in ('1201-1','1201-2')],sort_keys=True)
        key+=hot['toolkit_root']
        if self.geometry is None or self.geometry_key!=key:
            self.geometry=HOT3DStereoGeometry(hot['cameras'],hot['toolkit_root']);self.geometry_key=key
        g=self.geometry
        left,right=g.warp([frame.left_image[:,:,0],frame.right_image[:,:,0]])
        virtual=replace(frame,left_image=left,right_image=right,calibration_id=g.calibration.calibration_id,
            image_geometry='rectified',metadata={**frame.metadata,'rectification_id':g.calibration.calibration_id})
        # Runtime-only image references; not fields in JSON/recording metadata.
        virtual.debug_native_images=(frame.left_image,frame.right_image)
        # Display-only GT projections. Never passed as detector observations.
        truth=[{},{}]
        for side,world in hot.get('gt_world_m',{}).items():
            xyz=g.world_to_virtual(world,hot['cameras']['1201-1']['T_world_from_camera'])
            for camera in (0,1):
                uv,valid=g.project_virtual(xyz,camera)
                full=np.full((21,2),np.nan);mask=np.zeros(21,bool)
                full[list(CANONICAL_MP)]=uv;mask[list(CANONICAL_MP)]=valid
                truth[camera][side]=(full,mask)
        virtual.debug_ground_truth_uv=truth
        return g,virtual

    # prediction 仅传模型二维观测；ground_truth 直接显示数据集模型点，标记为非算法输出。
    def process(self,engine,frame,mode):
        if mode not in ('ground_truth','prediction'):raise ValueError('未知 HOT3D 显示模式')
        started=time.perf_counter();hot=frame.metadata['hot3d'];g,virtual=self.prepare(frame)
        if engine.calibration is None or engine.calibration.calibration_id!=g.calibration.calibration_id:engine.set_calibration(g.calibration)
        if self.sequence!=hot['sequence_id']:engine.reset_tracking();self.sequence=hot['sequence_id']
        if mode=='prediction':
            # 官方 FISHEYE624 生成公共虚拟校正图，在显示图上直接检测。
            metadata={k:v for k,v in virtual.metadata.items() if k!='hot3d'}
            predicted_frame=replace(virtual,source_type='benchmark:hot3d',metadata=metadata,
                left_detections=None,right_detections=None)
            result=engine.process(predicted_frame,dataset_pairing={'dataset_id':'hot3d',
                'sync_quality':'DATASET_DOCUMENTED','pair_id':hot['sequence_id']+'/'+hot['frame_key'],'source':FORMAT_SOURCE})
            # 三维与协方差一起回到原始左光学坐标；像素仍是虚拟校正视图，需保存坐标说明。
            rotation=g.R_virtual_left.T
            for h in result.hands:
                for j in h.joints:
                    if j.valid:j.xyz_m=g.virtual_to_original(j.xyz_m)
                    if j.position_covariance_m2 is not None:
                        j.position_covariance_m2=rotation@j.position_covariance_m2@rotation.T
                        j.depth_std_m=float(np.sqrt(max(j.position_covariance_m2[2,2],0)))
            result.diagnostics.update(result_kind='algorithm_prediction',detector_input='virtual_rectified_grayscale',
                pixel_coordinate_frame='virtual_rectified_pixels',output_rotation_native_from_virtual=rotation.tolist())
            result.diagnostics['detector_input_by_view']=['virtual_rectified_manual_roi' if getattr(d,'roi',None) is not None else 'virtual_rectified_full_frame' for d in (engine.detectors or [])]
            result.diagnostics['detection_rescue']=[getattr(d,'last_detection_diagnostics',{}) for d in (engine.detectors or [])]
            result.source_type='hot3d_clips'
            engine.last_frame=replace(engine.last_frame,source_type='hot3d_clips',metadata=virtual.metadata)
            # Tracking stores independent virtual-frame arrays; returned XYZ is native.
        else:
            result=StereoHandResult(frame.frame_id,frame.timestamp_ns,g.calibration.calibration_id,
                'hot3d_clips',session_id=engine.session_id)
            views=[[],[]]
            for side,world in hot.get('gt_world_m',{}).items():
                points=np.asarray(world,dtype=float)
                if points.shape!=(20,3) or not np.isfinite(points).all():raise ValueError('不接受无法解析的官方三维真值')
                xyz=g.world_to_virtual(points,hot['cameras']['1201-1']['T_world_from_camera'])
                hand=Hand3D(side.title(),None,1 if side=='left' else 2,tracking_status='dataset_ground_truth')
                projected=[]
                for camera in (0,1):
                    uv,valid=g.project_virtual(xyz,camera)
                    pixels=np.full((21,2),np.nan);pixels[list(CANONICAL_MP)]=uv
                    mask=np.zeros(21,bool);mask[list(CANONICAL_MP)]=valid
                    projected.append((pixels,mask))
                    views[camera].append(HandDetection(side.title(),1.,pixels/[g.calibration.image_width,g.calibration.image_height],pixels,
                        valid=mask,origin='official_umetrack_gt_projection'))
                for canonical,joint_id in enumerate(CANONICAL_MP):
                    j=hand.joints[joint_id];j.xyz_m=g.virtual_to_original(xyz[canonical]);j.valid=True
                    j.status='DATASET_GROUND_TRUTH';j.uncertainty_status='NOT_APPLICABLE_GROUND_TRUTH'
                    j.left_uv_px=tuple(projected[0][0][joint_id]) if projected[0][1][joint_id] else None
                    j.right_uv_px=tuple(projected[1][0][joint_id]) if projected[1][1][joint_id] else None
                hand.valid_joint_count=20;result.hands.append(hand)
            result.left_detections,result.right_detections=views
            available=bool(result.hands)
            result.reconstruction_status='Official Ground Truth' if available else 'Ground Truth Unavailable'
            result.diagnostics={'result_kind':'official_ground_truth' if available else 'ground_truth_unavailable',
                'left_detection_origin':'official_ground_truth','right_detection_origin':'official_ground_truth',
                'synthetic':False,'mean_reprojection_error_px':None,'metric_valid':False,
                'reason':None if available else hot.get('gt_unavailable_reason') or '该帧没有官方手部标注',
                'hardware_sync_precision_verified':False,'sync_quality':'DATASET_DOCUMENTED',
                'sync_error_ms':abs(frame.left_timestamp_ns-frame.right_timestamp_ns)/1e6,
                'quality_score_formula':'not applicable to official ground truth',
                'input_geometry':'raw_fisheye624','output_geometry':'rectified'}
            engine.last_frame=virtual
        result.diagnostics.update(hot3d_sequence_id=hot['sequence_id'],hot3d_frame_key=hot['frame_key'],
            ground_truth_available=bool(hot.get('gt_world_m')),ground_truth_model=hot.get('gt_model'),
            joint_mapping_version=MAPPING_VERSION,gt_unavailable_reason=hot.get('gt_unavailable_reason'),
            geometry=g.provenance(),official_hash_verified=hot['official_hash_verified'],
            input_domain='grayscale_replicated_to_rgb',pixel_coordinate_frame='virtual_rectified_pixels',
            real_stereo=True,hardware_sync_precision_verified=False)
        result.processing_time_ms=(time.perf_counter()-started)*1000
        overlay=[]
        for world in hot.get('gt_world_m',{}).values():
            points=np.asarray(world,dtype=float)
            if points.shape==(20,3) and np.isfinite(points).all():
                virtual_xyz=g.world_to_virtual(points,hot['cameras']['1201-1']['T_world_from_camera'])
                native=np.asarray([g.virtual_to_original(p) for p in virtual_xyz])
                overlay.append({'xyz_m':native.tolist(),'valid':np.isfinite(native).all(axis=1).tolist(),'mediapipe_indices':list(CANONICAL_MP)})
        engine.last_frame.metadata={**engine.last_frame.metadata,'ground_truth_left_camera':overlay}
        engine.last_frame.debug_native_images=(frame.left_image,frame.right_image)
        engine.last_frame.debug_ground_truth_uv=virtual.debug_ground_truth_uv
        return result
