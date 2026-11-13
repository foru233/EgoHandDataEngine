# HOT3D 离线适配与官方 FISHEYE624 几何；虚拟相机参数在 HOT3DStereoGeometry.__init__ 签名前部。
# width/height 是像素尺寸，focal_px 是虚拟焦距；改变会影响视域与手部采样，需单独记录评测。

"""HOT3D-Clips: official FISHEYE624 projection and explicit virtual stereo.

The runtime engine remains a pinhole/OpenCV-fisheye engine. Native FISHEYE624
images are resampled using the official model into a declared virtual pair;
they are never sent to OpenCV's four-coefficient fisheye implementation.
"""
from dataclasses import asdict,replace
import hashlib
import importlib
import json
from pathlib import Path
import sys
import tarfile
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from ..calibration.schema import StereoCalibration
from ..calibration.rectifier import StereoRectifier
from ..detection.landmarks import JOINT_NAMES
from .types import BenchmarkSample, BenchmarkBlocked, JointMapping
from .provenance import sha256,calibration_fingerprint

TOOLKIT_REVISION='9064a2fe7fda51bb1f1451a0d3d0da7f3ef5e64e'
MAPPING_VERSION='hot3d-official-canonical20-to-mp-v1'
# Official MANO_TO_CANONICAL_LANDMARK_MAPPING and UmeTrack's canonical output.
# Thumb CMC is absent. The 21st UmeTrack profile landmark is not canonical FK output.
# 官方 canonical20 对应 MediaPipe 索引；排除 THUMB_CMC=1，不把额外 profile 点冒充该关节。
CANONICAL_MP=(4,8,12,16,20,0,2,3,5,6,7,9,10,11,13,14,15,17,18,19)
HOT3D_MAPPING=tuple(JointMapping(i,JOINT_NAMES[m],m,JOINT_NAMES[m],True,
    'Official canonical20 order, shared by UmeTrack FK and MANO canonical mapping; model landmark correspondence, not anatomical equivalence proof')
    for i,m in enumerate(CANONICAL_MP))


# 恢复顺时针 90° 图像的像素中心坐标：u原=v正向，v原=H-1-u正向。
def restore_clockwise_pixels(uv,native_height):
    """Inverse of np.rot90(native, k=3), using pixel-center coordinates."""
    uv=np.asarray(uv,dtype=float)
    return np.column_stack((uv[:,1],native_height-1-uv[:,0]))


# 仅旋转完整原生图像适配官方正向；检测后恢复像素，不使用 GT 框裁剪。
def detect_native_upright(detector,image,timestamp_ns):
    # Matches official hot3d/clips/vis_clips.py's fixed upright orientation.
    rotated=np.ascontiguousarray(np.rot90(image,k=3))
    ds=detector.detect(cv2.cvtColor(rotated,cv2.COLOR_GRAY2BGR),timestamp_ns)
    result=[]
    for d in ds:
        uv=restore_clockwise_pixels(d.pixel_landmarks,image.shape[0])
        result.append(replace(d,pixel_landmarks=uv,normalized_landmarks=uv/[image.shape[1],image.shape[0]],
            bounding_box=(*uv.min(axis=0),*uv.max(axis=0)),origin=d.origin+'_official_upright_restored_native_pixels'))
    return result


# 数据四元数顺序为 wxyz；SciPy 接收 xyzw，必须显式重排，平移单位为米。
def transform(j):
    q=np.asarray(j['quaternion_wxyz'],dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-4:
        raise ValueError('Invalid quaternion_wxyz')
    T=np.eye(4);T[:3,:3]=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
    T[:3,3]=j['translation_xyz'];return T


def official_camera_module(root):
    root=Path(root).resolve()
    if not (root/'hand_tracking_toolkit/camera.py').is_file():
        raise BenchmarkBlocked('Official hand_tracking_toolkit camera module unavailable')
    for p in (root,root.parent/'runtime'):
        if str(p) not in sys.path:sys.path.insert(0,str(p))
    return importlib.import_module('hand_tracking_toolkit.camera')


# 先调用官方相机模型再构造公共虚拟双目，不把 FISHEYE624 直接当 OpenCV fisheye。
class HOT3DStereoGeometry:
    @property
    def epipolar_axis(self):
        """极线比较轴由内部虚拟双目校正器决定，供回放显示复用。"""
        return self.rectifier.epipolar_axis

    def __init__(self,cameras,toolkit_root,width=640,height=480,focal_px=160.):
        module=official_camera_module(toolkit_root)
        self.native_json={s:cameras[s] for s in ('1201-1','1201-2')}
        for s,j in self.native_json.items():
            if j['calibration']['projection_model_type']!='CameraModelType.FISHEYE624':
                raise BenchmarkBlocked('Unsupported native camera model: '+s)
        self.native=[module.from_json(self.native_json[s]) for s in ('1201-1','1201-2')]
        self.world=[transform(cameras[s]['T_world_from_camera']) for s in ('1201-1','1201-2')]
        device=[transform(cameras[s]['calibration']['T_device_from_camera']) for s in ('1201-1','1201-2')]
        # 从设备外参推左→右变换，并用世界相机位姿独立交叉核对方向和单位。
        relative=np.linalg.inv(device[1])@device[0]
        world_relative=np.linalg.inv(self.world[1])@self.world[0]
        self.extrinsic_check_max_abs=float(np.max(np.abs(relative-world_relative)))
        if self.extrinsic_check_max_abs>1e-5:
            raise BenchmarkBlocked('Device-derived and world-derived stereo extrinsics disagree')
        self.R=relative[:3,:3];self.t=relative[:3,3]
        # 把右相机中心写到左相机坐标；由基线和两路平均光轴建立虚拟相机正交基。
        baseline=-self.R.T@self.t
        self.baseline_m=float(np.linalg.norm(baseline))
        if not .01<self.baseline_m<.5:raise BenchmarkBlocked('Unexpected stereo baseline')
        x=baseline/self.baseline_m
        forward=np.array([0.,0.,1.])+self.R.T@np.array([0.,0.,1.])
        z=forward-x*np.dot(x,forward);z/=np.linalg.norm(z)
        y=np.cross(z,x);y/=np.linalg.norm(y)
        self.R_virtual_left=np.stack((x,y,z))
        if np.linalg.det(self.R_virtual_left)<.999:raise ValueError('Virtual rotation is not proper')
        K=np.array([[focal_px,0,(width-1)/2],[0,focal_px,(height-1)/2],[0,0,1.]])
        fingerprint=hashlib.sha256(json.dumps({'calibration':[j['calibration'] for j in self.native_json.values()],
            'virtual':{'width':width,'height':height,'focal_px':focal_px}},sort_keys=True).encode()).hexdigest()
        self.calibration=StereoCalibration(K,np.zeros(5),K.copy(),np.zeros(5),np.eye(3),
            np.array([-self.baseline_m,0,0]),width,height,'pinhole','hot3d-virtual-'+fingerprint[:16],
            'Official HOT3D FISHEYE624 forward warp; derived virtual parallel cameras; toolkit '+TOOLKIT_REVISION,'m',False)
        self.rectifier=StereoRectifier(self.calibration)
        self.K=self.rectifier.P1[:,:3]
        xx,yy=np.meshgrid(np.arange(width),np.arange(height))
        rays=np.column_stack((xx.ravel(),yy.ravel(),np.ones(width*height)))@np.linalg.inv(self.K).T
        left_rays=rays@self.R_virtual_left
        right_rays=left_rays@self.R.T
        # 用虚拟视线反查原生像素供 remap；原生视角/图像范围外保持黑区，不静默补图。
        self.maps=[];self.valid_masks=[]
        for camera,j,ray in zip(self.native,self.native_json.values(),(left_rays,right_rays)):
            uv=camera.eye_to_window(ray)
            angle=np.arctan2(np.linalg.norm(ray[:,:2],axis=1),ray[:,2])
            mask=(ray[:,2]>0)&(angle<j['calibration']['max_solid_angle'])&np.isfinite(uv).all(axis=1)
            mask&=(uv[:,0]>=0)&(uv[:,0]<camera.width-1)&(uv[:,1]>=0)&(uv[:,1]<camera.height-1)
            uv[~mask]=-1
            self.maps.append((uv[:,0].reshape(height,width).astype(np.float32),uv[:,1].reshape(height,width).astype(np.float32)))
            self.valid_masks.append(mask.reshape(height,width))

        # 两个独立针孔视图仅旋转至直立方向，并不执行双目极线校正。
        upright=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
        detection_rays=rays@upright
        self.detection_maps=[]
        for camera,j in zip(self.native,self.native_json.values()):
            uv=camera.eye_to_window(detection_rays)
            angle=np.arctan2(np.linalg.norm(detection_rays[:,:2],axis=1),detection_rays[:,2])
            mask=(detection_rays[:,2]>0)&(angle<j['calibration']['max_solid_angle'])&np.isfinite(uv).all(axis=1)
            mask&=(uv[:,0]>=0)&(uv[:,0]<camera.width-1)&(uv[:,1]>=0)&(uv[:,1]<camera.height-1)
            uv[~mask]=-1
            self.detection_maps.append((uv[:,0].reshape(height,width).astype(np.float32),uv[:,1].reshape(height,width).astype(np.float32)))
        rotations=(self.R_virtual_left,self.R_virtual_left@self.R.T)
        self.rectifier.undistorted_to_rectified=[self.K@rotation@upright.T@np.linalg.inv(self.K) for rotation in rotations]

    def warp(self,images):
        result=[]
        for image,mapping,cam in zip(images,self.maps,self.native):
            if image.ndim!=2 or image.shape!=(cam.height,cam.width):raise ValueError('Native monochrome dimensions mismatch')
            gray=cv2.remap(image,*mapping,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
            result.append(cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR))
        return result

    def warp_detection(self,images):
        result=[]
        for image,mapping,cam in zip(images,self.detection_maps,self.native):
            if image.ndim!=2 or image.shape!=(cam.height,cam.width):raise ValueError('Native monochrome dimensions mismatch')
            result.append(cv2.cvtColor(cv2.remap(image,*mapping,cv2.INTER_LINEAR),cv2.COLOR_GRAY2BGR))
        return result

    # 先用逆世界位姿转到当前帧原生左相机，再旋转到公共虚拟左相机。
    def world_to_virtual(self,xyz_world,T_world_left):
        T=transform(T_world_left)
        original=(np.asarray(xyz_world)-T[:3,3])@T[:3,:3]
        return original@self.R_virtual_left.T

    def virtual_to_original(self,xyz):return np.asarray(xyz)@self.R_virtual_left

    # 数值逆解官方畸变，仅接受正投影回代误差通过、角度/像域合法的射线。
    def native_pixels_to_virtual(self,uv,side):
        """Invert the OFFICIAL distortion numerically, check forward residual.

        The toolkit supplies forward projection and arctan unprojection, but
        no inverse FISHEYE624 distortion. Failed Newton inversions are invalid.
        """
        cam=self.native[side];uv=np.asarray(uv,dtype=float)
        finite=np.isfinite(uv).all(axis=1);indices=np.flatnonzero(finite)
        out=np.full_like(uv,np.nan);valid=np.zeros(len(uv),bool)
        if not len(indices):return out,valid
        target=(uv[indices]-cam.c)/cam.f;p=target.copy();eps=1e-6
        for _ in range(30):
            residual=cam.distort.evaluate(p.copy())-target
            columns=[]
            for axis in (0,1):
                step=np.zeros_like(p);step[:,axis]=eps
                columns.append((cam.distort.evaluate((p+step).copy())-cam.distort.evaluate((p-step).copy()))/(2*eps))
            J=np.stack(columns,axis=-1)
            determinant=np.linalg.det(J);safe=np.isfinite(J).all(axis=(1,2))&(abs(determinant)>1e-10)
            delta=np.zeros_like(p)
            if safe.any():delta[safe]=np.linalg.solve(J[safe],residual[safe])
            length=np.linalg.norm(delta,axis=1)
            delta*=np.minimum(1,.3/np.maximum(length,1e-12))[:,None]
            p-=delta
            if np.max(np.linalg.norm(residual,axis=1))<1e-12:break
        ray=cam.unproject(p)
        roundtrip=cam.eye_to_window(ray)
        acceptable=np.isfinite(ray).all(axis=1)&(np.linalg.norm(roundtrip-uv[indices],axis=1)<1e-5)
        acceptable&=np.linalg.norm(p,axis=1)<list(self.native_json.values())[side]['calibration']['max_solid_angle']
        acceptable&=(uv[indices]>=0).all(axis=1)&(uv[indices]<[cam.width,cam.height]).all(axis=1)
        virtual_ray=(ray if side==0 else ray@self.R)@self.R_virtual_left.T
        hp=virtual_ray@self.K.T
        projected=hp[:,:2]/hp[:,2,None]
        acceptable&=(virtual_ray[:,2]>0)&np.isfinite(projected).all(axis=1)
        acceptable&=(projected>=0).all(axis=1)&(projected<[self.calibration.image_width,self.calibration.image_height]).all(axis=1)
        good=indices[acceptable];out[good]=projected[acceptable];valid[good]=True
        return out,valid

    # 只变换二维像素和有效掩码；此阶段没有读取三维 GT 修正模型输出。
    def map_native_detections(self,detections,side):
        result=[]
        for d in detections:
            uv,mask=self.native_pixels_to_virtual(d.pixel_landmarks,side)
            result.append(replace(d,pixel_landmarks=uv,normalized_landmarks=uv/[self.calibration.image_width,self.calibration.image_height],
                valid=d.valid&mask,origin='mediapipe_native_fisheye624_mapped_to_virtual'))
        return result

    def project_virtual(self,xyz,side):
        X=np.asarray(xyz).copy()
        if side==1:X[:,0]-=self.baseline_m
        hp=X@self.K.T
        uv=hp[:,:2]/hp[:,2,None]
        valid=np.isfinite(uv).all(axis=1)&(X[:,2]>0)
        valid&=(uv[:,0]>=0)&(uv[:,0]<self.calibration.image_width)&(uv[:,1]>=0)&(uv[:,1]<self.calibration.image_height)
        good=np.flatnonzero(valid)
        if len(good):
            ij=np.rint(uv[good]).astype(int)
            ij[:,0]=np.clip(ij[:,0],0,self.calibration.image_width-1)
            ij[:,1]=np.clip(ij[:,1],0,self.calibration.image_height-1)
            valid[good]&=self.valid_masks[side][ij[:,1],ij[:,0]]
        return uv,valid

    def provenance(self):
        return {'native_camera_model':'CameraModelType.FISHEYE624','official_toolkit_revision':TOOLKIT_REVISION,
            'virtual_image_size':[self.calibration.image_width,self.calibration.image_height],
            'virtual_K':self.K.tolist(),'R_virtual_from_native_left':self.R_virtual_left.tolist(),
            'R_right_left':self.R.tolist(),'t_right_left_m':self.t.tolist(),'baseline_m':self.baseline_m,
            'extrinsic_world_device_max_abs':self.extrinsic_check_max_abs,
            'virtual_valid_image_fraction':[float(m.mean()) for m in self.valid_masks],
            'native_camera_parameters':self.native_json,
            'output_frame':'virtual_left_optical','native_output_transform':'X_native_left = X_virtual_left @ R_virtual_from_native_left',
            'resampling':'official FISHEYE624 forward projection, bilinear, no GT crop or GT-dependent camera',
            'calibration_fingerprint':calibration_fingerprint(self.calibration)}


# 固定时间顺序读取；官方 split 由摘要匹配确认，不信任本地 train/test 文件夹名称。
class HOT3DBenchmarkAdapter:
    def __init__(self,root,max_frames=200,toolkit_root=None,clips=None,frame_ids=None):
        self.root=Path(root);self.max_frames=max_frames
        self.toolkit_root=Path(toolkit_root) if toolkit_root else self.root/'official-hand-toolkit'
        self.clips=set(clips) if clips else None;self.frame_ids=set(frame_ids) if frame_ids else None
        metadata=self.root/'official_file_metadata.json'
        self.official=json.loads(metadata.read_text(encoding='utf8')) if metadata.exists() else []

    # 核对 tar/转换缓存/官方代码摘要和 GT 单位；无可靠真值时不输出虚构 MPJPE。
    def iter_samples(self):
        archives=sorted(self.root.rglob('clip-*.tar')) if self.root.is_dir() else [self.root]
        archives=[p for p in archives if p.is_file() and (self.clips is None or p.stem in self.clips)]
        if not archives:raise BenchmarkBlocked('No HOT3D-Clips tar archives found')
        count=0
        for path in archives:
            digest=sha256(path)
            official=next((r for r in self.official if r.get('lfs',{}).get('oid')==digest),None)
            converted=self.root/'prepared'/path.stem/'umetrack_world.json'
            ground_truth=json.loads(converted.read_text(encoding='utf8')) if converted.exists() else None
            if ground_truth and ground_truth.get('archive_sha256')!=digest:raise BenchmarkBlocked('Stale GT cache: archive hash mismatch')
            if ground_truth and ground_truth.get('model')!='official_umetrack':raise BenchmarkBlocked('GT is not official UmeTrack output')
            if ground_truth:
                if not official:raise BenchmarkBlocked('Archive fingerprint not verified against official HOT3D metadata; cannot claim GT accuracy')
                if ground_truth.get('unit')!='m' or ground_truth.get('coordinate_frame')!='world':
                    raise BenchmarkBlocked('Official FK cache unit/frame contract mismatch')
                if len(ground_truth.get('official_code_sha256',{}))<2:
                    raise BenchmarkBlocked('Official FK code provenance unavailable')
                for source,expected in ground_truth.get('official_code_sha256',{}).items():
                    if sha256(self.toolkit_root/source)!=expected:
                        raise BenchmarkBlocked('Official FK code changed: regenerate audited GT cache')
                for hands in ground_truth['frames'].values():
                    for points in hands.values():
                        a=np.asarray(points,dtype=float)
                        if a.shape!=(20,3) or not np.isfinite(a).all():raise BenchmarkBlocked('Unparsed/nonfinite FK landmarks cannot be GT')
            if official and official['path'].startswith('test_'):ground_truth=None
            with tarfile.open(path) as archive:
                names=archive.getnames()
                if len(set(names))!=len(names):raise BenchmarkBlocked('Duplicate archive members')
                frames=sorted(n.split('.')[0] for n in names if n.endswith('.info.json'))
                geometry=None;geometry_key=None
                for fid in frames:
                    if self.frame_ids is not None and f'{path.stem}/{fid}' not in self.frame_ids:continue
                    read_json=lambda suffix:json.load(archive.extractfile(fid+suffix))
                    cameras=read_json('.cameras.json');info=read_json('.info.json')
                    # Aria online calibration can change within a clip. Rebuild
                    # from the current frame instead of freezing frame-zero R/t/K.
                    key=json.dumps([cameras[s]['calibration'] for s in ('1201-1','1201-2')],sort_keys=True)
                    if geometry is None or key!=geometry_key:
                        geometry=HOT3DStereoGeometry(cameras,self.toolkit_root);geometry_key=key
                    relative=np.linalg.inv(transform(cameras['1201-2']['T_world_from_camera']))@transform(cameras['1201-1']['T_world_from_camera'])
                    if not np.allclose(relative[:3,:3],geometry.R,atol=1e-5) or not np.allclose(relative[:3,3],geometry.t,atol=1e-5):
                        raise BenchmarkBlocked('Time-varying native stereo extrinsic requires separate geometry')
                    images=[];image_hashes=[]
                    for stream in ('1201-1','1201-2'):
                        payload=archive.extractfile(fid+'.image_'+stream+'.jpg').read()
                        image_hashes.append(hashlib.sha256(payload).hexdigest())
                        image=cv2.imdecode(np.frombuffer(payload,np.uint8),cv2.IMREAD_UNCHANGED)
                        if image is None:raise BenchmarkBlocked('Cannot decode '+stream)
                        images.append(image)
                    left,right=geometry.warp(images)
                    hands=read_json('.hands.json') if fid+'.hands.json' in names else {}
                    gt_hands={}
                    if ground_truth:
                        for side,world in ground_truth['frames'].get(fid,{}).items():
                            gt_hands[side]=geometry.world_to_virtual(world,cameras['1201-1']['T_world_from_camera'])
                    first=next(iter(gt_hands.values()),None)
                    provenance={'dataset_id':'hot3d','archive':str(path.resolve()),'archive_sha256':digest,
                        'official_path':official['path'] if official else None,'official_hash_verified':bool(official),
                        'selected_streams':['1201-1','1201-2'],'input_domain':'grayscale_replicated_to_rgb',
                        'sync_quality':'DATASET_DOCUMENTED','source':'https://github.com/facebookresearch/hot3d/blob/main/hot3d/clips/README.md',
                        'sensor_timestamps_provided':True,'hardware_sync_precision_verified':False,
                        'image_sha256':image_hashes,'joint_mapping_version':MAPPING_VERSION,
                        'comparable_joints':20,'excluded_mediapipe_joints':[1],
                        'ground_truth_model':'official_umetrack' if gt_hands else None,
                        'ground_truth_reason':None if gt_hands else 'Official test GT withheld or official FK cache unavailable',
                        'geometry':geometry.provenance()}
                    yield BenchmarkSample('hot3d',path.stem,fid,left,right,geometry.calibration,first,
                        'virtual_left_optical' if gt_hands else None,tuple(JOINT_NAMES[m] for m in CANONICAL_MP),
                        HOT3D_MAPPING,np.ones(20,bool) if first is not None else None,provenance,
                        int(info['image_timestamps_ns']['1201-1']),int(info['image_timestamps_ns']['1201-2']),
                        'rectified',{'hands':hands,'info':info,'cameras':cameras,'gt_hands':gt_hands,
                            'geometry':geometry,'raw_images':images})
                    count+=1
                    if count>=self.max_frames:return
