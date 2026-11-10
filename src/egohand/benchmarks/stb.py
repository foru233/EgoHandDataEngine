# STB 只配 BB_left/BB_right 同序号，读取 BB 三维标签，毫米转米；不把 SK_color 混作双目。

"""Strict STB BB-pair loader. No inferred R/t or silently invented timestamps."""
import json
import re
from pathlib import Path
import cv2
import numpy as np
from scipy.io import loadmat
from .types import BenchmarkSample,JointMapping,BenchmarkBlocked
from .provenance import sha256,verify_projection_checks,calibration_fingerprint
from ..calibration.loader import load_calibration
from ..detection.landmarks import JOINT_NAMES

STB_NAMES=('palm_center','little_mcp','little_pip','little_dip','little_tip','ring_mcp','ring_pip','ring_dip','ring_tip','middle_mcp','middle_pip','middle_dip','middle_tip','index_mcp','index_pip','index_dip','index_tip','thumb_mcp','thumb_pip','thumb_dip','thumb_tip')
STB_SOURCE='https://github.com/zhjwustc/icip17_stereo_hand_pose_dataset'

# 明确掌心不等于腕点；仅17个语义明确关节参与比较，未确认拇指关节保持排除。
def stb_joint_mapping():
    result=[]
    for i,name in enumerate(STB_NAMES):
        target=17+i-1 if 1<=i<=4 else 13+i-5 if 5<=i<=8 else 9+i-9 if 9<=i<=12 else 5+i-13 if 13<=i<=16 else 4 if i==20 else None
        reason='same named finger joint / fingertip in official list' if target is not None else 'palm_center is not wrist' if i==0 else 'official thumb PIP/DIP terminology does not establish MediaPipe CMC/MCP/IP equivalence'
        result.append(JointMapping(i,name,target,JOINT_NAMES[target] if target is not None else None,target is not None,reason))
    return tuple(result)

# 没有独立标定/坐标核对时 calibration=None，保留 CALIBRATION_MAPPING_UNVERIFIED。
class STBBenchmarkAdapter:
    dataset_id='stb'
    def __init__(self,root,sequence=None,max_frames=200,frame_ids=None):
        self.root=Path(root);self.sequence=sequence;self.max_frames=max_frames;self.frame_ids=frame_ids
        self.calibration=None;self.calibration_status='CALIBRATION_MAPPING_UNVERIFIED';self.calibration_evidence={}
        sidecar=self.root/'benchmark_calibration.json'
        if sidecar.exists():
            evidence=json.loads(sidecar.read_text(encoding='utf-8'))
            c=load_calibration(self.root/evidence['calibration_file'])
            ok,details=verify_projection_checks(c,evidence)
            if ok and evidence.get('ground_truth_frame')=='camera_left_optical' and evidence.get('image_geometry')=='raw' and evidence.get('source'):
                self.calibration=c;self.calibration_status='VERIFIED_BY_INDEPENDENT_PROJECTION_CHECKS'
            self.calibration_evidence={**evidence,'verification':details}
        self.provenance={'dataset_id':'stb','source':STB_SOURCE,'official_source':STB_SOURCE,'sync_quality':'DATASET_DOCUMENTED','sync_provenance':'official BB_left / BB_right same-index pairs; no per-frame hardware timestamps in available README','calibration_status':self.calibration_status,'calibration_fingerprint':calibration_fingerprint(self.calibration),'joint_mapping_version':'stb-mediapipe-conservative-1','comparable_joint_count':17,'excluded_dataset_joints':[0,17,18,19],'timestamp_precision_verified':False}
    # 逐帧读取同序列左右图和 handPara；不人为产生左右硬件同步时间戳。
    def iter_samples(self):
        image_root=self.root/'images'
        seqs=[image_root/self.sequence] if self.sequence else sorted(p for p in image_root.glob('*') if p.is_dir())
        if not seqs or not any(p.is_dir() for p in seqs):raise BenchmarkBlocked('STB official BB image sequences not available; see prepare_stb.py and STB_INTEGRATION.md')
        count=0
        for directory in seqs:
            label=self.root/'labels'/f'{directory.name}_BB.mat'
            if not label.is_file():raise BenchmarkBlocked(f'Official BB labels missing: {label}')
            gt=np.asarray(loadmat(label).get('handPara'),dtype=np.float64)
            if gt.ndim!=3 or gt.shape[:2]!=(3,21):raise ValueError('STB handPara must have shape 3 x 21 x frames')
            lefts=sorted(directory.glob('BB_left_*'),key=lambda p:int(re.search(r'BB_left_(\d+)',p.name).group(1)))
            for left in lefts:
                index=int(re.search(r'BB_left_(\d+)',left.name).group(1))
                if self.frame_ids is not None and f'{directory.name}/{index}' not in self.frame_ids:continue
                right=directory/left.name.replace('BB_left_','BB_right_',1)
                if not right.is_file():raise ValueError(f'Unpaired official stereo image: {right}')
                if index>=gt.shape[2]:raise ValueError('STB frame index exceeds handPara; verify zero/one indexing explicitly')
                a,b=cv2.imread(str(left)),cv2.imread(str(right))
                if a is None or b is None or a.shape!=b.shape:raise ValueError('STB images cannot be decoded or dimensions differ')
                provenance={**self.provenance,'left_path':str(left.resolve()),'right_path':str(right.resolve()),'left_sha256':sha256(left),'right_sha256':sha256(right),'labels_sha256':sha256(label),'ground_truth_unit_original':'mm','original_frame_index':index,'pair_id':f'{directory.name}/{index}','image_size_actual':[a.shape[1],a.shape[0]]}
                xyz=gt[:,:,index].T*.001
                yield BenchmarkSample('stb',directory.name,str(index),a,b,self.calibration,xyz,'camera_left_optical' if self.calibration else 'STB_BB_FRAME_UNVERIFIED',STB_NAMES,stb_joint_mapping(),np.isfinite(xyz).all(axis=1),provenance)
                count+=1
                if count>=self.max_frames:return
        if not count:raise BenchmarkBlocked('No BB_left/BB_right pairs match the fixed selection')
