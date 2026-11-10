"""DexYCB 单序列、两相机 RGB 回放；标定使用官方 toolkit 的针孔模型。"""
from pathlib import Path
import cv2
import numpy as np
import yaml

from .base import StereoDataSource
from ..calibration.schema import StereoCalibration
from ..core.types import SourceMetadata, StereoFrame

OFFICIAL = 'https://dex-ycb.github.io/'
CAMERAS = ('840412060917', '841412060263')  # 默认夹角较小的一对；序列需包含两路。


class DexYCBSource(StereoDataSource):
    def __init__(self, path, loop=False):
        self.path = Path(path)
        self.loop = bool(loop)
        self.running = False
        self.index = 0
        self.rows = []
        self.calibration = None
        self.metadata = None

    def open(self):
        path = self.path.resolve()
        # 可选择数据集根、受试者目录，或单个含 meta.yml 的序列目录。
        candidates = [path] if (path / 'meta.yml').is_file() else sorted(p.parent for p in path.glob('*/meta.yml'))
        if not candidates:
            candidates = sorted(p.parent for p in path.glob('*/*/meta.yml'))
        if not candidates:
            raise ValueError('DexYCB 请选择已解压的数据集根目录、受试者或单个序列目录')
        self.sequence = candidates[0]
        root = next((p for p in (self.sequence, *self.sequence.parents) if (p / 'calibration').is_dir()), None)
        if root is None:
            raise ValueError('DexYCB 缺少官方 calibration 目录，请放在数据集根目录')
        meta = yaml.safe_load((self.sequence / 'meta.yml').read_text(encoding='utf8'))
        self.mano_annotations=[]
        for side,calib in zip(meta.get('mano_sides',[]),meta.get('mano_calib',[])):
            annotation=None
            path=root/'calibration'/('mano_'+str(calib))/'mano.yml'
            if path.is_file() and side in ('left','right'):
                betas=np.asarray(yaml.safe_load(path.read_text(encoding='utf8')).get('betas',[]),float)
                if betas.shape==(10,) and np.isfinite(betas).all():
                    annotation={'side':side,'betas':betas.tolist()}
            self.mano_annotations.append(annotation)
        if any(s not in meta['serials'] for s in CAMERAS):
            raise ValueError('该序列不含默认两路相机：' + ', '.join(CAMERAS))
        ext_path = root / 'calibration' / ('extrinsics_' + str(meta['extrinsics'])) / 'extrinsics.yml'
        extr = yaml.full_load(ext_path.read_text(encoding='utf8'))['extrinsics']
        mats = [np.asarray(extr[s], dtype=float).reshape(3, 4) for s in CAMERAS]
        intr = [yaml.full_load((root / 'calibration' / 'intrinsics' / (s + '_640x480.yml')).read_text(encoding='utf8'))['color'] for s in CAMERAS]
        def K(v): return np.array([[v['fx'], 0, v['ppx']], [0, v['fy'], v['ppy']], [0, 0, 1]])
        # 官方外参是 camera→world，换算 p_right = R p_left + t。
        a, b = mats
        R = b[:, :3].T @ a[:, :3]
        t = b[:, :3].T @ (a[:, 3] - b[:, 3])
        self.calibration = StereoCalibration(K(intr[0]), np.zeros(5), K(intr[1]), np.zeros(5), R, t, 640, 480, 'pinhole', 'dexycb-' + str(meta['extrinsics']), 'DexYCB official calibration; color pinhole as toolkit', 'm', False, rectification_alpha=1.0)
        self.rows = [i for i in range(int(meta['num_frames'])) if all((self.sequence / s / f'color_{i:06d}.jpg').is_file() for s in CAMERAS)]
        if not self.rows: raise ValueError('该序列没有可用的两路同帧 RGB 图像')
        self.index = 0
        self.metadata = SourceMetadata('dexycb', real_stereo=True, synchronized=False, real_calibration=True, metric_capable=True, ground_truth=True, synchronization_evidence=OFFICIAL, timestamp_kind='dataset_frame_index', extra={'frame_count': len(self.rows), 'sequence': str(self.sequence), 'cameras': CAMERAS})

    def get_calibration(self): return self.calibration
    def get_metadata(self): return self.metadata
    def start(self): self.running = True
    def stop(self): self.running = False
    def close(self): self.running = False
    def seek(self, index):
        if not 0 <= index < len(self.rows): raise IndexError(index)
        self.index = index

    def read(self):
        if not self.running: return None
        if self.index >= len(self.rows):
            if not self.loop: return None
            self.index = 0
        idx = self.index
        frame_id = self.rows[idx]
        self.index += 1
        images = [cv2.imread(str(self.sequence / s / f'color_{frame_id:06d}.jpg')) for s in CAMERAS]
        if any(image is None or image.shape != (480, 640, 3) for image in images): raise ValueError('DexYCB 图像解码失败或分辨率不是 640×480')
        # 官方 joint_3d 已在该相机坐标系内，-1 为缺失；不推断与模型关节的索引对应。
        truth=[];mano_truth=[]
        label=self.sequence / CAMERAS[0] / f'labels_{frame_id:06d}.npz'
        if label.is_file():
            with np.load(label,allow_pickle=False) as data:
                if 'pose_m' in data:
                    pose=np.asarray(data['pose_m'],dtype=float)
                    if pose.ndim==2 and pose.shape[1]==51:
                        for params,annotation in zip(pose,self.mano_annotations):
                            if annotation and np.isfinite(params).all() and np.any(params):
                                mano_truth.append({**annotation,'pose':params.tolist()})
                if 'joint_3d' in data:
                    xyz=np.asarray(data['joint_3d'],dtype=float)
                    if xyz.ndim==3 and xyz.shape[1:]==(21,3):
                        for hand in xyz:
                            valid=np.isfinite(hand).all(axis=1)&(hand[:,2]>0)
                            if valid.any():truth.append({'xyz_m':hand.tolist(),'valid':valid.tolist()})
        return StereoFrame(idx, (idx + 1) * 33333333, *images, None, None, self.calibration.calibration_id, 'benchmark:dexycb', False, 'raw', {'real_stereo': True, 'synthetic': False, 'sync_quality': 'DATASET_DOCUMENTED', 'synchronization_evidence': OFFICIAL, 'timestamp_kind': 'playback_ticks_from_dataset_index', 'dataset_pair_id': f'{self.sequence.name}/{frame_id}', 'ground_truth_left_camera':truth, 'ground_truth_mano':mano_truth})
