"""Read-only image cache reuse, independently verifying new replay identity."""
from pathlib import Path
import hashlib,json,sqlite3
import cv2
import numpy as np
from egohand.storage.replay import SavedResultReplay as NativeReplay

def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

class SharedSavedResultReplay(NativeReplay):
    def __init__(self,directory,results):
        root=Path(directory);marker=root/'shared_image_cache.json'
        self.shared_image_source=None
        self.fast_preview=False
        if marker.exists() and not (root/'display_cache.sqlite3').is_file():
            info=json.loads(marker.read_text(encoding='utf8'));source=Path(info['source'])
            if info['schema_version']!=1 or not info['read_only']:raise ValueError('Invalid shared cache contract')
            if digest(root/'results/frames.jsonl')!=info['result_sha256']:raise ValueError('New result fingerprint mismatch')
            if digest(root/'processing_calibration.yaml')!=info['calibration_sha256'] or digest(source/'processing_calibration.yaml')!=info['calibration_sha256']:
                raise ValueError('Shared image calibration mismatch')
            cache_path=source/'display_cache.sqlite3'
            with sqlite3.connect('file:'+cache_path.as_posix()+'?mode=ro',uri=True) as db:
                ids=db.execute('SELECT idx,frame_id,timestamp_ns FROM frames ORDER BY idx').fetchall()
            if len(ids)!=len(results) or any((r.frame_id,r.timestamp_ns)!=(fid,ts) for (i,fid,ts),r in zip(ids,results)):
                raise ValueError('Shared image frame identity mismatch')
            super().__init__(source,results);self.shared_image_source=source
        else:super().__init__(directory,results)

    def open(self):
        super().open()
        if self.cache is not None and self.shared_image_source is None:
            row=self.cache.execute("SELECT value FROM metadata WHERE key='joint_preview'").fetchone()
            if row is not None:
                info=json.loads(row[0])
                self.fast_preview=(info.get('version')==1 and info.get('count')==len(self.results)
                    and info.get('calibration_sha256')==digest(self.root/'processing_calibration.yaml'))

    def preview_poses(self):
        rows=self.cache.execute('SELECT idx,frame_id,timestamp_ns,pose FROM joint_preview ORDER BY idx').fetchall()
        if len(rows)!=len(self.results) or any(
                idx!=index or (fid,ts)!=(result.frame_id,result.timestamp_ns) or
                (pose is not None and len(pose)!=128)
                for index,((idx,fid,ts,pose),result) in enumerate(zip(rows,self.results))):
            raise ValueError('预计算轨迹与结果帧不匹配')
        return [None if pose is None else np.frombuffer(pose,'<f8').reshape(4,4).copy() for _,_,_,pose in rows]

    def read_preview(self,index):
        result=self.results[index]
        row=self.cache.execute(
            'SELECT frame_id,timestamp_ns,raw_left,raw_right,processed_left,processed_right,depth,caption '
            'FROM joint_preview WHERE idx=?',(index,)).fetchone()
        if row is None or row[:2]!=(result.frame_id,result.timestamp_ns):
            raise ValueError('预计算图像与结果帧不匹配')
        images=[cv2.imdecode(np.frombuffer(blob,np.uint8),cv2.IMREAD_COLOR) for blob in row[2:7]]
        if any(image is None for image in images):raise ValueError('预计算图像损坏')
        return images,row[7]
