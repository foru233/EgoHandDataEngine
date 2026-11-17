"""Append-only metric depth sidecar, bound to exact result frames and calibration."""
import hashlib
import json
import sqlite3
import zlib
from pathlib import Path
import numpy as np
from ..core.types import serializable
from ..visualization.stereo_depth import DepthCache,compute_depth

DEPTH_FILE='depth_results.sqlite3'

def file_sha256(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()

def calibration_sha256(calibration):
    return hashlib.sha256(json.dumps(serializable(calibration),sort_keys=True,allow_nan=False).encode()).hexdigest()

class DepthResultWriter:
    def __init__(self,root,calibration,*,input_pixels):
        self.path=Path(root)/DEPTH_FILE
        # Exclusive creation prevents overwriting a previous experiment/result.
        with self.path.open('xb'):pass
        self.connection=sqlite3.connect(self.path)
        self.connection.execute('CREATE TABLE metadata (key TEXT PRIMARY KEY,value TEXT NOT NULL)')
        self.connection.execute('CREATE TABLE frames (frame_id INTEGER,timestamp_ns INTEGER,calibration_id TEXT,status TEXT,info TEXT,depth_zlib BLOB,PRIMARY KEY(frame_id,timestamp_ns))')
        self.calibration=calibration;self.cache=DepthCache();self.count=0;self.failed=0
        self.metadata={'version':1,'status':'building','encoding':'zlib_float32_le',
            'calibration_sha256':calibration_sha256(calibration),'input_pixels':input_pixels,
            'purpose':'derived_stereo_depth_not_ground_truth','count':0,'failed':0}
        self._checkpoint()

    def _checkpoint(self):
        self.connection.executemany('INSERT OR REPLACE INTO metadata VALUES (?,?)',
            [(k,json.dumps(v,ensure_ascii=False,allow_nan=False)) for k,v in self.metadata.items()])
        self.connection.commit()

    def write(self,frame):
        try:
            depth,info=compute_depth(frame,self.calibration,self.cache)
            payload=zlib.compress(depth.astype('<f4',copy=False).tobytes(),level=3)
            status='available'
        except Exception as exc:
            info={'reason':str(exc),'unit':'m','calibration_id':self.calibration.calibration_id}
            payload=None;status='unavailable';self.failed+=1
        self.connection.execute('INSERT INTO frames VALUES (?,?,?,?,?,?)',
            (int(frame.frame_id),int(frame.timestamp_ns),frame.calibration_id,status,
             json.dumps(info,ensure_ascii=False,allow_nan=False),payload))
        self.count+=1
        if self.count%100==0:
            self.metadata.update(count=self.count,failed=self.failed);self._checkpoint()
        return {'file':DEPTH_FILE,'frame_id':int(frame.frame_id),'timestamp_ns':int(frame.timestamp_ns),
                'status':status,**info}

    def close(self,result_file,*,completed):
        self.metadata.update(count=self.count,failed=self.failed,
            status='completed' if completed and not self.failed else 'partial',
            result_sha256=file_sha256(result_file))
        self._checkpoint();self.connection.close()

class DepthResultReader:
    def __init__(self,root,result_file,calibration):
        path=Path(root)/DEPTH_FILE
        self.connection=sqlite3.connect(path.resolve().as_uri()+'?mode=ro',uri=True)
        try:
            self.metadata={k:json.loads(v) for k,v in self.connection.execute('SELECT key,value FROM metadata')}
            if (self.metadata.get('version')!=1 or self.metadata.get('status') not in ('completed','partial')
                or self.metadata.get('result_sha256')!=file_sha256(result_file)
                or self.metadata.get('calibration_sha256')!=calibration_sha256(calibration)):
                raise ValueError('深度结果与手部结果或标定不匹配')
        except Exception:
            self.connection.close();raise

    def read(self,result):
        row=self.connection.execute('SELECT calibration_id,status,info,depth_zlib FROM frames WHERE frame_id=? AND timestamp_ns=?',
            (int(result.frame_id),int(result.timestamp_ns))).fetchone()
        if row is None:return None
        if row[0]!=result.calibration_id:raise ValueError('深度帧标定身份不匹配')
        info=json.loads(row[2]);info['status']=row[1]
        if row[1]!='available':return None,info
        depth=np.frombuffer(zlib.decompress(row[3]),dtype='<f4').reshape(info['shape']).copy()
        if not np.isfinite(depth).all() or (depth<0).any():raise ValueError('深度结果包含非法值')
        return depth,info

    def close(self):self.connection.close()
