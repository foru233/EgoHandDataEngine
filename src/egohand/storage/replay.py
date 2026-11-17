# 回放已有结果不重新推理；尾行截断状态与结果列表同时返回，不能冒称完整会话。

from pathlib import Path
import sqlite3
import zlib
from . import exporter
from ..sources.recorded import read_jsonl_recover
from ..core.types import result_from_dict

# 用统一兼容加载器恢复旧/新结果字段，并保留JSONL末尾是否截断的信息。
def load_recorded_results(directory):
    root=Path(directory)
    requested_root=root
    if not (root/'results/frames.jsonl').is_file() and (root/'replay/results/frames.jsonl').is_file():
        root=root/'replay'
    if not (root/'results/frames.jsonl').is_file():
        candidate=root/'results.json'
        if not candidate.is_file():candidate=root/'export/results.json'
        if candidate.is_file():return exporter.load_json(candidate),False
    rows,truncated=read_jsonl_recover(root/"results/frames.jsonl")
    results=[result_from_dict(r) for r in rows]
    # Derived body/MANO data is additive and keyed by immutable frame identity.
    # A missing or malformed sidecar never prevents the original replay.
    sidecars=[root/'derived_pose.jsonl']
    if requested_root != root: sidecars.append(requested_root/'derived_pose.jsonl')
    sidecar=next((p for p in sidecars if p.is_file()),None)
    if sidecar is not None:
        try:
            manifest_path=sidecar.parent/'unified_smpl_manifest.json'
            manifest=(__import__('json').loads(manifest_path.read_text(encoding='utf8')) if manifest_path.is_file() else {})
            mesh_assets={}
            asset_names=['body_mesh_smpl.json','smpl_mesh.json']
            if manifest.get('mesh_file'):asset_names.append(manifest['mesh_file'])
            for name in asset_names:
                asset=sidecar.parent/name
                if asset.is_file(): mesh_assets[name]=__import__('json').loads(asset.read_text(encoding='utf8'))
            derived={}
            for index,line in enumerate(sidecar.read_text(encoding='utf8').splitlines()):
                row=__import__('json').loads(line); row.setdefault('body_mesh_index',index)
                if row.get('unified_smpl'):
                    row['body_mesh_vertices_path']=str(sidecar.parent/manifest.get('vertices_file','smpl_vertices.npy'))
                    row.setdefault('head_anchor_version',manifest.get('head_anchor_version'))
                elif (requested_root/'body_vertices.npy').is_file(): row['body_mesh_vertices_path']=str(requested_root/'body_vertices.npy')
                derived[(int(row['frame_id']),int(row['timestamp_ns']))]=row
            for result in results:
                row=derived.get((result.frame_id,result.timestamp_ns))
                if row is not None:
                    ref=row.get('body_mesh_ref')
                    if ref in mesh_assets: row={**row,'body_mesh':mesh_assets[ref]}
                    result.diagnostics={**result.diagnostics,'offline_derived_pose':row}
        except (OSError,ValueError,TypeError,KeyError):
            pass
    return results,truncated


class SavedResultReplay:
    """Display saved results with their original SDK pixels, without inference.

    Seeks are result indices. SDK decoding restarts when needed to retain HEVC
    dependencies; source frame identity must match exactly before presentation.
    """
    def __init__(self,directory,results):
        self.root=Path(directory)
        if (self.root/'replay/reprocessing.json').is_file():self.root=self.root/'replay'
        self.results=results
        self.rows=[{'frame_id':r.frame_id,'timestamp_ns':r.timestamp_ns} for r in results]
        self.source=None;self.calibration=None;self.index=0;self.cache=None
        self.depth_results=None;self.depth_error=None

    def open(self):
        import json
        from ..gui.shared.sources import open_recording_source
        from ..calibration.loader import load_calibration
        original=self.root
        descriptor=self.root/'reprocessing.json'
        if descriptor.is_file():
            info=json.loads(descriptor.read_text(encoding='utf8'))
            original=Path(info['original_recording'])
        saved=self.root/'processing_calibration.yaml'
        cache_path=self.root/'display_cache.sqlite3'
        if cache_path.is_file() and saved.is_file():
            # A complete saved replay must not require the original recording
            # or initialize its decoder. Cache identity is checked below.
            self.calibration=load_calibration(saved)
        else:
            self.source=open_recording_source(original,reuse_detections=True)
            self.source.open()
            self.calibration=self.source.get_calibration()
            if saved.is_file():self.calibration=load_calibration(saved)
            elif descriptor.is_file() and (self.root.parent/'input_config/kalibr_camchain.yaml').is_file():
                # Complete explicit CLI snapshot from earlier product runs.
                self.calibration=load_calibration(self.root.parent/'input_config/kalibr_camchain.yaml',kalibr_left_camera='cam0')
        source_rows=getattr(self.source,'rows',None)
        self.source_index=({(int(r['frame_id']),int(r['timestamp_ns'])):i for i,r in enumerate(source_rows)}
                           if source_rows is not None else None)
        if cache_path.is_file():
            import json, hashlib
            cache=sqlite3.connect(cache_path.resolve().as_uri()+'?mode=ro',uri=True)
            try:
                meta={k:json.loads(v) for k,v in cache.execute('SELECT key,value FROM metadata')}
                h=hashlib.sha256()
                with (self.root/'results/frames.jsonl').open('rb') as f:
                    for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
                if meta.get('version')!=1 or meta.get('count')!=len(self.results) or meta.get('result_sha256')!=h.hexdigest():
                    raise ValueError('回放缓存与结果文件不匹配')
                self.cache=cache
            except Exception:
                cache.close();raise
        from .depth_results import DEPTH_FILE,DepthResultReader
        if (self.root/DEPTH_FILE).is_file():
            try:self.depth_results=DepthResultReader(self.root,self.root/'results/frames.jsonl',self.calibration)
            except (OSError,ValueError,sqlite3.Error) as exc:
                self.depth_error=str(exc)

    def start(self):
        if self.source is not None:self.source.start()
    def close(self):
        if self.depth_results is not None:self.depth_results.close();self.depth_results=None
        if self.cache is not None:self.cache.close();self.cache=None
        if self.source is not None:self.source.close()
    def depth_for_result(self,result):
        if self.depth_results is None:return None
        try:return self.depth_results.read(result)
        except (ValueError,TypeError,KeyError,sqlite3.Error,zlib.error) as exc:
            self.depth_error=str(exc);return None
    def seek(self,index):
        if not 0<=index<len(self.results):raise IndexError(index)
        self.index=index;r=self.results[index]
        if self.cache is not None:return
        raw_index=(self.source_index[(r.frame_id,r.timestamp_ns)] if self.source_index is not None else r.frame_id)
        # Sequential saved-frame playback should reuse the decoder.
        if getattr(self.source,'index',None)!=raw_index:self.source.seek(raw_index)
    def read(self):
        from dataclasses import replace
        r=self.results[self.index]
        if self.cache is not None:
            import cv2, numpy as np
            from ..core.types import StereoFrame
            row=self.cache.execute('SELECT frame_id,timestamp_ns,left_jpeg,right_jpeg FROM frames WHERE idx=?',(self.index,)).fetchone()
            if row is None or (row[0],row[1])!=(r.frame_id,r.timestamp_ns):raise ValueError('缓存帧身份不匹配')
            images=[cv2.imdecode(np.frombuffer(data,dtype=np.uint8),cv2.IMREAD_COLOR) for data in row[2:]]
            if any(image is None for image in images):raise ValueError('缓存图像损坏')
            frame=StereoFrame(r.frame_id,r.timestamp_ns,*images,calibration_id=r.calibration_id,source_type=r.source_type,image_geometry='raw',metadata={'saved_display_cache':True})
        else:frame=self.source.read()
        self.index+=1
        if frame is None or (frame.frame_id,frame.timestamp_ns)!=(r.frame_id,r.timestamp_ns):
            raise ValueError('已有结果与原始双目采集身份不匹配，图像未显示')
        if frame.metadata.get('hot3d') or frame.metadata.get('show3d'):
            # These datasets restore per-frame official camera geometry in the
            # existing GUI processor; they have no static recording calibration.
            return frame
        if self.calibration is None or self.calibration.calibration_id!=r.calibration_id:
            raise ValueError('已有结果缺少匹配处理标定，不能叠加关键点')
        if frame.image_geometry=='raw':
            frame=replace(frame,calibration_id=r.calibration_id,
                metadata={**frame.metadata,'recorded_calibration_id':frame.calibration_id,
                    'saved_result_replay':True})
        return frame
