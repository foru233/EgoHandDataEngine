# 录制图像/视频读取与时间恢复；保持原始来源证据，不用相同 FPS 假定两路同步。

import csv
import json
from pathlib import Path
import cv2
from .base import StereoDataSource
from ..core.types import SourceMetadata,StereoFrame,detection_from_dict
from ..calibration.loader import load_calibration

# 只忽略被截断的最后一行并报告 truncated；中间损坏不是正常可恢复尾行。
def read_jsonl_recover(path):
    """Recover complete rows from an interrupted append, report malformed tail."""
    lines=Path(path).read_text(encoding="utf-8").splitlines()
    rows=[];truncated=False
    for i,line in enumerate(lines):
        try: rows.append(json.loads(line))
        except json.JSONDecodeError:
            if i!=len(lines)-1: raise ValueError("记录中间存在损坏，拒绝静默跳过")
            truncated=True
    return rows,truncated

# 统一读取图片清单、双视频或本工程录制；未知传感器时间保持未知。
class RecordedStereoSource(StereoDataSource):
    """Native recording directory or explicit external JSON manifest.

    External frames: left/right paths; optional sensor timestamps and evidence.
    External video: left_video/right_video plus optional timestamps_csv.
    No synchronization is inferred from matching video FPS.
    """
    def __init__(self,path,loop=False,reuse_detections=False):
        self.path=Path(path);self.loop=loop;self.reuse_detections=reuse_detections
        self.index=0;self.running=False;self.paused=False;self.captures=[];self.calibration=None
    def open(self):
        manifest=self.path/"manifest.json" if self.path.is_dir() else self.path
        self.root=manifest.parent
        self.info=json.loads(manifest.read_text(encoding="utf-8-sig"))
        self.native="raw_index" in self.info
        if self.native:
            self.rows,self.truncated=read_jsonl_recover(self.root/self.info["raw_index"])
            md=dict(self.info["metadata"])
        else:
            md=dict(self.info.get("metadata",{}));self.truncated=False
            if "left_video" in self.info:
                for name in ("left_video","right_video"):
                    cap=cv2.VideoCapture(str(self.root/self.info[name]))
                    if not cap.isOpened(): raise ValueError(f"视频无法打开：{name}")
                    self.captures.append(cap)
                if self.info.get("timestamps_csv"):
                    with (self.root/self.info["timestamps_csv"]).open(encoding="utf-8-sig",newline="") as f: self.rows=list(csv.DictReader(f))
                else:
                    count=min(int(c.get(cv2.CAP_PROP_FRAME_COUNT)) for c in self.captures)
                    fps=self.captures[0].get(cv2.CAP_PROP_FPS) or 30
                    self.rows=[{"timestamp_ns":round(i/fps*1e9)} for i in range(count)]
                    md.update(synchronized=False,synchronization_evidence=None,timestamp_kind="estimated_from_fps")
            else: self.rows=self.info.get("frames",[])
        allowed=SourceMetadata.__dataclass_fields__
        md={k:v for k,v in md.items() if k in allowed}
        md.setdefault("source_type","recorded")
        self.metadata=SourceMetadata(**md)
        if self.metadata.synchronized and not self.metadata.synchronization_evidence:
            self.metadata.synchronized=False
        self.metadata.extra.update(frame_count=len(self.rows),recovered_truncated_tail=self.truncated)
        if self.info.get("calibration"): self.calibration=load_calibration(self.root/self.info["calibration"])
        self.index=0
    def start(self): self.running=True;self.paused=False
    def pause(self): self.paused=True
    def resume(self): self.paused=False
    def stop(self): self.running=False
    def close(self):
        self.stop()
        for c in self.captures: c.release()
        self.captures=[]
    def get_metadata(self): return self.metadata
    def get_calibration(self): return self.calibration
    # 定位改变读取游标；消费端还需重置跟踪和 VIDEO 实例，避免沿用旧时间状态。
    def seek(self,index):
        if not 0<=index<len(self.rows): raise IndexError(index)
        self.index=index
        for cap in self.captures: cap.set(cv2.CAP_PROP_POS_FRAMES,index)
    # 解码一对图像并保留原时间/几何/来源；未知标定或同步时不能虚构可测深度。
    def read(self,force=False):
        if not self.running or (self.paused and not force): return None
        if self.index>=len(self.rows):
            if not self.loop or not self.rows: return None
            self.seek(0)
        i=self.index;row=self.rows[i];self.index+=1
        images=[]
        for side in ("left","right"):
            if self.captures:
                ok,img=self.captures[len(images)].read()
                if not ok: raise ValueError(f"视频 {side} 第 {i} 帧读取失败")
            else:
                rel=row.get(side+"_image",row.get(side))
                img=cv2.imread(str(self.root/rel)) if rel else None
                if img is None: raise ValueError(f"图像 {side} 第 {i} 帧缺失")
            images.append(img)
        def timestamp(key):
            v=row.get(key)
            return int(v) if v not in (None,"") else None
        left_ts,right_ts=timestamp("left_timestamp_ns"),timestamp("right_timestamp_ns")
        offset=int(self.info.get("right_timestamp_offset_ns",0))
        metadata={**row.get("metadata",{}),"synthetic":self.metadata.synthetic,"real_stereo":self.metadata.real_stereo,"synchronization_evidence":self.metadata.synchronization_evidence,"timestamp_kind":self.metadata.timestamp_kind,"processing_mode":"reprocessed","original_left_timestamp_ns":left_ts,"original_right_timestamp_ns":right_ts,"right_timestamp_offset_ns":offset}
        ts=timestamp("timestamp_ns")
        if ts is None: ts=left_ts
        if ts is None: raise ValueError("图像序列必须提供 timestamp_ns")
        ds=[]
        for side in ("left","right"):
            value=row.get(side+"_detections") if self.reuse_detections else None
            ds.append([detection_from_dict(d) for d in value] if value is not None else None)
        return StereoFrame(int(row.get("frame_id",i)),ts,*images,left_ts,right_ts+offset if right_ts is not None else None,row.get("calibration_id",self.calibration.calibration_id if self.calibration else None),self.metadata.source_type,self.metadata.synchronized,row.get("image_geometry",self.info.get("image_geometry","raw")),metadata,*ds)
