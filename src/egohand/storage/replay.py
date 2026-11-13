# 回放已有结果不重新推理；尾行截断状态与结果列表同时返回，不能冒称完整会话。

from pathlib import Path
from . import exporter
from ..sources.recorded import read_jsonl_recover
from ..core.types import result_from_dict

# 用统一兼容加载器恢复旧/新结果字段，并保留JSONL末尾是否截断的信息。
def load_recorded_results(directory):
    root=Path(directory)
    if not (root/'results/frames.jsonl').is_file() and (root/'replay/results/frames.jsonl').is_file():
        root=root/'replay'
    if not (root/'results/frames.jsonl').is_file():
        candidate=root/'results.json'
        if not candidate.is_file():candidate=root/'export/results.json'
        if candidate.is_file():return exporter.load_json(candidate),False
    rows,truncated=read_jsonl_recover(root/"results/frames.jsonl")
    return [result_from_dict(r) for r in rows],truncated


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
        self.source=None;self.calibration=None;self.index=0

    def open(self):
        import json
        from ..gui.shared.sources import open_recording_source
        from ..calibration.loader import load_calibration
        original=self.root
        descriptor=self.root/'reprocessing.json'
        if descriptor.is_file():
            info=json.loads(descriptor.read_text(encoding='utf8'))
            original=Path(info['original_recording'])
        self.source=open_recording_source(original,reuse_detections=True)
        self.source.open()
        self.calibration=self.source.get_calibration()
        saved=self.root/'processing_calibration.yaml'
        if saved.is_file():self.calibration=load_calibration(saved)
        elif descriptor.is_file() and (self.root.parent/'input_config/kalibr_camchain.yaml').is_file():
            # Complete explicit CLI snapshot from earlier product runs.
            self.calibration=load_calibration(self.root.parent/'input_config/kalibr_camchain.yaml',kalibr_left_camera='cam0')
        source_rows=getattr(self.source,'rows',None)
        self.source_index=({(int(r['frame_id']),int(r['timestamp_ns'])):i for i,r in enumerate(source_rows)}
                           if source_rows is not None else None)

    def start(self):self.source.start()
    def close(self):
        if self.source is not None:self.source.close()
    def seek(self,index):
        if not 0<=index<len(self.results):raise IndexError(index)
        self.index=index;r=self.results[index]
        raw_index=(self.source_index[(r.frame_id,r.timestamp_ns)] if self.source_index is not None else r.frame_id)
        # Sequential saved-frame playback should reuse the decoder.
        if getattr(self.source,'index',None)!=raw_index:self.source.seek(raw_index)
    def read(self):
        from dataclasses import replace
        r=self.results[self.index];frame=self.source.read();self.index+=1
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
