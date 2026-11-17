"""Add metric depth without changing existing hand, VIO or original recordings."""
import argparse
import json
import time
from pathlib import Path
from egohand.storage.replay import SavedResultReplay,load_recorded_results
from egohand.storage.depth_results import DepthResultWriter,DEPTH_FILE

def build(root):
    root=Path(root).resolve()
    if (root/DEPTH_FILE).exists():raise FileExistsError('已有深度结果，保留原文件')
    results,truncated=load_recorded_results(root)
    if truncated or not results:raise ValueError('结果为空或尾部截断')
    replay=SavedResultReplay(root,results);writer=None;completed=False
    try:
        replay.open();replay.start()
        writer=DepthResultWriter(root,replay.calibration,input_pixels=(
            'existing_full_resolution_jpeg_display_cache' if replay.cache is not None else 'original_recording_decode'))
        started=time.monotonic()
        for index,result in enumerate(results):
            replay.seek(index);frame=replay.read();writer.write(frame)
            if (index+1)%100==0:print(f'{index+1}/{len(results)} failed={writer.failed} elapsed={time.monotonic()-started:.1f}s',flush=True)
        completed=True
    finally:
        if writer is not None:writer.close(root/'results/frames.jsonl',completed=completed)
        replay.close()
    print(json.dumps(writer.metadata,ensure_ascii=False,indent=2),flush=True)
    if writer.failed:raise RuntimeError(f'{writer.failed} 帧深度不可用，详见逐帧记录')

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');build(parser.parse_args().directory)
