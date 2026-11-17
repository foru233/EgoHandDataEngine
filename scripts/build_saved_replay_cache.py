"""Build an optional random-access, full-resolution JPEG display cache."""
import argparse
import hashlib
import json
import os
import sqlite3
import time
from pathlib import Path
import cv2
from egohand.storage.replay import load_recorded_results, SavedResultReplay


def sha256(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def build(root):
    root=Path(root).resolve(); result_file=root/'results'/'frames.jsonl'
    final=root/'display_cache.sqlite3'; temporary=root/'display_cache.building.sqlite3'
    if final.exists() or temporary.exists():raise FileExistsError('缓存或未完成缓存已存在，保留原文件')
    results,truncated=load_recorded_results(root)
    if truncated or not results:raise ValueError('结果文件为空或尾部截断')
    replay=SavedResultReplay(root,results); connection=None
    try:
        replay.open(); replay.start()
        connection=sqlite3.connect(temporary)
        connection.execute('CREATE TABLE frames (idx INTEGER PRIMARY KEY, frame_id INTEGER NOT NULL, timestamp_ns INTEGER NOT NULL, left_jpeg BLOB NOT NULL, right_jpeg BLOB NOT NULL)')
        connection.execute('CREATE TABLE metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL)')
        info={'version':1,'result_sha256':sha256(result_file),'count':len(results),'quality':90,'encoding':'jpeg_bgr_full_resolution','original_recording':str(replay.source.recording_dir)}
        connection.executemany('INSERT INTO metadata VALUES (?,?)',[(k,json.dumps(v,ensure_ascii=False)) for k,v in info.items()])
        start=time.monotonic()
        for i,result in enumerate(results):
            replay.seek(i);frame=replay.read()
            left_ok,left=cv2.imencode('.jpg',frame.left_image,[cv2.IMWRITE_JPEG_QUALITY,90])
            right_ok,right=cv2.imencode('.jpg',frame.right_image,[cv2.IMWRITE_JPEG_QUALITY,90])
            if not left_ok or not right_ok:raise RuntimeError(f'JPEG 编码失败：{i}')
            connection.execute('INSERT INTO frames VALUES (?,?,?,?,?)',(i,result.frame_id,result.timestamp_ns,left.tobytes(),right.tobytes()))
            if (i+1)%100==0:
                connection.commit(); print(f'{i+1}/{len(results)} elapsed={time.monotonic()-start:.1f}s',flush=True)
        connection.commit()
        if connection.execute('PRAGMA quick_check').fetchone()[0]!='ok':raise RuntimeError('缓存完整性检查失败')
        connection.close();connection=None
        os.replace(temporary,final)
        print(f'完成 {final} bytes={final.stat().st_size}',flush=True)
    finally:
        if connection is not None:connection.close()
        replay.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');build(parser.parse_args().directory)
