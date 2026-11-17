import hashlib
import json
import sqlite3
from types import SimpleNamespace
import cv2
import numpy as np
import pytest
from egohand.storage.replay import SavedResultReplay


@pytest.mark.parametrize('cached_only',[False,True])
def test_cache_random_access_validates_frame_identity(tmp_path, monkeypatch,cached_only):
    from egohand.gui.shared import sources
    class Source:
        def open(self):pass
        def start(self):pass
        def close(self):pass
        def get_calibration(self):return SimpleNamespace(calibration_id='cal')
        def seek(self,index):raise AssertionError('cache should not seek source')
    monkeypatch.setattr(sources,'open_recording_source',lambda *a,**k:Source())
    if cached_only:
        from egohand.calibration import loader
        def forbidden(*a,**k):raise AssertionError('Complete cache must not open original recording')
        monkeypatch.setattr(sources,'open_recording_source',forbidden)
        monkeypatch.setattr(loader,'load_calibration',lambda path:SimpleNamespace(calibration_id='cal'))
        (tmp_path/'processing_calibration.yaml').touch()
        (tmp_path/'reprocessing.json').write_text(json.dumps({'original_recording':str(tmp_path/'missing')}),encoding='utf8')
    (tmp_path/'results').mkdir();result_file=tmp_path/'results'/'frames.jsonl';result_file.write_text('test',encoding='utf8')
    image=np.full((12,16,3),127,np.uint8);ok,encoded=cv2.imencode('.jpg',image);assert ok
    c=sqlite3.connect(tmp_path/'display_cache.sqlite3')
    c.execute('CREATE TABLE metadata (key TEXT PRIMARY KEY,value TEXT NOT NULL)')
    c.execute('CREATE TABLE frames (idx INTEGER PRIMARY KEY,frame_id INTEGER,timestamp_ns INTEGER,left_jpeg BLOB,right_jpeg BLOB)')
    metadata={'version':1,'count':1,'result_sha256':hashlib.sha256(result_file.read_bytes()).hexdigest()}
    c.executemany('INSERT INTO metadata VALUES (?,?)',[(k,json.dumps(v)) for k,v in metadata.items()])
    c.execute('INSERT INTO frames VALUES (?,?,?,?,?)',(0,3,123,encoded.tobytes(),encoded.tobytes()));c.commit();c.close()
    result=SimpleNamespace(frame_id=3,timestamp_ns=123,calibration_id='cal',source_type='vendor_vs030')
    replay=SavedResultReplay(tmp_path,[result]);replay.open();replay.start()
    try:
        replay.seek(0);frame=replay.read()
        assert frame.metadata['saved_display_cache'] and frame.left_image.shape==(12,16,3)
        if cached_only:assert replay.source is None
        result.timestamp_ns=124
        replay.seek(0)
        with pytest.raises(ValueError,match='身份不匹配'):replay.read()
    finally:replay.close()


def test_bad_depth_sidecar_without_image_cache_keeps_replay_usable(tmp_path,monkeypatch):
    from egohand.gui.shared import sources
    class Source:
        def open(self):pass
        def close(self):pass
        def get_calibration(self):return SimpleNamespace(calibration_id='cal')
    monkeypatch.setattr(sources,'open_recording_source',lambda *a,**k:Source())
    (tmp_path/'depth_results.sqlite3').write_bytes(b'not a database')
    replay=SavedResultReplay(tmp_path,[])
    try:
        replay.open()
        assert replay.depth_results is None and replay.depth_error
    finally:replay.close()
