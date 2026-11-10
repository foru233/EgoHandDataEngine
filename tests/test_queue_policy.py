import json
import threading
import time
from egohand.storage.recorder import AsyncRecorder
from egohand.gui.workers import PipelineWorker
from egohand.sources.synthetic import SyntheticStereoSource

def test_record_overflow_is_explicit(source,tmp_path):
    w=AsyncRecorder(tmp_path/"overload",source.get_metadata(),source.get_calibration(),max_queue=1)
    entered=threading.Event();release=threading.Event()
    original=w.writer.write_raw
    def slow(frame):
        entered.set();release.wait(3);original(frame)
    w.writer.write_raw=slow
    f=source.read()
    assert w.submit("raw",f)
    assert entered.wait(2)
    assert w.submit("raw",f)
    assert not w.submit("raw",f)
    assert "OVERFLOW" in w.error and not w.accepting
    release.set();w.close()
    m=json.loads((tmp_path/"overload/manifest.json").read_text(encoding="utf-8"))
    assert m["recording_status"]=="incomplete" and m["raw_count"]==2

def test_write_failure_is_explicit(source,tmp_path):
    w=AsyncRecorder(tmp_path/"failed",source.get_metadata())
    def fail(frame):raise OSError("test disk failure")
    w.writer.write_raw=fail
    w.submit("raw",source.read())
    w.thread.join(3)
    assert not w.thread.is_alive() and "WRITE_FAILED" in w.error
    m=json.loads((tmp_path/"failed/manifest.json").read_text(encoding="utf-8"))
    assert m["recording_status"]=="incomplete"

def test_offline_does_not_drop(qtbot):
    w=PipelineWorker(SyntheticStereoSource(frames=18),offline=True)
    w.start()
    qtbot.waitUntil(lambda:w.eof or not w.isRunning(),timeout=8000)
    qtbot.waitUntil(lambda:w.processed==18,timeout=8000)
    w.stop();assert w.wait(3000)
    assert w.dropped==0 and len(w.history)==18

def test_realtime_drops_inference_not_raw(qtbot,tmp_path,monkeypatch):
    from egohand.core.pipeline import EgoHandEngine
    original=EgoHandEngine.process
    def slow(self,frame,*a,**kw):
        time.sleep(.04);return original(self,frame,*a,**kw)
    monkeypatch.setattr(EgoHandEngine,"process",slow)
    w=PipelineWorker(SyntheticStereoSource(frames=25,fps=200))
    w.command("record_start",tmp_path/"raw")
    w.start()
    qtbot.waitUntil(lambda:w.read_count==25 and w.queue.empty(),timeout=8000)
    qtbot.wait(150);w.stop();assert w.wait(5000)
    assert w.dropped>0 and w.processed<25
    m=json.loads((tmp_path/"raw/manifest.json").read_text(encoding="utf-8"))
    assert m["raw_count"]==25
    assert m["raw_count"]>m["result_count"]
