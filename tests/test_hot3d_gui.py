from pathlib import Path
import pytest
from egohand.sources.hot3d import HOT3DSource,HOT3DFrameProcessor
from egohand.core.pipeline import EgoHandEngine
from egohand.gui.main_window import MainWindow

ROOT=Path(__file__).resolve().parents[1]/"samples/date/hot3d"

@pytest.fixture
def clip():
    p=ROOT/"train_aria/clip-001849.tar"
    if not p.exists():pytest.skip("Official local HOT3D clip unavailable")
    return p

def test_source_seek_truth_and_prediction_separation(clip):
    s=HOT3DSource(clip);s.open();s.start()
    try:
        assert len(s.rows)==150 and not s.get_metadata().synthetic
        f=s.read();p=HOT3DFrameProcessor();e=EgoHandEngine()
        r=p.process(e,f,"ground_truth")
        assert r.reconstruction_status=="Official Ground Truth"
        assert sum(h.valid_joint_count for h in r.hands)==40
        assert all(h.joints[1].valid is False for h in r.hands)
        assert all(j.quality_score is None for h in r.hands for j in h.joints)
        s.seek(62);f=s.read();assert f.frame_id==62
        other=p.process(e,f,"prediction")
        assert other.diagnostics["result_kind"]=="algorithm_prediction"
        assert not any(h.valid_joint_count for h in other.hands)
        assert all(d.origin!="official_umetrack_gt_projection" for d in other.left_detections)
    finally:s.close()

def test_open_extracted_folder_and_no_test_gt(clip,tmp_path):
    # 从真实tar建立临时“解包目录+同名tar”输入，不依赖用户保留重复的完整解包目录。
    import os,shutil,tarfile
    folder=tmp_path/clip.stem;folder.mkdir()
    sibling=folder.with_suffix('.tar')
    try:os.link(clip,sibling)
    except OSError:shutil.copyfile(clip,sibling)
    shutil.copyfile(ROOT/'official_file_metadata.json',tmp_path/'official_file_metadata.json')
    with tarfile.open(clip) as archive:
        (folder/'000000.cameras.json').write_bytes(archive.extractfile('000000.cameras.json').read())
    s=HOT3DSource(folder);s.open();s.start()
    try:assert s.read().metadata["hot3d"]["official_hash_verified"]
    finally:s.close()
    s=HOT3DSource(ROOT/"train_aria/clip-003365.tar");s.open();s.start()
    try:
        r=HOT3DFrameProcessor().process(EgoHandEngine(),s.read(),"ground_truth")
        assert not r.hands and r.reconstruction_status=="Ground Truth Unavailable"
    finally:s.close()

def test_gui_open_seek_switch_and_record(qtbot,clip,tmp_path,monkeypatch):
    from PySide6.QtWidgets import QFileDialog
    from egohand.storage.recorder import Recorder
    w=MainWindow();qtbot.addWidget(w);w.show()
    try:
        w.source_panel.kind.setCurrentIndex(4);w.source_panel.path.setText(str(clip));w.start_pipeline()
        qtbot.waitUntil(lambda:w.latest is not None,timeout=10000)
        assert w.latest.diagnostics["result_kind"]=="official_ground_truth"
        assert "GT" in w.provenance_badge.text() and w.left_view.pix is not None
        w.worker.seek(62);w.paused=True
        qtbot.waitUntil(lambda:w.latest.frame_id==62,timeout=8000)
        w.source_panel.hot3d_mode.setCurrentIndex(1)
        qtbot.waitUntil(lambda:w.latest.diagnostics["result_kind"]=="algorithm_prediction",timeout=10000)
        w.source_panel.hot3d_mode.setCurrentIndex(0)
        qtbot.waitUntil(lambda:w.latest.diagnostics["result_kind"]=="official_ground_truth",timeout=8000)
        w.stop_pipeline()
        source=HOT3DSource(clip);source.open();source.start();f=source.read()
        result=HOT3DFrameProcessor().process(EgoHandEngine(),f,"ground_truth")
        dest=tmp_path/"recording"
        writer=Recorder(dest,source.get_metadata());writer.write_raw(f);writer.write_result(result);writer.close();source.close()
        monkeypatch.setattr(QFileDialog,"getExistingDirectory",lambda *a,**k:str(dest))
        w.open_results()
        assert w.latest.reconstruction_status=="Official Ground Truth"
        assert w.left_view.pix is not None and w.right_view.pix is not None
    finally:w.stop_pipeline();w.close()
