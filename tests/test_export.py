import json
import numpy as np
from egohand.storage.exporter import export_json,load_json,export_csv,export_npz
from egohand.sources.synthetic import SyntheticStereoSource

def test_formats(engine,tmp_path):
    s=SyntheticStereoSource("missing_joints",frames=1);s.open();s.start()
    r=engine.process(s.read())
    export_json([r],tmp_path/"r.json");export_csv([r],tmp_path/"r.csv");export_npz([r],tmp_path/"r.npz")
    text=(tmp_path/"r.json").read_text(encoding="utf-8")
    assert "NaN" not in text
    d=json.loads(text)[0]
    assert d["schema_version"] and d["coordinate_frame"]=="camera_left_optical"
    assert d["hands"][0]["joints"][4]["xyz_m"]==[None,None,None]
    q=load_json(tmp_path/"r.json")[0]
    assert q.frame_id==r.frame_id
    assert np.isnan(q.hands[0].joints[4].xyz_m).all()
    with np.load(tmp_path/"r.npz",allow_pickle=False) as z:
        assert z["xyz_m"].shape==(1,2,21,3)
        assert not z["valid"][0,0,4]
        assert z["calibration_id"][0]==r.calibration_id
    assert "x_m,y_m,z_m" in (tmp_path/"r.csv").read_text(encoding="utf-8-sig")
