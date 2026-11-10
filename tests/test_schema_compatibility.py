import csv,json
import numpy as np
from egohand.core.types import serializable,result_from_dict
from egohand.storage.exporter import export_json,load_json,export_csv,export_npz

def test_legacy_json_and_new_uncertainty_roundtrip(engine,source,tmp_path):
    r=engine.process(source.read());old=serializable(r);old['schema_version']='1.0.0'
    for h in old['hands']:
        for j in h['joints']:
            for key in ('depth_std_m','position_std_m','position_covariance_m2','uncertainty_status'):j.pop(key)
    legacy=result_from_dict(old)
    assert legacy.schema_version=='1.0.0' and legacy.hands[0].joints[0].depth_std_m is None
    export_json([r],tmp_path/'r.json');q=load_json(tmp_path/'r.json')[0]
    np.testing.assert_allclose(q.hands[0].joints[0].position_covariance_m2,r.hands[0].joints[0].position_covariance_m2)
    export_csv([r],tmp_path/'r.csv');export_npz([r],tmp_path/'r.npz')
    with (tmp_path/'r.csv').open(encoding='utf-8-sig',newline='') as f:row=next(csv.DictReader(f))
    assert row['unit']=='m' and 'x_m' in row and float(row['depth_std_m'])>0
    with np.load(tmp_path/'r.npz',allow_pickle=False) as z:
        assert z['position_covariance_m2'].shape==(1,2,21,3,3)
        assert z['schema_version'].item()=='1.1.0'
        assert z['xyz_m'].shape==(1,2,21,3)
    assert 'NaN' not in (tmp_path/'r.json').read_text(encoding='utf-8')
