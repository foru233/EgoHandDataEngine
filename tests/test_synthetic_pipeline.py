import numpy as np
import pytest
from egohand.core.pipeline import EgoHandEngine
from egohand.sources.synthetic import SyntheticStereoSource,SCENARIOS
from egohand.storage.recorder import Recorder
from egohand.storage.replay import load_recorded_results

def test_end_to_end(tmp_path):
    s=SyntheticStereoSource("two_static",frames=4)
    e=EgoHandEngine();e.start(s)
    writer=Recorder(tmp_path/"record",s.get_metadata(),s.get_calibration())
    truths=[]
    for _ in range(4):
        f=s.read();truths.append(f.metadata["ground_truth_xyz_m"])
        writer.write_raw(f);writer.write_result(e.process(f))
    writer.close();e.stop()
    results,truncated=load_recorded_results(tmp_path/"record")
    assert not truncated
    for r,truth in zip(results,truths):
        for h,gt in zip(r.hands,truth):
            np.testing.assert_allclose([j.xyz_m for j in h.joints],gt,atol=1e-5)

@pytest.mark.parametrize("scenario",SCENARIOS)
def test_all_scenarios(scenario):
    s=SyntheticStereoSource(scenario,frames=8)
    e=EgoHandEngine();e.start(s)
    results=[e.process_next() for _ in range(8)]
    if scenario=="invalid_calibration": assert all(r.reconstruction_status=="2D Only" for r in results)
    if scenario=="timestamp_anomaly": assert results[5].diagnostics["reason"]=="TIMESTAMP_NON_MONOTONIC"
    assert all(len(h.joints)==21 for r in results for h in r.hands)
    e.close()
