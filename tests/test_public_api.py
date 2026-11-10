import numpy as np
from egohand import EgoHandEngine

def test_external_precomputed_api(calibration,source):
    f=source.read();e=EgoHandEngine(calibration=calibration)
    r=e.process_stereo(f.left_image,f.right_image,frame_id=f.frame_id,timestamp_ns=f.timestamp_ns,left_timestamp_ns=f.left_timestamp_ns,right_timestamp_ns=f.right_timestamp_ns,synchronized=f.synchronized,source_type=f.source_type,metadata=f.metadata,left_detections=f.left_detections,right_detections=f.right_detections)
    assert r.reconstruction_status=="Synthetic Ground Truth"
    assert r.hands[0].valid_joint_count==21
    np.testing.assert_allclose([j.xyz_m for j in r.hands[0].joints],f.metadata["ground_truth_xyz_m"][0],atol=1e-5)

def test_new_uncalibrated_source_cannot_reuse_old(engine):
    from egohand.sources.image import SingleImageSource
    from pathlib import Path
    from PIL import Image
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        p=Path(tmp)/"x.png";Image.new("RGB",(30,20)).save(p)
        engine.start(SingleImageSource(p))
        assert engine.rectifier is None
        assert engine.process_next().reconstruction_status=="2D Only"
        engine.close()
