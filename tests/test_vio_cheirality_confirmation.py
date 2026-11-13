import importlib.util
import json
from pathlib import Path

spec=importlib.util.spec_from_file_location('cheirality_confirmation',
    Path(__file__).resolve().parents[1]/'scripts/run_vio_cheirality_confirmation.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


def test_commit_audit_includes_mono_observations_and_rejects_nonfinite_depth(tmp_path):
    (tmp_path/'experiment.json').write_text(json.dumps({'error':None}))
    (tmp_path/'poses.jsonl').write_text(json.dumps({'raw_pose_valid':True,'pose_valid':True,
                                                  'inertial_scale_finalized':True})+'\n')
    d=tmp_path/'native_diagnostics';d.mkdir()
    (d/'map0_event5_after_commit_observations.csv').write_text(
        'point_id,rectified_stereo,perspective,camera_depth\n'
        '1,1,1,2\n2,0,1,-2\n3,0,1,nan\n4,1,1,inf\n5,0,0,-1\n')
    result=audit.summarize(tmp_path)['commits'][0]
    assert result['perspective_observations']==4
    assert result['invalid_depth']==3
    assert result['invalid_point_ids']==[2,3,4]
