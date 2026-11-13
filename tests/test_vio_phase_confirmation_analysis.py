import copy
import importlib.util
from pathlib import Path
import numpy as np

spec=importlib.util.spec_from_file_location('phase_confirmation_analysis',
    Path(__file__).resolve().parents[1]/'scripts/summarize_vio_phase_confirmation.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


def sample(phase):
    return {'inputs':{'timing':{'lattice_phase_ms':phase,'imu_rate_hz':480},
                      'python_sources':{'runtime':'same'},'files':['same inputs']},
            'timestamps':[1,2], 'image_means':[(20,30),(21,31)],
            'result':{'error':None,'worker_exit_code':0},'ba2_complete':True,'ba2_geometry':{}}


def test_only_phase_difference_is_a_valid_control():
    assert audit.pair_errors(sample(97),sample(32))==[]


def test_changed_sources_or_images_are_not_valid_controls():
    a,b=sample(97),sample(32)
    b['inputs']['python_sources']['runtime']='changed'
    b['timestamps'][0]=0
    assert audit.pair_errors(a,b)==['image_timestamps','input_python_sources']


def test_failed_ba2_is_explicit_outcome_not_zero_growth():
    a,b=sample(97),sample(32)
    a['ba2_complete']=False;a.pop('ba2_geometry')
    assert audit.pair_errors(a,b)==['ba2_incomplete']


def test_timeout_prefix_is_failure_outcome_not_input_mismatch():
    a,b=sample(97),sample(32)
    a['result']['error']='TimeoutError';a['result']['worker_exit_code']=1
    a['timestamps']=a['timestamps'][:1];a['image_means']=a['image_means'][:1]
    assert audit.pair_errors(a,b)==['failed_run']


def test_point_spread_is_independent_of_world_translation_and_rotation():
    points=np.array([[0,0,0],[1,0,0],[0,2,0],[1,2,3]],dtype=float)
    rotation=np.array([[0,-1,0],[1,0,0],[0,0,1]])
    a=audit.spread(points);b=audit.spread(points@rotation.T+[100,-200,50])
    np.testing.assert_allclose(list(a.values()),list(b.values()))
