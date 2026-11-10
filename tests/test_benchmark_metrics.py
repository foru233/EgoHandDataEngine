import numpy as np
from egohand.benchmarks.metrics import match_2d,grouped_joint_errors,distribution
from egohand.core.types import HandDetection


def detection(shift=0):
    uv=np.column_stack((np.arange(21)+shift,np.arange(21)))
    return HandDetection('Unknown',.1,uv/640,uv)


def test_evaluation_identity_never_uses_predicted_3d():
    a,b=detection(),detection(200)
    truth={'left':(a.pixel_landmarks,np.ones(21,bool)),'right':(b.pixel_landmarks,np.ones(21,bool))}
    found,_=match_2d([b,a],truth)
    assert found=={0:'right',1:'left'}


def test_independent_2d_assignment_rejects_ambiguity():
    d=detection();truth={s:(d.pixel_landmarks,np.ones(21,bool)) for s in ['left','right']}
    assert not match_2d([d],truth)[0]


def test_coverage_includes_rejected_and_missing_joints():
    rows=[{'joint':'thumb','valid':True,'error_mm':12.}]+[{'joint':'thumb','valid':False,'error_mm':None} for _ in range(4)]
    result=grouped_joint_errors(rows,'joint')[0]
    assert result['coverage']==.2 and result['error_mm_mean']==12 and result['gt_joints']==5


def test_missing_metrics_are_none_not_zero():
    assert distribution([])['mean'] is None
    assert distribution([np.nan])['count']==0
