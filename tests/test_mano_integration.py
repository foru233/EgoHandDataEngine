"""基于已下载的 DexYCB 和官方 MANO 文件验证真实模型与 GUI 接口。"""
import json
import subprocess
from pathlib import Path

import numpy as np
import pytest
import yaml

from egohand.core.pipeline import ROOT
from egohand.sources.dexycb import DexYCBSource

CONFIG=yaml.safe_load((ROOT/'configs'/'mano.yaml').read_text(encoding='utf8'))
PYTHON=Path(CONFIG['python'])
MODEL=ROOT/CONFIG['models']
DATA=ROOT/'samples'/'date'/'DexYCB'
READY=PYTHON.is_file() and (MODEL/'MANO_LEFT.pkl').is_file() and (MODEL/'MANO_RIGHT.pkl').is_file() and DATA.is_dir()
pytestmark=[pytest.mark.assets,pytest.mark.skipif(not READY,reason='需要本机 MANO 模型、PyTorch 和 DexYCB 数据')]

def test_dexycb_mano_mesh_and_fitting_use_same_frame():
    source=DexYCBSource(DATA);source.open();source.start();source.seek(15)
    frame=source.read();source.close()
    assert frame.metadata['ground_truth_mano']
    gt=np.asarray(frame.metadata['ground_truth_left_camera'][0]['xyz_m'])
    request={'frame':frame.frame_id,'sequence':'test','request_id':1,'prediction':True,'ground_truth':True,
        'truth':frame.metadata['ground_truth_mano'],
        'hands':[{'side':'right','track_id':1,'points':gt.tolist(),'valid':[True]*21}]}
    next_request={**request,'frame':frame.frame_id+1,'request_id':3,'ground_truth':False}
    process=subprocess.run([str(PYTHON),str(ROOT/'scripts'/'mano_worker.py')],
        input=json.dumps(request)+'\n'+json.dumps(next_request)+'\n',capture_output=True,text=True,encoding='utf8',timeout=60,check=True)
    reply,next_reply=[json.loads(line) for line in process.stdout.splitlines()]
    assert not reply['errors']
    assert reply['frame']==frame.frame_id and reply['request_id']==1
    assert len(reply['predicted'])==len(reply['truth'])==1
    assert reply['predicted'][0]['error_m']<.02
    assert reply['predicted'][0]['temporal_used'] is False
    assert next_reply['predicted'][0]['temporal_used'] is True
    # 官方 MANO 的可比关节需与数据集本帧左相机标注一致，拇指 CMC 不比较。
    mapping=[0,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
    np.testing.assert_allclose(reply['truth'][0]['joints20'],gt[mapping],atol=1e-5)
    assert np.asarray(reply['truth'][0]['vertices']).shape==(778,3)

def test_missing_or_sparse_skeleton_is_not_fabricated():
    request={'frame':1,'sequence':'test','request_id':2,'prediction':True,'ground_truth':True,'truth':[],
        'hands':[{'side':'Right','track_id':1,'points':np.zeros((21,3)).tolist(),'valid':[False]*21}]}
    process=subprocess.run([str(PYTHON),str(ROOT/'scripts'/'mano_worker.py')],input=json.dumps(request)+'\n',
        capture_output=True,text=True,encoding='utf8',timeout=60,check=True)
    reply=json.loads(process.stdout)
    assert reply['predicted']==[] and reply['truth']==[]
    assert any('有效关节不足' in s for s in reply['errors'])
    assert any('官方 MANO 标注' in s for s in reply['errors'])

def test_show3d_official_mano_uses_same_frame_and_left_camera():
    from egohand.sources.show3d import SHOW3DSource,world_to_camera
    scene=ROOT/'samples/date/SHOW3D/scenes/YQB827/none_wave-hands_95e6'
    mano=ROOT/'samples/date/SHOW3D/hand_pose/v3/scenes/YQB827/none_wave-hands_95e6/hand_pose_mano.json'
    if not mano.is_file():pytest.skip('需要该 SHOW3D 场景的官方 MANO 标注')
    source=SHOW3DSource(scene);source.open();source.start();source.seek(800);frame=source.read();source.close()
    request={'frame':800,'sequence':'show3d-test','request_id':4,'prediction':False,'ground_truth':True,
             'truth':frame.metadata['ground_truth_mano']}
    process=subprocess.run([str(PYTHON),str(ROOT/'scripts/mano_worker.py')],input=json.dumps(request)+'\n',
        capture_output=True,text=True,encoding='utf8',timeout=60,check=True)
    reply=json.loads(process.stdout)
    assert not reply['errors'] and {h['side'] for h in reply['truth']}=={'left','right'}
    right=next(h for h in reply['truth'] if h['side']=='right')
    show=frame.metadata['show3d']
    umetrack=world_to_camera(show['ground_truth']['right']['world_mm'],show['calibration'][0].t_world_from_camera)[:20]/1000
    assert np.linalg.norm(np.asarray(right['joints20'])-umetrack,axis=1).mean()<.03
