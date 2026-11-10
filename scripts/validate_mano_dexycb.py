"""使用本地 DexYCB 真图验证 MediaPipe→双目骨架→MANO，不用真值喂给检测或拟合。"""
import argparse
import json
import subprocess
from pathlib import Path
import numpy as np
import yaml
from egohand.core.pipeline import EgoHandEngine,ROOT
from egohand.detection.mediapipe_detector import MediaPipeHandDetector
from egohand.sources.dexycb import DexYCBSource

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--start',type=int,default=9)
    parser.add_argument('--end',type=int,default=30)
    parser.add_argument('--output',type=Path,default=ROOT/'results'/'mano'/'dexycb_validation.json')
    args=parser.parse_args()
    detector=yaml.safe_load((ROOT/'configs'/'detector.yaml').read_text(encoding='utf8'))
    config=yaml.safe_load((ROOT/'configs'/'mano.yaml').read_text(encoding='utf8'))
    options={k:detector[k] for k in ('num_hands','min_hand_detection_confidence','min_hand_presence_confidence','min_tracking_confidence')}
    detectors=[MediaPipeHandDetector(ROOT/detector['model_path'],mode='IMAGE',**options) for _ in range(2)]
    source=DexYCBSource(ROOT/'samples/date/DexYCB')
    engine=EgoHandEngine({'use_epipolar_error':False,'use_reprojection_error':False},detectors=detectors)
    rows=[];request=None;chosen=None
    try:
        engine.start(source)
        for index in range(args.start,min(args.end+1,len(source.rows))):
            source.seek(index);frame=source.read()
            pair={'dataset_id':'dexycb','sync_quality':'DATASET_DOCUMENTED','pair_id':frame.metadata['dataset_pair_id'],'source':frame.metadata['synchronization_evidence']}
            result=engine.process(frame,dataset_pairing=pair)
            rows.append({'frame':index,'left_detections':len(result.left_detections),'right_detections':len(result.right_detections),
                         'valid_joints':[h.valid_joint_count for h in result.hands]})
            hands=[]
            for hand in result.hands:
                xyz=np.asarray([j.xyz_m for j in hand.joints]);valid=np.asarray([j.valid for j in hand.joints])&np.isfinite(xyz).all(axis=1)
                if valid.sum()>=config['min_joints']:
                    hands.append({'side':hand.handedness,'track_id':hand.track_id,'points':np.where(valid[:,None],xyz,0).tolist(),'valid':valid.tolist()})
            if hands and frame.metadata['ground_truth_mano']:
                chosen=index
                request={'frame':index,'sequence':result.session_id,'prediction':True,'ground_truth':True,'hands':hands,'truth':frame.metadata['ground_truth_mano']}
                break
    finally:engine.close()
    report={'dataset':'DexYCB','sequence':str(source.sequence),'frames_examined':rows,
            'use_epipolar_error':False,'use_reprojection_error':False,'chosen_frame':chosen,
            'note':'单帧流程验证，非数据集整体精度；真值未用于检测、双目重建和预测拟合。'}
    if request:
        process=subprocess.run([config['python'],str(ROOT/'scripts'/'mano_worker.py')],input=json.dumps(request)+'\n',
            text=True,encoding='utf8',capture_output=True,timeout=120,check=True)
        reply=json.loads(process.stdout)
        report['errors']=reply['errors']
        report['prediction']=[{k:v for k,v in h.items() if k not in ('vertices','faces')} for h in reply['predicted']]
        report['truth_meshes']=len(reply['truth'])
        # 同手侧、同 MANO 拓扑才有逐顶点误差；不做对齐或缩放。
        if len(reply['predicted'])==len(reply['truth'])==1 and reply['predicted'][0]['side']==reply['truth'][0]['side']:
            p=np.asarray(reply['predicted'][0]['vertices']);g=np.asarray(reply['truth'][0]['vertices'])
            report['mean_vertex_error_mm']=float(np.linalg.norm(p-g,axis=1).mean()*1000)
    else:report['errors']=['所选帧范围内未找到足够的有效关节和官方 MANO 标注']
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
    print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
