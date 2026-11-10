"""真实 SHOW3D 单帧：校正图检测、双目预测、官方 UmeTrack 与预测 MANO 对照。"""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import yaml
from scipy.optimize import linear_sum_assignment
from egohand.core.pipeline import ROOT,EgoHandEngine
from egohand.core.types import serializable
from egohand.benchmarks.hot3d import CANONICAL_MP
from egohand.detection.landmarks import BONES,JOINT_NAMES
from egohand.detection.mediapipe_detector import MediaPipeHandDetector
from egohand.sources.show3d import SHOW3DSource,SHOW3DFrameProcessor,world_to_camera
from egohand.visualization.skeleton_view import optical_to_display

def evaluate(scene,index,gates):
    cfg=yaml.safe_load((ROOT/'configs/detector.yaml').read_text(encoding='utf8'))
    opts={k:cfg[k] for k in ('num_hands','min_hand_detection_confidence','min_hand_presence_confidence','min_tracking_confidence')}
    detectors=[MediaPipeHandDetector(ROOT/cfg['model_path'],mode='IMAGE',**opts) for _ in range(2)]
    source=SHOW3DSource(scene,view_mode='prediction')
    engine=EgoHandEngine({'use_epipolar_error':gates,'use_reprojection_error':gates},detectors=detectors)
    processor=SHOW3DFrameProcessor()
    try:
        engine.start(source);source.seek(index);raw=source.read();result=processor.process(engine,raw,'prediction')
        return raw,engine.last_frame,result
    finally:engine.close()

def match_2d(detections,truth):
    sides=sorted(truth)
    costs=np.full((len(detections),len(sides)),1e6)
    for i,det in enumerate(detections):
        for j,side in enumerate(sides):
            xy,visible=truth[side];valid=visible&det.valid&np.isfinite(xy).all(axis=1)
            if valid.sum()>=8:costs[i,j]=np.linalg.norm(det.pixel_landmarks[valid]-xy[valid],axis=1).mean()
    if not costs.size:return [],costs.tolist()
    rows,cols=linear_sum_assignment(costs)
    return [{'detection_index':int(i),'ground_truth_side':sides[j],'mean_px':float(costs[i,j]),
             'detector_label':detections[i].handedness} for i,j in zip(rows,cols) if costs[i,j]<50],costs.tolist()

def save_2d(frame,result,path):
    ims=[]
    for image,detections,truth in zip((frame.left_image,frame.right_image),
                                        (result.left_detections,result.right_detections),frame.debug_ground_truth_uv):
        img=image.copy()
        for det in detections:
            pts=det.pixel_landmarks
            for a,b in BONES:
                if det.valid[a] and det.valid[b]:cv2.line(img,tuple(np.rint(pts[a]).astype(int)),tuple(np.rint(pts[b]).astype(int)),(20,230,70),2,cv2.LINE_AA)
            for i in np.flatnonzero(det.valid):cv2.circle(img,tuple(np.rint(pts[i]).astype(int)),4,(20,230,70),-1,cv2.LINE_AA)
        for xy,mask in truth.values():
            for i in np.flatnonzero(mask):cv2.circle(img,tuple(np.rint(xy[i]).astype(int)),5,(225,65,235),2,cv2.LINE_AA)
        img=cv2.resize(img,(512,640))
        cv2.rectangle(img,(0,0),(512,44),(20,20,20),-1)
        cv2.putText(img,'Prediction green / UmeTrack GT magenta',(7,28),cv2.FONT_HERSHEY_SIMPLEX,.52,(255,255,255),1,cv2.LINE_AA)
        ims.append(img)
    cv2.imwrite(str(path),np.concatenate(ims,axis=1))

def save_3d(prediction,gt,mesh,truth_mesh,path):
    fig=plt.figure(figsize=(16,5.8),layout='constrained')
    predicted=optical_to_display(prediction);ground=optical_to_display(gt)
    vertices=optical_to_display(mesh['vertices']) if mesh else None
    gt_vertices=optical_to_display(truth_mesh['vertices']) if truth_mesh else None
    for i,title in enumerate(('Reconstructed joints vs UmeTrack','Fitted MANO vs UmeTrack joints','Fitted MANO vs official MANO'),1):
        ax=fig.add_subplot(1,3,i,projection='3d');ax.set_title(title)
        if i!=3:ax.scatter(ground[:,0],ground[:,1],ground[:,2],s=16,c='#bd35d1',label='UmeTrack GT')
        if i==1:ax.scatter(predicted[:,0],predicted[:,1],predicted[:,2],s=12,c='#18a76d',label='Stereo prediction')
        elif vertices is not None:
            faces=np.asarray(mesh['faces'],int)
            ax.add_collection3d(Poly3DCollection(vertices[faces],facecolor=(1.,.52,.12,.35),edgecolor='none'))
            ax.scatter([],[],[],c='#e88b2f',label='Fitted MANO')
        if i==3 and gt_vertices is not None:
            faces=np.asarray(truth_mesh['faces'],int)
            ax.add_collection3d(Poly3DCollection(gt_vertices[faces],facecolor=(.65,.12,.8,.12),edgecolor=(.55,.08,.75,.18),linewidth=.15))
            ax.scatter([],[],[],c='#a828c7',label='Official MANO')
        allpts=np.vstack([ground,predicted]+([vertices] if vertices is not None else [])+([gt_vertices] if gt_vertices is not None else []))
        center=(allpts.min(axis=0)+allpts.max(axis=0))/2;span=max(np.ptp(allpts,axis=0).max()/2,.04)
        ax.set_xlim(center[0]-span,center[0]+span);ax.set_ylim(center[1]-span,center[1]+span);ax.set_zlim(center[2]-span,center[2]+span)
        ax.set_xlabel('camera X (m)');ax.set_ylabel('camera Z (m)');ax.set_zlabel('-camera Y (m)')
        ax.view_init(elev=24,azim=-65);ax.legend(loc='upper left',fontsize=8)
    fig.savefig(path,dpi=160);plt.close(fig)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--frame',type=int,default=800)
    parser.add_argument('--output',type=Path,default=ROOT/'results/mano/show3d_wave_frame800')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    scene=ROOT/'samples/date/SHOW3D/scenes/YQB827/none_wave-hands_95e6'
    raw0,prepared0,r0=evaluate(scene,args.frame,True)
    raw,prepared,r=evaluate(scene,args.frame,False)
    checks=[]
    for det,truth in zip((r.left_detections,r.right_detections),prepared.debug_ground_truth_uv):
        pairs,costs=match_2d(det,truth);checks.append({'matches':pairs,'cost_matrix_mean_px':costs})
    gt_by_side={side:world_to_camera(entry['world_mm'],raw.metadata['show3d']['calibration'][0].t_world_from_camera)/1000
                for side,entry in raw.metadata['show3d']['ground_truth'].items()}
    matched=[h for h in r.hands if h.valid_joint_count>0 and h.track_id>0]
    cfg=yaml.safe_load((ROOT/'configs/mano.yaml').read_text(encoding='utf8'))
    hands=[]
    for h in matched:
        xyz=np.array([j.xyz_m for j in h.joints]);valid=np.array([j.valid for j in h.joints])&np.isfinite(xyz).all(axis=1)
        hands.append({'side':h.handedness,'track_id':h.track_id,'points':np.where(valid[:,None],xyz,0).tolist(),'valid':valid.tolist()})
    request={'frame':args.frame,'sequence':r.session_id,'prediction':True,'ground_truth':True,'hands':hands,'truth':raw.metadata.get('ground_truth_mano',[])}
    proc=subprocess.run([cfg['python'],str(ROOT/'scripts/mano_worker.py')],input=json.dumps(request)+'\n',
                        text=True,encoding='utf8',capture_output=True,timeout=120,check=True)
    fit=json.loads(proc.stdout);mesh=fit['predicted'][0] if len(fit['predicted'])==1 else None
    side=matched[0].handedness.lower() if len(matched)==1 else None
    truth_mesh=next((m for m in fit['truth'] if m['side']==side),None)
    entries=[];mesh_gt_error=None;mano_vertex_error=None;mano_joint_error=None
    if side in gt_by_side:
        # SHOW3D 第 21 点是 palm center，不属于官方 canonical20 映射。
        h=matched[0];gt=gt_by_side[side][:20]
        for i,mp in enumerate(CANONICAL_MP):
            if h.joints[mp].valid:
                pred=h.joints[mp].xyz_m;entries.append({'joint':JOINT_NAMES[mp],'gt_xyz_m':gt[i].tolist(),
                    'predicted_xyz_m':np.asarray(pred).tolist(),'error_mm':float(np.linalg.norm(pred-gt[i])*1000)})
        if mesh:
            # worker 的 MANO 20 点顺序与 canonical20 不同，按明确的 MediaPipe 下标重排。
            worker_mp=[0,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
            fit_joints=np.asarray(mesh['joints20'])[[worker_mp.index(k) for k in CANONICAL_MP]]
            # 拟合采用 DexYCB 指尖顶点；评测 SHOW3D 时替换为官方 MANO 指尖定义。
            fit_joints[:5]=np.asarray(mesh['vertices'])[[744,320,443,554,671]]
            mesh_gt_error=float(np.linalg.norm(fit_joints-gt,axis=1).mean()*1000)
            if truth_mesh and np.array_equal(np.asarray(mesh['faces']),np.asarray(truth_mesh['faces'])):
                mano_vertex_error=float(np.linalg.norm(np.asarray(mesh['vertices'])-np.asarray(truth_mesh['vertices']),axis=1).mean()*1000)
                mano_joint_error=float(np.linalg.norm(fit_joints-np.asarray(truth_mesh['joints20']),axis=1).mean()*1000)
            save_3d(np.asarray([h.joints[k].xyz_m for k in CANONICAL_MP if h.joints[k].valid]),gt,mesh,truth_mesh,args.output/'3d_comparison.png')
    save_2d(prepared,r,args.output/'rectified_2d_comparison.png')
    report={'scene':str(scene),'frame':args.frame,'inputs':'SHOW3D PinholePlane, rectified before detection',
      'default_gates':{'status':r0.reconstruction_status,'left_detections':len(r0.left_detections),'right_detections':len(r0.right_detections),'valid_3d':[h.valid_joint_count for h in r0.hands],'comparison':r0.diagnostics.get('show3d_gt_comparison')},
      'both_gates_disabled':{'status':r.reconstruction_status,'left_detections':len(r.left_detections),'right_detections':len(r.right_detections),'valid_3d':[h.valid_joint_count for h in r.hands],
          'comparison':r.diagnostics.get('show3d_gt_comparison'),'independent_2d_matches':checks,'joints':entries},
      'mano':{'predicted_mesh_count':len(fit['predicted']),'fit_errors':fit['errors'],
          'fit_to_predicted_skeleton_mm':mesh['error_m']*1000 if mesh else None,'compared_joints':mesh['joints_used'] if mesh else 0,
          'mano_landmarks_vs_umetrack_mm':mesh_gt_error,
          'official_mano_truth_available':bool(truth_mesh),'official_mano_truth_mesh_count':len(fit['truth']),
          'mano_mean_vertex_error_mm':mano_vertex_error,'mano_joint_error_mm':mano_joint_error,
          'comparison_note':'MANO vertex and landmark errors in original left camera coordinates; no alignment or scale adjustment'},
      'mano_annotation_revision':'81403d3e4254cef959027118bc668bb2f26c86f6',
      'mano_annotation_sha256':hashlib.sha256((ROOT/'samples/date/SHOW3D/hand_pose/v3/scenes/YQB827/none_wave-hands_95e6/hand_pose_mano.json').read_bytes()).hexdigest(),
      'mano_profile_sha256':hashlib.sha256((ROOT/'samples/date/SHOW3D/hand_pose/hand_profiles/YQB827/profile_mano.json').read_bytes()).hexdigest(),
      'toolkit_revision':'21ee64f0cd3c276e102f064094ac2ee81eb30cd5','no_ground_truth_used_in_prediction':True}
    (args.output/'report.json').write_text(json.dumps(serializable(report),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
    print(json.dumps(serializable({k:v for k,v in report.items() if k!='both_gates_disabled'}),ensure_ascii=False,indent=2))
    print('both gates disabled:',r.reconstruction_status,r.diagnostics.get('show3d_gt_comparison'))
    print('independent 2D matches:',checks)

if __name__=='__main__':main()
