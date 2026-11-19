"""Read legacy results under its Python >=3.11 runtime, export an isolated NPZ.

Reuses the existing triangulator and Hungarian associator with strict geometry
gates in a local config dictionary. Never edits the project's production YAML.
"""
from pathlib import Path
import json
import argparse
import sys
import numpy as np
from export_recording import VS030Pinhole,sha,write,PROJECT

sys.dont_write_bytecode=True
sys.path.insert(0,str(PROJECT/'src'))
from egohand.calibration.loader import load_calibration
from egohand.calibration.rectifier import StereoRectifier
from egohand.core.pipeline import default_config
from egohand.core.types import detection_from_dict
from egohand.stereo.triangulation import JointTriangulator
from egohand.stereo.hand_matching import StereoHandAssociator


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--input',type=Path,required=True);p.add_argument('--recording',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    base=args.source;inp=args.input;out=args.output
    if out.exists():raise FileExistsError(out)
    out.mkdir(parents=True)
    manifest=json.loads((inp/'manifest.json').read_text(encoding='utf8'))
    calibration=base/'processing_calibration.yaml'
    if manifest.get('calibration_hash')!=sha(calibration):
        calibration=args.recording/'calibration.yaml'
    a=VS030Pinhole(calibration)
    c=load_calibration(base/'processing_calibration.yaml')
    for k in ['K_left','K_right','D_left','D_right','R_right_left','t_right_left']:
        if not np.allclose(getattr(c,k),getattr(a.c,k),rtol=0,atol=1e-9):raise ValueError('Observation calibration mismatch '+k)
    rectifier=StereoRectifier(c);cfg=default_config()
    cfg.update(use_epipolar_error=True,use_reprojection_error=True,max_epipolar_px=5.,max_reprojection_px=5.)
    tri=JointTriangulator(rectifier,cfg);assoc=StereoHandAssociator(tri,cfg)
    rows=[json.loads(l) for l in (base/'results/frames.jsonl').read_text(encoding='utf8').splitlines()]
    frame_map=[json.loads(l) for l in (inp/'frame_map.jsonl').read_text().splitlines()]
    if len(rows)!=len(frame_map):raise ValueError('Frame counts differ')
    n=len(rows)
    uv=np.full((n,2,2,21,2),np.nan);valid=np.zeros((n,2,2,21),bool)
    labels=np.full((n,2,2),'',dtype='<U10');handscore=np.zeros((n,2,2))
    pairs=np.full((n,2),-1,int);geometry=np.zeros((n,2,2,21),bool)
    xyz=np.full((n,2,2,21,3),np.nan);epi=np.full((n,2,2,21),np.nan);cov=np.full((n,2,2,21,3,3),np.nan)
    sync=np.zeros(n,bool);ambiguous=np.zeros(n,bool);baselinexyz=np.full((n,2,21,3),np.nan)
    timestamps=np.array([r['timestamp_ns'] for r in rows],np.int64)
    with (out/'observations.jsonl').open('w',encoding='utf8') as stream:
        for i,(row,mapping) in enumerate(zip(rows,frame_map)):
            if (row['frame_id'],row['timestamp_ns'])!=(mapping['source_frame_index'],mapping['timestamp_ns']):
                raise ValueError(f'Exact-frame join failed {i}')
            diag=row['diagnostics']
            if diag.get('output_geometry')!='rectified':raise ValueError('Unknown saved pixel geometry')
            sync[i]=mapping['stereo_pair_transport_valid'] and diag.get('sync_quality')=='USER_CONFIRMED_SAME_BUNDLE'
            detections=[]
            for side,name in enumerate(['left','right']):
                ds=[detection_from_dict(d) for d in row.get(name+'_detections',[])]
                if len(ds)>2:raise ValueError('More candidates than the bimanual schema supports')
                detections.append(ds)
                for j,d in enumerate(ds):
                    pixels=a.rectified_pixels_to_view(d.pixel_landmarks,side)
                    uv[i,side,j]=pixels
                    inside=np.isfinite(pixels).all(1)&(pixels>=0).all(1)&(pixels<a.size).all(1)
                    good=d.valid&inside
                    indices=np.flatnonzero(good);xy=np.rint(pixels[indices]).astype(int)
                    xy=np.minimum(np.maximum(xy,0),np.array(a.size)-1)
                    good[indices]&=a.valid[side][xy[:,1],xy[:,0]]
                    valid[i,side,j]=good;labels[i,side,j]=d.handedness.lower();handscore[i,side,j]=d.handedness_score
            diagnostics={}
            if sync[i]:
                matches,ambiguity=assoc.associate(*detections,timestamp_ns=int(timestamps[i]))
                ambiguous[i]=ambiguity;diagnostics=assoc.diagnostics
                for m in matches:
                    pairs[i,m.left_index]=m.right_index
                    for k,joint in enumerate(m.joints):
                        geometry[i,m.left_index,m.right_index,k]=joint.valid
                        if joint.valid:xyz[i,m.left_index,m.right_index,k]=joint.xyz_m
                        epi[i,m.left_index,m.right_index,k]=joint.epipolar_error_px if joint.epipolar_error_px is not None else np.nan
                        if joint.position_covariance_m2 is not None:cov[i,m.left_index,m.right_index,k]=joint.position_covariance_m2
            for hand in row['hands']:
                label=hand['handedness'].lower()
                if label not in ['left','right']:continue
                side=['left','right'].index(label)
                for k,j in enumerate(hand['joints']):
                    if j['valid']:baselinexyz[i,side,k]=np.asarray(j['xyz_m'],float)
            stream.write(json.dumps(dict(frame_id=row['frame_id'],timestamp_ns=row['timestamp_ns'],
                stereo_transport_admitted=bool(sync[i]),strict_matches=pairs[i].tolist(),ambiguous=bool(ambiguous[i]),
                original_confidence='MediaPipe has no per-joint confidence in saved HandDetection; valid masks only',
                geometry_status='strict gates in isolated config; inherited calibration and sync assumptions'),allow_nan=False)+'\n')
    np.savez_compressed(out/'observations.npz',uv=uv,valid=valid,labels=labels,handedness_score=handscore,
          strict_pairs=pairs,geometry_valid=geometry,triangulated_xyz=xyz,triangulated_covariance_m2=cov,epipolar_error_px=epi,
          timestamps_ns=timestamps,stereo_admitted=sync,ambiguous=ambiguous,baseline_xyz=baselinexyz,
          K=np.stack(a.K),T_common_from_view=np.stack(a.T_common_from_view))
    report=dict(frames=n,exact_frame_join=True,source=str(base),source_results_sha256=sha(base/'results/frames.jsonl'),
          config=cfg,stereo_admitted_frames=int(sync.sum()),strict_pairs=int((pairs>=0).sum()),
          strict_stereo_joints=int(geometry.sum()),ambiguous_frames=int(ambiguous.sum()),
          left_observed_frames=int(valid[:,0].any((1,2)).sum()),right_observed_frames=int(valid[:,1].any((1,2)).sum()),
          per_joint_confidence_available=False,assumed_observation_weight='binary validity, not hand-level scores',
          gt_available=False,baseline_kind='selected same-recording saved detector observations',
          source_unchanged=True)
    write(out/'report.json',report)
    print(json.dumps(report,ensure_ascii=True),flush=True)


if __name__=='__main__':main()
