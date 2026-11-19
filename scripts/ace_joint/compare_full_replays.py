"""Compare the published geometry on exact shared frames and observations."""
import argparse
from itertools import zip_longest
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

from joint_work_paths import PROJECT

sys.path.insert(0,str(PROJECT/'src'))
from egohand.body.smpl_mapping import SMPL_JOINT_NAMES
from egohand.calibration.loader import load_calibration
from egohand.vio.hand_pose_binding import consumer_result_display_pose
from egohand.vio.hand_world import calibration_geometry_fingerprint
from egohand.vio.local_segments import display_reference_key

MP20=np.array([0,*range(2,21)])
LABELS={'legacy':'旧算法','previous':'上一版联合','one_pass':'一次重建版'}


def read_json(path):
    return json.loads(path.read_text(encoding='utf8'))


def identity(row):
    return int(row['frame_id']),int(row['timestamp_ns'])


def read_replay(root, keys):
    count=len(keys)
    index={key:i for i,key in enumerate(keys)}
    present=np.zeros(count,bool)
    body=np.full((count,24,3),np.nan)
    hand_world=np.full((count,2,20,3),np.nan)
    hand_camera=np.full_like(hand_world,np.nan)
    cameras=np.zeros(count,bool)
    refs=np.full(count,None,dtype=object)
    original_count=0
    with (root/'results/frames.jsonl').open(encoding='utf8') as rows, (root/'derived_pose.jsonl').open(encoding='utf8') as derived:
        for frame_line,pose_line in zip_longest(rows,derived):
            if frame_line is None or pose_line is None:raise ValueError(f'Incomplete replay: {root}')
            original_count+=1
            frame,pose=json.loads(frame_line),json.loads(pose_line)
            key=identity(frame)
            if key!=identity(pose):raise ValueError(f'Replay and pose identities differ: {root}')
            if key not in index:continue
            row=index[key]
            if present[row]:raise ValueError(f'Duplicate frame {key}: {root}')
            present[row]=True
            message=frame['diagnostics'].get('vio_pose') or frame['diagnostics'].get('live_vio_pose')
            transform=consumer_result_display_pose(SimpleNamespace(**frame))
            cameras[row]=transform is not None
            if message is not None:refs[row]=repr(display_reference_key(message))
            if pose.get('body'):
                body[row]=[pose['body']['joints'][joint] for joint in SMPL_JOINT_NAMES]
            for hand in (pose.get('mano') or {}).get('predicted',[]):
                side=('left','right').index(hand['side'].lower())
                joints=np.asarray(hand['joints20'],float)
                if not np.isfinite(joints).all() or joints.shape!=(20,3):
                    raise ValueError(f'Invalid MANO joints: {root}, {key}')
                if hand['coordinate_frame']=='body_world':
                    hand_world[row,side]=joints
                    if transform is not None:hand_camera[row,side]=(joints-transform[:3,3])@transform[:3,:3]
                elif hand['coordinate_frame']=='camera_left_optical':
                    hand_camera[row,side]=joints
                    if transform is not None:hand_world[row,side]=joints@transform[:3,:3].T+transform[:3,3]
                else:raise ValueError(f'Unrecognized MANO coordinate frame: {root}')
    return dict(count=original_count,present=present,body=body,hand_world=hand_world,
                hand_camera=hand_camera,cameras=cameras,refs=refs)


def stats(values):
    if not len(values):return dict(samples=0,mean=None,median=None,p95=None)
    if not np.isfinite(values).all():raise ValueError('Nonfinite metric sample')
    return dict(samples=len(values),mean=float(np.mean(values)),median=float(np.median(values)),
                p95=float(np.percentile(values,95)))


def strict_reference(archive):
    count=len(archive['timestamps_ns'])
    measured=np.full((count,2,20,2),np.nan)
    valid=np.zeros((count,2,20),bool)
    for row,pairs in enumerate(archive['strict_pairs']):
        selected=np.flatnonzero(pairs>=0)
        if len(selected)!=1 or not archive['stereo_admitted'][row] or archive['ambiguous'][row]:continue
        left=int(selected[0]);right=int(pairs[left])
        if (archive['labels'][row,0,left],archive['labels'][row,1,right])!=('right','right'):continue
        both=archive['geometry_valid'][row,left,right,MP20]
        for eye,candidate in enumerate((left,right)):
            measured[row,eye]=archive['uv'][row,eye,candidate,MP20]
            valid[row,eye]=both&archive['valid'][row,eye,candidate,MP20]
    return measured,valid


def projection(hand,K,left_from_view):
    view_from_left=np.linalg.inv(left_from_view)
    points=np.einsum('vij,nkj->nvki',view_from_left[:,:3,:3],hand)+view_from_left[None,:,:3,3][:,:,None,:]
    homogeneous=np.einsum('vij,nvkj->nvki',K,points)
    with np.errstate(divide='ignore',invalid='ignore'):
        uv=homogeneous[...,:2]/homogeneous[...,2:]
    return uv,points[...,2]


def temporal_metrics(records,keys,mask,field,hand=False):
    dt=np.diff(np.array([key[1] for key in keys],dtype=np.int64)).astype(float)/1e9
    good=(dt>0)&(dt<=.1)&(np.diff(np.array([key[0] for key in keys]))==1)
    selected=mask[:-2]&mask[1:-1]&mask[2:]&good[:-1]&good[1:]
    for data in records.values():
        ref=data['refs']
        selected &= (ref[:-2]==ref[1:-1])&(ref[1:-1]==ref[2:])
    starts=np.flatnonzero(selected)
    result={}
    for name,data in records.items():
        points=data[field][:,1] if hand else data[field]
        points=(points-points[:,:1])[:,1:]
        previous=(points[starts+1]-points[starts])/dt[starts,None,None]
        following=(points[starts+2]-points[starts+1])/dt[starts+1,None,None]
        acceleration=(following-previous)/((dt[starts]+dt[starts+1])*.5)[:,None,None]
        result[name]=stats(np.linalg.norm(acceleration,axis=-1).reshape(-1))
    return dict(triplets=len(starts),acceleration_m_s2=result)


def compare(roots):
    newest=roots['one_pass']
    delivery=read_json(newest/'delivery_validation.json')
    if delivery.get('status')!='passed':raise ValueError('Full GUI delivery validation did not pass')
    work=Path(read_json(newest/'pipeline.json')['work'])
    with (work/'input/frame_map.jsonl').open() as stream:
        keys=[(row['source_frame_index'],row['timestamp_ns']) for row in map(json.loads,stream)]
    if len(keys)!=delivery['full_frames'] or len(set(keys))!=len(keys):raise ValueError('Bad source frame map')
    with np.load(work/'observations/observations.npz') as archive:
        observations={key:archive[key] for key in archive.files}
    if not np.array_equal(observations['timestamps_ns'],[key[1] for key in keys]):
        raise ValueError('Observation timestamps differ from source frame map')
    calibrations={calibration_geometry_fingerprint(load_calibration(root/'processing_calibration.yaml'))
                  for root in roots.values()}
    if len(calibrations)!=1:raise ValueError('Replay camera calibrations differ')
    records={name:read_replay(root,keys) for name,root in roots.items()}
    if any(not data['present'].all() or data['count']!=len(keys) for data in records.values()):
        raise ValueError('Results do not cover the same raw frame identities')
    pixels,measurement_mask=strict_reference(observations)
    strict_frames=measurement_mask.any((1,2))
    coverage={}
    for name,data in records.items():
        data['body_valid']=np.isfinite(data['body']).all((1,2))
        hand_valid=np.isfinite(data['hand_world']).all((2,3))|np.isfinite(data['hand_camera']).all((2,3))
        data['right_valid']=hand_valid[:,1];data['left_valid']=hand_valid[:,0]
        manifest=read_json(roots[name]/'unified_smpl_manifest.json')
        coverage[name]=dict(frames=data['count'],camera=int(data['cameras'].sum()),body=int(data['body_valid'].sum()),
                            left_hand=int(data['left_valid'].sum()),right_hand=int(data['right_valid'].sum()),
                            right_on_strict_stereo=int((data['right_valid']&strict_frames).sum()),
                            configured_forehead_mm=float(manifest.get('camera_forehead_offset_mm') or 0.))
    projections={name:projection(data['hand_camera'][:,1],observations['K'],observations['T_common_from_view'])
                 for name,data in records.items()}
    groups={}
    for scope,names in (('all_three',tuple(roots)),('joint_pair',('previous','one_pass'))):
        group={name:records[name] for name in names}
        common_right=np.logical_and.reduce([data['right_valid'] for data in group.values()])
        common_body=np.logical_and.reduce([data['body_valid'] for data in group.values()])
        common_pixels=measurement_mask&common_right[:,None,None]
        projects={}
        for name in names:
            predicted,depth=projections[name]
            projects[name]=np.isfinite(predicted).all(-1)&(depth>.01)
        invalid={name:int((common_pixels&~valid).sum()) for name,valid in projects.items()}
        shared=common_pixels&np.logical_and.reduce(list(projects.values()))
        reprojection={name:stats(np.linalg.norm(projections[name][0]-pixels,axis=-1)[shared]) for name in names}
        groups[scope]=dict(common_body_frames=int(common_body.sum()),common_right_frames=int(common_right.sum()),
                           compared_pixel_frames=int(shared.any((1,2)).sum()),
                           inadmissible_projected_samples=invalid,reprojection_px=reprojection,
                           hand_temporal=temporal_metrics(group,keys,common_right,'hand_world',hand=True),
                           body_temporal=temporal_metrics(group,keys,common_body,'body'))
    return dict(status='passed',same_raw_frame_count=len(keys),strict_right_stereo_frames=int(strict_frames.sum()),
                independent_3d_ground_truth=False,coverage=coverage,groups=groups,
                result_paths={name:str(root) for name,root in roots.items()})


def markdown(report):
    count=report['same_raw_frame_count'];strict=report['strict_right_stereo_frames']
    rows=['# 三版全量公平对比','',
          f'三个结果的原始帧号、时间戳和处理标定一致，共 {count} 帧。左右目均识别为右手、且双目配对通过几何检查的参考帧有 {strict} 帧。','',
          '| 算法 | 可用相机 | 人体网格 | 左手网格 | 右手网格 | 严格双目帧有右手网格 | 相机至额头 |',
          '|---|---:|---:|---:|---:|---:|---:|']
    for name,data in report['coverage'].items():
        keys=('camera','body','left_hand','right_hand','right_on_strict_stereo')
        counts=' | '.join(str(data[key]) for key in keys)
        rows.append(f'| {LABELS[name]} | {counts} | {data["configured_forehead_mm"]:g} mm |')
    for scope,data in report['groups'].items():
        label='三版共同帧' if scope=='all_three' else '两版联合算法共同帧'
        samples=next(iter(data['reprojection_px'].values()))['samples']
        rows.extend(['',f'## {label}','',
          f'共同人体 {data["common_body_frames"]} 帧，右手 {data["common_right_frames"]} 帧；实际重投影比较 {data["compared_pixel_frames"]} 帧、{samples} 个相同像素样本。',
          f'时间指标使用同一连续三帧：人体 {data["body_temporal"]["triplets"]} 组，手部 {data["hand_temporal"]["triplets"]} 组。','',
          '| 算法 | 右手重投影中位 / P95（px） | 无效投影样本 | 手部加速度中位 / P95（m/s²） | 人体加速度中位 / P95（m/s²） |',
          '|---|---:|---:|---:|---:|'])
        def pair(values):
            return '—' if values['median'] is None else f'{values["median"]:.2f} / {values["p95"]:.2f}'
        for name,pixels in data['reprojection_px'].items():
            rows.append(f'| {LABELS[name]} | {pair(pixels)} | {data["inadmissible_projected_samples"][name]} | '
                        f'{pair(data["hand_temporal"]["acceleration_m_s2"][name])} | '
                        f'{pair(data["body_temporal"]["acceleration_m_s2"][name])} |')
    rows.extend(['','重投影使用实际发布的 MANO 网格关节，转回各自左相机坐标，再用同一套左右目标定投影。拇指 CMC 不纳入。三版共同集可能偏向三版都能识别的画面，所以另列两版联合算法的更大共同集。',
                 '时间指标以手腕或骨盆为原点，排除坐标段切换和过大的时间间隔。加速度包含真实动作，不能单独证明抖动更少。',
                 '本录制没有独立三维真值。重投影衡量与输入观测的吻合度。三版按实际保存配置比较，相机轨迹、初始化和额头偏移均可能不同，因此不是单因素消融。',''])
    return '\n'.join(rows)


def main():
    parser=argparse.ArgumentParser()
    for name in LABELS:parser.add_argument('--'+name.replace('_','-'),required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    report=compare({name:getattr(args,name).resolve() for name in LABELS})
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'comparison.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    (args.output/'comparison.md').write_text(markdown(report),encoding='utf8')
    print(markdown(report),flush=True)


if __name__=='__main__':
    main()
