"""Run with an existing Torch interpreter; exports official UmeTrack FK only.

No MANO assets, package installation, mesh reconstruction, or GUI imports.
"""
import argparse
import hashlib
import json
import sys
import tarfile
from pathlib import Path


def rigid_transform(j):
    # 官方字段顺序为 wxyz，平移为世界坐标米；构造 wrist→world 的齐次变换。
    import numpy as np
    w, x, y, z = np.asarray(j['quaternion_wxyz'], dtype=float)
    norm = (w*w+x*x+y*y+z*z)**.5
    w, x, y, z = np.array([w,x,y,z])/norm
    T = np.eye(4)
    T[:3,:3] = [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    T[:3,3] = j['translation_xyz']
    return T


def main():
    # 通过已有独立 Torch 环境调用官方 FK；不加载 MANO 许可资产，也不重写手模型蒙皮。
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--toolkit-root',type=Path,required=True)
    parser.add_argument('--archive',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    sys.path.insert(0,str(args.toolkit_root.resolve()))
    import numpy as np
    import torch
    from hand_tracking_toolkit.hand_models.umetrack_hand_model import (
        from_json, forward_kinematics, UmeTrackHandPose)
    source_files=['hand_tracking_toolkit/hand_models/umetrack_hand_model.py',
                  'hand_tracking_toolkit/hand_models/umetrack_skinning.py']
    code_hashes={p:hashlib.sha256((args.toolkit_root/p).read_bytes()).hexdigest() for p in source_files}
    output={'model':'official_umetrack','unit':'m','coordinate_frame':'world',
            'torch_version':torch.__version__,'python':sys.executable,
            'archive_sha256':hashlib.sha256(args.archive.read_bytes()).hexdigest(),
            'official_code_sha256':code_hashes,'frames':{}}
    with tarfile.open(args.archive) as archive:
        shapes=json.load(archive.extractfile('__hand_shapes.json__'))
        # 静态手形毫米转米、关节变换和右手镜像均由官方实现处理。
        model=from_json(shapes['umetrack'])  # official mm -> m conversion
        with torch.no_grad():
            for name in sorted(n for n in archive.getnames() if n.endswith('.hands.json')):
                hands=json.load(archive.extractfile(name));record={}
                for side in ('left','right'):
                    if not hands.get(side) or not hands[side].get('umetrack_pose'):continue
                    pose=hands[side]['umetrack_pose']
                    T=rigid_transform(pose['T_world_from_wrist'])
                    hand=UmeTrackHandPose(torch.tensor(int(side=='right')),
                        torch.tensor(pose['joint_angles']),torch.tensor(T,dtype=torch.float32))
                    xyz,_,_=forward_kinematics(hand,model,requires_mesh=False)
                    xyz=xyz.cpu().numpy()
                    if xyz.shape!=(20,3) or not np.isfinite(xyz).all():
                        raise ValueError('Official FK did not yield 20 finite canonical landmarks')
                    record[side]=xyz.tolist()
                output['frames'][name.split('.')[0]]=record
    output['status']='CONVERTED_OFFICIAL_MODEL' if output['frames'] else 'BLOCKED_BY_DATA'
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(output,indent=2,allow_nan=False),encoding='utf8')
    print(json.dumps({'archive':args.archive.name,'frames':len(output['frames']),'status':output['status']}))


if __name__=='__main__':main()
