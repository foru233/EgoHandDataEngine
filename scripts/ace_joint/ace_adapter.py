"""Execute official infer_video.main with ungated dumps and exact block offload."""
from pathlib import Path
import argparse
import json
import os
import sys
import time
import numpy as np

from joint_work_paths import ACE,MANO,is_joint_workspace
sys.dont_write_bytecode=True
os.environ['ACE_EGO_HAND_MANO_DIR']=str(MANO)
os.environ['VIDEOX_ATTENTION_TYPE']='SDPA'


def save_raw(pred,out,n,intr):
    keys=('global_orient','hand_pose','betas','cam_trans','direct_joints2d','direct_joints_cam','direct_wrist_cam','exists_2d','exists_3d')
    data={k:np.asarray(pred[k][:n]).copy() for k in keys}
    if any(len(v)!=n for v in data.values()):raise ValueError('ACE missing predicted frames')
    data['claim']=data['exists_2d']>.5
    data['visibility_finite']=np.isfinite(data['exists_2d'])
    data['existence_3d_finite']=np.isfinite(data['exists_3d'])
    data['exists_3d_gate_heuristic']=data['exists_3d']>.5
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(out/'raw_predictions.npz',**data)
    (out/'raw_metadata.json').write_text(json.dumps(dict(frames=n,intrinsics=intr,pred_intrinsics=pred.get('_pred_K'),
          pnp_accept_fraction=pred.get('_pnp_accept_frac'),slot_order=['left_hand','right_hand'],
          poses='rotation matrices; smplx MANO use_pca=False flat_hand_mean=False; camera translations in meters',
          exists_2d_scope='hand level, not per joint',scores_calibrated_probability=False,
          raw_nonfinite_counts={k:int((~np.isfinite(data[k])).sum()) for k in keys},
          gated_pose_values_never_used_as_raw=True),indent=2,allow_nan=False),encoding='utf8')


def move_nonblocks(model,device):
    import torch
    net=model.net;dit=net.backbone.dit
    for name,child in net.named_children():
        if name!='backbone':child.to(device)
    for name,child in dit.named_children():
        if name!='blocks':child.to(device)
    for p in dit.parameters(recurse=False):p.data=p.data.to(device)
    for name,b in dit.named_buffers(recurse=False):setattr(dit,name,b.to(device))
    for m in model.mano_models.values():m.to(device)
    model.device=torch.device(device)


def offload_blocks(model,device):
    """Immutable CPU weight store + Accelerate leaf hooks; no GPU->CPU copy.

    Repeated module.to(cpu) after forward crashed in Windows c10.dll on this
    host. Leaf offload keeps the original CPU values, loads them for compute,
    and releases temporary GPU parameters without copying weights back.
    """
    import torch
    from accelerate import cpu_offload
    move_nonblocks(model,device)
    for block in model.net.backbone.dit.blocks:
        cpu_offload(block,execution_device=torch.device(device),offload_buffers=False)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--video',type=Path,required=True);ap.add_argument('--camera',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--max-frames',type=int,default=5)
    ap.add_argument('--encode-width',type=int,default=832);ap.add_argument('--device',choices=['cuda-offload','cuda','cpu'],default='cuda-offload')
    args=ap.parse_args();out=args.out.resolve()
    if not is_joint_workspace(out):raise ValueError('Output outside joint workspace')
    if out.exists():raise FileExistsError(out)
    out.mkdir(parents=True)
    started=time.monotonic()
    from runtime_bootstrap import setup
    setup()
    import torch
    torch.set_num_threads(4)
    import infer_video as official
    from videox_fun.models.wan_transformer3d import Wan2_2Transformer3DModel
    loader=Wan2_2Transformer3DModel.from_pretrained.__func__
    def memory_loader(cls,*a,**kw):
        kw.setdefault('low_cpu_mem_usage',True)
        return loader(cls,*a,**kw)
    Wan2_2Transformer3DModel.from_pretrained=classmethod(memory_loader)
    original_model=official.GeoDitModel
    class LocalModel(original_model):
        def __init__(self,opt,device):
            super().__init__(opt,torch.device('cpu') if args.device=='cuda-offload' else device)
    official.GeoDitModel=LocalModel
    predict=official.predict_video
    def predict_capture(model,tap,ctrl,intr,mode,device,**kw):
        if args.device=='cuda-offload':offload_blocks(model,device)
        print('[adapter] predicting with',args.device,flush=True)
        pred=predict(model,tap,ctrl,intr,mode,device,**kw)
        n=min(official.frame_count(str(args.video)),args.max_frames) if args.max_frames else official.frame_count(str(args.video))
        save_raw(pred,out,n,intr)
        return pred
    official.predict_video=predict_capture
    os.chdir(ACE)
    sys.argv=['infer_video.py','--video',str(args.video.resolve()),'--camera',str(args.camera.resolve()),
        '--opt',str(ACE/'options/ace_ego_hand_k.yml'),'--ckpt',str(ACE/'checkpoints/ace_ego_hand_k.pt'),
        '--out',str(out),'--max_frames',str(args.max_frames),'--encode_w',str(args.encode_width),
        '--mode','tiled','--device','cpu' if args.device=='cpu' else 'cuda']
    official.main()
    (out/'runtime.json').write_text(json.dumps(dict(status='completed',elapsed_s=time.monotonic()-started,
          device=args.device,gpu_peak_allocated_bytes=torch.cuda.max_memory_allocated() if torch.cuda.is_available() else None,
          official_entrypoint='infer_video.main',precision='official BF16 backbone, FP32 LoRA/projectors',
          memory_adaptation='low_cpu_mem_usage loading and sequential CPU/GPU block movement; no quantization'),indent=2),encoding='utf8')
    print('ADAPTER COMPLETE',round(time.monotonic()-started,2),flush=True)


if __name__=='__main__':main()
