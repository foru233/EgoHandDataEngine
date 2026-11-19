"""ACE-only process: official tiled predictor, no co-resident VAE."""
from pathlib import Path
import argparse
import json
import os
import time
os.environ['VIDEOX_ATTENTION_TYPE']='SDPA'
os.environ.setdefault('PYTORCH_CUDA_ALLOC_CONF','max_split_size_mb:128')
from runtime_bootstrap import setup
from joint_work_paths import ACE,MANO,is_joint_workspace
os.environ['ACE_EGO_HAND_MANO_DIR']=str(MANO)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--latents',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args();out=args.out.resolve()
    if not is_joint_workspace(out):raise ValueError('Output outside joint workspace')
    out.mkdir(parents=True,exist_ok=True)
    latent_manifest=json.loads((args.latents/'latent_manifest.json').read_text())
    if latent_manifest['status']!='completed':raise ValueError('VAE stage incomplete')
    setup()
    import numpy as np
    import torch
    import yaml
    from ace_adapter import offload_blocks,save_raw
    from ace_ego_hand.inference import predict_video
    from ace_ego_hand.models.geodit_model import GeoDitModel
    from videox_fun.models.wan_transformer3d import Wan2_2Transformer3DModel
    torch.set_num_threads(4);os.chdir(ACE)
    loader=Wan2_2Transformer3DModel.from_pretrained.__func__
    def lowmem(cls,*a,**kw):kw.setdefault('low_cpu_mem_usage',True);return loader(cls,*a,**kw)
    Wan2_2Transformer3DModel.from_pretrained=classmethod(lowmem)
    config=yaml.safe_load((ACE/'options/ace_ego_hand_k.yml').read_text())
    model=GeoDitModel(config,torch.device('cpu'));model.load_inference(str(ACE/'checkpoints/ace_ego_hand_k.pt'))
    offload_blocks(model,'cuda');print('ACE OFFLOAD READY',flush=True)
    inp=Path(latent_manifest['input']);mapping=[json.loads(l) for l in (inp/'frame_map.jsonl').read_text().splitlines()]
    manifest=dict(status='running',input=str(inp),frames=latent_manifest['frames'],start=latent_manifest['start'],
                  inference_window=latent_manifest['window'],overlap=latent_manifest['overlap'],outputs=[],
                  temporal_context=latent_manifest['causal_context'],memory_strategy='separate VAE process + immutable CPU weights + leaf GPU offload')
    state=out/'sequence_manifest.json';state.write_text(json.dumps(manifest,indent=2),encoding='utf8')
    for row in latent_manifest['clips']:
        side,start,end=row['side'],row['start'],row['end'];directory=out/side/f'{start:06d}_{end:06d}'
        if (directory/'clip.json').exists():
            manifest['outputs'].append(dict(side=side,start=start,end=end,path=str(directory),reused=True));continue
        directory.mkdir(parents=True,exist_ok=False)
        camera=json.loads((inp/(side+'_camera.json')).read_text());intr={**camera['frames'][0]['intrinsics'],
                 'image_width':camera['image_width'],'image_height':camera['image_height']}
        ctrl=torch.load(row['path'],map_location='cpu',weights_only=True).float()
        started=time.monotonic();torch.cuda.reset_peak_memory_stats()
        print('ACE PREDICT',side,start,end,flush=True)
        pred=predict_video(model,15,ctrl,intr,'tiled',torch.device('cuda'),tile_w=22)
        save_raw(pred,directory,end-start,intr)
        report=dict(side=side,start=start,end=end,elapsed_s=time.monotonic()-started,
            timestamps_ns=[mapping[i]['timestamp_ns'] for i in range(start,end)],
            source_frames=[mapping[i]['source_frame_index'] for i in range(start,end)],
            gpu_peak_allocated_bytes=torch.cuda.max_memory_allocated(),claim_counts=(pred['exists_2d'][:end-start]>.5).sum(0).tolist(),
            raw_pose_finite=bool(np.isfinite(pred['global_orient'][:end-start]).all()))
        (directory/'clip.json').write_text(json.dumps(report,indent=2),encoding='utf8')
        manifest['outputs'].append(dict(side=side,start=start,end=end,path=str(directory),elapsed_s=report['elapsed_s']))
        state.write_text(json.dumps(manifest,indent=2),encoding='utf8')
        print('ACE COMPLETE',side,start,end,round(report['elapsed_s'],2),'seconds','claims',report['claim_counts'],flush=True)
        del ctrl,pred;torch.cuda.empty_cache()
    manifest['status']='completed';state.write_text(json.dumps(manifest,indent=2),encoding='utf8')


if __name__=='__main__':main()
