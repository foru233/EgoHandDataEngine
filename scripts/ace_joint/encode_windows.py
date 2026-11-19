"""VAE-only process: avoids co-resident 5B backbone + VAE host memory peaks."""
from pathlib import Path
import argparse
import json
import os
import time
os.environ.setdefault('PYTORCH_CUDA_ALLOC_CONF','max_split_size_mb:128')
from runtime_bootstrap import setup
from joint_work_paths import is_joint_workspace


def ranges(n,window,overlap):
    if window<1 or not 0<=overlap<window:raise ValueError('Window/overlap invalid')
    if not overlap:return [(start,min(n,start+window)) for start in range(0,n,window)]
    if n<=window:return [(0,n)]
    starts=list(range(0,n-window+1,window-overlap))
    if starts[-1]!=n-window:starts.append(n-window)
    return [(s,s+window) for s in starts]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--input',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--start',type=int,default=0);ap.add_argument('--frames',type=int,default=0)
    ap.add_argument('--window',type=int,default=81);ap.add_argument('--overlap',type=int,default=20)
    args=ap.parse_args();out=args.out.resolve()
    if not is_joint_workspace(out):raise ValueError('Outside joint workspace')
    out.mkdir(parents=True,exist_ok=True)
    setup()
    import cv2
    import numpy as np
    import torch
    from ace_ego_hand.video_vae import load_vae
    from streaming_vae import encode_streaming
    torch.set_num_threads(4);vae=load_vae(torch.device('cuda'))
    print('VAE GPU READY',flush=True)
    maps=[json.loads(l) for l in (args.input/'frame_map.jsonl').read_text().splitlines()]
    n=min(args.frames or len(maps)-args.start,len(maps)-args.start)
    windows=[(a+args.start,b+args.start) for a,b in ranges(n,args.window,args.overlap)]
    manifest=dict(status='running',input=str(args.input.resolve()),start=args.start,frames=n,window=args.window,
                  overlap=args.overlap,clips=[],causal_context='independent clip, exact 1+4 schedule')
    (out/'latent_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
    for side in ['left','right']:
        cap=cv2.VideoCapture(str(args.input/(side+'.avi')))
        try:
            for start,end in windows:
                path=out/side/f'{start:06d}_{end:06d}.pt';path.parent.mkdir(parents=True,exist_ok=True)
                if path.exists():
                    latent=torch.load(path,map_location='cpu',weights_only=True)
                    if latent.shape[1]!=1+(max(5,4*((end-start+2)//4)+1)-1)//4:raise ValueError('Stale latent length')
                    manifest['clips'].append(dict(side=side,start=start,end=end,path=str(path),reused=True));continue
                cap.set(cv2.CAP_PROP_POS_FRAMES,start);images=[]
                for i in range(start,end):
                    ok,image=cap.read()
                    if not ok:raise ValueError('Video decode missing frame '+str(i))
                    images.append(image[:,:,::-1].copy())
                pixels=torch.from_numpy(np.stack(images)).permute(0,3,1,2).float().div_(255).mul_(2).sub_(1)
                nenc=max(5,4*((len(images)+2)//4)+1)
                if nenc>len(images):pixels=torch.cat([pixels,pixels[-1:].repeat(nenc-len(images),1,1,1)])
                del images;started=time.monotonic();torch.cuda.reset_peak_memory_stats()
                latent=encode_streaming(vae,pixels,'cuda');torch.save(latent,path)
                clip=dict(side=side,start=start,end=end,path=str(path),shape=list(latent.shape),elapsed_s=time.monotonic()-started,
                          gpu_peak_allocated_bytes=torch.cuda.max_memory_allocated())
                manifest['clips'].append(clip)
                (out/'latent_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
                print('LATENT COMPLETE',side,start,end,round(clip['elapsed_s'],2),'seconds',clip['gpu_peak_allocated_bytes'],flush=True)
                del pixels,latent;torch.cuda.empty_cache()
        finally:cap.release()
    manifest['status']='completed'
    (out/'latent_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')


if __name__=='__main__':main()
