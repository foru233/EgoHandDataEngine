"""Exact official 1+4 causal encode schedule, transfer only current pixels.

The official encoder already processes one frame then four-frame chunks with
cached features. Moving the entire clip to GPU and patchifying it first keeps
two needless full-resolution clip copies alive. Here patchify is applied to
the same temporal slices, retaining every official encoder operation/cache.
"""
import torch


@torch.no_grad()
def encode_streaming(vae,pixels,device):
    from videox_fun.models.wan_vae3_8 import patchify
    model=vae.model
    if pixels.ndim!=4 or len(pixels)%4!=1:raise ValueError('Expected F,3,H,W with F=4k+1')
    model.clear_cache();outputs=[]
    dtype=next(model.parameters()).dtype
    scale=[item.to(device,dtype) for item in vae.scale]
    try:
        for i in range(1+(len(pixels)-1)//4):
            start,end=(0,1) if i==0 else (1+4*(i-1),1+4*i)
            chunk=pixels[start:end].permute(1,0,2,3).unsqueeze(0).to(device,dtype)
            chunk=patchify(chunk,patch_size=2)
            model._enc_conv_idx=[0]
            output=model.encoder(chunk,feat_cache=model._enc_feat_map,feat_idx=model._enc_conv_idx)
            outputs.append(output)
            del chunk
        encoded=torch.cat(outputs,dim=2)
        mu,_=model.conv1(encoded).chunk(2,dim=1)
        mu=(mu-scale[0].view(1,model.z_dim,1,1,1))*scale[1].view(1,model.z_dim,1,1,1)
        return mu[0].to(torch.float16).cpu()
    finally:model.clear_cache()
