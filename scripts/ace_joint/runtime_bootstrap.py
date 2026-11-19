"""Narrow VideoX imports on Windows; dense attention remains official SDPA.

The upstream package initializer eagerly imports unrelated video/image models.
Load only the three Wan modules ACE uses. The optional Triton sparse kernels are
lazy imports and still raise if invoked; no sparse or approximate kernel is used.
"""
from pathlib import Path
import importlib
import importlib.util
import sys
import types

from joint_work_paths import ACE
VIDEOX=ACE/'third_party/videox_fun'


def package(name,path):
    mod=types.ModuleType(name);mod.__path__=[str(path)]
    mod.__package__=name;mod.__spec__=importlib.util.spec_from_loader(name,loader=None,is_package=True)
    sys.modules[name]=mod
    if '.' in name:setattr(sys.modules[name.rsplit('.',1)[0]],name.rsplit('.',1)[1],mod)
    return mod


def setup():
    sys.path.insert(0,str(ACE));sys.path.insert(0,str(ACE/'third_party'))
    package('videox_fun',VIDEOX)
    models=package('videox_fun.models',VIDEOX/'models')
    utils=package('videox_fun.utils',VIDEOX/'utils')
    dist=package('videox_fun.dist',VIDEOX/'dist')
    fuser=importlib.import_module('videox_fun.dist.fuser')
    for name in ['get_sequence_parallel_rank','get_sequence_parallel_world_size','get_sp_group','xFuserLongContextAttention']:
        setattr(dist,name,getattr(fuser,name))
    dist.usp_attn_forward=importlib.import_module('videox_fun.dist.wan_xfuser').usp_attn_forward
    utils.cfg_skip=importlib.import_module('videox_fun.utils.cfg_optimization').cfg_skip
    name='videox_fun.models.attention_utils';path=VIDEOX/'models/attention_utils.py'
    source=path.read_text(encoding='utf8')
    old='from .attention_kernel import _sparse_linear_attention, get_block_map'
    if source.count(old)!=1:raise RuntimeError('Upstream attention import changed; audit adapter')
    replacement='''def _sparse_linear_attention(*args, **kwargs):
    from .attention_kernel import _sparse_linear_attention as fn
    return fn(*args, **kwargs)
def get_block_map(*args, **kwargs):
    from .attention_kernel import get_block_map as fn
    return fn(*args, **kwargs)
'''
    mod=types.ModuleType(name);mod.__package__='videox_fun.models';mod.__file__=str(path)
    mod.__spec__=importlib.util.spec_from_file_location(name,path)
    sys.modules[name]=mod
    exec(compile(source.replace(old,replacement),str(path),'exec'),mod.__dict__)
    # Windows torch 2.5 lacks the Linux FlashAttention path. Full math SDPA can
    # materialize O(L^2) scores. Split QUERIES only, retaining ALL keys/values:
    # the dense softmax is mathematically identical, with bounded workspace.
    import torch
    def bounded_attention(q,k,v,q_lens=None,k_lens=None,dropout_p=0.,softmax_scale=None,q_scale=None,
                          causal=False,window_size=(-1,-1),deterministic=False,dtype=torch.bfloat16,
                          fa_version=None,attention_type=None,attn_mask=None):
        if causal or dropout_p:raise ValueError('ACE adapter supports noncausal inference attention only')
        if softmax_scale is not None or q_scale is not None:raise ValueError('Audit nondefault attention scales before use')
        pieces=[];keys=k.transpose(1,2);values=v.transpose(1,2)
        for start in range(0,q.shape[1],128):
            mask=attn_mask
            if mask is not None and mask.shape[-2]>1:mask=mask[...,start:start+128,:]
            output=torch.nn.functional.scaled_dot_product_attention(q[:,start:start+128].transpose(1,2),keys,values,
                        attn_mask=mask,is_causal=False,dropout_p=0.)
            pieces.append(output.transpose(1,2))
        return torch.cat(pieces,dim=1).contiguous()
    mod.attention=bounded_attention
    models.AutoencoderKLWan3_8=importlib.import_module('videox_fun.models.wan_vae3_8').AutoencoderKLWan3_8
    models.WanT5EncoderModel=importlib.import_module('videox_fun.models.wan_text_encoder').WanT5EncoderModel
    return models
