"""Reuse verified, complete per-eye ACE predictions without caching fits."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
from uuid import uuid4

import yaml

from joint_work_paths import ACE, JOINT_WORK, MANO, SCRIPT_DIR

CACHE_VERSION = 1
INPUT_FILES = ('left.avi', 'right.avi', 'left_camera.json', 'right_camera.json', 'frame_map.jsonl')
ADAPTER_FILES = ('ace_adapter.py', 'encode_windows.py', 'infer_latents.py',
                 'runtime_bootstrap.py', 'streaming_vae.py')


def read_json(path):
    return json.loads(path.read_text(encoding='utf8'))


def sha256(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def inventory(root, patterns):
    return [path for path in sorted(root.rglob('*')) if path.is_file()
            and '__pycache__' not in path.parts and path.suffix.lower() in patterns]


def fingerprint(video, ace_python, window):
    options=ACE/'options/ace_ego_hand_k.yml'
    paths=yaml.safe_load(options.read_text(encoding='utf8'))['paths']
    model=ACE/paths['model_root']
    if not model.is_dir():
        raise FileNotFoundError(model)
    sources=inventory(ACE/'ace_ego_hand',{'.py'})+inventory(ACE/'third_party/videox_fun',{'.py'})
    models=inventory(model,{'.pt','.pth','.safetensors','.bin','.json','.model','.txt','.yaml','.yml'})
    mano=inventory(MANO,{'.pkl','.npz'})
    if not sources or not models or not mano:
        raise ValueError('ACE source, Wan weights or MANO model inventory is empty')
    explicit=[options, ACE/'checkpoints/ace_ego_hand_k.pt', ACE/paths['videox_config'],
              ACE/paths['caption_embed']]
    if paths.get('full_warmstart'):explicit.append(ACE/paths['full_warmstart'])
    if model.resolve() != (ACE/'ckpt/Wan2.2-Fun-5B-Control').resolve():
        models+=inventory(ACE/'ckpt/Wan2.2-Fun-5B-Control',{'.yaml','.yml','.json','.pth','.pt','.safetensors','.bin'})
    request=('import importlib.metadata as m,json,sys; '
             'print(json.dumps([sys.version,sorted((p.metadata["Name"],p.version) '
             'for p in m.distributions() if p.metadata["Name"])]))')
    runtime=json.loads(subprocess.check_output([str(ace_python),'-c',request], text=True))
    fingerprint_data=dict(version=CACHE_VERSION,window=window,overlap=0,runtime=runtime,
        inputs={name:sha256(video/name) for name in INPUT_FILES},
        adapters={name:sha256(SCRIPT_DIR/name) for name in ADAPTER_FILES},
        external={str(path):sha256(path) for path in dict.fromkeys(explicit+sources+models+mano)})
    raw=json.dumps(fingerprint_data,sort_keys=True,separators=(',',':')).encode()
    return hashlib.sha256(raw).hexdigest()


def expected_clips(frames, window):
    return {(side, first, min(frames,first+window)) for side in ('left','right')
            for first in range(0,frames,window)}


def clip_directory(clip):
    return Path(clip['side'])/f'{clip["start"]:06d}_{clip["end"]:06d}'


def manifest_clips(root, frames, window):
    manifest=read_json(root/'sequence_manifest.json')
    if (manifest['status']!='completed' or manifest['frames']!=frames or manifest['start']!=0
            or manifest['inference_window']!=window or manifest['overlap']!=0):
        raise ValueError('Incomplete or incompatible ACE sequence')
    clips=manifest['outputs']
    keys={(clip['side'],clip['start'],clip['end']) for clip in clips}
    if len(clips)!=len(keys) or keys!=expected_clips(frames,window):
        raise ValueError('Incomplete or duplicate ACE frame ranges')
    for clip in clips:
        directory=(root/clip['path']).resolve()
        if not directory.is_relative_to(root.resolve()):
            raise ValueError('ACE prediction path outside sequence directory')
        yield clip,directory


def lookup(key, frames, window, cache_root=None):
    root=(cache_root or JOINT_WORK/'ace_predictions')/key
    try:
        metadata=read_json(root/'cache.json')
        if metadata['version']!=CACHE_VERSION or metadata['key']!=key:return None
        verified={}
        for clip,directory in manifest_clips(root,frames,window):
            if directory!=root.resolve()/clip_directory(clip):return None
            target=directory/'raw_predictions.npz'
            verified[target.relative_to(root.resolve()).as_posix()]=sha256(target)
        if verified!=metadata['predictions']:return None
    except (OSError,ValueError,KeyError,TypeError):
        return None
    return root


def publish(source, key, frames, window, cache_root=None):
    cache_root=cache_root or JOINT_WORK/'ace_predictions'
    if lookup(key,frames,window,cache_root):return
    cache_root.mkdir(parents=True,exist_ok=True)
    staging=cache_root/f'.building_{uuid4().hex}'
    staging.mkdir()
    manifest=read_json(source/'sequence_manifest.json')
    predictions={}
    outputs=[]
    for clip,directory in manifest_clips(source,frames,window):
        relative=clip_directory(clip)
        target=staging/relative/'raw_predictions.npz'
        target.parent.mkdir(parents=True)
        shutil.copyfile(directory/target.name,target)
        predictions[target.relative_to(staging).as_posix()]=sha256(target)
        outputs.append({**clip,'path':relative.as_posix()})
    manifest['outputs']=outputs
    (staging/'sequence_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
    (staging/'cache.json').write_text(json.dumps(dict(version=CACHE_VERSION,key=key,predictions=predictions),
                                               indent=2),encoding='utf8')
    final=cache_root/key
    if final.exists():
        if lookup(key,frames,window,cache_root):return
        final.rename(cache_root/f'.invalid_{uuid4().hex}')
    staging.rename(final)
