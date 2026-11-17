"""Publish a validated versioned cache inside an existing result directory.

Existing vertices/parameters remain untouched, including Windows memory maps.
All previous small entrypoint files are preserved before the derived JSONL is
atomically replaced. Original frame results, calibration and depth are read-only.
"""
import argparse,hashlib,json,os,shutil
from pathlib import Path
import numpy as np


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('target',type=Path)
    p.add_argument('--version',required=True);args=p.parse_args()
    source=args.source.resolve();target=args.target.resolve();version=args.version
    if Path(version).name!=version or version in ('.','..'):raise ValueError('Invalid version directory')
    manifest=json.loads((source/'unified_smpl_manifest.json').read_text(encoding='utf8'))
    audit=json.loads((source/'audit_report.json').read_text(encoding='utf8'));assert audit['passed']
    assert digest(source/'results/frames.jsonl')==digest(target/'results/frames.jsonl')
    vname=manifest.get('vertices_file','smpl_vertices.npy');pname=manifest.get('pose_file','smpl_pose.npz')
    params=np.load(source/pname);vertices=np.load(source/vname,mmap_mode='r')
    assert len(vertices)==len(params['valid'])==manifest['frames']
    version_dir=target/version;version_dir.mkdir(exist_ok=False)
    names=['derived_pose.jsonl','unified_smpl_manifest.json','fit_report.json','audit_report.json']
    backup=version_dir/'previous_entrypoints';backup.mkdir()
    for name in names:
        if (target/name).exists():shutil.copyfile(target/name,backup/name)
    for old,new in [(vname,'smpl_vertices.npy'),(pname,'smpl_pose.npz'),
        (manifest.get('mesh_file','smpl_mesh.json'),'smpl_mesh.json'),('fit_report.json','fit_report.json'),('audit_report.json','audit_report.json')]:
        shutil.copyfile(source/old,version_dir/new)
    # A local manifest supports the persistent worker's compatible warm seed.
    local_manifest={**manifest,'source':str(target),'vertices_file':'smpl_vertices.npy','pose_file':'smpl_pose.npz','mesh_file':'smpl_mesh.json'}
    (version_dir/'unified_smpl_manifest.json').write_text(json.dumps(local_manifest,indent=2),encoding='utf8')
    published={**local_manifest,'vertices_file':f'{version}/smpl_vertices.npy',
        'pose_file':f'{version}/smpl_pose.npz','mesh_file':f'{version}/smpl_mesh.json',
        'previous_entrypoints':f'{version}/previous_entrypoints'}
    staged=version_dir/'derived_pose.jsonl'
    with staged.open('x',encoding='utf8') as stream:
        for line in (source/'derived_pose.jsonl').open(encoding='utf8'):
            row=json.loads(line);row['head_anchor_version']=manifest['head_anchor_version']
            if row.get('body'):
                row['body_mesh_ref']=published['mesh_file']
                row['body']['diagnostics']['body_mesh_vertices_path']=str(version_dir/'smpl_vertices.npy')
            stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
    # Install the manifest first; its frame-compatible mesh paths are ready
    # before the new sidecar becomes visible to readers.
    for name,value in [('unified_smpl_manifest.json',published),('audit_report.json',audit)]:
        temp=target/(name+'.publish');temp.write_text(json.dumps(value,indent=2),encoding='utf8');os.replace(temp,target/name)
    temp=target/'derived_pose.jsonl.publish';shutil.copyfile(staged,temp);os.replace(temp,target/'derived_pose.jsonl')
    temp=target/'fit_report.json.publish';shutil.copyfile(source/'fit_report.json',temp);os.replace(temp,target/'fit_report.json')
    report={'target':str(target),'version':version,'frames':manifest['frames'],'valid':manifest['valid'],
        'head_anchor_vertex':manifest['head_anchor_vertex'],'files':{str(x.relative_to(target)):digest(x) for x in version_dir.iterdir() if x.is_file()},
        'results_sha256':digest(target/'results/frames.jsonl')}
    (version_dir/'installation.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
