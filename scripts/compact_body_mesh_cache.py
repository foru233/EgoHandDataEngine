import json,sys
from pathlib import Path
root=Path(sys.argv[1]); path=root/'derived_pose.jsonl'; asset=root/'body_mesh_smpl.json'; tmp=path.with_suffix('.compact.tmp'); mesh=None; n=0
with path.open(encoding='utf8') as src:
    with tmp.open('w',encoding='utf8') as dst:
        for line in src:
            row=json.loads(line)
            if mesh is None and row.get('body_mesh'): mesh=row.pop('body_mesh')
            else: row.pop('body_mesh',None)
            if mesh is not None: row['body_mesh_ref']='body_mesh_smpl.json'
            dst.write(json.dumps(row,ensure_ascii=False,separators=(',', ':'))+'\n'); n+=1
tmp.replace(path)
if mesh is not None: asset.write_text(json.dumps(mesh,separators=(',', ':')),encoding='utf8')
print(n, bool(mesh), path.stat().st_size, asset.stat().st_size if asset.exists() else 0)
