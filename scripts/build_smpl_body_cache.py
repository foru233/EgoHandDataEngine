import json,sys
from pathlib import Path
from smpl_body_mesh import SmplBody

root=Path(sys.argv[1]).resolve(); path=root/'derived_pose.jsonl'; model=SmplBody(); count=0
tmp=path.with_suffix('.smpl.tmp')
with path.open(encoding='utf8') as src:
    with tmp.open('w',encoding='utf8') as tmpf:
        for line in src:
            row=json.loads(line); body=row.get('body')
            if body is not None:
                try: row['body_mesh']=model.mesh(body)
                except Exception as exc: row['body_mesh_error']=str(exc)
            tmpf.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n'); count+=1
tmp.replace(path)
(root/'derived_pose_manifest.json').write_text(json.dumps({'schema_version':3,'body_mesh_model':'SMPL_NEUTRAL','result_count':count},ensure_ascii=False,indent=2),encoding='utf8')
print(count)
