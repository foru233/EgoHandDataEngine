import os,subprocess,pathlib,datetime,json,shutil
root=pathlib.Path(__import__('pathlib').Path(__file__).resolve().parents[1])
repo=root/'third_party/ORB_SLAM3'
for dirname in ['build_windows','build_msvc_initial','logs','logs/vio','third_party/ORB_SLAM3/lib']:
 p=root/dirname if not dirname.startswith('build_') else repo/dirname
 print('DIR',p,p.exists())
 if p.exists():
  arr=sorted((x for x in p.rglob('*') if x.is_file() and (x.suffix.lower() in ['.log','.txt','.ninja','.exe','.dll','.lib','.json'] or 'build' in x.name.lower())),key=lambda x:x.stat().st_mtime,reverse=True)[:24]
  for x in arr:print(' ',x.relative_to(root),x.stat().st_size,datetime.datetime.fromtimestamp(x.stat().st_mtime))
git=shutil.which('git')
if pathlib.Path(git).exists():
 for args in [['rev-parse','--short','HEAD'],['status','--short'],['diff','--stat']]:
  p=subprocess.run([git,'-C',str(repo),*args],capture_output=True,text=True)
  print('GIT',args,'STATUS',p.returncode,'OUT',p.stdout[:4000],p.stderr[:300])
