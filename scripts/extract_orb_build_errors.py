import re, pathlib, collections
p=pathlib.Path(__import__('pathlib').Path(__file__).resolve().parents[1] / 'logs/vio_resume_build_stereo_allstatic.log')
s=p.read_text(encoding="utf-8",errors="replace").splitlines()
pattern=re.compile(r"error C\d+|fatal error|error LNK\d+|FAILED:|ninja:|undefined reference|CMake Error",re.I)
rows=[(i+1,x) for i,x in enumerate(s) if pattern.search(x)]
print('TOTAL_LINES',len(s),'ERROR_ROWS',len(rows))
progress=[x for x in s if x.startswith('[') and '/' in x[:10] and ']' in x[:12]]
print('LAST_BUILD_STEPS',progress[-8:])
for idx,line in rows[-130:]:print(f'{idx}: {line[:450]}'.encode('ascii','backslashreplace').decode())
print('PREV_ERRORS',flush=True)
for y in [r[0] for r in rows if re.search(r"error C\d+",r[1])][-7:]:
 for i in range(max(0,y-3),min(len(s),y+3)):
  print(f'{i+1}: {s[i][:300]}')
