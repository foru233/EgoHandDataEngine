import psutil,datetime
for p in psutil.process_iter(['pid','name','cpu_times','memory_info','cmdline','status']):
 try:
  if p.info['name'] and p.info['name'].lower() in ('cl.exe','ninja.exe','cmake.exe','link.exe','powershell.exe'):
   mem=p.info['memory_info'].rss/1e9 if p.info['memory_info'] else 0
   ct=p.info['cpu_times']
   print(p.pid,p.info['name'],p.info['status'],round(mem,2),"GB",round(ct.user+ct.system,1) if ct else 0,"CPU_SEC",' '.join(p.info['cmdline'] or [])[-200:],flush=True)
 except:pass
