from pathlib import Path
import urllib.request,hashlib,json
root=Path(__import__('pathlib').Path(__file__).resolve().parents[1] / 'samples/date/VIO')
sources={
"EuRoC/MH_01_easy/MH_01_easy_gt_openvins.csv":"https://raw.githubusercontent.com/rpng/open_vins/master/ov_data/euroc_mav/MH_01_easy.csv",
"EuRoC/MH_01_easy/MH_01_easy_gt_openvins.tum":"https://raw.githubusercontent.com/rpng/open_vins/master/ov_data/euroc_mav/MH_01_easy.txt",
"TUM-VI/room4/room4_gt_openvins.csv":"https://raw.githubusercontent.com/rpng/open_vins/master/ov_data/tum_vi/dataset-room4_512_16.csv",
}
results={}
for p,url in sources.items():
 try:
  with urllib.request.urlopen(urllib.request.Request(url,headers={"User-Agent":"Mozilla/5.0"}),timeout=20) as r:data=r.read()
  if len(data)<200 or data[:20].lstrip().startswith(b'<'):raise RuntimeError("invalid text")
  path=root/p;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
  results[p]={"url":url,"bytes":len(data),"sha256":hashlib.sha256(data).hexdigest()}
  print("OK",p,len(data),flush=True)
 except Exception as e:results[p]={"error":str(e)};print("FAIL",p,str(e),flush=True)
(root/"gt_sidecar_sources.json").write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding="utf8")
