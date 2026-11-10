import urllib.request,time
urls={
"euroc_hf_bag":"https://huggingface.co/datasets/kavehsgh/EuRoC_MAV_Dataset_Machine_Hall_Easy_01/resolve/main/MH_01_easy.bag?download=true",
"euroc_http_old":"http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip",
"tum_room4_official":"https://vision.in.tum.de/tumvi/calibrated/512_16/dataset-room4_512_16.bag"
}
for name,url in urls.items():
 start=time.monotonic()
 try:
  r=urllib.request.urlopen(urllib.request.Request(url,method="HEAD",headers={"User-Agent":"Mozilla/5.0"}),timeout=18)
  print(name,r.status,r.headers.get("Content-Length"),r.headers.get("Accept-Ranges"),r.geturl()[:160],round(time.monotonic()-start,2),flush=True)
  r.close()
 except Exception as e:print(name,"ERROR",repr(e)[:190],round(time.monotonic()-start,2),flush=True)
