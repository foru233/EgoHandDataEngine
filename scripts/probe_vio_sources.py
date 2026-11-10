import urllib.request, urllib.error, time
urls={
"euroc_mh01":"https://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip",
"euroc_v101":"https://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.zip",
"tum_room4_official":"https://vision.in.tum.de/tumvi/calibrated/512_16/dataset-room4_512_16.bag",
"tum_room4_mirror":"https://huggingface.co/datasets/gladiator7737/tum-vi/resolve/main/room4.bag?download=true",
"tum_room1_mirror":"https://huggingface.co/datasets/gladiator7737/tum-vi/resolve/main/room1.bag?download=true",
}
for k,u in urls.items():
 t=time.monotonic()
 try:
  r=urllib.request.urlopen(urllib.request.Request(u,method="HEAD",headers={"User-Agent":"Mozilla/5.0"}),timeout=12)
  print(k,r.status,r.headers.get("Content-Length"),r.headers.get("Accept-Ranges"),r.geturl()[:180],round(time.monotonic()-t,1),flush=True);r.close()
 except Exception as e:print(k,"ERROR",str(e)[:250],round(time.monotonic()-t,1),flush=True)
