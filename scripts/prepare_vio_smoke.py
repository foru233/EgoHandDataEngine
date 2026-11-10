from pathlib import Path
root=Path(__import__('pathlib').Path(__file__).resolve().parents[1])
seq=root/"samples/date/VIO/converted/EuRoC/MH_01_easy"
full=(seq/"timestamps.txt").read_text(encoding="utf8").splitlines()
N=160
sample=full[:N]
for stamp in [sample[0],sample[len(sample)//2],sample[-1]]:
 for cam in ("cam0","cam1"):
  p=seq/"mav0"/cam/"data"/f"{stamp}.png"
  if not p.is_file():raise FileNotFoundError(str(p))
dest=seq/"timestamps_smoke_160.txt"
dest.write_text("\n".join(sample)+"\n",encoding="utf8")
print("SMOKE_PREPARED",dest,len(sample),"from",sample[0],"to",sample[-1])

tum=root/"samples/date/VIO/converted/TUM-VI/room4"
t_all=(tum/"timestamps_inertial.txt").read_text(encoding="utf8").splitlines()
n_tum=220
subset=[v.split()[0] for v in t_all if v and not v.startswith("#")][:n_tum]
for stamp in [subset[0],subset[len(subset)//2],subset[-1]]:
 for cam in ("cam0","cam1"):
  pp=tum/"mav0"/cam/"data"/f"{stamp}.png"
  if not pp.is_file(): raise FileNotFoundError(str(pp))
(tum/"timestamps_inertial_smoke_220.txt").write_text("\n".join(subset)+"\n",encoding="utf8")
print("TUMVI_SMOKE_PREPARED",len(subset),subset[0],subset[-1],flush=True)
