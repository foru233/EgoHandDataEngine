from pathlib import Path
import tarfile,shutil,hashlib,time
root=Path(__import__('pathlib').Path(__file__).resolve().parents[1] / 'third_party/ORB_SLAM3/Vocabulary')
archive=root/"ORBvoc.txt.tar.gz"
out=root/"ORBvoc.txt"
with tarfile.open(archive,"r:gz") as tf:
  members=[m for m in tf.getmembers() if m.isfile()]
  print("ARCHIVE_FILES",[(m.name,m.size) for m in members],flush=True)
  matches=[m for m in members if Path(m.name).name=="ORBvoc.txt"]
  if len(matches)!=1:raise RuntimeError("ORBvoc missing or ambiguous")
  m=matches[0]
  if out.exists() and out.stat().st_size==m.size:
    print("VOCAB_EXISTS",out,out.stat().st_size,flush=True)
  else:
    temp=out.with_suffix(".txt.partial")
    with tf.extractfile(m) as src,temp.open("wb") as dst:
      shutil.copyfileobj(src,dst,length=8*1024*1024)
    if temp.stat().st_size!=m.size:raise RuntimeError("Vocabulary size mismatch")
    temp.replace(out)
    print("VOCAB_READY",out,out.stat().st_size,flush=True)
