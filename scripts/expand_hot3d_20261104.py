"""Incremental pinned HOT3D Aria train-only downloader and complete tar verifier."""
import sys, os, json, time, hashlib, shutil, datetime
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"tools/show3d_deps"))
from huggingface_hub import hf_hub_download
from prepare_hot3d import inspect_archive
DATA=ROOT/"samples/date/hot3d"
STAMP="20261104"
PLAN=json.loads((DATA/f"expansion_plan_{STAMP}.json").read_text(encoding="utf-8"))
REV="30fe9674782f32e1e5edba98476b6ff4300132c5"
assert PLAN["revision"]==REV and PLAN["split"]=="train_aria"
RESULTS=DATA/f"expansion_results_{STAMP}.json"
STATUS=DATA/f"expansion_status_{STAMP}.json"
OFFICIAL=DATA/"official_file_metadata.json"
INITIAL=shutil.disk_usage(DATA).free
MAX_INCREASE=5_200_000_000
assert 0<PLAN["planned_new_bytes"]<=5_000_000_000
passed,failed=[],[]
START=datetime.datetime.now(datetime.timezone.utc).isoformat()
def status(state,**extra):
    payload={"state":state,"started_utc":START,
       "updated_utc":datetime.datetime.now(datetime.timezone.utc).isoformat(),
       "revision":REV,"target_clips":len(PLAN["files"]),"passed":len(passed),
       "failed":len(failed),"verified_added_bytes":sum(r["size_bytes"] for r in passed),
       **extra}
    temp=STATUS.with_suffix(".tmp")
    temp.write_text(json.dumps(payload,indent=2,ensure_ascii=False),encoding="utf-8")
    os.replace(temp,STATUS)
    return payload
def persist():
    temp=RESULTS.with_suffix(".tmp")
    temp.write_text(json.dumps(passed+failed,indent=2,ensure_ascii=False),encoding="utf-8")
    os.replace(temp,RESULTS)
def check_sha(path,item):
    if not path.is_file() or path.stat().st_size!=item["size"]:return False
    hasher=hashlib.sha256()
    with path.open("rb") as f:
        for piece in iter(lambda:f.read(1024*1024),b""):hasher.update(piece)
    return hasher.hexdigest()==item["sha256"]
def official_record(item):
    return {"type":"file","oid":item["blob_id"],"size":item["size"],
        "lfs":{"oid":item["sha256"],"size":item["size"]},
        "path":item["path"]}
def update_official(item):
    records=json.loads(OFFICIAL.read_text(encoding="utf-8"))
    for old in records:
        if old.get("path")==item["path"]:
            if old.get("lfs",{}).get("oid")!=item["sha256"]:
                raise RuntimeError("Prior official metadata conflict "+item["path"])
            return
    records.append(official_record(item))
    temp=OFFICIAL.with_suffix(".json.tmp")
    temp.write_text(json.dumps(records,indent=2),encoding="utf-8")
    os.replace(temp,OFFICIAL)
def get_verified(item):
    path=DATA/item["path"]
    if path.is_file():
        if not check_sha(path,item):raise RuntimeError("Existing file invalid, not overwriting "+str(path))
    else:
        if INITIAL-shutil.disk_usage(DATA).free>MAX_INCREASE or shutil.disk_usage(DATA).free<12_000_000_000:
            raise RuntimeError("Free space or additional download budget exceeded")
        err=None
        for attempt in range(6):
            try:
                hf_hub_download(repo_id="bop-benchmark/hot3d",filename=item["path"],
                    repo_type="dataset",revision=REV,local_dir=DATA,token=False)
                if not check_sha(path,item):raise ValueError("Hash/size mismatch: "+str(path))
                err=None
                break
            except Exception as exc:
                err=exc
                print("RETRY",item["path"],attempt+1,type(exc).__name__,str(exc)[:140],flush=True)
                time.sleep(min(45,3*(attempt+1)))
        if err:raise err
    return path
def run():
    status("running")
    for index,item in enumerate(PLAN["files"],1):
        print("HOT3D_START",index,len(PLAN["files"]),item["path"],flush=True)
        try:
            assert item["path"].startswith("train_aria/clip-") and item["path"].endswith(".tar")
            path=get_verified(item)
            report=inspect_archive(path,[official_record(item)],DATA/"prepared")
            if report["frames"]!=150 or report["gt_hand_frames"]<1 or not report["has_hand_shape"]:
                raise ValueError("Missing annotated frames or hand shape in "+str(path))
            if report.get("camera_models")!=["CameraModelType.FISHEYE624"]:
                raise ValueError("Unexpected camera model "+str(report.get("camera_models")))
            if not check_sha(path,item):
                raise ValueError("Post-validation hash mismatch")
            update_official(item)
            ok={"path":item["path"],"status":"passed","size_bytes":item["size"],
                "sha256":item["sha256"],"frames":report["frames"],
                "gt_hand_frames":report["gt_hand_frames"],
                "camera_models":report["camera_models"],
                "verified_at_utc":datetime.datetime.now(datetime.timezone.utc).isoformat(),
                "annotation_model":"UmeTrack native GT parameters (not yet FK converted)"}
            passed.append(ok)
            print("HOT3D_PASS",item["path"],"GT_FRAMES",ok["gt_hand_frames"],flush=True)
        except Exception as e:
            failed.append({"path":item["path"],"status":"failed","reason":type(e).__name__+": "+str(e)[:300]})
            print("HOT3D_FAIL",item["path"],type(e).__name__,str(e)[:180],flush=True)
        persist()
        status("running",current_index=index)
    status("finished",failed_files=failed)
    print("HOT3D_FINISHED",len(passed),"PASS",len(failed),"FAIL",sum(x["size_bytes"] for x in passed),"verified_new_bytes",flush=True)
if __name__=="__main__":run()
