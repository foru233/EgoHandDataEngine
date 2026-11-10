"""Plan an incremental, reproducible ~10GB HOT3D + SHOW3D expansion."""
import sys, json, random, datetime
from pathlib import Path
from collections import defaultdict
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/show3d_deps"))
import pandas as pd
from huggingface_hub import HfApi
API = HfApi()
DATE = "20261104"
REV_SHOW = "81403d3e4254cef959027118bc668bb2f26c86f6"
REV_HOT = "30fe9674782f32e1e5edba98476b6ff4300132c5"
SROOT = ROOT / "samples/date/SHOW3D"
HROOT = ROOT / "samples/date/hot3d"
LIMIT_SHOW, LIMIT_HOT = 5_000_000_000, 5_000_000_000

def stamp():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()

def plan_hot():
    print("HOT3D: listing pinned train_aria files", flush=True)
    meta = list(API.list_repo_tree("bop-benchmark/hot3d", repo_type="dataset",
        path_in_repo="train_aria", recursive=False, expand=True,
        revision=REV_HOT, token=False))
    clips = []
    for item in meta:
        lfs = getattr(item, "lfs", None)
        path = getattr(item, "path", "")
        if not path.startswith("train_aria/clip-") or not path.endswith(".tar"):
            continue
        if not lfs or not getattr(lfs, "sha256", None):
            continue
        clips.append({"path": path, "size": item.size, "sha256": lfs.sha256,
                      "blob_id": getattr(item, "blob_id", None)})
    old = {p.name for p in (HROOT / "train_aria").glob("clip-*.tar")}
    fresh = [x for x in clips if Path(x["path"]).name not in old]
    random.Random(20261104).shuffle(fresh)
    chosen, total = [], 0
    for x in fresh:
        if total + x["size"] <= LIMIT_HOT:
            chosen.append(x)
            total += x["size"]
    assert 4_500_000_000 <= total <= LIMIT_HOT, f"HOT3D size insufficient: {total}"
    output = {"created_utc": stamp(), "revision": REV_HOT,
              "repo": "bop-benchmark/hot3d", "split": "train_aria",
              "additional_budget_bytes": LIMIT_HOT,
              "planned_new_bytes": total, "files": chosen,
              "note": "Existing files not counted; test split excluded."}
    name = HROOT / f"expansion_plan_{DATE}.json"
    name.write_text(json.dumps(output, indent=2), encoding="utf-8")
    print("HOT3D_PLAN", len(chosen), "files", total, "bytes", "remote_candidates", len(clips), flush=True)
    return output

def plan_show():
    print("SHOW3D: reading official indexes", flush=True)
    a = pd.read_parquet(SROOT / "dataset_index_train.parquet")
    b = pd.read_parquet(SROOT / "hand_pose/v3/index.parquet")
    d = a.merge(b, on=["subject_id","scene_id"], suffixes=("","_hand"))
    d = d[(d.has_headset0 == True) & (d.has_headset1 == True) &
          (d.has_hand_pose == True) & (d.coverage_left_accepted_pct >= 85) &
          (d.coverage_right_accepted_pct >= 85)].copy()
    existing_plan = json.loads((SROOT/"download_plan.json").read_text(encoding="utf-8"))
    exclude = {(x["subject"], x["scene"]) for x in existing_plan["scenes"]}
    exclude.update((p.parent.parent.name, p.parent.name) for p in (SROOT/"verification").glob("*/*/validation.json"))
    d = d[~d.apply(lambda r: (r["subject_id"], r["scene_id"]) in exclude, axis=1)].copy()
    d["score"] = d[["coverage_left_accepted_pct","coverage_right_accepted_pct"]].min(axis=1)
    d = d.sort_values(["score","num_frames"], ascending=False)
    subjects = list(d.subject_id.unique())
    random.Random(20261104).shuffle(subjects)
    groups = {s: list(d[d.subject_id==s].to_dict("records")) for s in subjects}
    taken_activities = {x["activity"] for x in existing_plan["scenes"]}
    selections = []
    while len(selections) < 115 and any(groups.values()):
        for s in subjects:
            if not groups[s]: continue
            pos = next((i for i,x in enumerate(groups[s])
                        if x["activity"] not in taken_activities
                        and not x["activity"].startswith("none-")),0)
            row = groups[s].pop(pos)
            selections.append(row)
            taken_activities.add(row["activity"])
            if len(selections)>=115: break
    print("SHOW3D_CANDIDATES",len(selections),flush=True)
    paths, by_scene = [], {}
    for row in selections:
        sub, scene = row["subject_id"], row["scene_id"]
        base=f"scenes/{sub}/{scene}"
        files=[f"{base}/headset0.mp4",f"{base}/headset1.mp4",
               f"{base}/camera_calibration/headset0.json",
               f"{base}/camera_calibration/headset1.json",
               f"{base}/metadata/frame_info.json",
               f"{base}/metadata/recording_info.json",
               f"hand_pose/v3/scenes/{sub}/{scene}/hand_pose_umetrack.json",
               f"hand_pose/hand_profiles/{sub}/profile_umetrack.json"]
        by_scene[(sub,scene)]=files
        paths.extend(files)
    paths=list(dict.fromkeys(paths))
    meta={}
    for start in range(0,len(paths),80):
        block=paths[start:start+80]
        for attempt in range(4):
            try:
                for x in API.get_paths_info("facebook/show3d-dataset",block,
                      repo_type="dataset", revision=REV_SHOW, expand=True, token=False):
                    lfs=getattr(x,"lfs",None)
                    if getattr(x,"size",None) is not None:
                        meta[x.rfilename]={"path": x.rfilename, "size": x.size,
                          "sha256": getattr(lfs,"sha256",None),
                          "blob_id": getattr(x,"blob_id",None)}
                break
            except Exception as e:
                print("META_RETRY",start,attempt,type(e).__name__,flush=True)
                if attempt==3: raise
        print("SHOW3D_METADATA",min(start+80,len(paths)),"/",len(paths),flush=True)
    chosen, total, skipped = [], 0, []
    for row in selections:
        sub,scene=row["subject_id"],row["scene_id"]
        fs=by_scene[(sub,scene)]
        absent=[x for x in fs if x not in meta]
        if absent:
            skipped.append({"subject":sub,"scene":scene,"missing":absent})
            continue
        cost=sum(meta[x]["size"] for x in fs if not (SROOT/x).exists())
        if total+cost>LIMIT_SHOW:
            continue
        chosen.append({"subject":sub,"scene":scene,"activity":row["activity"],
            "frames":int(row["num_frames"]),
            "left_accepted_pct":float(row["coverage_left_accepted_pct"]),
            "right_accepted_pct":float(row["coverage_right_accepted_pct"]),
            "files":[meta[x] for x in fs]})
        total += cost
    assert 4_500_000_000 <= total <= LIMIT_SHOW, f"SHOW3D size insufficient: {total}"
    output={"created_utc":stamp(),"revision":REV_SHOW,"repo":"facebook/show3d-dataset",
        "additional_budget_bytes":LIMIT_SHOW,"planned_new_bytes":total,
        "scenes":chosen,"incomplete":skipped}
    path=SROOT/f"expansion_plan_{DATE}.json"
    path.write_text(json.dumps(output,indent=2),encoding="utf-8")
    print("SHOW3D_PLAN",len(chosen),"scenes",total,"bytes",len({x["subject"] for x in chosen}),"participants",flush=True)
    return output

if __name__=="__main__":
    h=plan_hot()
    s=plan_show()
    print("TOTAL_ADDITIONAL_PLANNED_GB",round((h["planned_new_bytes"]+s["planned_new_bytes"])/1e9,4),flush=True)
