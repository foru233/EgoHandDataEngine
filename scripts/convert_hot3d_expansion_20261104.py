import json, os, subprocess, sys, time, datetime
from pathlib import Path
R=Path(__import__('pathlib').Path(__file__).resolve().parents[1])
D=R/"samples/date/hot3d"
plan=json.loads((D/"expansion_plan_20261104.json").read_text())
src=D/"expansion_results_20261104.json"; progress=D/"expansion_status_20261104.json"
report=D/"expansion_conversion_status_20261104.json"
tool=Path(os.environ.get("EGOHAND_HOT3D_PYTHON", sys.executable))
converted=set();failed={}
for cycle in range(900):
    items=json.loads(src.read_text()) if src.exists() else []
    for row in items:
        if row.get("status")!="passed":continue
        clip=Path(row["path"]).stem
        if clip in converted or clip in failed:continue
        out=D/"prepared"/clip/"umetrack_world.json"
        if out.exists():
            try:
                data=json.loads(out.read_text())
                if data.get("status")=="CONVERTED_OFFICIAL_MODEL" and len(data["frames"])==150:
                    converted.add(clip);continue
            except (ValueError,KeyError):pass
            failed[clip]="Existing conversion invalid; not overwriting"
            continue
        command=[str(tool),str(R/"scripts/convert_hot3d_umetrack.py"),
          "--toolkit-root",str(D/"official-hand-toolkit"),
          "--archive",str(D/row["path"]),"--output",str(out)]
        try:
            result=subprocess.run(command,cwd=R,capture_output=True,text=True,timeout=240)
            if result.returncode:raise RuntimeError(result.stderr[-500:])
            data=json.loads(out.read_text())
            if data.get("status")!="CONVERTED_OFFICIAL_MODEL":raise RuntimeError("Invalid conversion result")
            converted.add(clip);print("CONVERTED",clip,flush=True)
        except Exception as e:failed[clip]=str(e)[:300]
    state=json.loads(progress.read_text()) if progress.exists() else {}
    done=state.get("state")=="finished"
    report.write_text(json.dumps({"state":"finished" if done else "running",
      "converted":len(converted),"failed":failed,"target":len(plan["files"]),
      "updated_utc":datetime.datetime.now(datetime.timezone.utc).isoformat()},indent=2))
    if done:break
    time.sleep(12)
print("CONVERSION_FINISHED",len(converted),"failed",len(failed),flush=True)
