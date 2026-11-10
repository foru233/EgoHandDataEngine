"""Resume only failed SHOW3D expansion scenes after the original batch completes."""
import sys,time,json,subprocess,datetime
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]; D=ROOT/"samples/date/SHOW3D"
plan=json.loads((D/"expansion_plan_20261104.json").read_text(encoding="utf-8"))
result_path=D/"expansion_results_20261104.json";status=D/"expansion_retry_status_20261104.json"
template=D/"expand_download_20261104.py";log=D/"expansion_retry_20261104.log"
def state(label,**extra):
    status.write_text(json.dumps({"state":label,"timestamp_utc":datetime.datetime.now(datetime.timezone.utc).isoformat(),**extra},indent=2))
state("waiting_initial_batch",target=len(plan["scenes"]))
for i in range(1080):
    results=json.loads(result_path.read_text()) if result_path.is_file() else []
    if len(results)>=len(plan["scenes"]):break
    time.sleep(10)
else:
    state("blocked_timeout",record_count=len(results));sys.exit(2)
by_scene={(x["subject"],x["scene"]):x for x in results}
for attempt in range(1,4):
    failed=[x for x in plan["scenes"] if by_scene.get((x["subject"],x["scene"]),{}).get("status")!="passed"]
    if not failed:break
    state("retrying",attempt=attempt,remaining=len(failed))
    (D/"expansion_retry_plan_20261104.json").write_text(json.dumps({"revision":plan["revision"],"scenes":failed},indent=2))
    source=template.read_text(encoding="utf-8")
    for before,after in [("expansion_plan_20261104.json","expansion_retry_plan_20261104.json"),
                         ("expansion_results_20261104.json","expansion_retry_results_20261104.json"),
                         ("max_workers=2","max_workers=1")]:
        assert before in source
        source=source.replace(before,after)
    runner=D/"expand_retry_download_20261104.py"
    runner.write_text(source,encoding="utf-8")
    with log.open("a",encoding="utf-8") as out:
        out.write(f"\n=== RETRY {attempt} ===\n");out.flush()
        p=subprocess.run([sys.executable,str(runner)],cwd=ROOT,stdout=out,stderr=subprocess.STDOUT,timeout=7200)
    new=json.loads((D/"expansion_retry_results_20261104.json").read_text(encoding="utf-8"))
    for x in new:by_scene[(x["subject"],x["scene"])]=x
    result_path.write_text(json.dumps(list(by_scene.values()),indent=2),encoding="utf-8")
    if p.returncode:time.sleep(15)
remaining=[x for x in by_scene.values() if x["status"]!="passed"]
state("finished",passed=sum(x["status"]=="passed" for x in by_scene.values()),
      failed=len(remaining),failed_scenes=remaining)
print("SHOW3D_RETRY_FINISHED",len(by_scene),"records",len(remaining),"failed",flush=True)
