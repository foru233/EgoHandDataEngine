"""Inspect a local official STB subset; never download all sequences implicitly."""
import argparse
import json
from pathlib import Path
from datetime import datetime,timezone
from egohand.benchmarks.stb import STBBenchmarkAdapter,STB_SOURCE,stb_joint_mapping
from egohand.benchmarks.types import BenchmarkBlocked
from egohand.core.types import serializable

def main():
    p=argparse.ArgumentParser();p.add_argument('--dataset-root',default='samples/date/stb');p.add_argument('--sequence');p.add_argument('--max-frames',type=int,default=200)
    a=p.parse_args();root=Path(a.dataset_root);root.mkdir(parents=True,exist_ok=True)
    adapter=STBBenchmarkAdapter(root,a.sequence,a.max_frames)
    report={'official_source':STB_SOURCE,'inspected_at':datetime.now(timezone.utc).isoformat(),'joint_mapping':serializable(stb_joint_mapping()),'provenance':adapter.provenance,'automatic_bulk_download':False}
    try:
        samples=list(adapter.iter_samples());report.update(status='READABLE',pairs=len(samples),selected_frames=[s.provenance['pair_id'] for s in samples],observed_sizes=sorted(set(tuple(s.provenance['image_size_actual']) for s in samples)))
    except BenchmarkBlocked as e:report.update(status=e.status,reason=str(e),manual_layout='images/B1Counting/BB_left_0.png + BB_right_0.png; labels/B1Counting_BB.mat; independently verified benchmark_calibration.json')
    (root/'preparation_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
