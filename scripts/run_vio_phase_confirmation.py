"""Paired serial phase confirmation with fixed DLL, images and common time range."""
import argparse
import hashlib
import json
from pathlib import Path
import random
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--rounds',type=int,default=10)
    p.add_argument('--seed',type=int,default=20261112)
    p.add_argument('--wait-for',type=Path)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.wait_for:
        while not a.wait_for.exists():time.sleep(2)
    a.output.mkdir(parents=True,exist_ok=True)
    rows=[json.loads(l) for l in (ROOT/'results/vio/safe_exp_fail_closed_20261112/final_full/poses.jsonl').read_text(encoding='utf8').splitlines()]
    start=rows[0]['timestamp_ns']+100_000_000
    end=rows[-1]['timestamp_ns']-100_000_000
    lib=ROOT/'src/egohand/vio/native/bin/egohand_vio_fail_closed.dll'
    digest=hashlib.sha256(lib.read_bytes()).hexdigest()
    inputs={'dll_sha256':digest,'start_ns':start,'end_ns':end,'rounds':a.rounds,
            'phases_ms':[97,32],'seed':a.seed,
            'note':'Phase32 selected on independent stereo train half; physical exposure semantics unverified.'}
    (a.output/'design.json').write_text(json.dumps(inputs,indent=2))
    rng=random.Random(inputs['seed']);results=[]
    for repeat in range(a.rounds):
        order=[97,32];rng.shuffle(order)
        for phase in order:
            if hashlib.sha256(lib.read_bytes()).hexdigest()!=digest:
                raise RuntimeError('DLL changed during paired confirmation')
            out=a.output/f'round_{repeat:02d}_phase_{phase}'
            cmd=[sys.executable,str(ROOT/'scripts/run_vio_stability_experiment.py'),
                 '--recording',str(ROOT/'data/imu/session_20261111_105436_609132400'),
                 '--library',str(lib),'--output',str(out),'--phase-ms',str(phase),
                 '--start-stamp-ns',str(start),'--end-stamp-ns',str(end),
                 '--gauge-constraint','--diagnostics']
            if (out/'experiment.json').is_file():
                rc=0;retained=True
            else:
                with (a.output/f'{out.name}.log').open('w') as stream:
                    rc=subprocess.call(cmd,stdout=stream,stderr=subprocess.STDOUT,cwd=ROOT)
                retained=False
            results.append({'round':repeat,'phase_ms':phase,'exit_code':rc,'retained':retained,'command':cmd})
            (a.output/'commands.json').write_text(json.dumps(results,indent=2))
            print(json.dumps(results[-1]),flush=True)


if __name__=='__main__':main()
