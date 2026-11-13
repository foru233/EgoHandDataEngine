"""Serial randomized same-DLL cheirality guard confirmation; no deployment writes."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import random
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]


def summarize(folder):
    experiment=json.loads((folder/'experiment.json').read_text())
    poses=[json.loads(line) for line in (folder/'poses.jsonl').read_text().splitlines()]
    commits=[]
    for file in sorted((folder/'native_diagnostics').glob('map0_event*_after_commit_observations.csv')):
        with file.open() as stream:
            rows=[r for r in csv.DictReader(stream) if r.get('perspective')=='1']
        invalid=[r for r in rows if not (math.isfinite(float(r['camera_depth'])) and float(r['camera_depth'])>0)]
        commits.append({'snapshot':file.name,'perspective_observations':len(rows),'invalid_depth':len(invalid),
                        'invalid_point_ids':sorted({int(r['point_id']) for r in invalid})})
    return {'experiment':experiment,'frames':len(poses),
            'raw_valid':sum(r['raw_pose_valid'] for r in poses),
            'gated_valid':sum(r['pose_valid'] for r in poses),
            'ba2_complete':any(r.get('inertial_scale_finalized') for r in poses),
            'counters':poses[-1].get('optimization_counters') if poses else None,'commits':commits}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--rounds',type=int,default=10)
    p.add_argument('--library',type=Path,default=ROOT/'src/egohand/vio/native/bin/egohand_vio_cheirality_v3.dll')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    library=a.library.resolve();recording=ROOT/'data/imu/session_20261111_105436_609132400'
    tracked=sorted((ROOT/'src/egohand/vio').glob('*.py'))+[
        ROOT/'map_creat/atlas_replay.py',ROOT/'scripts/run_vio_stability_experiment.py',
        ROOT/'scripts/audit_vio_map_geometry.py',ROOT/'scripts/summarize_vio_phase_confirmation.py',Path(__file__).resolve(),
        *[ROOT/'third_party/ORB_SLAM3/src'/name for name in ('Optimizer.cc','LocalMapping.cc','LoopClosing.cc','System.cc')],
        *[ROOT/'third_party/ORB_SLAM3/include'/name for name in ('G2oTypes.h','VioCheiralityGuard.h','VioOptimizationDiagnostics.h')]]
    def hashes():return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    design={'library':str(library),'dll_sha256':hashlib.sha256(library.read_bytes()).hexdigest(),
            'sources':hashes(),'rounds':a.rounds,'seed':20261112,'phase_ms':34,
            'primary':'zero nonpositive/nonfinite conventional perspective observation depths in every committed map snapshot',
            'secondary':['execution failures','BA2 completion','raw/gated valid frame counts','native p50/p95 latency'],
            'mechanism':'remove physically invalid hemisphere observations, no range or displacement threshold',
            'limits':['recorded input only; not absolute precision or physical-device acceptance',
                      'same DLL, same settings/calibration/noise, only guard flag changes; geometry diagnostics optional post hoc']}
    (a.output/'design.json').write_text(json.dumps(design,indent=2),encoding='utf8')
    results=[];rng=random.Random(design['seed'])
    for repeat in range(a.rounds):
        order=[False,True];rng.shuffle(order)
        for enabled in order:
            if hashes()!=design['sources'] or hashlib.sha256(library.read_bytes()).hexdigest()!=design['dll_sha256']:
                raise RuntimeError('Frozen sources/library changed before trial')
            folder=a.output/f'round_{repeat:02d}_guard_{int(enabled)}'
            command=[sys.executable,str(ROOT/'scripts/run_vio_stability_experiment.py'),
                     '--recording',str(recording),'--library',str(library),'--output',str(folder),
                     '--phase-ms','34','--gauge-constraint','--diagnostics']
            if enabled:command.append('--cheirality-guard')
            with (a.output/f'{folder.name}.log').open('w') as log:
                exit_code=subprocess.call(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
            unchanged=hashes()==design['sources'] and hashlib.sha256(library.read_bytes()).hexdigest()==design['dll_sha256']
            row={'round':repeat,'enabled':enabled,'exit_code':exit_code,'frozen_inputs_unchanged':unchanged,
                 'path':str(folder),'command':command}
            if (folder/'experiment.json').is_file():row.update(summarize(folder))
            results.append(row)
            (a.output/'summary.json').write_text(json.dumps(results,indent=2),encoding='utf8')
            print(json.dumps(row),flush=True)
            if not unchanged:raise RuntimeError('Frozen sources/library changed during trial; reject control')
            # Stop a broken implementation before spending ten rounds on it.
            if repeat==0 and enabled:
                counters=row.get('counters') or {}
                if (row.get('experiment',{}).get('error') or exit_code or not row.get('commits')
                        or not row.get('ba2_complete') or any(c['invalid_depth'] or not c['perspective_observations'] for c in row['commits'])
                        or not (counters.get('invalid_depth_checks') or counters.get('invalid_depth_removed'))):
                    raise RuntimeError('First enabled trial failed physical invariant or did not exercise guard')


if __name__=='__main__':main()
