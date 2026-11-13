"""Audit frozen paired trials, then run serial release checks; never deploys a DLL."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
import psutil

ROOT=Path(__file__).resolve().parents[1]


def read(path):return json.loads(path.read_text(encoding='utf8'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--suite',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--wait',action='store_true');a=p.parse_args()
    design=read(a.suite/'design.json');expected=design['rounds']*2
    while True:
        try:trials=read(a.suite/'summary.json')
        except (OSError,json.JSONDecodeError):trials=[]
        if len(trials)==expected:break
        if not a.wait:raise RuntimeError(f'Only {len(trials)}/{expected} trials complete')
        active=any((q.info['name'] or '').lower().startswith('python') and
                   any(Path(v).name=='run_vio_cheirality_confirmation.py' for v in (q.info['cmdline'] or []))
                   for q in psutil.process_iter(['name','cmdline']))
        if not active:raise RuntimeError(f'Confirmation controller stopped at {len(trials)}/{expected}; not restarted')
        time.sleep(10)
    library=Path(design['library'])
    assert hashlib.sha256(library.read_bytes()).hexdigest()==design['dll_sha256']
    for name,digest in design['sources'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
    a.output.mkdir(parents=True,exist_ok=False)
    report={'candidate':str(library),'sha256':design['dll_sha256'],'tests':[],
            'test_kind':'recording/native/synthetic regression; not physical-device precision acceptance'}
    def persist():
        (a.output/'release_validation.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    try:
        primary=[]
        for repeat in range(design['rounds']):
            pair=[next(t for t in trials if t['round']==repeat and t['enabled']==v) for v in (False,True)]
            input_rows=[];pose_rows=[]
            for trial in pair:
                assert trial['frozen_inputs_unchanged'] and trial['exit_code']==0
                assert trial['experiment']['error'] is None and trial['ba2_complete']
                folder=Path(trial['path']);inputs=read(folder/'inputs.json')
                assert inputs.pop('cheirality_guard')==trial['enabled']
                input_rows.append(inputs)
                poses=[json.loads(s) for s in (folder/'poses.jsonl').read_text().splitlines()]
                pose_rows.append([(r['timestamp_ns'],r['image_diagnostics']) for r in poses])
                assert trial['frames']==1727 and len(trial['commits'])>=3
                assert all(c['perspective_observations']>0 for c in trial['commits'])
                if trial['enabled']:
                    assert all(c['invalid_depth']==0 for c in trial['commits'])
                    assert trial['counters']['invalid_depth_checks'] or trial['counters']['invalid_depth_removed']
                else:
                    assert all(trial['counters'][k]==0 for k in ('invalid_depth_checks','invalid_depth_removed','invalid_depth_points'))
            assert input_rows[0]==input_rows[1],f'Input mismatch in pair {repeat}'
            assert pose_rows[0]==pose_rows[1],f'Image/timestamp mismatch in pair {repeat}'
            primary.append({'round':repeat,'off_invalid':[c['invalid_depth'] for c in pair[0]['commits']],
                            'on_invalid':[c['invalid_depth'] for c in pair[1]['commits']]})
        metrics={}
        for enabled in (False,True):
            rows=[t for t in trials if t['enabled']==enabled]
            def describe(values):return {'median':float(np.median(values)),'min':min(values),'max':max(values)}
            metrics[str(int(enabled))]={
                'raw_valid':describe([t['raw_valid'] for t in rows]),
                'gated_valid':describe([t['gated_valid'] for t in rows]),
                'track_p50_ms':describe([t['experiment']['track_ms_p50_p95'][0] for t in rows]),
                'track_p95_ms':describe([t['experiment']['track_ms_p50_p95'][1] for t in rows])}
        assert any(any(x['off_invalid']) for x in primary),'Baseline never exercised invalid depth'
        report.update(controlled_pairs=design['rounds'],physical_invariant_passed=True,primary=primary,metrics=metrics)
        persist();print(json.dumps({'stage':'formal_audit_passed','metrics':metrics}),flush=True)
        env=os.environ.copy();env.update(EGOHAND_VIO_LIBRARY=str(library),EGOHAND_VIO_CHEIRALITY_GUARD='1',
            EGOHAND_VIO_GAUGE_CONSTRAINT='1',EGOHAND_VIO_WORLD_FRAME_DISABLE='0',EGOHAND_VIO_DEPTH_GATE='0',
            EGOHAND_VIO_POINT_CLAMP='0',EGOHAND_VIO_ENTRY_PRUNE='0',EGOHAND_VIO_GAUGE_FIX='0',
            EGOHAND_VIO_STEREO_ONLY='0',EGOHAND_VIO_DISABLE_RECOVERY='0')
        def run(name,arguments,environment=env):
            command=[sys.executable,*map(str,arguments)]
            with (a.output/f'{name}.log').open('w',encoding='utf8') as log:
                code=subprocess.call(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,env=environment)
            report['tests'].append({'name':name,'command':command,'exit_code':code});persist()
            print(json.dumps({'stage':name,'exit_code':code}),flush=True)
            assert code==0,name
        run('long_3200',['scripts/run_vio_stability_experiment.py','--recording','map_creat/data/map_scan_1794299836937477500',
            '--settings','configs/vio/vs030_rectified_half.yaml','--output',a.output/'long_3200',
            '--library',library,'--gauge-constraint','--cheirality-guard','--frames','3200'])
        long=read(a.output/'long_3200/experiment.json');assert long['frames']==3200 and long['error'] is None
        recording='data/imu/session_20261110_112020_689246900';config='results/vio/stability_baseline/atlas_compat_config'
        run('atlas_export',['map_creat/atlas_replay.py',recording,a.output/'atlas_export','--config-snapshot',config,'--limit','340'])
        atlas=a.output/'atlas_export/atlas.osa';before=hashlib.sha256(atlas.read_bytes()).hexdigest()
        run('atlas_roundtrip',['map_creat/atlas_replay.py',recording,a.output/'atlas_roundtrip','--config-snapshot',config,'--atlas',atlas,'--limit','100'])
        assert hashlib.sha256(atlas.read_bytes()).hexdigest()==before,'Read-only localization changed original Atlas'
        exported=read(a.output/'atlas_export/report.json');reloaded=read(a.output/'atlas_roundtrip/report.json')
        assert exported['valid_poses']>0 and reloaded['valid_poses']>0
        report['atlas']={'build_valid':exported['valid_poses'],'reload_valid':reloaded['valid_poses'],
                         'reload_epochs':reloaded['map_epochs'],'original_unchanged':True};persist()
        run('vocabulary_reuse',['scripts/verify_vio_live_phase.py','--recording','data/imu/session_20261111_105436_609132400',
                               '--output',a.output/'vocabulary_reuse.json'])
        ordinary=os.environ.copy();ordinary.pop('EGOHAND_VIO_LIBRARY',None);ordinary.pop('EGOHAND_VIO_CHEIRALITY_GUARD',None)
        run('python_regression',['-m','pytest','tests','--ignore=tests/test_adaptive_workspace.py','-q'],ordinary)
        report['success']=True;persist();print('Release validation completed; deployment still requires review of this report.',flush=True)
    except Exception as exc:
        report['success']=False;report['error']=repr(exc);persist();raise


if __name__=='__main__':main()
