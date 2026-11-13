"""Serial 2x2 diagnostic screen of existing calibration noise sources, never deploys.

These are supplied configuration densities, NOT newly measured sensor noise.
One round is exploratory; it cannot support a statistical/accuracy conclusion.
"""
import argparse
import hashlib
import json
from pathlib import Path
import random
import re
import subprocess
import sys
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from summarize_vio_phase_confirmation import analyze_run


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--rounds',type=int,default=1)
    a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    recording=ROOT/'data/imu/session_20261111_105436_609132400'
    library=ROOT/'src/egohand/vio/native/bin/egohand_vio_fail_closed.dll'
    noise_source=ROOT/'标定结果/2/data3-imu.yaml'
    noise=yaml.safe_load(noise_source.read_text(encoding='utf8'))['imu0']
    base=(recording/'vio_settings.yaml').read_text(encoding='utf8')
    candidates={'old_noise':{},'cal2_acc':{'IMU.NoiseAcc':noise['accelerometer_noise_density']},
                'cal2_gyro':{'IMU.NoiseGyro':noise['gyroscope_noise_density']},
                'cal2_both':{'IMU.NoiseAcc':noise['accelerometer_noise_density'],
                             'IMU.NoiseGyro':noise['gyroscope_noise_density']}}
    tracked=sorted((ROOT/'src/egohand/vio').glob('*.py'))+[
        ROOT/'map_creat/atlas_replay.py',ROOT/'scripts/run_vio_stability_experiment.py',Path(__file__).resolve()]
    def source_hashes():
        return {str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in tracked}
    config_dir=a.output/'configs';config_dir.mkdir()
    for name,options in candidates.items():
        text=base
        for key,value in options.items():
            text,count=re.subn(r'^'+re.escape(key)+r':[^\r\n]*',f'{key}: {value}',text,flags=re.M)
            if count!=1:raise RuntimeError('Noise key missing or duplicated')
        (config_dir/f'{name}.yaml').write_text(text,encoding='utf8')
    design={'rounds':a.rounds,'phase_ms':34,'seed':20261112,'candidates':candidates,
            'python_sources':source_hashes(),
            'source':str(noise_source),'source_sha256':hashlib.sha256(noise_source.read_bytes()).hexdigest(),
            'library_sha256':hashlib.sha256(library.read_bytes()).hexdigest(),
            'constraints':'same recorded images, calibration, DLL, phase, gauge, random walks, recovery; only two noise-density factors',
            'limitations':['Supplied calibration priors are not verified noise measurements.',
                           'Scale/misalignment calibration is not applied by this test.',
                           'Screen only; no accuracy truth, no default changes.'],
            'definitions':'ORB Tracking.cc uses density*sqrt(rate); Kalibr labels noise_density as continuous-time density.',
            'references':['https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model',
                          'https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc']}
    (a.output/'design.json').write_text(json.dumps(design,indent=2,ensure_ascii=False),encoding='utf8')
    results=[];rng=random.Random(design['seed'])
    for round_id in range(a.rounds):
        order=list(candidates);rng.shuffle(order)
        for name in order:
            if hashlib.sha256(library.read_bytes()).hexdigest()!=design['library_sha256']:
                raise RuntimeError('Library changed during screen')
            if source_hashes()!=design['python_sources']:
                raise RuntimeError('Python sources changed during screen; do not compare these trials')
            out=a.output/f'round_{round_id:02d}_{name}'
            command=[sys.executable,str(ROOT/'scripts/run_vio_stability_experiment.py'),
                     '--recording',str(recording),'--library',str(library),'--output',str(out),
                     '--settings',str((config_dir/f'{name}.yaml').resolve()),'--phase-ms','34',
                     '--gauge-constraint','--diagnostics']
            with (a.output/f'{out.name}.log').open('w') as log:
                code=subprocess.call(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
            if source_hashes()!=design['python_sources']:
                raise RuntimeError('Python sources changed during a trial; retain outputs but reject control')
            row={'round':round_id,'candidate':name,'exit_code':code,'command':command}
            if (out/'experiment.json').is_file():
                analyzed=analyze_run(out)
                row.update({k:v for k,v in analyzed.items() if k not in ('inputs','timestamps','image_means')})
            results.append(row)
            (a.output/'summary.json').write_text(json.dumps(results,indent=2),encoding='utf8')
            print(json.dumps(row),flush=True)


if __name__=='__main__':main()
