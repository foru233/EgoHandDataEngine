"""Run a versioned offline recipe after checking frozen code and dependencies.

Examples:
  python scripts/run_offline_baseline.py MANIFEST --verify-only
  python scripts/run_offline_baseline.py MANIFEST --recipe 111833_stereo_product --output NEW_RUN
  python scripts/run_offline_baseline.py MANIFEST --recipe 184714_two_stage --output NEW_WORKFLOW

The two-stage workflow runs camera inference, SI poses, then exact-frame rejoin
serially. Its timings remain separate; it is not a fresh integrated FPS claim.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]


def sha(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest',type=Path)
    parser.add_argument('--verify-only',action='store_true')
    parser.add_argument('--recipe')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--camera-run',type=Path)
    parser.add_argument('--pose-run',type=Path)
    args=parser.parse_args()
    manifest=json.loads(args.manifest.read_text(encoding='utf8'))
    for name,value in manifest.get('artifacts_sha256',{}).items():
        if sha(args.manifest.parent/name)!=value:raise RuntimeError('Frozen reconstruction artifact differs: '+name)
    for path,value in manifest.get('delivery_tool_sha256',{}).items():
        if sha(ROOT/path)!=value:raise RuntimeError('Frozen delivery tool differs: '+path)
    if manifest.get('data_protocol_sha256') and sha(Path(manifest['data_protocol']))!=manifest['data_protocol_sha256']:
        raise RuntimeError('Frozen data protocol differs')
    for path,value in manifest.get('regression_evidence',{}).items():
        if sha(ROOT/path)!=value:raise RuntimeError('Frozen regression evidence differs: '+path)
    for path,value in manifest['source_sha256'].items():
        if sha(ROOT/path)!=value:raise RuntimeError('Frozen source/config differs: '+path)
    for path,value in manifest['production_protected_files'].items():
        if sha(ROOT/path)!=value:raise RuntimeError('Production file differs: '+path)
    for run in manifest['proven_runs'].values():
        for path,value in run.get('evidence_sha256',{}).items():
            if sha(ROOT/path)!=value:raise RuntimeError('Frozen run evidence differs: '+path)
        inputs=run['input_snapshot']
        for path,value in inputs['inputs'].items():
            resolved=run.get('historical_input_locations',{}).get(path,path)
            if sha(Path(resolved))!=value:raise RuntimeError('Frozen input/dependency differs: '+path)
        if inputs.get('library') and sha(Path(inputs['library']))!=inputs['library_sha256']:
            raise RuntimeError('Frozen library differs: '+inputs['library'])
    if args.verify_only:
        print(json.dumps({'frozen_sources_and_dependencies_verified':True,
            'recipes':sorted(manifest['recipes'])+['184714_two_stage']}));return
    if not args.recipe or args.output is None:parser.error('--recipe and --output are required')
    output=args.output.resolve()
    if output.exists():raise FileExistsError(output)
    allowed={ROOT/'scripts/process_recording.py',ROOT/'scripts/reprocess_recorded_hands.py',
        ROOT/'handoff/goal_delivery_20261113/rejoin_saved_camera_hands.py'}
    def command(name,destination,**substitutions):
        values={'<NEW_OUTPUT_DIRECTORY>':str(destination),**substitutions}
        cmd=[values.get(item,item) for item in manifest['recipes'][name]]
        assert Path(cmd[0]).resolve()==(ROOT/'.venv/Scripts/python.exe').resolve()
        assert Path(cmd[1]).resolve() in allowed
        if any('<' in item for item in cmd):raise ValueError('Recipe has unresolved dependencies: '+name)
        return cmd
    if args.recipe=='184714_two_stage':
        commands=[command('184714_stereo_sim3_fixed',output/'camera'),
            command('184714_si_sim3_compare',output/'poses'),
            command('184714_saved_hands_fusion',output/'fused',
                **{'<COMPLETED_CAMERA_RUN>':str(output/'camera'),'<COMPLETED_SI_POSE_RUN>':str(output/'poses')})]
    else:
        values={}
        if args.camera_run:values['<COMPLETED_CAMERA_RUN>']=str(args.camera_run.resolve())
        if args.pose_run:values['<COMPLETED_SI_POSE_RUN>']=str(args.pose_run.resolve())
        commands=[command(args.recipe,output,**values)]
    # Do not race native builds or a known active VIO worker. Unrelated Python
    # processes, including processes without readable command lines, are untouched.
    import psutil
    for process in psutil.process_iter(['name','cmdline']):
        try:
            name=(process.info['name'] or '').lower();line=' '.join(process.info['cmdline'] or [])
            if name in ('cl.exe','ninja.exe','link.exe') or 'egohand.vio.live_subprocess' in line:
                raise RuntimeError('Native build/VIO worker already running; serialize this workflow')
        except (psutil.AccessDenied,psutil.NoSuchProcess):pass
    if len(commands)>1:output.mkdir(parents=True)
    for cmd in commands:
        print(json.dumps({'running':cmd},ensure_ascii=False),flush=True)
        subprocess.run(cmd,cwd=ROOT,check=True)


if __name__=='__main__':main()
