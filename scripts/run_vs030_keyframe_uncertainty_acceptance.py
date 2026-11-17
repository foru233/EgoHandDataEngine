"""Exclusive, same-policy full recorded worker A/B; never deploy a DLL."""
import argparse
import json
import os
from pathlib import Path
import sys

from run_public_recovery_acceptance import ROOT, execute, save, sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--experiment', choices=['short_recovery', 'lost_keyframes', 'geometric_retry', 'merge_observation', 'keyframe_uncertainty'], default='keyframe_uncertainty')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    base = ROOT / 'handoff/vio_gate_optimization_20261114'
    profiles = base / 'vs030_184714_stability_matrix_20261114'
    recordings = ['data/session_20261112_184714_636910800',
                  'data/hand/session_20261110_111807_489019700',
                  'data/hand/session_20261110_111833_589265500']
    library = args.library.resolve(strict=True)
    inputs = [library, ROOT / 'scripts/process_recording.py',
              profiles / 'quality_0_5.yaml', profiles / 'recovery_0_5.yaml',
              ROOT / '\u6807\u5b9a\u7ed3\u679c/2/data3-camchain-imucam-measured.yaml',
              ROOT / 'handoff/vio_architecture_execution_20261113/bootstrap_full_settings.yaml']
    inputs += [Path(__file__).resolve(), ROOT / 'scripts/audit_recorded_world_hands.py',
               ROOT / 'configs/detector.yaml', ROOT / 'configs/stereo.yaml',
               *sorted((ROOT / 'src/egohand').rglob('*.py'))]
    digests = {str(p): sha(p) for p in inputs}
    save(args.output / 'suite_plan.json', {
        'recordings': recordings, 'stable_seconds': .5, 'full_hand_worker': True,
        'hashes': digests, 'independent_ground_truth': False,
        'scope': 'Candidate versus feature-disabled control, same DLL/policy/settings/raw input; no device or visible GUI operation. Separate native runs may vary.',
        'experiment': args.experiment,
        'native_observation_journals': args.experiment in ('merge_observation', 'keyframe_uncertainty'),
        'algorithm_variable': {
            'short_recovery': 'SHORT_RECOVERY',
            'lost_keyframes': 'LOST_KEYFRAME_GUARD; SHORT_RECOVERY=RIGHT_INDEX_FIX=RIGHT_WEIGHT_FIX=1 in both arms',
            'geometric_retry': 'GEOMETRIC_RETRY; SHORT_RECOVERY=RIGHT_INDEX_FIX=RIGHT_WEIGHT_FIX=LOST_KEYFRAME_GUARD=VISUAL_VELOCITY_SEED=1 in both arms',
            'keyframe_uncertainty': 'KF_COVARIANCE_PRIOR only; KF_COVARIANCE_CAPTURE=SHORT_RECOVERY=RIGHT_INDEX_FIX=RIGHT_WEIGHT_FIX=LOST_KEYFRAME_GUARD=VISUAL_VELOCITY_SEED=MERGE_OBSERVATION_FIX=1; GEOMETRIC_RETRY=FRAME_SEARCH_SEED=0 both arms',
            'merge_observation': 'MERGE_OBSERVATION_FIX; SHORT_RECOVERY=RIGHT_INDEX_FIX=RIGHT_WEIGHT_FIX=LOST_KEYFRAME_GUARD=VISUAL_VELOCITY_SEED=1; GEOMETRIC_RETRY=0 both arms',
        }[args.experiment],
        'performance_limit': 'Other read-only public runs may overlap; offline timings do not certify realtime or matched resource conditions.',
        'production_deployed': False})
    outcomes = []
    plans = {}
    for recording in recordings:
        root = ROOT / recording
        tag = root.name.split('_')[2]
        for kind in ('candidate', 'control'):
            if any(sha(Path(p)) != value for p, value in digests.items()):
                raise RuntimeError('Frozen DLL/source/configuration changed')
            name = f'{tag}_{kind}'
            output = args.output / name
            environment = os.environ.copy()
            for key in list(environment):
                if key.startswith(('EGOHAND_VIO_', 'EGOHAND_TEST_', 'TASK_RECOVERY_', 'EGOHAND_COORDINATOR_')):
                    environment.pop(key)
            environment.update(EGOHAND_VIO_SHORT_RECOVERY='1' if kind == 'candidate' else '0',
                EGOHAND_VIO_RECOVERY_CONFIG=str(profiles / 'recovery_0_5.yaml'),
                EGOHAND_VIO_LIBRARY=str(library), PYTHONIOENCODING='utf-8',
                EGOHAND_NATIVE_FAULT_TRACE='1')
            if args.experiment in ('lost_keyframes', 'geometric_retry', 'merge_observation', 'keyframe_uncertainty'):
                environment.update(EGOHAND_VIO_SHORT_RECOVERY='1',
                    EGOHAND_VIO_RIGHT_INDEX_FIX='1', EGOHAND_VIO_RIGHT_WEIGHT_FIX='1',
                    EGOHAND_VIO_LOST_KEYFRAME_GUARD='1' if args.experiment in ('geometric_retry','merge_observation', 'keyframe_uncertainty') or kind=='candidate' else '0')
            if args.experiment in ('geometric_retry', 'merge_observation', 'keyframe_uncertainty'):
                environment.update(EGOHAND_VIO_VISUAL_VELOCITY_SEED='1',
                    EGOHAND_VIO_GEOMETRIC_RETRY='1' if args.experiment=='geometric_retry' and kind=='candidate' else '0',
                    EGOHAND_VIO_MERGE_OBSERVATION_FIX='1' if args.experiment=='keyframe_uncertainty' or (args.experiment=='merge_observation' and kind=='candidate') else '0')
            if args.experiment in ('merge_observation', 'keyframe_uncertainty'):
                environment['EGOHAND_VIO_COMMIT_LOG'] = str(
                    (args.output / (name + '_native_commits.jsonl')).resolve())
            if args.experiment == 'keyframe_uncertainty':
                environment.update(EGOHAND_VIO_KF_COVARIANCE_CAPTURE='1',
                    EGOHAND_VIO_KF_COVARIANCE_PRIOR='1' if kind=='candidate' else '0',
                    EGOHAND_VIO_FRAME_SEARCH_SEED='0', EGOHAND_VIO_GEOMETRIC_RETRY='0')
            command = [sys.executable, str(ROOT / 'scripts/process_recording.py'), str(root), str(output),
                '--mode', 'stereo_inertial', '--image-scale', '1', '--session-auto-phase',
                '--calibration', str(inputs[4]), '--settings', str(inputs[5]),
                '--timing', str(root / 'vio_timing.yaml'), '--quality', str(inputs[2]), '--library', str(library)]
            print(json.dumps({'starting': name, 'command': command}), flush=True)
            result = {'name': name, 'command': command,
                      **execute(command, args.output / (name + '.log'), environment)}
            if result['exit_code'] == 0:
                validation = json.loads((output / 'validation.json').read_text(encoding='utf-8'))
                result['validation'] = validation
                plan = json.loads((output / 'processing_inputs.json').read_text(encoding='utf-8'))
                comparable = {k: plan[k] for k in ('inputs', 'source_hashes', 'config', 'library_sha256')}
                if tag in plans:
                    if comparable != plans[tag]:
                        raise RuntimeError('A/B input/source/settings mismatch')
                    result['same_input_policy_AB_verified'] = True
                plans[tag] = comparable
                audit = [sys.executable, str(ROOT / 'scripts/audit_recorded_world_hands.py'),
                         str(output / 'archive/frames.jsonl'), str(output / 'world_hand_audit.json')]
                # Find the worker archive without assuming a legacy folder layout.
                archive = list(output.rglob('frames.jsonl'))
                if len(archive) == 1:
                    audit[2] = str(archive[0])
                    result['world_hand_audit'] = execute(audit, args.output / (name + '_hands.log'), environment)
                else:
                    result['world_hand_audit'] = {'unavailable': 'missing_or_ambiguous_frames_archive', 'paths': [str(p) for p in archive]}
            outcomes.append(result)
            save(args.output / (name + '_outcome.json'), result)
            print(json.dumps({'finished': name, 'exit_code': result['exit_code'],
                'admitted': result.get('validation', {}).get('pose_admitted')}), flush=True)
    save(args.output / 'suite_outcomes.json', {'outcomes': outcomes, 'production_deployed': False})


if __name__ == '__main__':
    main()
