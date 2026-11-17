"""Frozen same-DLL A/B for isolated native observation and recovery fixes.

Uses development sequences already inspected; no independent-holdout claim.
"""
import argparse
import json
import os
from pathlib import Path
import sys

from run_public_recovery_acceptance import ROOT, execute, save, sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--experiment', choices=['index', 'right_weight', 'lost_keyframes', 'visual_velocity', 'geometric_retry', 'merge_observation', 'frame_search_seed', 'preoptimizer_retry', 'keyframe_uncertainty'], default='index')
    parser.add_argument('--exact-graph-range', type=int, nargs=2, metavar=('FIRST', 'LAST'))
    parser.add_argument('--sequences', nargs='+', choices=['room1', 'room2', 'room3_holdout', 'corridor4'],
                        default=['room3_holdout'])
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    library = args.library.resolve(strict=True)
    runner = ROOT / 'scripts/run_tumvi_geometry_recovery.py'
    scripts = [runner, Path(__file__), ROOT / 'scripts/audit_native_translation_stages.py',
               ROOT / 'scripts/evaluate_public_recovery_trajectory.py', ROOT / 'scripts/audit_public_gate_gt.py']
    frozen = {str(p): sha(p) for p in [library, *scripts, *sorted((ROOT / 'src/egohand').rglob('*.py'))]}
    sizes = {'room1': 850, 'room2': 850, 'room3_holdout': 2821, 'corridor4': 1927}
    cases = [dict(sequence=s, frames=sizes[s], mask_frames=0 if s == 'corridor4' else 40, corrected=corrected)
             for s in args.sequences for corrected in [False, True]]
    save(args.output / 'freeze.json', dict(cases=cases, hashes=frozen,
        experiment=args.experiment,
        change={'index': 'RIGHT_INDEX_FIX only',
                'right_weight': 'RIGHT_WEIGHT_FIX only, RIGHT_INDEX_FIX=1 in both arms',
                'lost_keyframes': 'LOST_KEYFRAME_GUARD only, RIGHT_INDEX_FIX=RIGHT_WEIGHT_FIX=1 in both arms',
                'visual_velocity': 'VISUAL_VELOCITY_SEED only, RIGHT_INDEX_FIX=RIGHT_WEIGHT_FIX=LOST_KEYFRAME_GUARD=1 in both arms',
                'geometric_retry': 'GEOMETRIC_RETRY only, RIGHT_INDEX_FIX=RIGHT_WEIGHT_FIX=LOST_KEYFRAME_GUARD=VISUAL_VELOCITY_SEED=1 in both arms',
                'merge_observation': 'MERGE_OBSERVATION_FIX only, RIGHT_INDEX_FIX=RIGHT_WEIGHT_FIX=LOST_KEYFRAME_GUARD=VISUAL_VELOCITY_SEED=1; GEOMETRIC_RETRY=0 both arms',
                'frame_search_seed': 'FRAME_SEARCH_SEED only, RIGHT_INDEX_FIX=RIGHT_WEIGHT_FIX=LOST_KEYFRAME_GUARD=VISUAL_VELOCITY_SEED=MERGE_OBSERVATION_FIX=1; GEOMETRIC_RETRY=0 both arms',
                'preoptimizer_retry': 'GEOMETRIC_RETRY only (saved preoptimizer visual seed), RIGHT_INDEX_FIX=RIGHT_WEIGHT_FIX=LOST_KEYFRAME_GUARD=VISUAL_VELOCITY_SEED=MERGE_OBSERVATION_FIX=FRAME_SEARCH_SEED=1 both arms',
                'keyframe_uncertainty': 'KF_COVARIANCE_PRIOR only, KF_COVARIANCE_CAPTURE=RIGHT_INDEX_FIX=RIGHT_WEIGHT_FIX=LOST_KEYFRAME_GUARD=VISUAL_VELOCITY_SEED=MERGE_OBSERVATION_FIX=1 both arms; GEOMETRIC_RETRY=FRAME_SEARCH_SEED=0'}[args.experiment]
               + '; same DLL, host, input, calibration, SHORT_RECOVERY and .5s policy.',
        acceptance='Diagnostic development A/B only. Do not promote from admission counts or one pair. Assess complete reference trajectories, preloss anchored GT, all jump events and actual right graph edges.',
        independent_holdout=False, production_deployed=False))
    env = os.environ.copy()
    for key in list(env):
        if key.startswith(('EGOHAND_VIO_', 'EGOHAND_TEST_', 'EGOHAND_COORDINATOR_')):
            env.pop(key)
    env.update(TUMVI_RECOVERY_LIBRARY=str(library), PYTHONIOENCODING='utf-8', PYTHONPATH=str(ROOT / 'src'))
    outcomes = []
    for case in cases:
        assert all(sha(Path(p)) == h for p, h in frozen.items())
        name = case['sequence'] + ('_corrected' if case['corrected'] else '_legacy')
        output = args.output / name
        command = [sys.executable, str(runner), 'run', str(output), '--sequence', case['sequence'],
                   '--frames', str(case['frames']), '--mask-start', '550', '--mask-frames', str(case['mask_frames']),
                   '--stable-seconds', '.5', '--kind', 'candidate', '--native-diagnostics']
        if args.experiment in ('right_weight', 'lost_keyframes', 'visual_velocity', 'geometric_retry', 'merge_observation', 'frame_search_seed', 'preoptimizer_retry') or case['corrected'] or args.experiment == 'keyframe_uncertainty':
            command.append('--right-index-fix')
        if args.experiment in ('lost_keyframes', 'visual_velocity', 'geometric_retry', 'merge_observation', 'frame_search_seed', 'preoptimizer_retry') or (args.experiment == 'right_weight' and case['corrected']) or args.experiment == 'keyframe_uncertainty':
            command.append('--right-weight-fix')
        if args.experiment in ('visual_velocity', 'geometric_retry', 'merge_observation', 'frame_search_seed', 'preoptimizer_retry') or (args.experiment == 'lost_keyframes' and case['corrected']) or args.experiment == 'keyframe_uncertainty':
            command.append('--lost-keyframe-guard')
        if args.experiment in ('geometric_retry', 'merge_observation', 'frame_search_seed', 'preoptimizer_retry') or (args.experiment == 'visual_velocity' and case['corrected']) or args.experiment == 'keyframe_uncertainty':
            command.append('--visual-velocity-seed')
        if args.experiment in ('geometric_retry', 'preoptimizer_retry') and case['corrected']:
            command.append('--geometric-retry')
        if args.experiment in ('frame_search_seed', 'preoptimizer_retry') or (args.experiment == 'merge_observation' and case['corrected']) or args.experiment == 'keyframe_uncertainty':
            command.append('--merge-observation-fix')
        if args.experiment == 'preoptimizer_retry' or (args.experiment == 'frame_search_seed' and case['corrected']):
            command.append('--frame-search-seed')
        if args.experiment == 'keyframe_uncertainty':
            command.append('--kf-covariance-capture')
            if case['corrected']:
                command.append('--kf-covariance-prior')
        if args.exact_graph_range is not None:
            command.extend(['--exact-graph-range', *map(str, args.exact_graph_range)])
        print(json.dumps(dict(starting=name)), flush=True)
        result = dict(case=case, command=command, **execute(command, args.output / (name + '.log'), env))
        if result['exit_code'] == 0:
            actions = [('score', runner, ['score']),
                       ('trajectory', ROOT / 'scripts/evaluate_public_recovery_trajectory.py', []),
                       ('gate_gt', ROOT / 'scripts/audit_public_gate_gt.py', []),
                       ('stages', ROOT / 'scripts/audit_native_translation_stages.py', [])]
            for action, script, prefix in actions:
                extra = [] if action == 'score' else ['--output', str(output / (action + '.json'))]
                result[action] = execute([sys.executable, str(script), *prefix, str(output), *extra],
                                         args.output / (name + '_' + action + '.log'), env)
        outcomes.append(result)
        save(args.output / (name + '_outcome.json'), result)
        print(json.dumps(dict(finished=name, exit_code=result['exit_code'])), flush=True)
    assert all(sha(Path(p)) == h for p, h in frozen.items())
    save(args.output / 'outcomes.json', dict(outcomes=outcomes, source_integrity=True, production_deployed=False))


if __name__ == '__main__':
    main()
