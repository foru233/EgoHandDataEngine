"""Exercise the actual customer GUI with an explicit offline profile and recording."""
import argparse
from collections import Counter
import importlib.util
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    spec = importlib.util.spec_from_file_location('customer_gui_launcher', ROOT / 'userrr/run_client_gui.py')
    launcher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launcher)
    state = dict(finished=False, pose_messages=0, displayed_frames=0, errors=[])
    seen = set()
    sources = Counter()
    started = time.perf_counter()
    stream = (args.output / 'display_audit.jsonl').open('x', encoding='utf-8')

    def bind(window):
        from PySide6.QtCore import QTimer
        from egohand.vio.localization_state import consumer_map_pose
        def sample():
            result = window.latest
            if result is None or result.timestamp_ns in seen:
                return
            seen.add(result.timestamp_ns)
            state['displayed_frames'] += 1
            message = result.diagnostics.get('vio_pose') or {}
            best = getattr(window.vio_panel, '_display_pose', None)
            source = best.source if best is not None else 'unavailable'
            sources[source] += 1
            admitted = consumer_map_pose(message, world_hands=True,
                expected_timestamp_ns=result.timestamp_ns) is not None
            world = result.world_hands or {}
            if world.get('status') == 'available' and not admitted:
                state['errors'].append(dict(timestamp_ns=result.timestamp_ns, reason='world_hands_without_same_frame_pose'))
            if best is not None and best.trusted and source != 'trusted':
                state['errors'].append(dict(timestamp_ns=result.timestamp_ns, reason='unverified_display_marked_trusted'))
            row = dict(timestamp_ns=result.timestamp_ns, pose_timestamp_ns=message.get('timestamp_ns'),
                tracking_state=message.get('tracking_state'), admitted=admitted,
                display_source=source, display_timestamp_ns=best.timestamp_ns if best is not None else None,
                age_ms=best.age_ms if best is not None else None,
                display_trusted=best.trusted if best is not None else None,
                map_id=message.get('map_id'), reset_id=message.get('reset_id'),
                segment_id=(message.get('local_motion') or {}).get('segment_id'),
                world_hands_status=world.get('status'), status_text=window.state_label.text(),
                world_text=window.world_meta.text(), footer=window.footer.text())
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
            stream.flush()
            if sources[source] == 1:
                window.grab().save(str(args.output / ('display_' + source + '.png')))
        def message(value):
            state['pose_messages'] += value.get('type') == 'pose'
        def finished():
            sample()
            state['finished'] = True
            state['history_frames'] = len(window.session.history)
            state['last_error'] = getattr(window, 'last_error', None)
            window.grab().save(str(args.output / 'display_finished.png'))
        timer = QTimer(window)
        timer.timeout.connect(sample)
        timer.start(100)
        window.session.vio_status.connect(message)
        window.session.finished.connect(finished)

    sys.argv = [str(ROOT / 'userrr/run_client_gui.py'), '--offline-recovery-profile', str(args.profile.resolve()),
        '--recording', str(args.recording.resolve()), '--output-dir', str((args.output / 'run').resolve()),
        '--start', '--exit-when-finished']
    try:
        exit_code = launcher.main(on_window=bind)
    finally:
        stream.close()
    state.update(exit_code=exit_code, elapsed_s=time.perf_counter()-started,
        display_sources=dict(sources), scope='Actual Qt customer GUI and complete recorded worker; sampled display audit, not every frame. No physical device.',
        production_deployed=False, physical_accuracy_verified=False)
    with (args.output / 'gui_validation.json').open('x', encoding='utf-8') as output:
        json.dump(state, output, indent=2, ensure_ascii=False)
    print(json.dumps(state, ensure_ascii=False))
    return 0 if exit_code == 0 and state['finished'] and not state['errors'] and not state.get('last_error') else 1


if __name__ == '__main__':
    sys.exit(main())
