"""Limit an isolated process_recording replay for frame-stage diagnosis."""
import os
from pathlib import Path
import runpy
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from egohand.vio.replay_runtime import ReplayVio

maximum = int(os.environ['TASK_DIAGNOSTIC_MAX_FRAMES'])
if maximum < 1:
    raise ValueError('Diagnostic frame limit must be positive')
original = ReplayVio.__init__


def limited(self, *args, **kwargs):
    if 'max_frames' in kwargs:
        raise ValueError('Conflicting diagnostic frame limit')
    original(self, *args, max_frames=maximum, **kwargs)


ReplayVio.__init__ = limited
runpy.run_path(str(ROOT / 'scripts/process_recording.py'), run_name='__main__')
