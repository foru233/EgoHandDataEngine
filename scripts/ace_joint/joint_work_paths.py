import os
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT = SCRIPT_DIR.parents[1]
JOINT_WORK = Path(os.environ.get('EGOHAND_JOINT_WORK_DIR', PROJECT / 'data/.joint_work')).expanduser().resolve()
ACE = Path(os.environ.get('EGOHAND_ACE_ROOT', PROJECT / 'third_party/ACE-Ego-Hand')).expanduser().resolve()
MANO = Path(os.environ.get('EGOHAND_JOINT_MANO_DIR', PROJECT / 'models/mano')).expanduser().resolve()


def is_joint_workspace(path):
    return path.resolve().is_relative_to(JOINT_WORK)
