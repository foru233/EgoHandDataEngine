"""PyCharm Run/Debug entry point for EgoHand Studio.

Use the project's .venv interpreter; no batch-file plugin is required.
"""
import os
import sys
from pathlib import Path


def main():
    project_root = Path(__file__).resolve().parent
    os.chdir(project_root)
    sys.path.insert(0, str(project_root / "src"))
    from egohand.main import main as studio_main

    return studio_main()


if __name__ == "__main__":
    sys.exit(main())
