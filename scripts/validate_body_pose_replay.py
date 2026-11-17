"""Audit recorded results for body-pose inputs without changing the recording."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from types import SimpleNamespace

from egohand.body.body_pose_gate import evaluate_body_gate
from egohand.body.body_pose_estimator import BodyPoseEstimator
from egohand.core.types import result_from_dict


def audit(path: Path):
    estimator = BodyPoseEstimator()
    counts = {"frames": 0, "dual_camera_hands": 0, "world_hand_frames": 0,
              "strict_valid": 0, "preview_estimated": 0, "rejected": 0}
    reasons = {}
    for line in path.open(encoding="utf8"):
        if not line.strip():
            continue
        result = result_from_dict(json.loads(line))
        counts["frames"] += 1
        if sum(hand.valid_joint_count > 0 for hand in result.hands) >= 2:
            counts["dual_camera_hands"] += 1
        world = result.world_hands or {}
        if world.get("status") in ("available", "no_valid_hand_joints"):
            counts["world_hand_frames"] += 1
        pose = result.diagnostics.get("live_vio_pose") or result.diagnostics.get("vio_pose")
        estimate, gate = estimator.update(result, pose, strict_gate=True)
        if estimate is not None and estimate.strict_valid:
            counts["strict_valid"] += 1
        else:
            counts["rejected"] += 1
            reasons[gate.reason] = reasons.get(gate.reason, 0) + 1
        if not gate.strict_valid:
            preview, preview_gate = estimator.update(result, pose, strict_gate=False)
            if preview is not None and preview.status == "estimated_preview":
                counts["preview_estimated"] += 1
    return {"source": str(path.resolve()), "counts": counts, "rejection_reasons": reasons,
            "note": "No world hand frame is treated as a real three-input acceptance."}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("results", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = audit(args.results)
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf8")
    print(text)


if __name__ == "__main__":
    main()
