"""Construct neutral SMPL parameters and admitted camera inputs without fitting."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

from joint_work_paths import PROJECT
sys.path[:0] = [str(PROJECT / 'src'), str(PROJECT / 'scripts')]
from egohand.body.smpl_binding import mesh_asset
from egohand.body.smpl_mapping import SMPL_FROM_BODY, SMPL_JOINT_NAMES, SMPL_PARENT_IDS
from egohand.body.smpl_mount import HEAD_ANCHOR_VERSION
from egohand.body.smpl_runtime import GroundReference
from egohand.body.smpl_unified import UnifiedSmpl
from egohand.storage.replay import load_recorded_results
from egohand.vio.hand_pose_binding import consumer_result_display_pose
from smpl_body_mesh import SmplBody


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('camera', type=Path)
    parser.add_argument('--initial-camera-height-m', type=float, required=True)
    args = parser.parse_args()
    source = args.source
    rows, truncated = load_recorded_results(source)
    if not rows or truncated:
        raise ValueError('Missing or incomplete camera observations')
    floor = GroundReference(args.initial_camera_height_m)
    count = len(rows)
    transforms = np.full((count, 4, 4), np.nan)
    ground = np.full(count, np.nan)
    references = []
    for index, row in enumerate(rows):
        transform = consumer_result_display_pose(row)
        references.append(repr(floor.observe(row)))
        if transform is not None:
            transforms[index] = transform
            ground[index] = floor.floor_for(row)
    if not np.isfinite(ground).any():
        raise ValueError('No admitted camera trajectory; joint fitting requires at least one located frame')
    frame_id = np.array([row.frame_id for row in rows])
    timestamps = np.array([row.timestamp_ns for row in rows])
    np.savez_compressed(args.camera, frame_id=frame_id, timestamp_ns=timestamps,
                        T_world_from_camera=transforms, ground_y_m=ground, reference=np.array(references))

    smpl = SmplBody()
    neutral = UnifiedSmpl(smpl.model)
    arrays = dict(body_pose=np.zeros((count, 69), np.float32), global_orient=np.zeros((count, 3), np.float32),
                  transl=np.zeros((count, 3), np.float32), joints_world=np.zeros((count, 24, 3), np.float32),
                  rotations_world=np.zeros((count, 24, 3, 3), np.float32),
                  betas=np.repeat(neutral.betas.detach().numpy(), count, axis=0), valid=np.zeros(count, bool))
    np.savez_compressed(source / 'smpl_pose.npz', **arrays, frame_id=frame_id, timestamp_ns=timestamps)
    (source / 'smpl_mesh.json').write_text(json.dumps(mesh_asset(smpl, neutral)), encoding='utf8')
    metadata = dict(initialization_only=True, body_initialization='neutral_smpl', frames=count,
                    head_anchor_vertex=neutral.head_vertex, head_anchor_version=HEAD_ANCHOR_VERSION,
                    mesh_file='smpl_mesh.json', pose_file='smpl_pose.npz', vertices_file='smpl_vertices.npy',
                    initial_camera_height_m=args.initial_camera_height_m,
                    model='SMPL_NEUTRAL', vertices_coordinate_frame='body_world',
                    world_to_smpl=SMPL_FROM_BODY.tolist(), joint_names=SMPL_JOINT_NAMES,
                    joint_tree=SMPL_PARENT_IDS)
    (source / 'unified_smpl_manifest.json').write_text(json.dumps(metadata, indent=2), encoding='utf8')
    print('NEUTRAL SEED', int(np.isfinite(ground).sum()), '/', count, flush=True)


if __name__ == '__main__':
    main()
