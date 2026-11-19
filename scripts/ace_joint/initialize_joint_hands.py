"""Associate camera evidence and create hand parameters by direct forward evaluation."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import torch

from fusion import associate, collect_sequence
from offline_mano import landmarks, make_model, mean_rotations


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ace-run', type=Path, required=True)
    parser.add_argument('--observations', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(2)
    raw, manifest = collect_sequence(args.ace_run)
    data = associate(raw, manifest, args.observations)
    count = manifest['frames']
    orientations = np.broadcast_to(np.eye(3, dtype=np.float32), (count, 2, 16, 3, 3)).copy()
    position = np.zeros((count, 2, 3), np.float32)
    wrist = np.full((count, 2, 3), np.nan, np.float32)
    betas = np.broadcast_to(data['betas'].astype(np.float32), (count, 2, 10)).copy()
    support = data['valid'].any(axis=1)
    active = (support.sum(axis=-1) >= 4) & (data['identity_ids'] >= 0)
    stereo = data['stereo_valid'] & np.isfinite(data['stereo_xyz']).all(axis=-1)
    for hand, side in enumerate(('left', 'right')):
        model = make_model(side, 'cpu')
        for index in np.flatnonzero(active[:, hand]):
            rotations = data['prior_R'][index, hand]
            translations = data['prior_t'][index, hand]
            weights = data['prior_weight'][index, hand]
            good = (weights > 0) & np.isfinite(rotations).all((1, 2, 3)) & np.isfinite(translations).all(axis=1)
            has_stereo = stereo[index, hand].sum() >= 4
            if not has_stereo and not good.any():
                active[index, hand] = False
                continue
            if good.any():
                orientations[index, hand] = mean_rotations(rotations[good], weights[good])
                position[index, hand] = np.average(translations[good], axis=0, weights=weights[good])
        active_rows = np.flatnonzero(active[:, hand])
        for first in range(0, len(active_rows), 128):
            indices = active_rows[first:first+128]
            pose = Rotation.from_matrix(orientations[indices, hand].reshape(-1, 3, 3)).as_rotvec()
            pose = torch.from_numpy(pose.astype(np.float32)).reshape(-1, 48)
            with torch.no_grad():
                output = model(global_orient=pose[:, :3], hand_pose=pose[:, 3:],
                               betas=torch.from_numpy(betas[indices, hand]))
                local = landmarks(output, side).numpy()
            for row, index in enumerate(indices):
                mask = stereo[index, hand]
                if mask.sum() >= 4:
                    position[index, hand] = np.median(data['stereo_xyz'][index, hand, mask] - local[row, mask], axis=0)
                wrist[index, hand] = (data['stereo_xyz'][index, hand, 0] if mask[0]
                                      else local[row, 0] + position[index, hand])
    if not np.isfinite(position).all():
        raise ValueError('Nonfinite direct hand initialization')
    np.savez_compressed(args.output / 'associated_observations.npz',
                        **{key: value for key, value in data.items() if key != 'diagnostics'})
    np.savez_compressed(args.output / 'mano_params.npz', rotations=orientations, translation_m=position,
                        betas=betas, timestamps_ns=data['timestamps_ns'], initial_hand_mask=active, wrist_camera_m=wrist)
    (args.output / 'manifest.json').write_text(json.dumps(dict(
        initialization='ACE rotations and direct stereo translation', optimized=False, frames=count,
        active_hand_frames=active.sum(axis=0).tolist()), indent=2), encoding='utf8')
    print('HAND INITIALIZATION', active.sum(axis=0).tolist(), flush=True)


if __name__ == '__main__':
    main()
