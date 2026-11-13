"""Stereo surface observations and conservative sparse voxel evidence in Atlas metres."""
from collections import Counter
from itertools import product

import cv2
import numpy as np

NEIGHBOR_OFFSETS = tuple(product((-1, 0, 1), repeat=3))


class StereoDepth:
    def __init__(self, projection, baseline_m, scale):
        self.fx, self.fy = projection[0, 0] * scale, projection[1, 1] * scale
        self.cx = (projection[0, 2] + .5) * scale - .5
        self.cy = (projection[1, 2] + .5) * scale - .5
        self.baseline_m = baseline_m
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=96, blockSize=5,
            P1=200, P2=800, uniquenessRatio=12, speckleWindowSize=60, speckleRange=2,
        )

    def estimate(self, left, right, *, min_depth_m=.25, max_depth_m=4.):
        disparity = self.matcher.compute(left, right).astype(np.float32) / 16
        reverse = self.matcher.compute(np.fliplr(right), np.fliplr(left))[:, ::-1] / 16
        y, x = np.indices(left.shape)
        rx = np.rint(x - disparity).astype(int)
        depth = np.divide(self.fx * self.baseline_m, disparity, out=np.zeros_like(disparity),
                          where=disparity > 0)
        matched = reverse[y, np.clip(rx, 0, left.shape[1] - 1)]
        valid = (rx >= 0) & (np.abs(matched - disparity) <= .75) & (depth >= min_depth_m) & (depth <= max_depth_m)
        return np.where(valid, depth, 0)

    def unproject(self, x, y, z):
        return np.column_stack(((x - self.cx) * z / self.fx, (y - self.cy) * z / self.fy, z))

    def sample(self, depth, *, stride, foreground=None):
        y, x = np.mgrid[:depth.shape[0]:stride, :depth.shape[1]:stride]
        z = depth[y, x]
        valid = z > 0
        if foreground is not None:
            valid &= ~foreground[y, x]
        return self.unproject(x[valid], y[valid], z[valid])

    def points(self, left, right, *, stride, foreground=None):
        return self.sample(self.estimate(left, right), stride=stride, foreground=foreground)


class VoxelEvidence:
    def __init__(self, size_m=.08, view_spacing_m=.18):
        self.size_m = size_m
        self.view_spacing_m = view_spacing_m
        self.occupied = Counter()
        self.free = Counter()
        self.origins = []

    def observe(self, points, pose):
        if not len(points):
            return
        origin = pose[:3, 3]
        if any(np.linalg.norm(origin - other) < self.view_spacing_m for other in self.origins):
            return
        self.origins.append(origin.copy())
        rays = points @ pose[:3, :3].T
        ends = np.floor((rays + origin) / self.size_m).astype(np.int32)
        occupied = set(map(tuple, np.unique(ends, axis=0)))

        # Reserve extra margin before the stereo hit. A ray terminating at an
        # occluder provides no free-space evidence behind that occluder.
        distances = np.linalg.norm(rays, axis=1)
        directions = rays / distances[:, None]
        free_parts = []
        margin = 2 * self.size_m + .025 * distances**2
        for step in np.arange(.15, np.max(distances), self.size_m / 2):
            active = step < distances - margin
            if active.any():
                free_parts.append(np.floor((origin + step * directions[active]) / self.size_m)
                                  .astype(np.int32))
        free = set()
        if free_parts:
            free = set(map(tuple, np.unique(np.concatenate(free_parts), axis=0)))
        self.occupied.update(occupied)
        self.free.update(free - occupied)

    def resolved(self, min_views=2):
        occupied = {key for key, hits in self.occupied.items()
                    if hits >= min_views and hits >= self.free[key]}
        free = {key for key, hits in self.free.items()
                if hits >= min_views and self.occupied[key] == 0}
        return occupied, free

    def differences(self, baseline_occupied, baseline_free, min_views=3):
        occupied, free = self.resolved(min_views)
        def near_surface(key, surfaces):
            x, y, z = key
            return any((x+dx, y+dy, z+dz) in surfaces for dx, dy, dz in NEIGHBOR_OFFSETS)

        additions = {key for key in occupied if not near_surface(key, baseline_occupied)}
        removals = {key for key in free & baseline_occupied if not near_surface(key, occupied)}
        return additions & baseline_free, additions - baseline_free, removals


def voxel_centers(keys, size):
    return (np.asarray(sorted(keys), dtype=np.float32).reshape(-1, 3) + .5) * size

