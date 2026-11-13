"""Background map comparison in the imported Atlas coordinate system."""
from pathlib import Path
from tempfile import TemporaryDirectory
import shutil

import numpy as np

from .map_depth import StereoDepth, VoxelEvidence, voxel_centers
from .map_foreground import ForegroundDetector
from .map_package import write_package


class MapComparison:
    def __init__(self, directory, manifest, rectifier, scale):
        self.directory = directory
        self.manifest = manifest
        self.size = float(manifest["voxel_size_m"])
        with np.load(directory / "space.npz", allow_pickle=False) as space:
            self.occupied = set(map(tuple, space["occupied"]))
            self.free = set(map(tuple, space["free"]))
        self.camera = StereoDepth(rectifier.P1, -rectifier.P2[0, 3] / rectifier.P2[0, 0], scale)
        self.detector = ForegroundDetector()
        self.evidence = VoxelEvidence(self.size)

    def close(self):
        self.detector.close()

    def evaluate(self, left, right, pose, continuous):
        if not continuous:
            self.detector.reset()
        depth = self.camera.estimate(left, right)
        mask = self.detector.mask(left, depth, pose, self.camera)
        dynamic = self.camera.sample(depth, stride=8, foreground=~mask)
        dynamic = dynamic @ pose[:3, :3].T + pose[:3, 3]
        self.evidence.observe(self.camera.sample(depth, stride=6, foreground=mask | (depth < .45)), pose)
        known, unknown, removed = self.evidence.differences(self.occupied, self.free)
        return {"dynamic": dynamic, "added": voxel_centers(known, self.size),
                "new_area": voxel_centers(unknown, self.size),
                "removed": voxel_centers(removed, self.size)}, (known, unknown, removed)

    def save_revision(self, destination, candidates):
        known, unknown, removed = candidates
        # Only extend into unknown voxels immediately adjacent to observed space.
        observed = self.occupied | self.free | known
        connected = set()
        for x, y, z in unknown:
            neighbors = ((x+1,y,z), (x-1,y,z), (x,y+1,z), (x,y-1,z), (x,y,z+1), (x,y,z-1))
            if any(key in observed for key in neighbors):
                connected.add((x, y, z))
        occupied = (self.occupied - removed) | known | connected
        free = (self.free | removed) - occupied
        if occupied == self.occupied:
            raise ValueError("没有可保存的已确认地图变化")
        destination = Path(destination)
        if destination.exists():
            raise FileExistsError(destination)
        with TemporaryDirectory(prefix=".map-revision-", dir=destination.parent) as temporary:
            staging = Path(temporary) / "map"
            staging.mkdir()
            for name in ("atlas.osa", "calibration.yaml", "orb.yaml"):
                shutil.copyfile(self.directory / name, staging / name)
            write_package(staging, occupied, free, self.size, self.manifest["atlas_map_id"],
                          self.manifest["processing"], previous=self.manifest)
            staging.rename(destination)

