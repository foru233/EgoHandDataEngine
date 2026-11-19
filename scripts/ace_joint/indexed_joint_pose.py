"""Load the saved MANO/SMPL JSON sidecar one frame at a time."""
import json
from pathlib import Path
from uuid import uuid4

import numpy as np
from egohand.core.types import result_from_dict
from egohand.sources.recorded import read_jsonl_recover
from egohand.storage.replay import load_recorded_results as native_load
from shared_replay_cache import digest

INDEX_NAME = 'joint_pose_index.npz'


def build_index(root):
    index_path = root / INDEX_NAME
    if index_path.is_file():
        return
    source = root / 'derived_pose.jsonl'
    records = []
    with source.open('rb') as stream:
        offset = 0
        for line in stream:
            row = json.loads(line)
            records.append((offset, len(line), int(row['frame_id']), int(row['timestamp_ns']), bool(row.get('body'))))
            offset += len(line)
    temporary = root / f'joint_pose_index_{uuid4().hex}.building.npz'
    np.savez(temporary, version=1, rows=np.asarray(records, dtype=np.int64), source_sha256=digest(source))
    temporary.rename(index_path)


class IndexedResults(list):
    def __init__(self, values, root, offsets, manifest, mesh):
        super().__init__(values)
        self.root = root
        self.offsets = offsets
        self.manifest = manifest
        self.mesh = mesh
        self.valid_body = offsets[:, 4].astype(bool)
        self.loaded = None

    def attach_pose(self, index):
        result = self[index]
        if index == self.loaded:
            return
        offset, length = map(int, self.offsets[index, :2])
        with (self.root / 'derived_pose.jsonl').open('rb') as source:
            source.seek(offset)
            row = json.loads(source.read(length))
        if (int(row['frame_id']), int(row['timestamp_ns'])) != (result.frame_id, result.timestamp_ns):
            raise ValueError('Indexed joint pose does not match the selected frame')
        row.setdefault('body_mesh_index', index)
        row['body_mesh_vertices_path'] = str(self.root / self.manifest.get('vertices_file', 'smpl_vertices.npy'))
        row.setdefault('head_anchor_version', self.manifest.get('head_anchor_version'))
        if row.get('body_mesh_ref') == self.manifest.get('mesh_file', 'smpl_mesh.json'):
            row['body_mesh'] = self.mesh
        if self.loaded is not None:
            self[self.loaded].diagnostics.pop('offline_derived_pose', None)
        result.diagnostics['offline_derived_pose'] = row
        self.loaded = index


def load_joint_results(directory):
    root = Path(directory)
    if not (root / INDEX_NAME).is_file() or not (root / 'display_cache.sqlite3').is_file():
        return native_load(directory)
    try:
        with np.load(root / INDEX_NAME, allow_pickle=False) as index:
            if int(index['version']) != 1 or str(index['source_sha256']) != digest(root / 'derived_pose.jsonl'):
                raise ValueError('Joint pose index fingerprint mismatch')
            offsets = index['rows']
        manifest = json.loads((root / 'unified_smpl_manifest.json').read_text(encoding='utf8'))
        if manifest.get('algorithm') != 'joint_smpl_mano_v2':
            return native_load(directory)
        mesh = json.loads((root / manifest.get('mesh_file', 'smpl_mesh.json')).read_text(encoding='utf8'))
        rows, truncated = read_jsonl_recover(root / 'results/frames.jsonl')
        if offsets.shape != (len(rows), 5) or truncated or any(
                (int(row['frame_id']), int(row['timestamp_ns'])) != tuple(position[2:4])
                for row, position in zip(rows, offsets)):
            raise ValueError('Joint pose index frame identities differ')
        return IndexedResults(map(result_from_dict, rows), root, offsets, manifest, mesh), False
    except (OSError, ValueError, TypeError, KeyError):
        return native_load(directory)
