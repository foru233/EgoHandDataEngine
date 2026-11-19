"""Build a self-contained native replay database and pre-rendered preview images."""
import argparse
from contextlib import closing
import json
from pathlib import Path
import shutil
import sqlite3
import sys
from uuid import uuid4
import zlib

from joint_work_paths import PROJECT
sys.path.insert(0, str(PROJECT / 'src'))

import cv2
import numpy as np
from egohand.calibration.loader import load_calibration
from egohand.core.types import StereoFrame, result_from_dict
from egohand.storage.depth_results import DEPTH_FILE, DepthResultWriter, calibration_sha256
from egohand.vio.hand_pose_binding import consumer_result_display_pose
from egohand.vio.local_segments import consumer_display_pose
from egohand.visualization.stereo_depth import DepthCache, colorize_depth, compute_depth
from shared_replay_cache import digest
from indexed_joint_pose import build_index


def preview_jpeg(image, width):
    if image.shape[1] > width:
        image = cv2.resize(image, (width, round(image.shape[0] * width / image.shape[1])),
                           interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 88])
    if not ok:
        raise ValueError('Preview JPEG encoding failed')
    return encoded.tobytes()


def open_readonly(path):
    return sqlite3.connect(path.resolve().as_uri() + '?mode=ro', uri=True)


def reuse_metric_depth(root, source, image_source):
    target = root / DEPTH_FILE
    if target.exists():
        raise FileExistsError(target)
    calibration = load_calibration(root / 'processing_calibration.yaml')
    temporary = root / f'depth_import_{uuid4().hex}.building.sqlite3'
    shutil.copyfile(source / DEPTH_FILE, temporary)
    with closing(sqlite3.connect(temporary)) as depth_db, closing(open_readonly(
            source / 'display_cache.sqlite3')) as original, closing(open_readonly(
            image_source / 'display_cache.sqlite3')) as images:
        metadata = {key: json.loads(value) for key, value in depth_db.execute('SELECT key,value FROM metadata')}
        with (root / 'results/frames.jsonl').open(encoding='utf8') as results:
            ids = [(int(row['frame_id']), int(row['timestamp_ns'])) for row in map(json.loads, results)]
        if (metadata.get('version') != 1 or metadata.get('status') != 'completed'
                or metadata.get('count') != len(ids) or metadata.get('failed') != 0
                or metadata.get('calibration_sha256') != calibration_sha256(calibration)
                or metadata.get('result_sha256') != digest(source / 'results/frames.jsonl')):
            raise ValueError('Original depth metadata does not match the new result')
        depth_ids = set(depth_db.execute(
            'SELECT frame_id,timestamp_ns FROM frames WHERE calibration_id=? AND status="available"',
            (calibration.calibration_id,)))
        if len(depth_ids) != len(ids) or depth_ids != set(ids):
            raise ValueError('Original metric depth frame identities differ')
        query = 'SELECT frame_id,timestamp_ns,left_jpeg,right_jpeg FROM frames ORDER BY idx'
        for expected, old, new in zip(ids, original.execute(query), images.execute(query), strict=True):
            if old[:2] != expected or old != new:
                raise ValueError('Original depth was computed on different image pixels')
        depth_db.execute('UPDATE metadata SET value=? WHERE key=?',
                         (json.dumps(digest(root / 'results/frames.jsonl')), 'result_sha256'))
        depth_db.commit()
        if depth_db.execute('PRAGMA quick_check').fetchone()[0] != 'ok':
            raise ValueError('Metric depth failed SQLite integrity check')
    temporary.rename(target)
    print(f'METRIC DEPTH {len(ids)}/{len(ids)} complete: {target}', flush=True)


def build(root, source):
    cv2.setNumThreads(2)
    target = root / 'display_cache.sqlite3'
    metric_target = root / DEPTH_FILE
    if target.exists() or metric_target.exists():
        raise FileExistsError('A replay or depth cache already exists')
    calibration_path = root / 'processing_calibration.yaml'
    calibration_sha = digest(calibration_path)
    if calibration_sha != digest(source / calibration_path.name):
        raise ValueError('Image source calibration differs from the saved result')
    result_path = root / 'results/frames.jsonl'
    result_sha = digest(result_path)
    source_cache = source / target.name
    if not source_cache.is_file():
        raise FileNotFoundError(source_cache)
    if shutil.disk_usage(root).free < source_cache.stat().st_size + 512 * 1024**2:
        raise OSError('Not enough space for the standalone replay cache')
    temporary = root / f'display_cache.{uuid4().hex}.building.sqlite3'
    shutil.copyfile(source_cache, temporary)
    calibration = load_calibration(calibration_path)
    depth_cache = DepthCache()
    metric_work = root / f'depth_build_{uuid4().hex}'
    metric_work.mkdir()
    metric = DepthResultWriter(metric_work, calibration, input_pixels='existing_full_resolution_jpeg_display_cache')
    with closing(metric.connection), closing(sqlite3.connect(temporary)) as db, result_path.open(encoding='utf8') as results:
        metadata = {key: json.loads(value) for key, value in db.execute('SELECT key,value FROM metadata')}
        if metadata.get('version') != 1 or metadata.get('result_sha256') != digest(source / 'results/frames.jsonl'):
            raise ValueError('Source display cache failed its result fingerprint check')
        db.execute('CREATE TABLE joint_preview (idx INTEGER PRIMARY KEY, frame_id INTEGER, timestamp_ns INTEGER, '
                   'raw_left BLOB, raw_right BLOB, processed_left BLOB, processed_right BLOB, depth BLOB, caption TEXT, pose BLOB)')
        count = 0
        for index, line in enumerate(results):
            result = result_from_dict(json.loads(line))
            row = db.execute('SELECT frame_id,timestamp_ns,left_jpeg,right_jpeg FROM frames WHERE idx=?',(index,)).fetchone()
            if row is None or row[:2] != (result.frame_id, result.timestamp_ns):
                raise ValueError(f'Image/result identity mismatch at frame {index}')
            images = [cv2.imdecode(np.frombuffer(blob, np.uint8), cv2.IMREAD_COLOR) for blob in row[2:]]
            if any(image is None for image in images) or result.calibration_id != calibration.calibration_id:
                raise ValueError(f'Invalid image or calibration at frame {index}')
            frame = StereoFrame(result.frame_id, result.timestamp_ns, *images,
                                calibration_id=result.calibration_id, source_type=result.source_type,
                                image_geometry='raw')
            rectifier = depth_cache.acquire(calibration, frame)[0]
            rectified = rectifier.rectify(frame)
            depth, info = compute_depth(rectified, calibration, depth_cache)
            if not np.isfinite(depth).all() or (depth < 0).any():
                raise ValueError(f'Invalid metric depth at frame {index}')
            metric.connection.execute('INSERT INTO frames VALUES (?,?,?,?,?,?)',
                (frame.frame_id, frame.timestamp_ns, frame.calibration_id, 'available',
                 json.dumps(info, allow_nan=False), zlib.compress(depth.astype('<f4', copy=False).tobytes(), level=3)))
            metric.count += 1
            color, valid = colorize_depth(depth, .15, 3., False)
            use_rectified = result.diagnostics.get('output_geometry') == 'rectified'
            processed = (rectified.left_image, rectified.right_image) if use_rectified else images
            raw_blobs = [preview_jpeg(image, 480) for image in images]
            processed_blobs = [preview_jpeg(image, 800) for image in processed]
            caption = f'深度 · 预计算 · {info["reference_camera"]} · {valid.mean():.0%}'
            message = result.diagnostics.get('vio_pose') or result.diagnostics.get('live_vio_pose')
            pose = None
            if isinstance(message, dict):
                pose = (consumer_result_display_pose(result) if 'live_vio_pose' in result.diagnostics
                        else consumer_display_pose(message, expected_timestamp_ns=(
                            result.timestamp_ns if 'local_motion' in message else None)))
            db.execute('INSERT INTO joint_preview VALUES (?,?,?,?,?,?,?,?,?,?)',
                       (index, *row[:2], *raw_blobs, *processed_blobs, preview_jpeg(color, 480), caption,
                        None if pose is None else np.asarray(pose, dtype='<f8').tobytes()))
            count += 1
            if count % 100 == 0:
                db.commit()
                metric.connection.commit()
                print(f'CACHE {count}/{metadata["count"]}', flush=True)
        if count != metadata['count'] or db.execute('SELECT count(*) FROM frames').fetchone()[0] != count:
            raise ValueError('Preview and result frame counts differ')
        db.executemany('INSERT OR REPLACE INTO metadata VALUES (?,?)', [
            ('result_sha256', json.dumps(result_sha)),
            ('joint_preview', json.dumps({'version': 1, 'count': count, 'calibration_sha256': calibration_sha})),
        ])
        db.commit()
        if db.execute('PRAGMA quick_check').fetchone()[0] != 'ok':
            raise ValueError('Standalone cache failed SQLite integrity check')
        if metric.connection.execute('PRAGMA quick_check').fetchone()[0] != 'ok':
            raise ValueError('Metric depth failed SQLite integrity check')
        metric.close(result_path, completed=True)
    (metric_work / DEPTH_FILE).rename(metric_target)
    metric_work.rmdir()
    build_index(root)
    temporary.rename(target)
    print(f'CACHE {count}/{count} complete: {target}', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    parser.add_argument('--source', type=Path)
    parser.add_argument('--depth-only-from', type=Path)
    args = parser.parse_args()
    source = args.source or Path(json.loads((args.root / 'shared_image_cache.json').read_text(encoding='utf8'))['source'])
    if args.depth_only_from:
        reuse_metric_depth(args.root, args.depth_only_from, source)
        build_index(args.root)
    else:
        build(args.root, source)
