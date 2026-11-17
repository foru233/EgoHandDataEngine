"""Create a standalone SDK clip, preserving HEVC decoder preroll and IMU packets."""
import json
import hashlib
import shutil
import tempfile
from pathlib import Path

from .sdk_recording import iter_paired_frames,iter_sdk_records,iter_imu_rows,CAMERA_SINKS,IMU_SINK
from .recorder import Recorder
from ..calibration.loader import load_calibration
from ..core.types import SourceMetadata


def save_sdk_clip_time_range(source, destination, start_ts, end_ts):
    """Resolve the requested device-time range to actual decoded pairs before saving."""
    if start_ts>end_ts:raise ValueError('截取范围无效')
    start=end=None
    for pair in iter_paired_frames(source):
        if pair.device_ts < start_ts:continue
        if pair.device_ts > end_ts:break
        if start is None:start=pair.index
        end=pair.index
    if start is None:raise ValueError('所选范围内没有可解码的双目帧')
    return save_sdk_clip(source,destination,start,end)


def save_sdk_clip(source, destination, start, end):
    """Save inclusive paired-frame indices into a new self-contained directory.

    Original camera packets preceding the start are decoder preroll, not exposed
    as replay frames. IMU packets include a 2 s lead and tail for replay.
    """
    source=Path(source).resolve();destination=Path(destination).resolve()
    if destination.exists():raise FileExistsError(str(destination))
    if source==destination or source in destination.parents:raise ValueError('截取目录不能位于原录制内部')
    if not (0 <= start <= end):raise ValueError('截取范围无效')
    info=json.loads((source/'manifest.json').read_text(encoding='utf-8-sig'))
    if info.get('sdk_clip'):raise ValueError('暂不支持再次截取已截取的录制')
    if not info.get('sdk_records'):raise ValueError('当前仅支持 VS030 SDK 录制')
    timestamps=[];boundary_hashes={};frame_rows=[]
    for pair in iter_paired_frames(source,limit=end+1):
        if pair.index>=start:
            timestamps.append(int(pair.device_ts))
            frame_rows.append({'frame_id':pair.index-start,'timestamp_ns':int(pair.device_ts)*1000,
                'source_type':'vendor_vs030','calibration_id':None,'image_geometry':'raw',
                'left_shape':list(pair.left_bgr.shape),'right_shape':list(pair.right_bgr.shape),
                'metadata':{'sdk_sequence':int(pair.sequence),'sdk_device_timestamp_raw':int(pair.device_ts),
                    'sdk_capture_ts_ns_channel_4':int(pair.capture_ts_left_ns),
                    'sdk_capture_ts_ns_channel_5':int(pair.capture_ts_right_ns),
                    'sdk_arrival_ts_ns_channel_4':int(pair.arrival_ts_left_ns),
                    'sdk_arrival_ts_ns_channel_5':int(pair.arrival_ts_right_ns),
                    'sdk_imu_packet_count':0,'sdk_imu_frame_timing':[]}})
            if pair.index in (start,end):
                boundary_hashes[pair.index-start]=tuple(hashlib.sha256(image.tobytes()).hexdigest() for image in (pair.left_bgr,pair.right_bgr))
    if len(timestamps)!=end-start+1:raise ValueError('所选帧超出可解码范围')
    if any(b<=a for a,b in zip(timestamps,timestamps[1:])):raise ValueError('截取范围时间戳不递增')
    first,last=timestamps[0],timestamps[-1]
    destination.parent.mkdir(parents=True,exist_ok=True)
    meta={k:v for k,v in info.get('metadata',{}).items() if k in SourceMetadata.__dataclass_fields__}
    meta['source_type']='vendor_vs030'
    calibration=load_calibration(source/info['calibration']) if info.get('calibration') else None
    with tempfile.TemporaryDirectory(prefix='.clip_',dir=destination.parent) as staging:
        working=Path(staging)/'recording'
        recorder=Recorder(working,SourceMetadata(**meta),calibration,info.get('algorithm_config'),info.get('model'))
        try:
            recorder.manifest['sdk_clip']={'start_pair_index':start,'end_pair_index':end,
                'first_device_timestamp_raw':first,'last_device_timestamp_raw':last,
                'imu_preroll_us':2_000_000,'imu_tail_us':2_000_000,'source':str(source),
                'note':'camera packets before selected start retained for HEVC decode only'}
            imu_expected=[]
            for record in iter_sdk_records(source/info['sdk_records']):
                keep=(record.sink in CAMERA_SINKS and record.device_ts<=last) or (
                    record.sink==IMU_SINK and first-2_000_000<=record.device_ts<=last+2_000_000)
                if keep:
                    if record.sink==IMU_SINK:imu_expected.append((record.device_ts,hashlib.sha256(record.payload).digest()))
                    recorder.write_sdk((record.sink,record.sequence,record.device_ts,record.capture_ts_ns,
                        record.arrival_ts_ns,record.host_monotonic_ns,record.host_unix_ns,record.payload))
            if not imu_expected or imu_expected[0][0]>first or imu_expected[-1][0]<last:
                raise ValueError('IMU 数据未覆盖所选首尾帧，无法保证截取同步')
            for row in frame_rows:
                row['calibration_id']=calibration.calibration_id if calibration else None
                recorder.frame_index.write(json.dumps(row,ensure_ascii=False)+'\n')
            recorder.manifest['frame_index_count']=len(frame_rows)
            for name in ('vio_quality.yaml','vio_settings.yaml','vio_timing.yaml','kalibr_camchain.yaml'):
                if (source/name).is_file():shutil.copy2(source/name,working/name)
            recorder.close()
            check=recorder.manifest.get('replay_check')
            if isinstance(check,dict):
                check['recording']=str(destination)
                (working/'replay_check.json').write_text(json.dumps(check,ensure_ascii=False,indent=2),encoding='utf8')
                recorder._manifest()
            actual=[]
            for pair in iter_paired_frames(working):
                actual.append(pair.device_ts)
                if pair.index in boundary_hashes:
                    images=tuple(hashlib.sha256(image.tobytes()).hexdigest() for image in (pair.left_bgr,pair.right_bgr))
                    if images!=boundary_hashes[pair.index]:raise ValueError('截取边界图像与原录制不一致')
            if actual!=timestamps:raise ValueError('截取结果双目帧与原录制不一致')
            imu_actual=[(r.device_ts,hashlib.sha256(r.payload).digest()) for r in iter_sdk_records(working/'raw/imu_records.bin')]
            if imu_actual!=imu_expected:raise ValueError('截取结果 IMU 包与原录制不一致')
            rows=iter_imu_rows(working)
            try:first_row=next(rows)
            except StopIteration:raise ValueError('截取结果缺少 IMU 样本') from None
            last_row=first_row
            for last_row in rows:pass
            if first_row[1]>first*1000 or last_row[1]<last*1000:
                raise ValueError('IMU 样本时间未覆盖所选首尾帧，无法保证截取同步')
            working.rename(destination)
        finally:
            if not recorder.closed:recorder.close(incomplete_reason='clip_failed')
    return destination
