"""Read-only SDK recording adapter for the shared hand-processing pipeline."""
import json
from pathlib import Path
from threading import Event

from .base import StereoDataSource
from ..calibration.loader import load_calibration
from ..core.types import SourceMetadata, StereoFrame
from ..storage.sdk_recording import iter_paired_frames


class SdkRecordedStereoSource(StereoDataSource):
    """Sequential HEVC replay using the same decoder/pairing as offline VIO.

    Seeking restarts decoding from the beginning to preserve HEVC references.
    Device timestamps drive replay order, without claiming exposure synchrony.
    """
    def __init__(self, path, loop=False):
        self.path = Path(path)
        self.recording_dir = self.path if self.path.is_dir() else self.path.parent
        self.loop = loop
        self.running = False
        self.paused = False
        self.calibration = None
        self.index = 0
        self.fps = 30
        self._frames = None
        self._skip = 0
        self._interrupted = Event()

    def open(self):
        manifest = self.recording_dir / 'manifest.json'
        self.info = json.loads(manifest.read_text(encoding='utf-8-sig'))
        sdk = self.recording_dir / str(self.info.get('sdk_records') or 'raw/sdk_records.bin')
        if not sdk.is_file() or not sdk.stat().st_size:
            raise ValueError('录制中没有可读取的 SDK 双目数据')
        saved_calibration = self.info.get('calibration')
        self.calibration = load_calibration(self.recording_dir / saved_calibration) if saved_calibration else None
        original = self.info.get('metadata', {})
        self.metadata = SourceMetadata(
            'vs030_recorded', real_stereo=True, synchronized=False,
            real_calibration=bool(self.calibration and not self.calibration.synthetic),
            metric_capable=False, contains_imu=bool(original.get('contains_imu')),
            timestamp_kind='sdk_device_timestamp_raw_us_to_ns',
            extra={**original.get('extra', {}), 'recording_dir': str(self.recording_dir),
                   'original_source_type': original.get('source_type'),
                   'device_timestamp_semantics': 'unverified'},
        )
        count = int(self.info.get('frame_index_count') or self.info.get('raw_count') or 0)
        if count: self.metadata.extra['frame_count'] = count
        self.index = 0
        self._skip = 0
        self._close_iterator()
        self._interrupted.clear()

    def _close_iterator(self):
        if self._frames is not None:
            close = getattr(self._frames, 'close', None)
            if close: close()
        self._frames = None

    def start(self):
        self.running = True
        self.paused = False
        self._interrupted.clear()

    def pause(self): self.paused = True
    def resume(self): self.paused = False
    def stop(self):
        self.running = False
        self._interrupted.set()

    def interrupt_read(self): self._interrupted.set()

    def close(self):
        self.stop()
        self._close_iterator()

    def get_metadata(self): return self.metadata
    def get_calibration(self): return self.calibration

    def seek(self, index):
        if index < 0: raise IndexError(index)
        self._close_iterator()
        self.index = int(index)
        self._skip = int(index)

    def frame_from_pair(self, pair):
        timestamp = int(pair.device_ts) * 1000
        return StereoFrame(
            frame_id=int(pair.index), timestamp_ns=timestamp,
            left_image=pair.left_bgr, right_image=pair.right_bgr,
            calibration_id=self.calibration.calibration_id if self.calibration else None,
            source_type='vendor_vs030', synchronized=False, image_geometry='raw',
            metadata={
                'real_stereo': True, 'synthetic': False,
                'processing_mode': 'reprocessed',
                'timestamp_kind': 'sdk_device_timestamp_raw_us_to_ns',
                'clock_domain': 'sdk_device',
                'sdk_device_timestamp_raw': int(pair.device_ts),
                'sdk_sequence': int(pair.sequence),
                'sdk_capture_ts_ns_channel_4': int(pair.capture_ts_left_ns),
                'sdk_capture_ts_ns_channel_5': int(pair.capture_ts_right_ns),
                'sdk_arrival_ts_ns_channel_4': int(pair.arrival_ts_left_ns),
                'sdk_arrival_ts_ns_channel_5': int(pair.arrival_ts_right_ns),
                'sdk_device_timestamp_semantics': 'unverified',
                'transport_pairing': 'sdk_sequence_and_equal_device_timestamp',
                'left_right_mapping_evidence': 'user_confirmed_matches_visionviewer',
                'hardware_sync_precision_verified': False,
                'recording_dir': str(self.recording_dir),
            },
        )

    def read(self, force=False):
        if not self.running or self._interrupted.is_set() or (self.paused and not force):
            return None
        if self._frames is None:
            self._frames = iter(iter_paired_frames(self.recording_dir))
        while not self._interrupted.is_set():
            try:
                pair = next(self._frames)
            except StopIteration:
                if self.loop and self.index:
                    self.seek(0)
                    self._frames = iter(iter_paired_frames(self.recording_dir))
                    continue
                return None
            if self._skip:
                self._skip -= 1
                continue
            self.index = int(pair.index) + 1
            return self.frame_from_pair(pair)
        return None
