import io
import queue
import threading
from types import SimpleNamespace

import pytest

from egohand.sources.vendor_xdsdk import (
    read_exact, RECORD, RECORD_VERSION, MAGIC, XdSdkStereoSource,
)


class Fragmented(io.BytesIO):
    def read(self, n=-1):
        return super().read(min(n, 3))


def test_worker_ipc_v2_framing_survives_short_pipe_reads():
    header = RECORD.pack(MAGIC, RECORD_VERSION, RECORD.size, 7, 5, 123, 456, 789, 1011)
    stream = Fragmented(header + b"hello")
    assert RECORD.unpack(read_exact(stream, RECORD.size)) == (
        MAGIC, RECORD_VERSION, RECORD.size, 7, 5, 123, 456, 789, 1011,
    )
    assert read_exact(stream, 5) == b"hello"
    with pytest.raises(EOFError):
        read_exact(stream, 1)


def test_preview_queue_overflow_drops_only_preview_packets_and_resets_decoder():
    source = XdSdkStereoSource()
    source.decode_queue = queue.Queue(maxsize=2)
    packet = (4, 1, 2, 3, 4, 5, 6, b"encoded")
    source._enqueue_preview(packet)
    source._enqueue_preview(packet)
    source._enqueue_preview(packet)

    assert source.decode_queue.qsize() == 1
    generation, queued = source.decode_queue.get_nowait()
    assert generation == 1
    assert queued == packet
    stats = source.get_capture_stats()
    assert stats["preview_decode_queue_resets"] == 1
    assert stats["preview_decode_packets_dropped"] == 2


def test_source_metadata_declares_raw_imu_and_decoupled_preview():
    source = XdSdkStereoSource()
    metadata = source.get_metadata()
    assert metadata.contains_imu
    assert metadata.extra["channels"] == [4, 5, 7]
    assert metadata.extra["raw_imu_payload"]
    assert metadata.extra["preview_and_raw_recording_decoupled"]


def test_first_preview_distinguishes_no_data_from_decode_failure():
    source = XdSdkStereoSource()
    cancelled = threading.Event()
    with pytest.raises(TimeoutError, match="未收到相机数据"):
        source.wait_for_preview(cancelled, timeout=0)
    source.capture_stats["sdk_packets_received"] = 1
    with pytest.raises(TimeoutError, match="未获得首对双目画面"):
        source.wait_for_preview(cancelled, timeout=0)
    source.preview_ready.set()
    assert source.wait_for_preview(cancelled, timeout=0)
    cancelled.set()
    assert not source.wait_for_preview(cancelled, timeout=0)


def test_preview_pairs_decoder_output_by_its_original_packet(monkeypatch):
    import av
    import numpy as np

    class Decoder:
        def __init__(self, delayed):
            self.delayed = delayed
            self.previous = None

        def decode(self, packet):
            frame = SimpleNamespace(pts=packet.pts, to_ndarray=lambda format: np.zeros((3, 4, 3), np.uint8))
            if not self.delayed:
                return [frame]
            previous, self.previous = self.previous, frame
            return [previous] if previous is not None else []

    decoders = iter((Decoder(False), Decoder(True)))
    monkeypatch.setattr(av, 'CodecContext', SimpleNamespace(create=lambda *_: next(decoders)))
    source = XdSdkStereoSource()
    for sequence in (1, 2):
        for sink in (4, 5):
            source._enqueue_preview((sink, sequence, sequence * 100, 0, 0, 0, 0, b'encoded'))
    source.reader_done.set()
    source._decode_preview()

    assert source.preview_ready.is_set()
    frame = source.read()
    assert frame.metadata['sdk_sequence'] == 1
    assert frame.metadata['sdk_device_timestamp_raw'] == 100


@pytest.mark.parametrize('timeout_during_discovery', [False, True])
def test_gui_retries_startup_without_waiting_for_vio(qtbot, monkeypatch, timeout_during_discovery):
    from egohand.gui.workers import PipelineWorker
    from egohand.vio import live_runtime

    source = XdSdkStereoSource()
    events = []
    def open_source():
        events.append('open')
        if timeout_during_discovery and events.count('open') == 1:
            raise TimeoutError('camera not found')

    monkeypatch.setattr(source, 'open', open_source)
    monkeypatch.setattr(source, 'start', lambda: events.append('start'))
    monkeypatch.setattr(source, 'read', lambda: None)

    def wait_for_preview(cancelled):
        if not timeout_during_discovery and events.count('start') == 1:
            raise TimeoutError('no packets')
        return True

    monkeypatch.setattr(source, 'wait_for_preview', wait_for_preview)

    class SlowVio:
        def __init__(self, source, on_status):
            pass

        def start(self, *, defer_imu):
            assert defer_imu
            events.append('vio launched')

        def close(self):
            events.append('vio closed')

    monkeypatch.setattr(live_runtime, 'LiveVio', SlowVio)
    worker = PipelineWorker(source, live_vio=True)
    with qtbot.waitSignal(worker.finished, timeout=3000):
        with qtbot.waitSignal(worker.ready, timeout=2000):
            worker.start()
    assert worker.last_error is None
    first_attempt = ['open'] if timeout_during_discovery else ['open', 'start']
    assert events == first_attempt + ['open', 'start', 'vio launched', 'vio closed']


def test_raw_reader_emits_all_sdk_and_host_clock_fields_to_recorder_callback():
    source = XdSdkStereoSource()
    payload = b"raw camera packet"
    header = RECORD.pack(MAGIC, RECORD_VERSION, RECORD.size, 4, len(payload), 77, 88, 99, 111)
    seen = []
    source.sdk_record_callback = seen.append
    source.process = SimpleNamespace(
        stdout=io.BytesIO(header + payload),
        stderr=io.BytesIO(),
        poll=lambda: 0,
    )
    source.stop_event.clear()
    source._receive_raw()

    assert len(seen) == 1
    packet = seen[0]
    assert packet[:5] == (4, 77, 88, 99, 111)
    assert packet[5] > 0
    assert packet[6] > 0
    assert packet[7] == payload
    assert source.reader_done.is_set()
