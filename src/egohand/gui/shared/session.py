"""Worker ownership and signal isolation independent of either GUI layout."""
from PySide6.QtCore import QObject, Signal, Slot
from ..workers import PipelineWorker


def create_worker(source, *, parent=None, callbacks=None, worker_class=None, **options):
    """Construct and bind before starting; preserves direct Qt sender identity."""
    worker = (worker_class or PipelineWorker)(source, parent=parent, **options)
    for name, callback in (callbacks or {}).items():
        getattr(worker, name).connect(callback)
    return worker


def stop_worker(worker, timeout_ms=15000):
    if worker is None:
        return True
    worker.stop()
    return worker.wait(timeout_ms)


class SessionController(QObject):
    ready = Signal(dict)
    error = Signal(str)
    startup_status = Signal(str)
    vio_status = Signal(dict)
    recording_state_changed = Signal(bool, bool, str)
    finished = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.worker = None
        self.history = []

    def start(self, source, **options):
        if not self.stop():
            raise RuntimeError('后台数据源尚未停止，请稍后重试')
        self.history = []
        self.worker = create_worker(source, parent=self, callbacks={
            'ready': self._ready, 'error': self._error,
            'startup_status': self._startup_status, 'vio_status': self._vio_status,
            'recording_state_changed': self._recording_state,
            'finished': self._finished,
        }, **options)
        self.worker.start()
        return self.worker

    def _current(self):
        return self.sender() is self.worker

    @Slot(dict)
    def _ready(self, value):
        if self._current(): self.ready.emit(value)

    @Slot(str)
    def _error(self, value):
        if self._current(): self.error.emit(value)

    @Slot(str)
    def _startup_status(self, value):
        if self._current(): self.startup_status.emit(value)

    @Slot(dict)
    def _vio_status(self, value):
        if self._current(): self.vio_status.emit(value)

    @Slot(bool, bool, str)
    def _recording_state(self, active, paused, message):
        if self._current(): self.recording_state_changed.emit(active, paused, message)

    @Slot()
    def _finished(self):
        if self._current():
            self.history = list(self.worker.history)
            self.finished.emit()

    def take_latest(self):
        if self.worker is not None and self.worker.output:
            return self.worker.output.pop()
        return None

    def request_stop(self):
        if self.worker is not None: self.worker.stop()

    def stop(self, timeout_ms=15000):
        worker = self.worker
        if not stop_worker(worker, timeout_ms):
            return False
        if worker is not None:
            self.history = list(worker.history)
            self.worker = None
            worker.deleteLater()
        return True

    def pause(self, value=True):
        if self.worker is not None: self.worker.pause(value)

    def seek(self, index):
        if self.worker is not None: self.worker.seek(index)

    def command(self, name, value=None):
        if self.worker is not None: self.worker.command(name, value)
