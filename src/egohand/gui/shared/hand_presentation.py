"""Latest immutable display inputs; never modifies recorder or localization output."""
import copy
import threading


class HandPresentationSnapshot:
    def __init__(self):
        self._lock = threading.Lock()
        self._pose = None
        self._result = None

    def publish_pose(self, message):
        if message.get('type') not in ('pose', 'vio_reset', 'status', 'error'):
            return
        pose = copy.deepcopy(message)
        with self._lock:
            self._pose = pose
            if pose.get('type') != 'pose':
                self._result = None

    def publish_pair(self, result):
        # Own a display copy. Recording/history retain their original objects.
        owned = copy.deepcopy(result)
        with self._lock:
            if self._result is None or owned.timestamp_ns > self._result.timestamp_ns:
                self._result = owned

    def read(self):
        # Published copies are never mutated. Both references are read together.
        with self._lock:
            return self._result, self._pose
