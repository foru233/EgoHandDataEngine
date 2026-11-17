"""Single in-flight stereo image buffer; acknowledgement controls reuse."""
from multiprocessing.shared_memory import SharedMemory
import json
import numpy as np

MAX_PIXELS = 1600*1200
MAX_DESCRIPTOR = 1024
SHARED_STEREO = 2


class StereoPublisher:
    def __init__(self):
        self.memory = None
        self.shape = None

    def publish(self, left, right):
        if (not isinstance(left, np.ndarray) or not isinstance(right, np.ndarray) or
                left.dtype != np.uint8 or right.dtype != np.uint8 or left.ndim != 2 or
                left.shape != right.shape or not left.size or left.size > MAX_PIXELS):
            raise ValueError('shared stereo requires matching bounded mono8 arrays')
        if self.shape != left.shape:
            self.close()
            self.memory = SharedMemory(create=True, size=2*left.size)
            self.shape = left.shape
        view = np.ndarray((2, *left.shape), dtype=np.uint8, buffer=self.memory.buf)
        np.copyto(view[0], left); np.copyto(view[1], right)
        del view
        return json.dumps(dict(version=1, name=self.memory.name, shape=list(left.shape),
                               nbytes=2*left.size, dtype='uint8'), separators=(',',':')).encode('utf8')

    def close(self):
        if self.memory is not None:
            self.memory.close()
            self.memory.unlink()
            self.memory = None
            self.shape = None


class StereoReader:
    def __init__(self):
        self.memory = None
        self.shape = None

    def read(self, descriptor, width, height):
        if not 0 < width*height <= MAX_PIXELS or width <= 0 or height <= 0:
            raise ValueError('invalid stereo dimensions')
        if not isinstance(descriptor, (bytes, bytearray)) or len(descriptor) > MAX_DESCRIPTOR:
            raise ValueError('invalid shared stereo descriptor length')
        value = json.loads(descriptor)
        if (not isinstance(value, dict) or value.get('version') != 1 or value.get('dtype') != 'uint8' or
                value.get('shape') != [height,width] or value.get('nbytes') != 2*width*height or
                not isinstance(value.get('name'), str) or not value['name'] or len(value['name']) > 128):
            raise ValueError('shared stereo descriptor does not match frame')
        if self.memory is None or self.memory.name != value['name']:
            self.close()
            self.memory = SharedMemory(name=value['name'], create=False)
            self.shape = (height,width)
        if self.shape != (height,width) or self.memory.size < 2*width*height or self.memory.size > 2*MAX_PIXELS+65536:
            raise ValueError('shared stereo buffer size mismatch')
        view = np.ndarray((2,height,width), dtype=np.uint8, buffer=self.memory.buf)
        return view[0], view[1]

    def close(self):
        if self.memory is not None:
            self.memory.close()
            self.memory = None
            self.shape = None


CAPABILITY = 'shared_stereo_v1'


def negotiate(ready, protocol, *, requested=False):
    # Legacy peers retain their byte stream. Recovery remains a separate protocol.
    if requested and protocol == 2 and CAPABILITY in ready.get('stereo_transports', []):
        return StereoPublisher()
    return None
