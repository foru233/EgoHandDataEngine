"""Source construction shared by desktop entry points; no widget dependencies."""
from dataclasses import dataclass
from pathlib import Path
import json

from ...core.pipeline import ROOT


@dataclass(frozen=True)
class SourceRequest:
    kind: str = 'vendor'
    path: str = ''
    loop: bool = False
    reuse_detections: bool = False
    scenario: str = 'crossing'
    fps: float = 30
    view_mode: str = 'prediction'


def open_recording_source(path, *, loop=False, reuse_detections=False):
    """Choose the original SDK stream when a recording contains raw packets."""
    from ...sources.recorded import RecordedStereoSource
    from ...sources.sdk_recorded import SdkRecordedStereoSource
    path = Path(path)
    manifest = path / 'manifest.json' if path.is_dir() else path
    info = json.loads(manifest.read_text(encoding='utf-8-sig'))
    sdk = manifest.parent / str(info.get('sdk_records') or 'raw/sdk_records.bin')
    if sdk.is_file() and sdk.stat().st_size:
        return SdkRecordedStereoSource(manifest, loop=loop)
    return RecordedStereoSource(path, loop, reuse_detections)


def create_source(request):
    if request.kind == 'synthetic':
        from ...sources.synthetic import SyntheticStereoSource
        return SyntheticStereoSource(request.scenario, frames=18000, fps=request.fps)
    if request.kind == 'recorded':
        return open_recording_source(request.path, loop=request.loop,
                                     reuse_detections=request.reuse_detections)
    if request.kind == 'image':
        from ...sources.image import SingleImageSource
        return SingleImageSource(request.path or ROOT / 'samples/public/hand.jpg')
    if request.kind == 'hot3d':
        from ...sources.hot3d import HOT3DSource
        return HOT3DSource(request.path, loop=request.loop, view_mode=request.view_mode)
    if request.kind == 'show3d':
        from ...sources.show3d import SHOW3DSource
        return SHOW3DSource(request.path or ROOT / 'samples/date/SHOW3D', loop=request.loop, view_mode=request.view_mode)
    if request.kind == 'dexycb':
        from ...sources.dexycb import DexYCBSource
        return DexYCBSource(request.path or ROOT / 'samples/date/DexYCB', loop=request.loop)
    if request.kind == 'vendor':
        from ...sources.vendor_xdsdk import XdSdkStereoSource
        return XdSdkStereoSource()
    raise ValueError('不支持的数据源：' + str(request.kind))
