"""Download a pinned TUM-VI room3 mirror, preserving it for holdout testing."""
import hashlib
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'samples10-15gb/VIO/TUM-VI/room3_holdout'
REV = '57fc43a8b9dcddd508fe85dc458cabc5cd75a7ff'
EXPECTED = '6ff4806f5a1f8898638ac85ea9823c0eec9bcdcf84af4a051b896096ac3a9753'
SIZE = 2971328666
URL = f'https://huggingface.co/datasets/gladiator7737/tum-vi/resolve/{REV}/room3.bag'


def main():
    DEST.mkdir(parents=True, exist_ok=True)
    if (DEST / 'room3.bag').exists():
        raise FileExistsError('verified destination already exists')
    record = {'source': URL, 'revision': REV, 'mirror_lfs_sha256': EXPECTED,
              'expected_bytes': SIZE, 'official_dataset': 'https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset',
              'official_format_reference': 'https://cvg.cit.tum.de/_media/spezial/bib/schubert2018vidataset.pdf',
              'license': 'CC BY 4.0 (TUM-VI; cite Schubert et al. 2018)',
              'purpose': 'candidate final holdout; input integrity only until test freeze',
              'status': 'downloading; not yet verified'}
    manifest = DEST / 'download_provenance.json'
    if manifest.exists():
        previous = json.loads(manifest.read_text(encoding='utf-8'))
        if previous.get('source') != URL or previous.get('mirror_lfs_sha256') != EXPECTED:
            raise ValueError('existing download has different provenance')
    else:
        manifest.write_text(json.dumps(record, indent=2), encoding='utf-8')
    h = hashlib.sha256()
    partial = DEST / 'room3.bag.partial'
    size = partial.stat().st_size if partial.exists() else 0
    if size > SIZE:
        raise ValueError('existing partial is longer than expected')
    if size:
        with partial.open('rb') as f:
            for chunk in iter(lambda: f.read(1024 * 1024), b''):
                h.update(chunk)
    request = urllib.request.Request(URL, headers={'Range': f'bytes={size}-'} if size else {})
    with urllib.request.urlopen(request, timeout=90) as response, partial.open('ab') as f:
        if size and (response.status != 206 or not response.headers.get('Content-Range', '').startswith(f'bytes {size}-')):
            raise ValueError('server did not honor resume range; refusing to append')
        while chunk := response.read(1024 * 1024):
            f.write(chunk)
            h.update(chunk)
            size += len(chunk)
    if size != SIZE or h.hexdigest() != EXPECTED:
        raise ValueError(f'download verification failed: {size} bytes, sha256 {h.hexdigest()}')
    partial.rename(DEST / 'room3.bag')
    record.update(status='verified against pinned mirror LFS sha256; official upstream full hash not verified',
                  actual_bytes=size, actual_sha256=h.hexdigest())
    manifest.write_text(json.dumps(record, indent=2), encoding='utf-8')
    print(json.dumps(record, indent=2))


if __name__ == '__main__':
    main()
