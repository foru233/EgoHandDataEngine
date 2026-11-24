import json
from pathlib import Path

import pytest


@pytest.fixture
def cache(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'scripts/ace_joint'))
    import ace_prediction_cache
    return ace_prediction_cache


def test_prediction_cache_admits_only_complete_unchanged_predictions(cache, tmp_path):
    source = tmp_path / 'inference'
    store = tmp_path / 'cache'
    outputs = []
    for side, start, end in sorted(cache.expected_clips(5, 3)):
        directory = source / f'{side}_{start}'
        directory.mkdir(parents=True)
        (directory / 'raw_predictions.npz').write_bytes(f'{side}/{start}/{end}'.encode())
        outputs.append(dict(side=side, start=start, end=end, path=str(directory)))
    manifest = dict(status='completed', frames=5, start=0, inference_window=3, overlap=0, outputs=outputs)
    path = source / 'sequence_manifest.json'
    path.write_text(json.dumps({**manifest, 'outputs': outputs[:-1]}), encoding='utf8')
    key = 'a' * 64
    with pytest.raises(ValueError, match='frame ranges'):
        cache.publish(source, key, 5, 3, store)
    assert cache.lookup(key, 5, 3, store) is None
    path.write_text(json.dumps(manifest), encoding='utf8')
    cache.publish(source, key, 5, 3, store)
    result = cache.lookup(key, 5, 3, store)
    assert result == store / key
    assert cache.lookup(key, 5, 5, store) is None
    (result / 'right/000003_000005/raw_predictions.npz').write_bytes(b'changed')
    assert cache.lookup(key, 5, 3, store) is None
    cache.publish(source, key, 5, 3, store)
    assert cache.lookup(key, 5, 3, store) == result


def test_fingerprint_changes_when_calibration_runtime_or_model_changes(cache, tmp_path, monkeypatch):
    ace, mano, video = (tmp_path / path for path in ('ace', 'mano', 'video'))
    for directory in (ace/'options', ace/'checkpoints', ace/'ace_ego_hand',
                      ace/'third_party/videox_fun', ace/'ckpt/model', mano, video):
        directory.mkdir(parents=True)
    assets = {
        ace/'options/ace_ego_hand_k.yml': 'paths: {model_root: ckpt/model, videox_config: videox.yml, caption_embed: caption.pt}',
        ace/'videox.yml': 'config',
        ace/'caption.pt': 'embeddings',
        ace/'checkpoints/ace_ego_hand_k.pt': 'checkpoint',
        ace/'ace_ego_hand/inference.py': 'code',
        ace/'ckpt/model/model.pth': 'weights',
        mano/'MANO_RIGHT.pkl': 'shape',
    }
    assets.update({video/name: name for name in cache.INPUT_FILES})
    for path, value in assets.items():
        path.write_text(value, encoding='utf8')
    monkeypatch.setattr(cache, 'ACE', ace)
    monkeypatch.setattr(cache, 'MANO', mano)
    runtime = ['["runtime version 1"]']
    monkeypatch.setattr(cache.subprocess, 'check_output', lambda *args, **kwargs: runtime[0])
    first = cache.fingerprint(video, 'python', 81)
    assert cache.fingerprint(video, 'python', 81) == first
    for path in (video/'left_camera.json', ace/'ckpt/model/model.pth'):
        path.write_text('changed', encoding='utf8')
        assert cache.fingerprint(video, 'python', 81) != first
        path.write_text(assets[path], encoding='utf8')
    runtime[0] = '["runtime version 2"]'
    assert cache.fingerprint(video, 'python', 81) != first
