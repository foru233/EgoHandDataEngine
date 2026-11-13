import cv2
import json
import numpy as np
import pytest

from egohand.vio.session_phase import verify_session_phase, prepare_recording_phase
from egohand.vio.imu_lattice import PacketLatticeImuBuffer, packet_lattice_phase_ns
from egohand.vio.config_snapshot import resolve_live_config, resolve_recording_config
from dataclasses import replace


def test_gray_preparation_skips_identity_resize_without_changing_pixels(monkeypatch):
    from egohand.vio.session_phase import _phase_gray_pair
    rng = np.random.default_rng(19)
    left = rng.integers(0, 256, (117, 151, 3), dtype=np.uint8)
    right = rng.integers(0, 256, left.shape, dtype=np.uint8)
    old = [cv2.resize(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), None,
                      fx=1., fy=1., interpolation=cv2.INTER_AREA) for im in (left, right)]
    monkeypatch.setattr(cv2, 'resize', lambda *a, **k: pytest.fail('identity resize called'))
    for actual, expected in zip(_phase_gray_pair(left, right, 1.), old):
        np.testing.assert_array_equal(actual, expected)
        assert actual.flags.c_contiguous


def test_gray_preparation_retains_nonidentity_resizing():
    from egohand.vio.session_phase import _phase_gray_pair
    rng = np.random.default_rng(19)
    images = [rng.integers(0, 256, (117, 151, 3), dtype=np.uint8) for _ in range(2)]
    expected = [cv2.resize(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), None,
                           fx=.5, fy=.5, interpolation=cv2.INTER_AREA) for im in images]
    for actual, reference in zip(_phase_gray_pair(*images, .5), expected):
        np.testing.assert_array_equal(actual, reference)


def synthetic_reference(second_phase=37, constant=False):
    anchor=1_794_300_000_000_000_000
    seconds=np.arange(0,8,1/480)
    def signal(t):
        return np.stack((.7*np.sin(3*t),.4*np.cos(5*t),.3*np.sin(7*t)),axis=-1)
    gyro=signal(seconds)
    camera=cv2.Rodrigues(np.array([.3,-.2,.1]))[0]
    samples=[]
    for t0 in np.arange(.3,7.3,1/15):
        t1=t0+1/15
        phase=(37 if t0<3.8 else second_phase)/1000
        prediction=np.eye(3)
        for t in np.linspace(t0,t1,32,endpoint=False)+1/960:
            w=np.array([.2,0,0]) if constant else signal(t-phase)
            prediction=prediction@cv2.Rodrigues(w/480)[0]
        samples.append((anchor+round(t0*1e9),anchor+round(t1*1e9),np.eye(3),
                        camera@prediction@camera.T))
    if constant:gyro[:]=[.2,0,0]
    return samples,anchor+np.rint(seconds*1e9).astype(np.int64),gyro,camera


def test_known_phase_with_independent_holdout_and_camera_axes():
    result=verify_session_phase(*synthetic_reference())
    assert result['verified'],result
    assert abs(result['phase_ms']-37)<=1
    assert result['holdout_rms_deg']<.002


def test_constant_rotation_does_not_certify_timing():
    result=verify_session_phase(*synthetic_reference(constant=True))
    assert not result['verified']
    assert result['reason']=='insufficient_changing_rotation'


def test_changing_offset_between_halves_fails_closed():
    result=verify_session_phase(*synthetic_reference(second_phase=97))
    assert not result['verified']
    assert 'train_holdout_disagree' in result['reason']


def test_rephase_preserves_values_and_rejects_consumed_history():
    from collections import deque
    buffer=PacketLatticeImuBuffer(sample_rate_hz=480,camera_to_imu_drift_ppm=0,phase_ns=97_000_000)
    buffer.rows=deque([(100_000_000,1,2,3,4,5,6)])
    buffer._last_stamp=100_000_000;buffer.covered_until_ns=101_000_000
    buffer.rephase_before_tracking(34_000_000)
    assert list(buffer.rows)==[(37_000_000,1,2,3,4,5,6)]
    assert buffer.covered_until_ns==38_000_000
    buffer.consumed=1
    buffer.reset()
    with pytest.raises(RuntimeError,match='consumed'):
        buffer.rephase_before_tracking(32_000_000)


def test_auto_recording_requires_verified_matching_sidecar(tmp_path):
    live=resolve_live_config()
    live=replace(live,timing={**live.timing,'lattice_phase_mode':'session_auto'})
    live.write(tmp_path)
    recorded=resolve_recording_config(tmp_path)
    with pytest.raises(ValueError,match='尚未保存'):
        packet_lattice_phase_ns(recorded.timing)
    model={'schema_version':1,'config_hashes':recorded.describe()['hashes'],
           'verification':{'verified':True,'phase_ms':34}}
    path=tmp_path/'vio_session_phase.json'
    path.write_text(json.dumps({'schema_version':1,'models':[model]}),encoding='utf8')
    recorded=resolve_recording_config(tmp_path)
    assert packet_lattice_phase_ns(recorded.timing)==34_000_000
    model['config_hashes']['calibration']='wrong'
    path.write_text(json.dumps({'schema_version':1,'models':[model]}),encoding='utf8')
    with pytest.raises(ValueError,match='不匹配'):
        resolve_recording_config(tmp_path)


def test_async_recorder_persists_phase_and_refuses_mixed_session_replay(tmp_path):
    from egohand.storage.recorder import AsyncRecorder
    from egohand.core.types import SourceMetadata
    path=tmp_path/'session'
    recorder=AsyncRecorder(path,SourceMetadata('vendor_vs030'),max_queue=8)
    config=resolve_recording_config(path)
    model={'schema_version':1,'config_hashes':config.describe()['hashes'],
           'verification':{'verified':True,'phase_ms':34}}
    assert recorder.submit('session_phase',model)
    assert recorder.submit('session_phase',model)  # Cached notification must not duplicate a session.
    recorder.close()
    assert not recorder.error
    data=json.loads((path/'vio_session_phase.json').read_text(encoding='utf8'))
    assert len(data['models'])==1
    assert packet_lattice_phase_ns(resolve_recording_config(path).timing)==34_000_000
    data['models'].append({**model,'verified_at_device_ns':123})
    (path/'vio_session_phase.json').write_text(json.dumps(data),encoding='utf8')
    with pytest.raises(ValueError,match='多个时序会话'):
        resolve_recording_config(path)


@pytest.mark.parametrize('content',['{','[]','{"schema_version":1,"models":42}',
                                  '{"schema_version":1,"models":[null]}'])
def test_unreadable_phase_model_is_a_clear_failure(tmp_path,content):
    config=resolve_live_config()
    replace(config,timing={**config.timing,'lattice_phase_mode':'session_auto'}).write(tmp_path)
    (tmp_path/'vio_session_phase.json').write_text(content,encoding='utf8')
    with pytest.raises(ValueError,match='时序'):
        resolve_recording_config(tmp_path)


def test_cancelled_offline_preparation_never_opens_recording(tmp_path):
    from threading import Event
    event=Event();event.set()
    config=resolve_live_config()
    config=replace(config,timing={**config.timing,'lattice_phase_mode':'session_auto'})
    with pytest.raises(InterruptedError,match='已取消'):
        prepare_recording_phase(tmp_path/'does_not_exist',config,tmp_path/'output',cancelled=event)
    assert not (tmp_path/'output').exists()
