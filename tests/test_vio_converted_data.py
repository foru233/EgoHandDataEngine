"""不依赖 GUI 的数据转换验收；真实 bag 和转换结果缺失时跳过。"""
import json
from pathlib import Path
import pytest

ROOT=Path('samples/date/VIO/converted')
@pytest.mark.parametrize('relative,frames,imu,bit_depth', [('EuRoC/MH_01_easy',3682,36820,8),('TUM-VI/room4',2228,22212,16)])
def test_converted_sensor_pairing_and_samples(relative,frames,imu,bit_depth):
    p=ROOT/relative
    if not (p/'conversion_report.json').exists():pytest.skip('真实 VIO 数据尚未转换')
    r=json.loads((p/'conversion_report.json').read_text(encoding='utf8'))
    assert r['paired_exact_timestamps']==frames and r['left_only']==r['right_only']==0
    assert r['topics']['cam0']['count']==r['topics']['cam1']['count']==frames
    assert r['topics']['imu0']['count']==imu
    assert r['image']['cam0']['bit_depth']==r['image']['cam1']['bit_depth']==bit_depth
    assert all(x['pixel_exact'] for x in json.loads((p/'sample_pixel_verification.json').read_text(encoding='utf8')))
    ticks=[int(x) for x in (p/'timestamps_inertial.txt').read_text().splitlines()]
    assert ticks[0]>=r['topics']['imu0']['first_ns'] and ticks[-1]<=r['topics']['imu0']['last_ns']
    assert len(ticks)==frames-(1 if bit_depth==16 else 0)
