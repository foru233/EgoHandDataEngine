from dataclasses import replace
import pytest
from egohand.benchmarks.injections import measure_case,OFFSETS_MS,SPEEDS,AXES,SYNC_QUALITIES


@pytest.mark.parametrize('offset',OFFSETS_MS)
@pytest.mark.parametrize('speed',SPEEDS.values())
@pytest.mark.parametrize('axis',AXES)
def test_physical_offsets(offset,speed,axis):
    result=measure_case(offset,speed,axis)
    assert not result['hardware_sync_precision_verified']
    assert result['sync_quality']=='SOFTWARE_ESTIMATED'
    if offset==0 or speed==0:assert result['raw_geometric_error_mm']<1e-8
    if offset>5:
        assert result['runtime_reason']=='SYNC_ERROR_EXCEEDED'
        assert result['valid_rate_with_runtime_sync_gate']==0


def test_horizontal_timing_bias_can_hide_from_reprojection():
    r=measure_case(5,1.,'horizontal')
    assert r['raw_geometric_error_mm']>30
    assert r['reprojection_error_px']<1e-8 and r['epipolar_error_px']<1e-8
    assert r['valid_rate_with_runtime_sync_gate']==1
    assert r['classification']=='sync_injection_systematic_bias'


def test_real_missing_timestamps_not_promoted_to_hardware(engine,source):
    f=source.read();f=replace(f,left_timestamp_ns=None,right_timestamp_ns=None)
    r=engine.process(f)
    assert not r.diagnostics['hardware_sync_precision_verified']
    assert r.diagnostics['reason']=='MISSING_STEREO_TIMESTAMPS'


def test_metadata_alone_does_not_bypass_sync_gate(engine,source):
    f=source.read();f=replace(f,synchronized=False,metadata={**f.metadata,'sync_quality':'DATASET_DOCUMENTED'})
    r=engine.process(f)
    assert r.diagnostics['reason']=='Synchronization Unverified'
    assert not r.hands


def test_sync_grade_contract():
    assert set(SYNC_QUALITIES)=={'HARDWARE_VERIFIED','DATASET_DOCUMENTED','USER_ASSERTED','SOFTWARE_ESTIMATED','UNKNOWN'}


def test_all_geometrically_rejected_uncertainty_is_none():
    r=measure_case(5,1.,'vertical')
    assert r['valid_rate_without_sync_gate']==0
    assert r['depth_std_m'] is None
    assert r['ungated_triangulation_error_mm']>0
