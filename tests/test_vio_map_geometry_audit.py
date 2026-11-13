import importlib.util
from pathlib import Path
import numpy as np

spec=importlib.util.spec_from_file_location('map_geometry_audit',Path(__file__).resolve().parents[1]/'scripts/audit_vio_map_geometry.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


def test_pairwise_metric_does_not_mislabel_one_remote_point_as_global_scale():
    x=np.random.default_rng(7).normal(size=(200,3));y=x.copy();y[-1]+=[10000,0,0]
    result=audit.pairwise_geometry(x,y)
    assert result['centroid_radius_p50_growth']>20
    assert result['same_pair_ratio_p05_p50_p95'][1]==1


def test_pairwise_metric_detects_uniform_scale_and_ignores_rigid_origin_change():
    x=np.random.default_rng(8).normal(size=(100,3))
    r=np.array([[0,-1,0],[1,0,0],[0,0,1]])
    result=audit.pairwise_geometry(x,3*x@r.T+[100,-50,2])
    np.testing.assert_allclose(result['same_pair_ratio_p05_p50_p95'],[3,3,3],atol=1e-12)
