from scripts.build_vio_cross_dataset_report import compact_metrics, public_catalog


def test_compact_metrics_preserves_only_comparable_fields():
    assert compact_metrics({'count': 4, 'rmse': 0.2, 'debug': 'ignored'}) == {
        'count': 4, 'rmse': 0.2,
    }


def test_public_catalog_keeps_unavailable_datasets_explicit():
    catalog = {item['dataset']: item for item in public_catalog()}
    assert catalog['KITTI Raw/Odometry']['split'] == 'unavailable'
    assert catalog['Hilti SLAM Challenge']['gt'] is None
    assert any(item['sequence'] == 'room3_holdout' and item['split'] == 'final_test_candidate'
               for item in catalog.values())
