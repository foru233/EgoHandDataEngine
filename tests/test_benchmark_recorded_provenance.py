import importlib.util
from pathlib import Path


spec = importlib.util.spec_from_file_location('recorded_benchmark',
    Path(__file__).resolve().parents[1] / 'scripts/benchmark_recorded_pipeline.py')
benchmark = importlib.util.module_from_spec(spec)
spec.loader.exec_module(benchmark)


def test_file_snapshot_detects_changes_and_missing_files(tmp_path):
    source = tmp_path / 'source.txt'
    missing = tmp_path / 'missing.txt'
    source.write_text('first', encoding='utf8')
    before = benchmark.snapshot_files([source, missing])
    assert before[str(source)] is not None
    assert before[str(missing)] is None
    source.write_text('second', encoding='utf8')
    assert benchmark.snapshot_files([source, missing]) != before
