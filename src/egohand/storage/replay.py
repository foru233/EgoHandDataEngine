# 回放已有结果不重新推理；尾行截断状态与结果列表同时返回，不能冒称完整会话。

from pathlib import Path
from . import exporter
from ..sources.recorded import read_jsonl_recover
from ..core.types import result_from_dict

# 用统一兼容加载器恢复旧/新结果字段，并保留JSONL末尾是否截断的信息。
def load_recorded_results(directory):
    rows,truncated=read_jsonl_recover(Path(directory)/"results/frames.jsonl")
    return [result_from_dict(r) for r in rows],truncated
