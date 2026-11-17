"""Read OpenCV FileStorage JSON while preserving explicit nonfinite values."""
import json
import math
import re


TOKEN = re.compile(r'"(?:\\.|[^"\\])*"|(?P<special>-?\.(?:Nan|Inf))')


def read_graph(path):
    # Match quoted strings first: a literal ".Nan" inside a string is untouched.
    text = TOKEN.sub(lambda m: ({'.Nan': 'NaN', '.Inf': 'Infinity', '-.Inf': '-Infinity'}[m.group('special')]
        if m.group('special') else m.group(0)), path.read_text(encoding='utf8'))
    return json.loads(text)


def nonfinite_paths(value, path=''):
    if isinstance(value, dict):
        return [p for key, item in value.items() for p in nonfinite_paths(item, f'{path}.{key}' if path else key)]
    if isinstance(value, list):
        return [p for i, item in enumerate(value) for p in nonfinite_paths(item, f'{path}.{i}')]
    return [path] if isinstance(value, float) and not math.isfinite(value) else []


def tracking_depth_only(paths):
    return all(re.fullmatch(r'map_points\.\d+\.tracking_depth', p) for p in paths)
