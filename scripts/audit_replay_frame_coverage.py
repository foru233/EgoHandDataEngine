"""Account for recorded stereo packet identities absent from replay output."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def digest(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def account(index_rows, pose_rows):
    streams = {4: {}, 5: {}}
    duplicates = Counter()
    for row in index_rows:
        sink = row.get('sink')
        if sink not in streams:
            continue
        seq = int(row['sequence']); stamp = int(row['device_timestamp_raw'])
        if seq in streams[sink]:
            duplicates[str(sink)] += 1
        streams[sink][seq] = stamp
    if duplicates:
        raise ValueError(f'duplicate camera sequence in SDK index: {dict(duplicates)}')
    common = set(streams[4]) & set(streams[5])
    mismatches = sorted(s for s in common if streams[4][s] != streams[5][s])
    paired = {s: streams[4][s] for s in common if s not in mismatches}
    actual = []
    wrong_timestamp = []
    for row in pose_rows:
        replay = row.get('replay') or {}
        seq = int(replay['sequence']); actual.append(seq)
        if seq in paired and (int(replay['device_ts_raw']) != paired[seq]
                              or int(row['timestamp_ns']) != paired[seq] * 1000):
            wrong_timestamp.append(seq)
    if len(actual) != len(set(actual)) or any(a >= b for a, b in zip(actual, actual[1:])):
        raise ValueError('replay sequence is duplicated or nonmonotonic')
    missing = sorted(set(paired) - set(actual)); extra = sorted(set(actual) - set(paired))
    leading = [x for x in missing if actual and x < actual[0]]
    trailing = [x for x in missing if actual and x > actual[-1]]
    interior = [x for x in missing if actual and actual[0] < x < actual[-1]]
    return {'index_camera_packets': {str(k): len(v) for k, v in streams.items()},
            'paired_packet_identities': len(paired), 'index_unpaired_sequences': len(set(streams[4]) ^ set(streams[5])),
            'device_timestamp_mismatch_sequences': mismatches, 'replay_output_frames': len(actual),
            'missing': {'leading': leading, 'interior': interior, 'trailing': trailing,
                        'all_if_no_output': missing if not actual else []},
            'unexpected_replay_sequences': extra, 'replay_timestamp_mismatch_sequences': wrong_timestamp,
            'limitation': 'SDK packet identity is not proof of a separately decodable image or physical exposure time; decoder output may be delayed.'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('index', type=Path); p.add_argument('poses', type=Path); p.add_argument('output', type=Path)
    args = p.parse_args()
    def rows(path):
        with path.open(encoding='utf-8') as f:
            return [json.loads(line) for line in f if line.strip()]
    report = account(rows(args.index), rows(args.poses))
    report['input_sha256'] = {'index': digest(args.index), 'poses': digest(args.poses)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
