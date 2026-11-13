"""Audit diagnostic-only event-frame logs without inferring coordinate continuity."""
import argparse
import csv
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('poses',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    events={};changes=[];previous=None;frames=0;partial=0;unbound=0;unknown=0
    with args.poses.open(encoding='utf8') as source:
        for line in source:
            row=json.loads(line);frames+=1
            event_frame=row.get('map_event_frame')
            if not isinstance(event_frame,dict):unknown+=1
            elif not event_frame.get('frame_bound'):unbound+=1
            else:
                partial+=event_frame.get('coverage')=='partial'
                for event in event_frame.get('events',[]):
                    key=(event.get('map_id'),event.get('sequence'))
                    if key not in events:
                        events[key]={'first_seen_frame_ns':row.get('timestamp_ns'),**event}
            epoch=row.get('map_epoch');world=row.get('world_frame_revision')
            if previous is not None and (epoch,world)!=(previous['epoch'],previous['world']):
                changes.append({'timestamp_ns':row.get('timestamp_ns'),'before_epoch':previous['epoch'],
                    'after_epoch':epoch,'before_world':previous['world'],'after_world':world,
                    'tracking_state':row.get('tracking_state'),'raw_pose_valid':row.get('raw_pose_valid'),
                    'gate_action':'accepted' if row.get('pose_valid') else 'rejected',
                    'quality_status':row.get('quality_status'),'jump_detected':row.get('jump_detected'),
                    'coordinate_continuity':'unverified'})
            previous={'epoch':epoch,'world':world}
    def write_csv(path, rows):
        with path.open('w',encoding='utf-8-sig',newline='') as out:
            if rows:
                keys=list(dict.fromkeys(key for row in rows for key in row))
                writer=csv.DictWriter(out,fieldnames=keys);writer.writeheader();writer.writerows(rows)
    write_csv(args.output/'revision_decisions.csv',changes)
    write_csv(args.output/'event_history.csv',list(events.values()))
    summary={'frames':frames,'partial_event_frames':partial,'unbound_event_frames':unbound,
             'missing_event_frames':unknown,'unique_historical_events':len(events),
             'observed_revision_changes':len(changes),
             'note':'first_seen is not commit time; event reset generation and full coverage are unknown'}
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf8')
    print(json.dumps(summary))


if __name__=='__main__':main()
