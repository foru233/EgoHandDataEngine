"""Read-only packet-origin PTS audit, independent of SDK physical timestamp semantics."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import av

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from egohand.storage.sdk_recording import iter_sdk_records


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('recording',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--packets-per-channel',type=int,default=2000)
    a=p.parse_args();decoders={s:av.CodecContext.create('hevc','r') for s in (4,5)}
    counts=Counter();delays={s:Counter() for s in decoders};unknown=Counter();errors=Counter();output=Counter()
    for record in iter_sdk_records(a.recording/'raw/sdk_records.bin'):
        s=record.sink
        if s not in decoders or counts[s]>=a.packets_per_channel:continue
        ordinal=counts[s];counts[s]+=1
        packet=av.Packet(record.payload);packet.pts=ordinal;packet.dts=ordinal
        try:frames=decoders[s].decode(packet)
        except av.error.FFmpegError:errors[s]+=1;continue
        for frame in frames:
            output[s]+=1
            if frame.pts is None:unknown[s]+=1
            else:delays[s][ordinal-frame.pts]+=1
        if all(counts[k]>=a.packets_per_channel for k in decoders):break
    report={'recording':str(a.recording.resolve()),'submitted_packets':dict(counts),
            'decoded_frames':dict(output),'errors':dict(errors),'unknown_pts':dict(unknown),
            'output_packet_ordinal_minus_origin_pts':{s:dict(v) for s,v in delays.items()},
            'limitations':['Synthetic packet-origin PTS tracks decoder buffering, not sensor exposure truth.',
                           'No decoder flush; final buffered frames are not attributed to a later packet.']}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2),encoding='utf8');print(json.dumps(report))


if __name__=='__main__':main()
