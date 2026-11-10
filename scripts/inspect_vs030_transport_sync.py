"""只检查 VS030 发包顺序和节奏；不会将发包时间当作曝光时间。"""
import argparse
import json
import statistics
import struct
import subprocess
from pathlib import Path
import numpy as np
from egohand.sources.vendor_placeholder import ADB,CAMERA_SERIAL,CAPTURE_INTERFACE,CAMERA_UDP_PORT,HOST_UDP_PORT,MAGIC
from egohand.core.pipeline import ROOT

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--packets',type=int,default=2000)
    parser.add_argument('--output',type=Path,default=ROOT/'results/camera/vs030_transport_sync.json');args=parser.parse_args()
    if not 10<=args.packets<=100000:raise ValueError('packets 必须在 10..100000 之间')
    # 快照长度只覆盖厂商头部，不保存或输出用户画面。
    command=f'timeout 20 tcpdump -i {CAPTURE_INTERFACE} -U -s 90 -c {args.packets} -w - udp src port {CAMERA_UDP_PORT} and dst port {HOST_UDP_PORT} 2>/dev/null'
    process=subprocess.run([str(ADB),'-s',CAMERA_SERIAL,'exec-out',command],capture_output=True,timeout=25,check=True,
        creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    data=process.stdout
    if len(data)<24 or data[:4]!=b'\xd4\xc3\xb2\xa1':raise RuntimeError('未获得有效的小端 pcap 数据')
    pos=24;groups=[];active=None
    while pos+16<=len(data):
        sec,usec,length,_=struct.unpack_from('<IIII',data,pos);pos+=16
        if pos+length>len(data):raise ValueError('pcap 尾部不完整')
        packet=data[pos:pos+length];pos+=length
        if len(packet)<42:continue
        ip=packet[14:];ihl=(ip[0]&15)*4
        if len(ip)<ihl+8:continue
        payload=ip[ihl+8:]
        if len(payload)<8 or not payload.startswith(MAGIC):continue
        kind=struct.unpack_from('<I',payload,4)[0];timestamp=sec+usec/1e6
        if kind==1 and len(payload)>=48:
            if active is not None:groups.append(active)
            active={'start':timestamp,'channels':[],'closed':False}
        elif kind==4 and len(payload)>=16 and active is not None:
            active['channels'].append({'id':struct.unpack_from('<I',payload,8)[0],'time':timestamp})
        elif kind==2 and active is not None:
            active['closed']=True;groups.append(active);active=None
    if active is not None:groups.append(active)
    complete=[g for g in groups if g['closed'] and [c['id'] for c in g['channels']]==[4,5]]
    intervals=np.diff([g['start'] for g in complete])*1000
    channel_delays=np.array([(g['channels'][1]['time']-g['channels'][0]['time'])*1000 for g in complete])
    def stats(a):
        return {'count':int(len(a)),'mean':float(np.mean(a)),'median':float(np.median(a)),
                'p95':float(np.percentile(a,95)),'min':float(np.min(a)),'max':float(np.max(a))} if len(a) else None
    report={'device':'VS030','packet_count_requested':args.packets,'bundle_count_observed':len(groups),
        'complete_4_then_5_bundle_count':len(complete),'other_or_incomplete_bundle_count':len(groups)-len(complete),
        'bundle_start_interval_ms':stats(intervals),'channel_header_send_interval_ms':stats(channel_delays),
        'evidence':'same UDP transport bundle only; device packet-capture timestamps',
        'hardware_exposure_timestamps_observed':False,'hardware_exposure_sync_verified':False,
        'note':'发送间隔受打包、编码和传输排序影响，不代表左右相机曝光时间差；不能据此宣称硬件同步。'}
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
