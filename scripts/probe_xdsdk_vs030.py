"""把本机相机 UDP 分组转发给 SDK 被动接收器，统计兼容性，不保存图像。"""
import collections
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import time
from egohand.sources.vendor_placeholder import VendorStereoSource,MAGIC

ROOT=Path(__file__).resolve().parents[1]

class ProbeSource(VendorStereoSource):
    def __init__(self):
        super().__init__();self.counts=collections.Counter()
    def _capture_udp(self):
        forward=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);heartbeat=time.perf_counter()
        try:
            while not self.stop_event.is_set():
                if time.perf_counter()>=heartbeat:
                    self.control.send(self._control_packet(4,struct.pack('<IQI',0,time.time_ns()//1_000_000,0)))
                    heartbeat=time.perf_counter()+1
                try:payload,peer=self.data_socket.recvfrom(65535)
                except socket.timeout:continue
                if peer[0]!=self.camera_ip or peer[1]!=45679:continue
                if payload.startswith(MAGIC) and len(payload)>=8:
                    kind=struct.unpack_from('<I',payload,4)[0]
                    self.counts[(kind,len(payload))]+=1
                forward.sendto(payload,('127.0.0.1',45685))
        finally:forward.close();self.running=False

def main():
    exe=ROOT/'build/xdsdk_probe/probe_xdsdk_stream.exe'
    env=os.environ.copy();env['PATH']=str(ROOT/'SDK/delivery/xdclient')+os.pathsep+env.get('PATH','')
    p=subprocess.Popen([str(exe)],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,env=env,
        creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    src=ProbeSource();message='';error=None
    try:
        message=p.stdout.readline().strip()
        if message!='SDK_LISTENING':raise RuntimeError('SDK 接收器未启动：'+message)
        src.open();src.start()
        # 先停止转发，给 SDK 接收线程时间退出，避免持续输入影响 stop。
        time.sleep(5);src.stop()
        tail,stderr=p.communicate(timeout=20);message+='\n'+tail
    except Exception as exc:error=f'{type(exc).__name__}: {exc}'
    finally:
        src.stop()
        if p.poll() is None:p.terminate();p.wait(timeout=5)
    report={'sdk':'xdsdk 0.2.0 VC020 V2','device':'VS030','sdk_output':message,
        'camera_header_counts':{f'kind={k[0]}, bytes={k[1]}':v for k,v in src.counts.items()},
        'error':error,'stored_images':False,
        'note':'仅测试给定 SDK 对当前固件包的被动解析，不代表其他 SDK 版本或固件能力。'}
    path=ROOT/'results/camera/vs030_xdsdk_compatibility.json';path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)

if __name__=='__main__':main()
