"""两路同时观看闪光的帧级检查；不保存图像，不推断亚帧曝光时间。"""
import argparse
import json
import time
from pathlib import Path
import cv2
import numpy as np
from egohand.core.pipeline import ROOT
from egohand.sources.vendor_placeholder import VendorStereoSource

def main():
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=12.)
    p.add_argument('--output',type=Path,default=ROOT/'results/camera/vs030_flash_sync.json');args=p.parse_args()
    if not 3<=args.seconds<=60:raise ValueError('测试时间必须为 3 至 60 秒')
    source=VendorStereoSource();rows=[]
    try:
        source.open();source.start();deadline=time.monotonic()+args.seconds
        while time.monotonic()<deadline:
            frame=source.read()
            brightness=[]
            for image in (frame.left_image,frame.right_image):
                small=cv2.resize(image,(160,120),interpolation=cv2.INTER_AREA)
                gray=cv2.cvtColor(small,cv2.COLOR_BGR2GRAY)
                brightness.append(float(gray.mean()))
            rows.append((frame.timestamp_ns,*brightness))
    finally:source.close()
    values=np.asarray(rows,float)
    if len(values)<10:raise RuntimeError('采集帧不足，无法比较')
    a=values[:,1];b=values[:,2];differences=np.diff(values[:,0])/1e6
    correlations={}
    for lag in range(-3,4):
        # 正数表示右通道的图像亮度变化在后续配对组中出现。
        left,right=(a[:-lag],b[lag:]) if lag>0 else (a[-lag:],b[:lag]) if lag<0 else (a,b)
        correlations[str(lag)]=float(np.corrcoef(left,right)[0,1]) if np.std(left)>1e-8 and np.std(right)>1e-8 else None
    usable={k:v for k,v in correlations.items() if v is not None and np.isfinite(v)}
    best=int(max(usable,key=usable.get)) if usable else None
    ranges=[float(np.percentile(x,95)-np.percentile(x,5)) for x in (a,b)]
    sufficient=min(ranges)>=3 and best is not None and usable[str(best)]>=.5
    report={'frames':len(rows),'duration_s':float((values[-1,0]-values[0,0])/1e9),
      'received_pair_interval_ms':{'median':float(np.median(differences)),'p95':float(np.percentile(differences,95)),'max':float(np.max(differences))},
      'brightness_5_to_95_percentile_range':ranges,'correlation_by_pair_lag':correlations,
      'best_lag_in_received_pairs':best if sufficient else None,'visible_flash_signal_sufficient':bool(sufficient),
      'interpretation':'仅对收到的图像组比较亮度变化，帧丢失、自动曝光和场景均可影响结果；不能推断曝光相差毫秒数或证明硬件同步。'}
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
    print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
