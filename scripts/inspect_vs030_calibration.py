"""采集少量实际图像，统计校正后静态场景匹配；不保存图像。"""
import json
from pathlib import Path
import cv2
import numpy as np
from egohand.sources.vendor_placeholder import VendorStereoSource
from egohand.calibration.rectifier import StereoRectifier
from egohand.core.pipeline import ROOT

def main():
    source=VendorStereoSource();rows=[]
    try:
        source.open();source.start();rectifier=StereoRectifier(source.get_calibration())
        sift=cv2.SIFT_create(nfeatures=3000)
        for _ in range(8):
            frame=rectifier.rectify(source.read())
            features=[sift.detectAndCompute(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),None)
                      for im in (frame.left_image,frame.right_image)]
            (k0,d0),(k1,d1)=features
            if d0 is None or d1 is None:continue
            matches=cv2.BFMatcher().knnMatch(d0,d1,k=2)
            good=[a for pair in matches if len(pair)==2 for a,b in [pair] if a.distance<0.7*b.distance]
            if not good:continue
            a=np.array([k0[m.queryIdx].pt for m in good]);b=np.array([k1[m.trainIdx].pt for m in good])
            epi=np.abs(a[:,rectifier.epipolar_axis]-b[:,rectifier.epipolar_axis])
            signed=(a[:,rectifier.disparity_axis]-b[:,rectifier.disparity_axis])*rectifier.P2[rectifier.disparity_axis,3]
            rows.append({'matches':len(good),'epipolar_median_px':float(np.median(epi)),
                         'epipolar_p90_px':float(np.percentile(epi,90)),
                         'within_3px':int(np.sum(epi<3)),
                         'positive_depth_sign_within_3px':int(np.sum((epi<3)&(signed<0)))})
    finally:source.close()
    report={'frames':rows,'interpretation':'SIFT 特征可能包含误匹配和动态物体；此统计不能替代标定板复验，也不能证明曝光同步或三维精度。'}
    path=ROOT/'results/camera/vs030_calibration_check.json';path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
