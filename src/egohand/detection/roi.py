"""手动检测 ROI：统一采用完整检测输入图上的归一化 (x,y,w,h)。"""
import math
import numpy as np


def validate_roi(roi):
    if roi is None:return None
    if not isinstance(roi,(list,tuple)) or len(roi)!=4:raise ValueError('ROI 必须为 [x,y,w,h] 或 null')
    try:x,y,w,h=map(float,roi)
    except (TypeError,ValueError) as error:raise ValueError('ROI的四个值必须是有限数字') from error
    if not np.isfinite([x,y,w,h]).all() or min(x,y)<0 or max(x,y)>=1 or min(w,h)<=0 or x+w>1+1e-9 or y+h>1+1e-9:
        raise ValueError('ROI 必须位于图像内，归一化范围为 0～1，宽高必须为正')
    if (x,y,w,h)==(0.,0.,1.,1.):return None
    return x,y,min(w,1-x),min(h,1-y)


def roi_pixel_box(roi,image_shape):
    height,width=image_shape[:2];roi=validate_roi(roi)
    if roi is None:return 0,0,width,height
    x,y,w,h=roi
    x0=max(0,int(math.floor(x*width+1e-9)));y0=max(0,int(math.floor(y*height+1e-9)))
    x1=min(width,int(math.ceil((x+w)*width-1e-9)));y1=min(height,int(math.ceil((y+h)*height-1e-9)))
    if x1-x0<2 or y1-y0<2:raise ValueError('ROI 太小，至少需要 2×2 个输入像素')
    return x0,y0,x1-x0,y1-y0
