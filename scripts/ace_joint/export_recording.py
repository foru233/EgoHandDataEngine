"""Export calibrated pinhole videos using the project's read-only SDK decoder."""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import time
import cv2
import numpy as np

from joint_work_paths import PROJECT, is_joint_workspace
sys.dont_write_bytecode=True
sys.path.insert(0,str(PROJECT/'src'))
from egohand.calibration.loader import load_calibration
from egohand.calibration.rectifier import StereoRectifier
from egohand.storage.sdk_recording import iter_paired_frames


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(8*1024*1024),b''): h.update(b)
    return h.hexdigest()


def write(path,data):
    Path(path).write_text(json.dumps(data,indent=2,ensure_ascii=True,allow_nan=False),encoding='utf8')


class VS030Pinhole:
    def __init__(self,calibration_path,width=832):
        self.calibration_path=Path(calibration_path)
        self.c=load_calibration(calibration_path)
        self.r=StereoRectifier(self.c)
        scale=min(width/self.c.image_width,1.)
        self.size=(max(32,round(self.c.image_width*scale/32)*32),max(32,round(self.c.image_height*scale/32)*32))
        self.H=np.diag([self.size[0]/self.c.image_width,self.size[1]/self.c.image_height,1.])
        self.K=[self.H@k for k in [self.c.K_left,self.c.K_right]]
        T=np.eye(4); T[:3,:3]=self.c.R_right_left; T[:3,3]=self.c.t_right_left
        self.T_common_from_view=[np.eye(4),np.linalg.inv(T)]
        self.valid=[]
        ones=np.ones((self.c.image_height,self.c.image_width),np.float32)
        for maps in self.r.undistort_maps:
            mask=cv2.remap(ones,*maps,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
            self.valid.append(cv2.warpPerspective(mask,self.H,self.size,flags=cv2.INTER_LINEAR)>.99999)

    def image(self,image,side):
        if image.shape[:2]!=(self.c.image_height,self.c.image_width): raise ValueError('Image/calibration size mismatch')
        undistorted=cv2.remap(image,*self.r.undistort_maps[side],cv2.INTER_LINEAR)
        return cv2.warpPerspective(undistorted,self.H,self.size,flags=cv2.INTER_LINEAR)

    def rectified_pixels_to_view(self,uv,side):
        M=self.H@np.linalg.inv(self.r.undistorted_to_rectified[side])
        p=np.c_[uv,np.ones(len(uv))]@M.T
        with np.errstate(divide='ignore',invalid='ignore'): return p[:,:2]/p[:,2:]

    def provenance(self,side):
        return dict(K=self.K[side].tolist(),distortion_model='pinhole',output_camera_frame=f'camera_{["left","right"][side]}_optical',
                    T_common_from_view=self.T_common_from_view[side].tolist(),image_size=list(self.size),
                    raw_to_virtual_rotation=np.eye(3).tolist(),raw_to_virtual_translation=[0.,0.,0.],
                    pixel_transform=dict(undistorted_original_K_to_view=self.H.tolist(),
                    rectified_to_view=(self.H@np.linalg.inv(self.r.undistorted_to_rectified[side])).tolist(),
                    raw_to_undistorted='StereoRectifier fisheye four-parameter R=I maps; not a homography'),
                    calibration_hash=sha(self.calibration_path),valid_fraction=float(self.valid[side].mean()))


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--recording',type=Path,required=True); ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--max-frames',type=int,default=0);ap.add_argument('--width',type=int,default=832)
    ap.add_argument('--calibration',type=Path)
    args=ap.parse_args(); root=args.recording.resolve();out=args.output.resolve()
    if not is_joint_workspace(out): raise ValueError('Output must remain in an isolated joint workspace')
    if out.exists(): raise FileExistsError(out)
    out.mkdir(parents=True)
    adapter=VS030Pinhole(args.calibration or root/'calibration.yaml',args.width)
    writers=[]; names=['left','right'];started=time.monotonic()
    for side in range(2):
        writer=cv2.VideoWriter(str(out/(names[side]+'.avi')),cv2.VideoWriter_fourcc(*'FFV1'),30.,adapter.size)
        if not writer.isOpened(): raise RuntimeError('Lossless FFV1 writer not available')
        writers.append(writer)
        k=adapter.K[side]
        write(out/(names[side]+'_camera.json'),dict(image_width=adapter.size[0],image_height=adapter.size[1],model='pinhole',
             frames=[dict(intrinsics=dict(fx=float(k[0,0]),fy=float(k[1,1]),cx=float(k[0,2]),cy=float(k[1,2])))]))
        cv2.imwrite(str(out/(names[side]+'_valid.png')),adapter.valid[side].astype(np.uint8)*255)
    count=0;previous=None;rows=[]
    try:
        with (out/'frame_map.jsonl').open('w',encoding='utf8') as f:
            for pair in iter_paired_frames(root):
                timestamp_ns=int(pair.device_ts)*1000
                if previous is not None and timestamp_ns<=previous: raise ValueError('Non-monotonic timestamps')
                previous=timestamp_ns
                for side,image in enumerate([pair.left_bgr,pair.right_bgr]): writers[side].write(adapter.image(image,side))
                row=dict(video_frame_index=count,source_frame_index=int(pair.index),sdk_sequence=int(pair.sequence),
                         timestamp_ns=timestamp_ns,device_timestamp_raw=int(pair.device_ts),
                         timestamp_semantics='device raw * 1000, inherited project convention; exposure origin unverified',
                         sdk_capture_left_ns=int(pair.capture_ts_left_ns),sdk_capture_right_ns=int(pair.capture_ts_right_ns),
                         sdk_arrival_left_ns=int(pair.arrival_ts_left_ns),sdk_arrival_right_ns=int(pair.arrival_ts_right_ns),
                         pairing='equal SDK sequence and device timestamp; project user-confirmed transport pairing',
                         hardware_sync_precision_verified=False,stereo_pair_transport_valid=True)
                f.write(json.dumps(row)+'\n');rows.append(row); count+=1
                if count%100==0: print('EXPORTED',count,round(time.monotonic()-started,2),flush=True)
                if args.max_frames and count>=args.max_frames: break
    finally:
        for w in writers:w.release()
    decoded=[]
    for name in names:
        cap=cv2.VideoCapture(str(out/(name+'.avi'))); n=0
        while cap.read()[0]: n+=1
        cap.release();decoded.append(n)
    if decoded!=[count,count]: raise ValueError(f'Export frame accounting mismatch: {decoded} != {count}')
    write(out/'camera_transforms.json',{names[i]:adapter.provenance(i) for i in range(2)})
    hashes={str(p):sha(p) for p in [root/'raw/sdk_records.bin',root/'calibration.yaml',root/'manifest.json',out/'left.avi',out/'right.avi',out/'frame_map.jsonl']}
    write(out/'input_hashes.json',hashes)
    write(out/'manifest.json',dict(status='completed',recording=str(root),frames=count,video_frame_counts=decoded,
          source_count_is_complete=not args.max_frames,coordinate_frame='camera_left_optical',unit='m',
          video_container_fps=30,temporal_optimization_clock='frame_map.jsonl timestamp_ns; never container fps',
          video_codec='FFV1 lossless',elapsed_s=time.monotonic()-started,calibration_hash=sha(adapter.calibration_path)))
    print('COMPLETE',count,'frames',round(time.monotonic()-started,2),'seconds',flush=True)


if __name__=='__main__':main()
