"""生成 VS030 已校正双目 ORB 配置；只准备几何，不启动相机或 VIO。"""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
from egohand.core.pipeline import ROOT
from egohand.vio.vs030_geometry import prepare_vs030_vio_geometry

def main():
    source=ROOT/'标定结果/1/data3-camchain-imucam.yaml'
    report=ROOT/'标定结果/1/data3-results-imucam.txt'
    template=ROOT/'samples/date/VIO/converted/EuRoC/MH_01_easy/calibration/EuRoC.yaml'
    if not all(p.is_file() for p in (source,report,template)):raise FileNotFoundError('缺少 VS030 或参考配置')
    geo=prepare_vs030_vio_geometry(source);r=geo.rectifier;data=yaml.safe_load(source.read_text(encoding='utf8'))
    dest=ROOT/'configs/vio/vs030_rectified_preliminary.yaml'
    ref=cv2.FileStorage(str(template),cv2.FILE_STORAGE_READ)
    if not ref.isOpened():raise RuntimeError('无法读取 ORB 参考配置')
    values={}
    try:
        values['File.version']='"1.0"';values['Camera.type']='"Rectified"'
        for name,value in zip(('fx','fy','cx','cy'),(r.P1[0,0],r.P1[1,1],r.P1[0,2],r.P1[1,2])):
            values['Camera1.'+name]=repr(float(value))
        values.update({'Camera.width':str(r.c.image_width),'Camera.height':str(r.c.image_height),
            'Camera.fps':'30','Camera.RGB':'0','Stereo.b':repr(float(-r.P2[0,3]/r.P1[0,0])),
            'Stereo.ThDepth':'60.0'})
        # 下列数值直接对应同目录 Kalibr results 的 IMU0 段，后续需重新核查硬件和噪声定义。
        for name,value in (('IMU.NoiseGyro',8.5e-5),('IMU.NoiseAcc',0.0012),
                           ('IMU.GyroWalk',4e-5),('IMU.AccWalk',0.0001),('IMU.Frequency',459.0)):
            values[name]=repr(float(value))
        for name in ('ORBextractor.nFeatures','ORBextractor.scaleFactor','ORBextractor.nLevels',
                     'ORBextractor.iniThFAST','ORBextractor.minThFAST','Viewer.KeyFrameSize',
                     'Viewer.KeyFrameLineWidth','Viewer.GraphLineWidth','Viewer.PointSize',
                     'Viewer.CameraSize','Viewer.CameraLineWidth','Viewer.ViewpointX',
                     'Viewer.ViewpointY','Viewer.ViewpointZ','Viewer.ViewpointF'):
            node=ref.getNode(name)
            if node.empty():raise ValueError('参考配置缺少 '+name)
            value=node.real()
            values[name]=str(int(value)) if name in ('ORBextractor.nFeatures','ORBextractor.nLevels','ORBextractor.iniThFAST','ORBextractor.minThFAST') else repr(float(value))
    finally:ref.release()
    # OpenCV 写入器不允许点号键；ORB 的官方 YAML 使用点号，手动构造标量。
    matrix=', '.join(repr(float(x)) for x in geo.T_imu_from_orb_left.reshape(-1))
    text='%YAML:1.0\n---\n'+'\n'.join(f'{k}: {v}' for k,v in values.items())
    text+='\nIMU.T_b_c1: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: f\n   data: [ '+matrix+' ]\n'
    dest.write_text(text,encoding='utf8')
    info={'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'source':str(source),
          'imu_noise_source':str(report),'orb_left':'cam1 / channel 5','orb_right':'cam0 / channel 4',
          'projection_left':r.P1.tolist(),'projection_right':r.P2.tolist(),
          'T_orb_left_from_gui_left':geo.T_orb_left_from_gui_left.tolist(),
          'T_imu_from_orb_left':geo.T_imu_from_orb_left.tolist(),
          'kalibr_cam1_shift_s':float(data['cam1']['timeshift_cam_imu']),
          'shift_applied':False,'status':'PRELIMINARY_NOT_SYNC_VALIDATED',
          'note':'仅几何和参数准备；不证明时间戳来源、噪声单位、传感器同步或定位精度。'}
    audit=dest.with_suffix('.json');audit.write_text(json.dumps(info,ensure_ascii=False,indent=2),encoding='utf8')
    print('configuration:',dest,'\naudit:',audit)

if __name__=='__main__':main()
