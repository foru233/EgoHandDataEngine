"""复现 ORB-SLAM3 官方 EuRoC 的 R1，并返回 T_rectified_from_body。"""
import json
import sys
from pathlib import Path
import cv2
import numpy as np

root = Path(__file__).resolve().parents[1]
if len(sys.argv) > 1 and sys.argv[1] == 'tumvi':
    fs = cv2.FileStorage(str(root / 'samples/date/VIO/converted/TUM-VI/room4/calibration/TUM-VI.yaml'), cv2.FILE_STORAGE_READ)
    try:
        T_body_camera = fs.getNode('IMU.T_b_c1').mat()
        if T_body_camera is None:
            raise ValueError('缺少 IMU 外参')
        print(json.dumps(np.linalg.inv(T_body_camera).tolist()))
    finally:
        fs.release()
    raise SystemExit(0)
fs = cv2.FileStorage(str(root / 'configs/vio/euroc_stereo.yaml'), cv2.FILE_STORAGE_READ)
imu = cv2.FileStorage(str(root / 'samples/date/VIO/converted/EuRoC/MH_01_easy/calibration/EuRoC.yaml'), cv2.FILE_STORAGE_READ)
try:
    intrinsics, distortions = [], []
    for camera in (1, 2):
        get = lambda key: fs.getNode(f'Camera{camera}.{key}').real()
        intrinsics.append(np.array([[get('fx'), 0, get('cx')], [0, get('fy'), get('cy')], [0, 0, 1]], dtype=np.float64))
        distortions.append(np.array([get(k) for k in ('k1', 'k2', 'p1', 'p2')]))
    T_c1_c2 = fs.getNode('Stereo.T_c1_c2').mat()
    T_body_camera = imu.getNode('IMU.T_b_c1').mat()
    if T_c1_c2 is None or T_body_camera is None:
        raise ValueError('缺少双目或 IMU 外参')
    T_c2_c1 = np.linalg.inv(T_c1_c2)
    R1, _, _, _, _, _, _ = cv2.stereoRectify(intrinsics[0], distortions[0], intrinsics[1], distortions[1], (752, 480), T_c2_c1[:3,:3], T_c2_c1[:3,3], flags=cv2.CALIB_ZERO_DISPARITY, alpha=-1, newImageSize=(752, 480))
    T_rect_camera = np.eye(4)
    T_rect_camera[:3,:3] = R1
    print(json.dumps((T_rect_camera @ np.linalg.inv(T_body_camera)).tolist()))
finally:
    fs.release()
    imu.release()
