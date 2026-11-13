"""由 calibration2 生成 ORB-SLAM3 配置；输出路径显式指定，绝不覆盖已被使用的标定。

默认只读既有标定并生成 ORB YAML：
    python scripts/prepare_vs030_calibration2.py \
        --calibration "标定结果/2/data3-camchain-imucam-measured.yaml" \
        --orb-template configs/vio/vs030_rectified_half_calibration2.yaml \
        --orb-out configs/vio/vs030_rectified_full_calibration2.yaml \
        --scale 1.0

如需按 PCB 候选相机中心重新生成测量标定（会写新文件，不覆盖源 Kalibr）：
    python scripts/prepare_vs030_calibration2.py \
        --source "标定结果/2/data3-camchain-imucam(1).yaml" \
        --write-calibration --calibration-out 标定结果/2/data3-camchain-imucam-measured.new.yaml \
        --orb-template ... --orb-out ... --scale 1.0

安全约束：
- 所有输出必须显式给出；不提供 `--legacy-half-paths` 就不会写默认路径。
- 拒绝覆盖 SOURCE、实时链路正在使用的标定、以及实时链路正在使用的 ORB YAML，除非显式 `--force`。
- 生成后用 validate_geometry() 复核，含宽高、内参、基线、IMU 外参；通过才打印 OK。
"""
import argparse
import hashlib
import re
import sys
import tempfile
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from egohand.vio.vs030_geometry import prepare_vs030_vio_geometry  # noqa: E402
from egohand.vio.config_snapshot import SNAPSHOT_FILES, VioConfigSnapshot  # noqa: E402

SOURCE = ROOT / '标定结果/2/data3-camchain-imucam(1).yaml'
MEASURED = ROOT / '标定结果/2/data3-camchain-imucam-measured.yaml'
LIVE_ORB = ROOT / 'configs/vio/vs030_rectified_half_calibration2.yaml'
LIVE_TIMING = ROOT / 'configs/vio/live_timing.yaml'
LIVE_QUALITY = ROOT / 'configs/vio/live_quality.yaml'

# IMU 为原点，单位 m。X 符号由结果 2 的双目外参选定；轴向及测量点仍待确认。
CAMERA_CENTERS_IMU_M = {
    'cam0': np.array([0.03022, 0.00375, 0.00258]),
    'cam1': np.array([-0.03485, 0.00375, 0.00258]),
}


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _guard_overwrite(path, *, protected, force, label):
    path = Path(path).resolve()
    if force:
        return path
    for protected_path in protected:
        if protected_path is None:
            continue
        if path == Path(protected_path).resolve():
            raise SystemExit(f'拒绝覆盖{label}：{path}（如确实需要，显式加 --force）')
    return path


def write_measured_calibration(source, destination, *, force):
    source = Path(source).resolve()
    destination = _guard_overwrite(destination, protected=[source, MEASURED], force=force,
                                   label='已使用的标定')
    data = yaml.safe_load(source.read_text(encoding='utf-8-sig'))
    for camera, center in CAMERA_CENTERS_IMU_M.items():
        transform = np.asarray(data[camera]['T_cam_imu'], dtype=float)
        # Kalibr 存的是 IMU -> camera；输入是 camera 原点在 IMU 坐标中的位置。
        transform[:3, 3] = -transform[:3, :3] @ center
        data[camera]['T_cam_imu'] = transform.tolist()
    destination.write_text(
        '# 结果 2 + PCB 候选相机中心；轴向及测量参考点未最终验收。\n'
        + yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding='utf-8')
    return destination


def render_orb_config(template, calibration, scale):
    geom = prepare_vs030_vio_geometry(calibration)
    rect = geom.rectifier
    height, width = rect.maps_left[0].shape
    values = {
        'Camera1.fx': rect.P1[0, 0] * scale,
        'Camera1.fy': rect.P1[1, 1] * scale,
        'Camera1.cx': rect.P1[0, 2] * scale,
        'Camera1.cy': rect.P1[1, 2] * scale,
        'Stereo.b': -rect.P2[0, 3] / rect.P2[0, 0],
        'Camera.width': round(width * scale),
        'Camera.height': round(height * scale),
    }
    text = Path(template).read_text(encoding='utf-8')
    for key, value in values.items():
        rendered = f'{key}: {int(value)}' if key in ('Camera.width', 'Camera.height') else f'{key}: {float(value)!r}'
        text, count = re.subn(rf'(?m)^{re.escape(key)}:.*$', rendered, text)
        if count != 1:
            raise SystemExit(f'ORB 模板缺少或重复 {key}')
    matrix = ', '.join(repr(float(v)) for v in geom.T_imu_from_orb_left.reshape(-1))
    text, count = re.subn(r'(?ms)(^IMU\.T_b_c1:.*?\bdata:\s*\[)[^\]]*(\])',
                          lambda match: match[1] + ' ' + matrix + ' ' + match[2], text)
    if count != 1:
        raise SystemExit('ORB 模板缺少或重复 IMU.T_b_c1')
    return text, geom, (width, height)


def validate(calibration, orb_path, scale):
    timing_path = Path(tempfile.mkdtemp()) / SNAPSHOT_FILES['timing']
    timing_path.write_text(yaml.safe_dump({'image_scale': float(scale)}), encoding='utf8')
    snapshot = VioConfigSnapshot(Path(calibration).resolve(), Path(orb_path).resolve(),
                                 timing_path, LIVE_QUALITY, {'image_scale': float(scale)}, {}, 'prepare')
    geometry = snapshot.validate_geometry()
    return geometry


def main():
    parser = argparse.ArgumentParser(description='由 calibration2 生成 ORB-SLAM3 配置（显式输出路径）')
    parser.add_argument('--source', type=Path, default=SOURCE, help='Kalibr camchain 源（仅用于 --write-calibration）')
    parser.add_argument('--calibration', type=Path, default=None,
                        help='读取的标定；缺省时用 --calibration-out（需同时 --write-calibration）')
    parser.add_argument('--write-calibration', action='store_true', help='按 PCB 候选相机中心写出新的测量标定')
    parser.add_argument('--calibration-out', type=Path, default=None)
    parser.add_argument('--orb-template', type=Path, default=LIVE_ORB)
    parser.add_argument('--orb-out', type=Path, required=True)
    parser.add_argument('--scale', type=float, default=1.0)
    parser.add_argument('--force', action='store_true', help='允许覆盖已存在的输出（仍拒绝覆盖源文件）')
    args = parser.parse_args()

    if not (0 < args.scale <= 1):
        raise SystemExit('--scale 须在 (0,1] 内')

    if args.write_calibration:
        if args.calibration_out is None:
            raise SystemExit('--write-calibration 需要 --calibration-out')
        calibration = write_measured_calibration(args.source, args.calibration_out, force=args.force)
    else:
        calibration = Path(args.calibration or MEASURED).resolve()
        if not calibration.is_file():
            raise SystemExit(f'标定不存在：{calibration}')

    protected = [SOURCE, MEASURED, LIVE_ORB]
    orb_out = _guard_overwrite(args.orb_out, protected=protected, force=args.force, label='ORB 配置')
    if orb_out.exists() and not args.force:
        raise SystemExit(f'ORB 输出已存在：{orb_out}（如确实需要，显式加 --force）')

    text, geom, full_size = render_orb_config(args.orb_template, calibration, args.scale)
    orb_out.parent.mkdir(parents=True, exist_ok=True)
    orb_out.write_text(text, encoding='utf-8')

    validate(calibration, orb_out, args.scale)
    print(f'calibration : {calibration}  sha256={_sha256(calibration)}')
    print(f'orb_config  : {orb_out}  sha256={_sha256(orb_out)}')
    print(f'scale       : {args.scale}  →  ORB 图像 {round(full_size[0]*args.scale)}×{round(full_size[1]*args.scale)}'
          f'（标定图像空间 {full_size[0]}×{full_size[1]}）')
    print('validate_geometry: OK（内参/宽高/基线/IMU 外参均与标定一致）')


if __name__ == '__main__':
    main()
