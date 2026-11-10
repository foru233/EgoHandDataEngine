"""Headless, reproducible STB/HOT3D benchmark. Raw data are read-only."""
import argparse
from datetime import datetime
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from egohand.benchmarks.runner import run


def main():
    # 评测参数统一在入口声明：dataset-root 只读，output 保存独立结果，不覆盖原数据。
    # --config 是双目几何配置；检测阈值另由 --detector-config 指定，默认使用 configs/detector.yaml。
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',choices=['stb','hot3d'],required=True)
    p.add_argument('--dataset-root',required=True)
    p.add_argument('--max-frames',type=int,default=200)
    # oracle-2d 仅验证投影几何；真实图片检测成绩必须单独运行 mediapipe-stereo。
    p.add_argument('--mode',choices=['oracle-2d','mediapipe-stereo'],default='mediapipe-stereo')
    p.add_argument('--config')
    p.add_argument('--detector-config',default='configs/detector.yaml',help='MediaPipe threshold and rescue settings, same default as GUI')
    p.add_argument('--output',default='benchmark-results/run_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--manifest',help='Previously emitted sample_manifest.json; fixed sample set, kept in dataset order')
    p.add_argument('--toolkit-root')
    p.add_argument('--clips',help='Comma-separated clip stems, e.g. clip-001849,clip-001851')
    p.add_argument('--model',default='models/hand_landmarker.task')
    p.add_argument('--native-detection',action='store_true',help='历史选项，现已停用')
    p.add_argument('--detector-input',choices=['virtual','native'],default='virtual',help='virtual：去畸变及极线校正后检测；native 已停用')
    args=p.parse_args()
    if args.max_frames<1:p.error('--max-frames must be positive')
    if args.detector_input!='virtual' or args.native_detection:p.error('原生图检测已停用；请使用 --detector-input virtual 并去掉 --native-detection')
    return run(args)


if __name__=='__main__':sys.exit(main())

