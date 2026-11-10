"""FreiHAND RGB 单目检测测试；与双目重建及第一人称表现分开报告。"""
import argparse
import csv
import json
import statistics
import time
from pathlib import Path

import cv2
import yaml
from egohand.detection.mediapipe_detector import MediaPipeHandDetector

ROOT = Path(__file__).resolve().parents[1]
IMAGES = ROOT / 'samples/date/RGB_Hands/FreiHAND/mirror_eval_rgb'
OUT = ROOT / 'benchmark-results/freihand_rgb_eval'

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--max-images', type=int, default=0)
    args = parser.parse_args()
    images = sorted(IMAGES.glob('*.jpg'))
    if args.max_images:
        images = images[:args.max_images]
    if not images:
        raise SystemExit('没有可用图片')
    config = yaml.safe_load((ROOT/'configs/detector.yaml').read_text(encoding='utf8'))
    detector = MediaPipeHandDetector(ROOT/config['model_path'], mode='IMAGE', **{key:config[key] for key in ('num_hands','min_hand_detection_confidence','min_hand_presence_confidence','min_tracking_confidence')})
    OUT.mkdir(parents=True, exist_ok=True)
    rows=[]
    try:
        for path in images:
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is None:
                rows.append({'image':path.name,'status':'decode_error'})
                continue
            started = time.perf_counter()
            detections = detector.detect(image)
            elapsed = (time.perf_counter()-started)*1000
            rows.append({'image':path.name,'status':'ok','detections':len(detections),
                'elapsed_ms':round(elapsed,3),'rescue_attempts':detector.last_detection_diagnostics['crop_attempts'],
                'rescue_hands':detector.last_detection_diagnostics['rescue_hands'],
                'width':image.shape[1],'height':image.shape[0],
                'channels_differ':bool((image[:,:,0]!=image[:,:,1]).any() or (image[:,:,1]!=image[:,:,2]).any())})
    finally:
        detector.close()
    valid=[row for row in rows if row['status']=='ok']
    report={'dataset':'FreiHAND evaluation image subset (third-party mirror)', 'annotation_status':'not verified; no ground-truth accuracy claimed',
            'method':'MediaPipe IMAGE with current detector.yaml and rescue; no manual ROI', 'model_sha256':detector.sha256,
            'images_attempted':len(rows),'images_decoded':len(valid), 'detected_at_least_one':sum(r['detections']>0 for r in valid),
            'detected_two_or_more':sum(r['detections']>1 for r in valid), 'rescue_invoked':sum(r['rescue_attempts']>0 for r in valid),
            'rescue_detected':sum(r['rescue_hands']>0 for r in valid),
            'median_ms':statistics.median(r['elapsed_ms'] for r in valid) if valid else None,
            'mean_ms':statistics.mean(r['elapsed_ms'] for r in valid) if valid else None,
            'all_have_nonidentical_color_channels':all(r['channels_differ'] for r in valid),
            'limitations':'Hand-centred single-view crops; no evidence for stereo or egocentric performance; detection counts are not annotated recall.'}
    (OUT/'summary.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf8')
    with (OUT/'per_image.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=['image','status','detections','elapsed_ms','rescue_attempts','rescue_hands','width','height','channels_differ'])
        writer.writeheader();writer.writerows(rows)
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)

if __name__=='__main__':
    main()
