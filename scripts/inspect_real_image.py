import json
from pathlib import Path
import cv2
from egohand.detection.mediapipe_detector import MediaPipeHandDetector
from egohand.core.types import serializable
from egohand.detection.landmarks import BONES

def main():
    img=cv2.imread("samples/public/hand.jpg")
    d=MediaPipeHandDetector("models/hand_landmarker.task","IMAGE")
    try:hands=d.detect(img)
    finally:d.close()
    if not hands:raise RuntimeError("真实图片没有检测到手")
    Path("logs").mkdir(exist_ok=True)
    Path("logs/real_image_detection.json").write_text(json.dumps({"mode":"Real Image Detection Mode; monocular 2D only","source":"samples/public/provenance.json","hands":serializable(hands)},indent=2,allow_nan=False),encoding="utf-8")
    for h in hands:
        uv=h.pixel_landmarks.astype(int)
        for a,b in BONES:cv2.line(img,tuple(uv[a]),tuple(uv[b]),(40,220,60),4)
        for j,p in enumerate(uv):cv2.circle(img,tuple(p),7,(40,220,60),-1);cv2.putText(img,str(j),tuple(p),cv2.FONT_HERSHEY_SIMPLEX,.65,(40,90,255),2)
    cv2.putText(img,"REAL PHOTO / 2D ONLY / NO METRIC STEREO",(20,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,80,255),2)
    cv2.imwrite("logs/real_image_detection.jpg",img)
    print(f"Actual detected hands: {len(hands)}; each has 21 landmarks")

if __name__=="__main__":main()
