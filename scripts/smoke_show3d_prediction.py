"""One-frame real MediaPipe SHOW3D prediction smoke (no GT fed to detector)."""
from pathlib import Path
from egohand.core.pipeline import EgoHandEngine,ROOT
from egohand.sources.show3d import SHOW3DSource,SHOW3DFrameProcessor
from egohand.detection.mediapipe_detector import MediaPipeHandDetector

root=ROOT/"samples/date/SHOW3D"
scene=root/"scenes/LWA828/bbq_tapping_33e9"
model=ROOT/"models/hand_landmarker.task"
src=SHOW3DSource(scene,view_mode="prediction")
detectors=[MediaPipeHandDetector(model,mode="IMAGE",min_hand_detection_confidence=.2) for _ in range(2)]
engine=EgoHandEngine(detectors=detectors)
processor=SHOW3DFrameProcessor()
try:
    engine.start(src)
    for frame_index in (0,30,60):
        src.seek(frame_index)
        frame=src.read()
        result=processor.process(engine,frame,"prediction")
        print("FRAME",frame_index,"status",result.reconstruction_status,
              "detections",len(result.left_detections),len(result.right_detections),
              "hands",len(result.hands),
              "valid_3d",sum(h.valid_joint_count for h in result.hands),
              "pairing",result.diagnostics.get("dataset_pairing"),
              "error",result.diagnostics.get("reason"),flush=True)
        assert result.diagnostics["result_kind"]=="algorithm_prediction"
        assert result.diagnostics["left_detection_origin"]=="mediapipe"
        assert result.diagnostics["right_detection_origin"]=="mediapipe"
        assert result.diagnostics["native_camera_model"]=="PinholePlane"
        assert result.source_type=="show3d"
finally:
    engine.close()
print("SHOW3D_REAL_MEDIAPIPE_SMOKE_PASS",flush=True)
