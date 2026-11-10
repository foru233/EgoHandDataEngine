from enum import StrEnum

class Status(StrEnum):
    READY = "Ready"
    RUNNING = "Running"
    PAUSED = "Paused"
    STOPPED = "Stopped"
    MISSING_CALIBRATION = "Missing Calibration"
    MODEL_NOT_LOADED = "Model Not Loaded"
    INVALID_MATCH = "Invalid Stereo Match"
    SYNTHETIC = "Synthetic Ground Truth"
    TWO_D = "2D Only"
    METRIC = "Metric Stereo Reconstruction"
    ERROR = "Error"

class JointStatus(StrEnum):
    OBSERVED = "STEREO_OBSERVED"
    UNMATCHED = "UNMATCHED"
    EPIPOLAR = "EPIPOLAR_REJECTED"
    DISPARITY = "INVALID_DISPARITY"
    NEGATIVE = "NEGATIVE_DEPTH"
    REPROJECTION = "REPROJECTION_REJECTED"
    TEMPORAL = "TEMPORAL_OUTLIER"
    BONE = "BONE_LENGTH_REJECTED"
    DEPTH = "DEPTH_RANGE_REJECTED"
    INVALID = "INVALID"
    IDENTITY = "IDENTITY_AMBIGUOUS"
    UNCERTAINTY = "HIGH_UNCERTAINTY"
