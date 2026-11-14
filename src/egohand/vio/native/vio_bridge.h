#pragma once
#include <stdint.h>

#if defined(_WIN32)
  #if defined(EGOHAND_VIO_BRIDGE_BUILD)
    #define EGOHAND_VIO_API __declspec(dllexport)
  #else
    #define EGOHAND_VIO_API __declspec(dllimport)
  #endif
  #define EGOHAND_VIO_CALL __cdecl
#else
  #define EGOHAND_VIO_API __attribute__((visibility("default")))
  #define EGOHAND_VIO_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct EgohandVioImuSample {
    int64_t timestamp_ns;
    float accel_x_mps2, accel_y_mps2, accel_z_mps2;
    float gyro_x_rps, gyro_y_rps, gyro_z_rps;
} EgohandVioImuSample;

typedef struct EgohandVioResult {
    int32_t tracking_state;
    int32_t pose_valid;
    int32_t is_lost;
    int32_t imu_initialized;
    /* Legacy name: 1 means inertial BA2 completed, not accuracy certification.
       Stereo has metric scale from its calibrated baseline before BA2. */
    int32_t inertial_scale_finalized;
    double imu_init_elapsed_sec;
    double frame_timestamp_sec;
    uint64_t map_epoch;
    /* Row-major T_world_from_rectified_left_camera, 4x4; NaN if pose_valid=0. */
    float T_world_from_rectified_left_camera[16];
} EgohandVioResult;


/* Read-only map appearance, NOT Tracking correspondences or recovery evidence.
   Point selection is bounded map-container order, independent of candidate pose.
   status: 0 unavailable, 1 copied, 2 stale/version mismatch, 3 unsupported model,
   4 insufficient output capacity. flags: 1 partial writer coverage, 2 truncated.
   Even status=1 does not certify all map writers, static scene, or continuity. */
typedef struct EgohandVioMapAppearancePoint {
    uint64_t point_id;
    float position[3];
    uint8_t descriptor[32];
    uint32_t observations;
} EgohandVioMapAppearancePoint;
typedef struct EgohandVioMapAppearanceHeader {
    uint32_t schema_version,struct_size,status,flags;
    uint32_t point_count,sampled_count,capacity,source_kind,image_width,image_height;
    uint64_t frame_id,map_id,reset_id,atlas_generation,map_revision,map_change_index,total_map_points,world_frame_revision;
    int64_t timestamp_ns;
    float fx,fy,cx,cy,bf;
    uint32_t reserved;
} EgohandVioMapAppearanceHeader;
EGOHAND_VIO_API int32_t EGOHAND_VIO_CALL egohand_vio_map_appearance_header_size(void);
EGOHAND_VIO_API int32_t EGOHAND_VIO_CALL egohand_vio_map_appearance_point_size(void);
/* Field index follows declaration order. -1 means invalid kind/index. */
EGOHAND_VIO_API int32_t EGOHAND_VIO_CALL egohand_vio_map_appearance_offset(int32_t kind,int32_t field);
EGOHAND_VIO_API int EGOHAND_VIO_CALL egohand_vio_get_map_appearance(
    void* handle,uint64_t expected_frame_id,int64_t expected_timestamp_ns,
    EgohandVioMapAppearanceHeader* header,int32_t header_size,
    EgohandVioMapAppearancePoint* points,int32_t point_size,int32_t capacity);

/* Independent extension. EgohandVioResult's existing ABI remains unchanged. */
typedef struct EgohandVioFrameSnapshot {
    uint32_t schema_version, struct_size;
    uint64_t frame_id, map_id, map_revision, reset_id, world_frame_id, map_change_index;
    int64_t timestamp_ns;
    int32_t snapshot_consistent, tracking_state, pose_valid, imu_initialized;
    int32_t ba1_complete, ba2_complete, body_valid;
    int32_t left_features, right_features, stereo_valid_depths, tracked_inliers;
    float T_world_from_rectified_left_camera[16];
    float T_world_from_body[16];
    float velocity_world_mps[3], accel_gyro_bias[6];
} EgohandVioFrameSnapshot;

EGOHAND_VIO_API int32_t EGOHAND_VIO_CALL egohand_vio_snapshot_size(void);
EGOHAND_VIO_API int EGOHAND_VIO_CALL egohand_vio_get_frame_snapshot(
    void* handle, EgohandVioFrameSnapshot* out_snapshot, int32_t out_size);

/* Diagnostic-only partial journal. map_update_lock_matches_caller checks only
   pointer equality with the caller's claimed locked map, not mutex ownership.
   coverage=1 never establishes that the
   reference stayed fixed. Event time is the source KF's relative seconds,
   NOT commit or camera time. Transform direction: p_after=s*R*p_before+t. */
#define EGOHAND_VIO_MAP_EVENT_CAPACITY 16
typedef struct EgohandVioMapEvent {
    uint64_t sequence, map_id, source_keyframe_id;
    uint64_t revision_before, revision_after, change_before, change_after;
    double source_keyframe_time_sec;
    float scale, rotation[9], translation[3];
    uint32_t type, map_update_lock_matches_caller, reset_generation_known, reserved;
} EgohandVioMapEvent;
typedef struct EgohandVioMapEventFrame {
    uint32_t schema_version, struct_size;
    uint64_t frame_id, map_id, reset_id;
    int64_t frame_timestamp_ns;
    uint32_t snapshot_consistent, frame_bound, coverage, event_count;
    uint64_t overwritten_count;
    EgohandVioMapEvent events[EGOHAND_VIO_MAP_EVENT_CAPACITY];
} EgohandVioMapEventFrame;
/* type 0 unknown, 1 initial alignment, 2 inertial realignment, 3 merge alignment.
   Return 0 for a report, 1 if no frame snapshot, -1 on argument/lifecycle error. */
EGOHAND_VIO_API int32_t EGOHAND_VIO_CALL egohand_vio_map_event_frame_size(void);
EGOHAND_VIO_API int EGOHAND_VIO_CALL egohand_vio_get_map_event_frame(
    void* handle, EgohandVioMapEventFrame* out_frame, int32_t out_size);

/* Independent extension: world-frame revision counter.
   A free-gauge FullInertialBA can translate the whole map without changing map_epoch
   (evidence: docs/VIO_27m跳变取证_20261111.md, 75 keyframes moved 22.82 m inside one
   solve while map_id/reset_revision/big_change_revision stayed 0/0/9). This additive
   report lets a caller tell "whole map re-anchored" apart from tracking error.
   EgohandVioResult's existing field order and size are unchanged: the report is a
   separate struct behind size-checked getters, so an older Python binding keeps
   working and an older DLL simply fails the capability probe. */
typedef struct EgohandVioWorldFrameInfo {
    uint32_t schema_version, struct_size;
    int32_t available;         /* 1 when this DLL tracks world-frame revisions */
    int32_t verdict;           /* last fit: 0 re-anchor, 1 below threshold, 2 non-rigid, 3 insufficient */
    int32_t last_keyframes;    /* keyframes used by the last SE(3) fit */
    int32_t reserved;
    uint64_t revision;         /* confirmed whole-map re-anchors in this handle's lifetime */
    uint64_t detections;       /* equals revision; kept for audit symmetry */
    uint64_t evaluations;      /* free-gauge full BA solves that were fit */
    uint64_t rejected_nonrigid;/* large displacement that was not rigid over the same ids */
    uint64_t map_id;           /* map of the last evaluation */
    uint64_t last_max_keyframe_id;
    double last_timestamp_sec;
    double last_translation_m; /* fitted whole-map displacement */
    double last_rotation_deg;
    double last_scale;         /* ~1.0 means rigid translation, not scale drift */
    double last_residual_p50_m, last_residual_p95_m, last_residual_p99_m, last_residual_max_m;
    double last_residual_limit_m;
} EgohandVioWorldFrameInfo;

EGOHAND_VIO_API int32_t EGOHAND_VIO_CALL egohand_vio_world_frame_info_size(void);
EGOHAND_VIO_API int EGOHAND_VIO_CALL egohand_vio_get_world_frame_info(
    void* handle, EgohandVioWorldFrameInfo* out_info, int32_t out_size);
/* ';'-separated additive capability tokens, e.g. "snapshot;world_frame_revision". */
EGOHAND_VIO_API const char* EGOHAND_VIO_CALL egohand_vio_capabilities(void);

EGOHAND_VIO_API const char* EGOHAND_VIO_CALL egohand_vio_api_version(void);
EGOHAND_VIO_API const char* EGOHAND_VIO_CALL egohand_vio_last_error(void);
/* Optional additive export: map points dropped by the opt-in clamp (0 when the clamp is
   disabled or the build predates it). Deliberately NOT a struct field so that
   EgohandVioWorldFrameInfo keeps its size and older/newer DLLs stay interchangeable. */
EGOHAND_VIO_API uint64_t EGOHAND_VIO_CALL egohand_vio_point_clamp_count(void);

/* Optional additive export: map points rejected by the opt-in absolute depth gate. */
EGOHAND_VIO_API uint64_t EGOHAND_VIO_CALL egohand_vio_depth_gate_rejected(void);

/* Optional additive export: map points pruned because they already exceeded their range bound. */
EGOHAND_VIO_API uint64_t EGOHAND_VIO_CALL egohand_vio_entry_pruned(void);
EGOHAND_VIO_API uint64_t EGOHAND_VIO_CALL egohand_vio_invalid_depth_checks(void);
EGOHAND_VIO_API uint64_t EGOHAND_VIO_CALL egohand_vio_invalid_depth_removed(void);
EGOHAND_VIO_API uint64_t EGOHAND_VIO_CALL egohand_vio_invalid_depth_points(void);

/* sizeof(EgohandVioResult); Python 侧用来校验 ABI 是否一致。 */
EGOHAND_VIO_API int32_t EGOHAND_VIO_CALL egohand_vio_result_size(void);

/* Candidate ORB state, available only after a tracked inertial frame.
   The caller must apply its own trust and continuity checks. */
EGOHAND_VIO_API int EGOHAND_VIO_CALL egohand_vio_get_body_state(
    void* handle, float* T_world_from_body_16, float* velocity_3,
    float* accel_gyro_bias_6, int* ba2_complete);

/* 1 consistent, 0 reject, -1 unavailable. Callback runs synchronously in TrackStereo. */
typedef int (EGOHAND_VIO_CALL *EgohandRecoveryCallback)(const float* Twb16, uint64_t epoch, void* context);
EGOHAND_VIO_API int EGOHAND_VIO_CALL egohand_vio_set_recovery_callback(
    void* handle, EgohandRecoveryCallback callback, void* context);

/* Optional short-recovery geometry ABI marker.
   The current bridge deliberately reports unavailable until an independent
   native stereo/map geometry solver is wired in.  Presence of this export
   lets the recovery worker fail closed with a structured unavailable result
   instead of silently selecting a legacy DLL. */
EGOHAND_VIO_API const char* EGOHAND_VIO_CALL egohand_vio_short_recovery_report(void);

EGOHAND_VIO_API int EGOHAND_VIO_CALL egohand_vio_create(
    const char* vocabulary_path, const char* settings_path,
    int use_viewer, void** out_handle);

EGOHAND_VIO_API int EGOHAND_VIO_CALL egohand_vio_track_stereo(
    void* handle,
    const uint8_t* left_gray, const uint8_t* right_gray,
    int width, int height, int stride_bytes,
    int64_t frame_timestamp_ns,
    const EgohandVioImuSample* imu_samples, int imu_count,
    EgohandVioResult* out_result);

EGOHAND_VIO_API int EGOHAND_VIO_CALL egohand_vio_create_map(
    const char* vocabulary_path, const char* settings_path, int localize_only, void** out_handle);
EGOHAND_VIO_API int EGOHAND_VIO_CALL egohand_vio_export_map(
    void* handle, const char* atlas_path, const char* pose_path);

EGOHAND_VIO_API int EGOHAND_VIO_CALL egohand_vio_reset(void* handle);
/* Start a fresh map without resetting the sensor timestamp epoch. */
EGOHAND_VIO_API int EGOHAND_VIO_CALL egohand_vio_start_new_map(void* handle);
EGOHAND_VIO_API int EGOHAND_VIO_CALL egohand_vio_shutdown(void* handle);
EGOHAND_VIO_API void EGOHAND_VIO_CALL egohand_vio_destroy(void* handle);

#ifdef __cplusplus
}
#endif
