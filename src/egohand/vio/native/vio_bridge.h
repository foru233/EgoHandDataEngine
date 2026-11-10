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
    double imu_init_elapsed_sec;
    double frame_timestamp_sec;
    uint64_t map_epoch;
    /* Row-major T_world_from_rectified_left_camera, 4x4; NaN if pose_valid=0. */
    float T_world_from_rectified_left_camera[16];
} EgohandVioResult;

EGOHAND_VIO_API const char* EGOHAND_VIO_CALL egohand_vio_api_version(void);
EGOHAND_VIO_API const char* EGOHAND_VIO_CALL egohand_vio_last_error(void);

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

EGOHAND_VIO_API int EGOHAND_VIO_CALL egohand_vio_reset(void* handle);
EGOHAND_VIO_API int EGOHAND_VIO_CALL egohand_vio_shutdown(void* handle);
EGOHAND_VIO_API void EGOHAND_VIO_CALL egohand_vio_destroy(void* handle);

#ifdef __cplusplus
}
#endif
