#include "vio_bridge.h"
#include <System.h>
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace {
thread_local std::string g_last_error;
struct VioHandle {
    std::unique_ptr<ORB_SLAM3::System> system;
    std::mutex mutex;
    int64_t epoch_ns = 0, last_frame_ns = 0;
    bool has_epoch = false, has_last_frame = false, shutdown = false;
};
void clear_error() { g_last_error.clear(); }
void set_error(const std::string& s) { g_last_error = s; }
double seconds_from_epoch(int64_t t, int64_t epoch) {
    return static_cast<double>(t - epoch) * 1e-9;
}
bool finite_sample(const EgohandVioImuSample& s) {
    return std::isfinite(s.accel_x_mps2) && std::isfinite(s.accel_y_mps2) &&
           std::isfinite(s.accel_z_mps2) && std::isfinite(s.gyro_x_rps) &&
           std::isfinite(s.gyro_y_rps) && std::isfinite(s.gyro_z_rps);
}
void invalid_result(EgohandVioResult* r) {
    r->tracking_state=-1; r->pose_valid=0; r->is_lost=1; r->imu_initialized=0;
    r->imu_init_elapsed_sec=-1.0;
    r->frame_timestamp_sec=std::numeric_limits<double>::quiet_NaN();
    r->map_epoch=0;
    const float nan=std::numeric_limits<float>::quiet_NaN();
    std::fill(std::begin(r->T_world_from_rectified_left_camera),
              std::end(r->T_world_from_rectified_left_camera),nan);
}
}

extern "C" {
const char* EGOHAND_VIO_CALL egohand_vio_api_version() {
    return "egohand-vio-bridge/0.1.0";
}
const char* EGOHAND_VIO_CALL egohand_vio_last_error() { return g_last_error.c_str(); }

int EGOHAND_VIO_CALL egohand_vio_create(
    const char* vocabulary_path,const char* settings_path,int use_viewer,void** out_handle) {
    clear_error();
    if(!out_handle){set_error("out_handle is null");return -1;}
    *out_handle=nullptr;
    if(!vocabulary_path||!settings_path||!*vocabulary_path||!*settings_path){
        set_error("vocabulary_path and settings_path are required");return -1;
    }
    try {
        auto h=std::make_unique<VioHandle>();
        ORB_SLAM3::Verbose::SetTh(ORB_SLAM3::Verbose::VERBOSITY_NORMAL);
        h->system=std::make_unique<ORB_SLAM3::System>(
            vocabulary_path,settings_path,ORB_SLAM3::System::IMU_STEREO,use_viewer!=0);
        *out_handle=h.release();
        return 0;
    } catch(const std::exception& e){set_error(e.what());return -1;}
      catch(...){set_error("unknown exception creating ORB-SLAM3");return -1;}
}

int EGOHAND_VIO_CALL egohand_vio_track_stereo(
    void* opaque,const uint8_t* left_gray,const uint8_t* right_gray,
    int width,int height,int stride_bytes,int64_t frame_ns,
    const EgohandVioImuSample* imu,int imu_count,EgohandVioResult* out) {
    clear_error();
    if(!opaque||!left_gray||!right_gray||!out){set_error("null handle, image or output");return -1;}
    invalid_result(out);
    if(width<=0||height<=0||stride_bytes<width){set_error("invalid image size/stride");return -1;}
    if(frame_ns<=0||imu_count<0||(imu_count>0&&!imu)){set_error("invalid timestamp or IMU buffer");return -1;}
    auto* h=static_cast<VioHandle*>(opaque);
    std::lock_guard<std::mutex> lock(h->mutex);
    if(h->shutdown||!h->system){set_error("ORB-SLAM3 is shut down");return -1;}
    if(h->has_last_frame&&frame_ns<=h->last_frame_ns){
        set_error("camera timestamps must be strictly increasing");return -1;
    }
    if(!h->has_last_frame&&imu_count){
        set_error("first stereo frame must have no IMU samples, matching ORB-SLAM3 dataset input");return -1;
    }
    int64_t previous=h->has_last_frame?h->last_frame_ns:std::numeric_limits<int64_t>::min();
    for(int i=0;i<imu_count;++i){
        if(!finite_sample(imu[i])){set_error("IMU sample contains NaN/Inf");return -1;}
        if(imu[i].timestamp_ns<=previous){
            set_error("IMU timestamps must increase and follow previous image timestamp");return -1;
        }
        if(imu[i].timestamp_ns>frame_ns){
            set_error("IMU sample is later than current camera frame");return -1;
        }
        previous=imu[i].timestamp_ns;
    }
    try {
        if(!h->has_epoch){h->epoch_ns=frame_ns;h->has_epoch=true;}
        cv::Mat lview(height,width,CV_8UC1,const_cast<uint8_t*>(left_gray),static_cast<size_t>(stride_bytes));
        cv::Mat rview(height,width,CV_8UC1,const_cast<uint8_t*>(right_gray),static_cast<size_t>(stride_bytes));
        cv::Mat left=lview.clone(),right=rview.clone();
        std::vector<ORB_SLAM3::IMU::Point> measurements;
        measurements.reserve(static_cast<size_t>(imu_count));
        for(int i=0;i<imu_count;++i){
            const auto& s=imu[i];
            measurements.emplace_back(s.accel_x_mps2,s.accel_y_mps2,s.accel_z_mps2,
                s.gyro_x_rps,s.gyro_y_rps,s.gyro_z_rps,
                seconds_from_epoch(s.timestamp_ns,h->epoch_ns));
        }
        const double frame_sec=seconds_from_epoch(frame_ns,h->epoch_ns);
        // Caller must supply rectified, synchronized mono8 images and IMU samples
        // timestamped in the same device clock domain. No host-clock mapping occurs here.
        const Sophus::SE3f Tcw=h->system->TrackStereo(left,right,frame_sec,measurements);
        const int state=h->system->GetTrackingState();
        const bool lost=h->system->isLost();
        const double imu_elapsed=h->system->GetTimeFromIMUInit();
        out->tracking_state=state;out->is_lost=lost?1:0;
        out->imu_init_elapsed_sec=imu_elapsed;
        // ORB-SLAM3 在 IMU 未初始化时返回 0，而非负值。
        // 只有正的初始化经过时间才能保守地确认初始化已发生。
        out->imu_initialized=(std::isfinite(imu_elapsed)&&imu_elapsed>0.0)?1:0;
        out->frame_timestamp_sec=frame_sec;
        out->map_epoch=h->system->GetMapEpoch();
        out->pose_valid=(state==2&&!lost)?1:0;
        if(out->pose_valid){
            const Eigen::Matrix4f Twc=Tcw.inverse().matrix();
            if(!Twc.allFinite()) out->pose_valid=0;
            else for(int r=0;r<4;++r)for(int c=0;c<4;++c)
                out->T_world_from_rectified_left_camera[r*4+c]=Twc(r,c);
        }
        if(!out->pose_valid){
            const float nan=std::numeric_limits<float>::quiet_NaN();
            std::fill(std::begin(out->T_world_from_rectified_left_camera),
                      std::end(out->T_world_from_rectified_left_camera),nan);
        }
        h->last_frame_ns=frame_ns;h->has_last_frame=true;
        return 0;
    } catch(const std::exception& e){set_error(e.what());return -1;}
      catch(...){set_error("unknown exception in TrackStereo");return -1;}
}

int EGOHAND_VIO_CALL egohand_vio_reset(void* opaque){
    clear_error();if(!opaque){set_error("null handle");return -1;}
    auto* h=static_cast<VioHandle*>(opaque);std::lock_guard<std::mutex> lock(h->mutex);
    if(h->shutdown||!h->system){set_error("ORB-SLAM3 is shut down");return -1;}
    try{h->system->Reset();h->has_last_frame=false;h->last_frame_ns=0;h->has_epoch=false;h->epoch_ns=0;return 0;}
    catch(const std::exception& e){set_error(e.what());return -1;}
    catch(...){set_error("unknown exception in Reset");return -1;}
}
int EGOHAND_VIO_CALL egohand_vio_shutdown(void* opaque){
    clear_error();if(!opaque){set_error("null handle");return -1;}
    auto* h=static_cast<VioHandle*>(opaque);std::lock_guard<std::mutex> lock(h->mutex);
    if(h->shutdown||!h->system)return 0;
    try{h->system->Shutdown();h->shutdown=true;return 0;}
    catch(const std::exception& e){set_error(e.what());return -1;}
    catch(...){set_error("unknown exception in Shutdown");return -1;}
}
void EGOHAND_VIO_CALL egohand_vio_destroy(void* opaque){
    if(!opaque)return;auto* h=static_cast<VioHandle*>(opaque);
    {std::lock_guard<std::mutex> lock(h->mutex);
     if(!h->shutdown&&h->system){try{h->system->Shutdown();}catch(...){}h->shutdown=true;}}
    delete h;
}
}
