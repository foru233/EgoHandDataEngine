#include "vio_bridge.h"
#include <cstddef>
#include <VioCommitDiagnostics.h>
#include <System.h>
#include <VioSafeExp.h>
#include <VioWorldFrameRevision.h>
#include <VioDepthGate.h>
#include <VioEntryPrune.h>
#include <VioCheiralityGuard.h>
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
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
    bool map_export_enabled = false, localization_only = false;
    EgohandRecoveryCallback recovery_callback = nullptr;
    void* recovery_context = nullptr;
    int64_t epoch_ns = 0, last_frame_ns = 0;
    bool has_epoch = false, has_last_frame = false, shutdown = false;
    /* ORB-SLAM3 keeps one process-wide world-frame counter; subtract the value seen at
       creation so one process can host several sessions and still report deltas. */
    unsigned long long world_frame_baseline = 0;
    ORB_SLAM3::TrackingFrameSnapshot last_snapshot;
    int last_width=0,last_height=0;
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
    r->inertial_scale_finalized=0;
    r->imu_init_elapsed_sec=-1.0;
    r->frame_timestamp_sec=std::numeric_limits<double>::quiet_NaN();
    r->map_epoch=0;
    const float nan=std::numeric_limits<float>::quiet_NaN();
    std::fill(std::begin(r->T_world_from_rectified_left_camera),
              std::end(r->T_world_from_rectified_left_camera),nan);
}
}

extern "C" {
uint64_t EGOHAND_VIO_CALL egohand_vio_invalid_depth_checks() {
    return ORB_SLAM3::VioCheirality::rejectedStereoChecks.load();
}
uint64_t EGOHAND_VIO_CALL egohand_vio_invalid_depth_removed() {
    return ORB_SLAM3::VioCheirality::removedObservations.load();
}
uint64_t EGOHAND_VIO_CALL egohand_vio_invalid_depth_points() {
    return ORB_SLAM3::VioCheirality::invalidatedPoints.load();
}
EGOHAND_VIO_API uint64_t EGOHAND_VIO_CALL egohand_vio_numerical_fault_count() {
    return ORB_SLAM3::VioNumericalFaultCount();
}
#ifdef EGOHAND_VIO_TEST_HOOKS
EGOHAND_VIO_API void EGOHAND_VIO_CALL egohand_vio_test_inject_numerical_fault() {
    ORB_SLAM3::VioSafeExp(Eigen::Matrix3f::Identity(),
        Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN()));
}
#endif
const char* EGOHAND_VIO_CALL egohand_vio_api_version() {
    return "egohand-vio-bridge/0.2.0";
}
const char* EGOHAND_VIO_CALL egohand_vio_last_error() { return g_last_error.c_str(); }
int32_t EGOHAND_VIO_CALL egohand_vio_result_size() { return (int32_t)sizeof(EgohandVioResult); }

int32_t EGOHAND_VIO_CALL egohand_vio_map_appearance_header_size() {return sizeof(EgohandVioMapAppearanceHeader);}
int32_t EGOHAND_VIO_CALL egohand_vio_map_appearance_point_size() {return sizeof(EgohandVioMapAppearancePoint);}
int32_t EGOHAND_VIO_CALL egohand_vio_map_appearance_offset(int32_t kind,int32_t field) {
    const int32_t header[]={offsetof(EgohandVioMapAppearanceHeader,schema_version),offsetof(EgohandVioMapAppearanceHeader,struct_size),offsetof(EgohandVioMapAppearanceHeader,status),offsetof(EgohandVioMapAppearanceHeader,flags),offsetof(EgohandVioMapAppearanceHeader,point_count),offsetof(EgohandVioMapAppearanceHeader,sampled_count),offsetof(EgohandVioMapAppearanceHeader,capacity),offsetof(EgohandVioMapAppearanceHeader,source_kind),offsetof(EgohandVioMapAppearanceHeader,image_width),offsetof(EgohandVioMapAppearanceHeader,image_height),offsetof(EgohandVioMapAppearanceHeader,frame_id),offsetof(EgohandVioMapAppearanceHeader,map_id),offsetof(EgohandVioMapAppearanceHeader,reset_id),offsetof(EgohandVioMapAppearanceHeader,atlas_generation),offsetof(EgohandVioMapAppearanceHeader,map_revision),offsetof(EgohandVioMapAppearanceHeader,map_change_index),offsetof(EgohandVioMapAppearanceHeader,total_map_points),offsetof(EgohandVioMapAppearanceHeader,world_frame_revision),offsetof(EgohandVioMapAppearanceHeader,timestamp_ns),offsetof(EgohandVioMapAppearanceHeader,fx),offsetof(EgohandVioMapAppearanceHeader,fy),offsetof(EgohandVioMapAppearanceHeader,cx),offsetof(EgohandVioMapAppearanceHeader,cy),offsetof(EgohandVioMapAppearanceHeader,bf),offsetof(EgohandVioMapAppearanceHeader,reserved)};
    const int32_t point[]={offsetof(EgohandVioMapAppearancePoint,point_id),offsetof(EgohandVioMapAppearancePoint,position),offsetof(EgohandVioMapAppearancePoint,descriptor),offsetof(EgohandVioMapAppearancePoint,observations)};
    if (kind==0 && field>=0 && field<sizeof(header)/sizeof(header[0])) return header[field];
    if (kind==1 && field>=0 && field<sizeof(point)/sizeof(point[0])) return point[field];
    return -1;
}
int EGOHAND_VIO_CALL egohand_vio_get_map_appearance(void* opaque,uint64_t expected_frame_id,int64_t stamp,
    EgohandVioMapAppearanceHeader* out,int32_t out_size,EgohandVioMapAppearancePoint* points,int32_t point_size,int32_t capacity) {
    clear_error();
    if (!opaque || !out || out_size!=sizeof(*out) || !points || point_size!=sizeof(*points) || capacity<1 || capacity>4096) {
        set_error("map appearance ABI, argument or capacity mismatch");return -1;
    }
    *out=EgohandVioMapAppearanceHeader();out->schema_version=1;out->struct_size=sizeof(*out);out->capacity=capacity;
    out->flags=1;out->source_kind=1;
    auto* h=static_cast<VioHandle*>(opaque);std::lock_guard<std::mutex> lock(h->mutex);
    if (h->shutdown || !h->system) {set_error("map appearance after shutdown");return -1;}
    if (!h->has_last_frame || stamp!=h->last_frame_ns || expected_frame_id!=h->last_snapshot.frame_id || !h->last_snapshot.consistent) {
        out->status=2;return 1;
    }
    try {
        ORB_SLAM3::MapAppearanceSnapshot sample;
        const bool ok=h->system->GetMapAppearanceSnapshot(sample,capacity);
        const auto& s=sample.frame;
        out->frame_id=s.frame_id;out->map_id=s.map_id;out->reset_id=s.reset_generation;out->atlas_generation=s.atlas_generation;
        out->map_revision=s.big_change_revision;out->map_change_index=s.map_change_index;out->timestamp_ns=stamp;
        out->image_width=h->last_width;out->image_height=h->last_height;
        out->status=sample.status;out->sampled_count=sample.sampled_count;out->total_map_points=sample.total_map_points;
        if (sample.total_map_points>sample.sampled_count) out->flags|=2;
        out->world_frame_revision=sample.world_revision>=h->world_frame_baseline ? sample.world_revision-h->world_frame_baseline : 0;
        out->fx=sample.fx;out->fy=sample.fy;out->cx=sample.cx;out->cy=sample.cy;out->bf=sample.bf;
        if (!ok) return 1;
        if (s.frame_id!=expected_frame_id || s.timestamp_sec!=h->last_snapshot.timestamp_sec || s.map_id!=h->last_snapshot.map_id ||
            s.reset_generation!=h->last_snapshot.reset_generation || s.atlas_generation!=h->last_snapshot.atlas_generation ||
            s.big_change_revision!=h->last_snapshot.big_change_revision || s.map_change_index!=h->last_snapshot.map_change_index) {
            out->status=2;return 1;
        }
        if (sample.points.size()>static_cast<size_t>(capacity)) {out->status=4;return 1;}
        for (size_t i=0;i<sample.points.size();++i) {
            const auto& p=sample.points[i];points[i]=EgohandVioMapAppearancePoint();points[i].point_id=p.id;
            std::copy(p.position.begin(),p.position.end(),points[i].position);
            std::copy(p.descriptor.begin(),p.descriptor.end(),points[i].descriptor);points[i].observations=p.observations;
        }
        out->point_count=static_cast<uint32_t>(sample.points.size());return 0;
    } catch(const std::exception& e) {set_error(e.what());return -1;}
      catch(...) {set_error("map appearance capture failed");return -1;}
}

int32_t EGOHAND_VIO_CALL egohand_vio_snapshot_size() { return (int32_t)sizeof(EgohandVioFrameSnapshot); }
int32_t EGOHAND_VIO_CALL egohand_vio_map_event_frame_size() { return (int32_t)sizeof(EgohandVioMapEventFrame); }
uint64_t EGOHAND_VIO_CALL egohand_vio_entry_pruned() {
    return ORB_SLAM3::EntryPruneCounter();
}

uint64_t EGOHAND_VIO_CALL egohand_vio_depth_gate_rejected() {
    return ORB_SLAM3::Rejected();
}

uint64_t EGOHAND_VIO_CALL egohand_vio_point_clamp_count() {
    return ORB_SLAM3::VioWorldFrame::PointClampCounter();
}

int32_t EGOHAND_VIO_CALL egohand_vio_world_frame_info_size() { return (int32_t)sizeof(EgohandVioWorldFrameInfo); }

/* Additive tokens only: existing names keep their meaning and new ones are appended,
   so a caller can probe for optional features instead of sniffing the version string. */
const char* EGOHAND_VIO_CALL egohand_vio_capabilities() {
    return "map_appearance_v1;result;snapshot;body_state;recovery_callback;start_new_map;world_frame_revision;cheirality_guard;partial_map_event_frame_v1";
}

int EGOHAND_VIO_CALL egohand_vio_get_map_event_frame(void* opaque, EgohandVioMapEventFrame* out, int32_t out_size) {
    clear_error();
    if(!opaque || !out || out_size != sizeof(EgohandVioMapEventFrame)) {
        set_error("map event frame size mismatch or null argument"); return -1;
    }
    auto* h=static_cast<VioHandle*>(opaque); std::lock_guard<std::mutex> lock(h->mutex);
    if(h->shutdown||!h->system) { set_error("ORB-SLAM3 is shut down"); return -1; }
    const auto& s=h->last_snapshot;
    if(!s.available || !h->has_last_frame) return 1;
    *out={}; out->schema_version=1; out->struct_size=sizeof(*out);
    out->frame_id=s.frame_id; out->map_id=s.map_id; out->reset_id=s.reset_generation;
    // The integer is the original bridge request, never reconstructed from seconds.
    out->frame_timestamp_ns=h->last_frame_ns;
    out->snapshot_consistent=s.consistent;
    out->frame_bound=s.consistent && s.timestamp_sec==seconds_from_epoch(h->last_frame_ns,h->epoch_ns);
    out->coverage=out->frame_bound ? s.map_events.coverage : 0;
    if(!out->frame_bound) return 0;
    out->event_count=s.map_events.count;
    out->overwritten_count=s.map_events.overwritten_count;
    for(std::uint32_t i=0;i<out->event_count && i<EGOHAND_VIO_MAP_EVENT_CAPACITY;++i){
        const auto& src=s.map_events.events[i]; auto& dst=out->events[i];
        dst.sequence=src.sequence; dst.map_id=src.map_id; dst.source_keyframe_id=src.source_keyframe_id;
        dst.revision_before=src.revision_before; dst.revision_after=src.revision_after;
        dst.change_before=src.change_before; dst.change_after=src.change_after;
        dst.source_keyframe_time_sec=src.source_keyframe_time_sec; dst.scale=src.scale;
        std::copy(src.rotation.begin(),src.rotation.end(),dst.rotation);
        std::copy(src.translation.begin(),src.translation.end(),dst.translation);
        dst.type=static_cast<uint32_t>(src.type);
        dst.map_update_lock_matches_caller=src.map_update_lock_matches_caller;
        dst.reset_generation_known=src.reset_generation_known;
    }
    return 0;
}

int EGOHAND_VIO_CALL egohand_vio_get_world_frame_info(void* opaque, EgohandVioWorldFrameInfo* out, int32_t out_size) {
    clear_error();
    if(!opaque || !out || out_size != sizeof(EgohandVioWorldFrameInfo)) {
        set_error("world frame info size mismatch or null argument");return -1;
    }
    auto* h=static_cast<VioHandle*>(opaque);std::lock_guard<std::mutex> lock(h->mutex);
    // Read-only history: it stays reportable after shutdown, unlike the live snapshot.
    const ORB_SLAM3::VioWorldFrame::Change change=ORB_SLAM3::VioWorldFrame::Last();
    *out={};
    out->schema_version=1;out->struct_size=sizeof(*out);out->available=1;
    // -1 = no free-gauge full BA has been fitted yet; the enum's 0 is Verdict::Reanchor,
    // so an unevaluated state must not be reported as a re-anchor.
    out->verdict=change.evaluations?change.verdict:-1;out->last_keyframes=change.keyframes;
    out->revision=change.revision>h->world_frame_baseline?change.revision-h->world_frame_baseline:0;
    out->detections=change.detections;out->evaluations=change.evaluations;
    out->rejected_nonrigid=change.rejected_nonrigid;
    out->map_id=change.map_id;out->last_max_keyframe_id=change.max_keyframe_id;
    out->last_timestamp_sec=change.timestamp_sec;
    out->last_translation_m=change.translation_m;out->last_rotation_deg=change.rotation_deg;
    out->last_scale=change.scale;
    out->last_residual_p50_m=change.residual_p50_m;out->last_residual_p95_m=change.residual_p95_m;
    out->last_residual_p99_m=change.residual_p99_m;out->last_residual_max_m=change.residual_max_m;
    out->last_residual_limit_m=change.residual_limit_m;
    return 0;
}

int EGOHAND_VIO_CALL egohand_vio_get_frame_snapshot(void* opaque, EgohandVioFrameSnapshot* out, int32_t out_size) {
    clear_error();
    if(!opaque || !out || out_size != sizeof(EgohandVioFrameSnapshot)) {
        set_error("snapshot size mismatch or null argument");return -1;
    }
    auto* h=static_cast<VioHandle*>(opaque);std::lock_guard<std::mutex> lock(h->mutex);
    if(h->shutdown||!h->system){set_error("ORB-SLAM3 is shut down");return -1;}
    auto s=h->last_snapshot;
    if (s.geometry_commit_state && !s.geometry_commit_state->Stable(s.geometry_commit_sequence))
        s.consistent=s.pose_valid=s.body_valid=false;
    if(!s.available)return 1;
    *out={};out->schema_version=1;out->struct_size=sizeof(*out);
    out->frame_id=s.frame_id;out->map_id=s.map_id;out->map_revision=s.big_change_revision;
    out->reset_id=s.reset_generation;out->map_change_index=s.map_change_index;
    out->world_frame_id=(s.map_id<<40)|(s.reset_generation<<20)|s.big_change_revision;
    out->timestamp_ns=h->last_frame_ns;out->snapshot_consistent=s.consistent;
    out->tracking_state=s.tracking_state;out->pose_valid=s.pose_valid&&s.consistent;
    out->imu_initialized=s.imu_initialized;out->ba1_complete=s.ba1_complete;out->ba2_complete=s.ba2_complete;
    out->body_valid=s.body_valid&&s.consistent;
    out->left_features=s.left_features;out->right_features=s.right_features;
    out->stereo_valid_depths=s.stereo_valid_depths;out->tracked_inliers=s.tracked_inliers;
    for(int r=0;r<4;++r)for(int c=0;c<4;++c){
        out->T_world_from_rectified_left_camera[r*4+c]=s.world_from_camera(r,c);
        out->T_world_from_body[r*4+c]=s.world_from_body(r,c);
    }
    for(int i=0;i<3;++i)out->velocity_world_mps[i]=s.velocity_world(i);
    for(int i=0;i<6;++i)out->accel_gyro_bias[i]=s.bias(i);
    // Recheck after copying scalar/value fields too. No map pointer or map lock is needed.
    if (s.geometry_commit_state && !s.geometry_commit_state->Stable(s.geometry_commit_sequence))
        out->snapshot_consistent=out->pose_valid=out->body_valid=0;
    return 0;
}

int EGOHAND_VIO_CALL egohand_vio_set_recovery_callback(void* opaque,EgohandRecoveryCallback callback,void* context) {
    clear_error();if(!opaque){set_error("null handle");return -1;}
    auto* h=static_cast<VioHandle*>(opaque);std::lock_guard<std::mutex> lock(h->mutex);
    if(h->shutdown||!h->system){set_error("ORB-SLAM3 is shut down");return -1;}
    h->recovery_callback=callback;h->recovery_context=context;
    if(!callback){h->system->SetRecoveryCandidateCheck({});return 0;}
    h->system->SetRecoveryCandidateCheck([h](const Eigen::Matrix4f& T,unsigned long long epoch){
        float value[16];for(int r=0;r<4;++r)for(int c=0;c<4;++c)value[r*4+c]=T(r,c);
        return h->recovery_callback(value,static_cast<uint64_t>(epoch),h->recovery_context);
    });
    return 0;
}

int EGOHAND_VIO_CALL egohand_vio_get_body_state(void* opaque,float* transform,float* velocity,float* bias,int* ba2) {
    clear_error();
    if(!opaque||!transform||!velocity||!bias||!ba2){set_error("null body state argument");return -1;}
    auto* h=static_cast<VioHandle*>(opaque);std::lock_guard<std::mutex> lock(h->mutex);
    if(h->shutdown||!h->system){set_error("ORB-SLAM3 is shut down");return -1;}
    // The compatibility getter consumes the same immutable frame snapshot.
    const auto& s=h->last_snapshot;
    if(!s.available||!s.consistent||!s.body_valid)return 1;
    const auto& T=s.world_from_body;const auto& v=s.velocity_world;const auto& b=s.bias;
    const bool ready=s.ba2_complete;
    for(int r=0;r<4;++r)for(int c=0;c<4;++c)transform[r*4+c]=T(r,c);
    for(int i=0;i<3;++i)velocity[i]=v(i);
    for(int i=0;i<6;++i)bias[i]=b(i);
    *ba2=ready?1:0;return 0;
}

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
        const char* stereoOnly=std::getenv("EGOHAND_VIO_STEREO_ONLY");
        const bool diagnosticStereo=stereoOnly&&std::string(stereoOnly)=="1";
        std::cout << "Egohand sensor: " << (diagnosticStereo?"STEREO (diagnostic)":"IMU_STEREO") << std::endl;
        h->system=std::make_unique<ORB_SLAM3::System>(
            vocabulary_path,settings_path,diagnosticStereo?ORB_SLAM3::System::STEREO:ORB_SLAM3::System::IMU_STEREO,use_viewer!=0);
        h->world_frame_baseline=ORB_SLAM3::VioWorldFrame::Revision();
        *out_handle=h.release();
        return 0;
    } catch(const std::exception& e){set_error(e.what());return -1;}
      catch(...){set_error("unknown exception creating ORB-SLAM3");return -1;}
}

int EGOHAND_VIO_CALL egohand_vio_create_map(
    const char* vocabulary_path, const char* settings_path, int localize_only, void** out_handle) {
    clear_error();
    if(!out_handle || !vocabulary_path || !settings_path) {
        set_error("Missing map session parameters");
        return -1;
    }
    *out_handle = nullptr;
    try {
        cv::FileStorage settings(settings_path, cv::FileStorage::READ);
        const auto load = settings["System.LoadAtlasFromFile"];
        const auto save = settings["System.SaveAtlasToFile"];
        const auto loop = settings["loopClosing"];
        if(!settings.isOpened() || !save.empty() || loop.empty() || static_cast<int>(loop) != 0 ||
           (localize_only != 0) != (!load.empty() && !static_cast<std::string>(load).empty()))
            throw std::runtime_error("Invalid Atlas session settings");
        auto h = std::make_unique<VioHandle>();
        h->map_export_enabled = !localize_only;
        h->localization_only = localize_only != 0;
        const auto sensor = localize_only ? ORB_SLAM3::System::STEREO : ORB_SLAM3::System::IMU_STEREO;
        h->system = std::make_unique<ORB_SLAM3::System>(vocabulary_path, settings_path, sensor, false);
        h->world_frame_baseline = ORB_SLAM3::VioWorldFrame::Revision();
        *out_handle = h.release();
        return 0;
    } catch(const std::exception& e) { set_error(e.what()); return -1; }
      catch(...) { set_error("Could not create Atlas session"); return -1; }
}

int EGOHAND_VIO_CALL egohand_vio_export_map(
    void* opaque, const char* atlas_path, const char* pose_path) {
    clear_error();
    if(!opaque || !atlas_path || !pose_path) { set_error("Missing map export parameters"); return -1; }
    auto* h = static_cast<VioHandle*>(opaque);
    std::lock_guard<std::mutex> lock(h->mutex);
    if(h->shutdown || !h->map_export_enabled) {
        set_error("This session cannot export an Atlas");
        return -1;
    }
    if(ORB_SLAM3::VioNumericalFaultCount()) {
        set_error("VIO_NUMERICAL_FAULT: invalid inertial rotation; restart worker before exporting");
        return -1;
    }
    h->shutdown = true;
    try {
        h->system->ExportMap(atlas_path, pose_path);
        return 0;
    } catch(const std::exception& e) { set_error(e.what()); return -1; }
      catch(...) { set_error("Could not export Atlas"); return -1; }
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
        h->last_snapshot=ORB_SLAM3::TrackingFrameSnapshot();
        if(ORB_SLAM3::VioNumericalFaultCount()) {
            set_error("VIO_NUMERICAL_FAULT: invalid inertial rotation; restart worker");
            return -1;
        }
        h->system->TrackStereo(left,right,frame_sec,measurements);
        if(ORB_SLAM3::VioNumericalFaultCount()) {
            set_error("VIO_NUMERICAL_FAULT: invalid inertial rotation; pose rejected, restart worker");
            return -1;
        }
        h->system->GetTrackingFrameSnapshot(h->last_snapshot);
        const auto& snapshot=h->last_snapshot;
        ORB_SLAM3::VioCommitDiagnostics::BridgeIdentity(snapshot,frame_ns,h->epoch_ns);
        out->tracking_state=snapshot.tracking_state;
        out->is_lost=snapshot.tracking_state==4?1:0;
        out->imu_init_elapsed_sec=h->localization_only?0.0:snapshot.imu_init_elapsed_sec;
        out->imu_initialized=(!h->localization_only&&snapshot.imu_initialized)?1:0;
        out->frame_timestamp_sec=frame_sec;
        if(snapshot.map_id>=(1ULL<<24)||snapshot.reset_generation>=(1ULL<<20)||snapshot.big_change_revision>=(1ULL<<20)) {
            set_error("snapshot map epoch counter overflow");return -1;
        }
        out->map_epoch=(snapshot.map_id<<40)|(snapshot.reset_generation<<20)|snapshot.big_change_revision;
        // Legacy field name retained for ABI: BA2 completion, not metric accuracy.
        out->inertial_scale_finalized=snapshot.ba2_complete?1:0;
        out->pose_valid=(snapshot.available&&snapshot.consistent&&snapshot.pose_valid)?1:0;
        if(out->pose_valid){
            for(int r=0;r<4;++r)for(int c=0;c<4;++c)
                out->T_world_from_rectified_left_camera[r*4+c]=snapshot.world_from_camera(r,c);
        }
        if(!out->pose_valid){
            const float nan=std::numeric_limits<float>::quiet_NaN();
            std::fill(std::begin(out->T_world_from_rectified_left_camera),
                      std::end(out->T_world_from_rectified_left_camera),nan);
        }
        h->last_frame_ns=frame_ns;h->has_last_frame=true;h->last_width=width;h->last_height=height;
        return 0;
    } catch(const std::exception& e){set_error(e.what());return -1;}
      catch(...){set_error("unknown exception in TrackStereo");return -1;}
}

int EGOHAND_VIO_CALL egohand_vio_start_new_map(void* opaque){
    clear_error();if(!opaque){set_error("null handle");return -1;}
    auto* h=static_cast<VioHandle*>(opaque);std::lock_guard<std::mutex> lock(h->mutex);
    if(h->shutdown||!h->system){set_error("ORB-SLAM3 is shut down");return -1;}
    if(h->localization_only || h->map_export_enabled){set_error("Atlas sessions cannot start a new map");return -1;}
    // Schedule the active-map reset through ORB's synchronized tracking path.
    // It discards the failed active map and starts a fresh coordinate frame.
    try{h->system->ResetActiveMap();return 0;}
    catch(const std::exception& e){set_error(e.what());return -1;}
    catch(...){set_error("could not start a new map");return -1;}
}

int EGOHAND_VIO_CALL egohand_vio_reset(void* opaque){
    clear_error();if(!opaque){set_error("null handle");return -1;}
    auto* h=static_cast<VioHandle*>(opaque);std::lock_guard<std::mutex> lock(h->mutex);
    if(h->shutdown||!h->system){set_error("ORB-SLAM3 is shut down");return -1;}
    if(h->localization_only || h->map_export_enabled){set_error("Atlas sessions cannot be reset");return -1;}
    try{h->system->Reset();h->has_last_frame=false;h->last_frame_ns=0;h->has_epoch=false;h->epoch_ns=0;
        h->last_snapshot=ORB_SLAM3::TrackingFrameSnapshot();return 0;}
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
    ORB_SLAM3::VioCommitDiagnostics::Flush();
}
}
