// Real native regression for frame-bound stereo recovery and reference commits.
#include "ShortRecovery.h"
#include "CameraModels/Pinhole.h"
#include "CameraModels/KannalaBrandt8.h"
#include "MLPnPsolver.h"
#include "../src/egohand/vio/native/vio_reference_snapshot.h"
#include <cassert>
#include <iostream>
#include <limits>

using namespace ORB_SLAM3;

template<class F> void rejects(F f) {
    bool threw=false; try { f(); } catch(const std::exception&) { threw=true; }
    assert(threw);
}

void predictionReadinessRegression() {
    Frame last,current;
    IMU::Calib calibration(Sophus::SE3f(),.001f,.01f,.0001f,.001f);
    IMU::Preintegrated delta(IMU::Bias(),calibration);
    delta.IntegrateNewMeasurement(Eigen::Vector3f(0,0,IMU::GRAVITY_VALUE),Eigen::Vector3f::Zero(),.05f);
    last.SetPose(Sophus::SE3f());last.SetVelocity(Eigen::Vector3f::Zero());last.mTimeStamp=1;
    last.mImuBias=IMU::Bias();current.mpPrevFrame=&last;current.mTimeStamp=1.05;
    current.mImuCalib=calibration;current.mpImuPreintegratedFrame=&delta;
    auto ready=[&]{return ShortRecovery::PredictionReady(current,last,nullptr,nullptr,false);};
    assert(ready());
    current.mTimeStamp=1;assert(!ready()); // duplicate frame time
    current.mTimeStamp=1.2;assert(!ready()); // discontinuity
    current.mTimeStamp=1.05;delta.dT=.02f;assert(!ready()); // missing interval
    delta.dT=.05f;current.mImuCalib.mbIsSet=false;assert(!ready());
    current.mImuCalib=calibration;last.mImuBias.bax=std::numeric_limits<float>::quiet_NaN();assert(!ready());
    last.mImuBias=IMU::Bias();assert(ready());
    assert(!ShortRecovery::PredictionReady(current,last,nullptr,&delta,true)); // no KF for updated map
    delta.dP.x()=std::numeric_limits<float>::quiet_NaN();assert(!ready());
}

void boundaryRegression() {
    Map map, other;
    auto state=map.GetGeometryCommitState();
    assert(map.GetInertialReferenceBoundary().first==0);
    std::unique_lock<std::mutex> update(map.mMutexMapUpdate);
    std::unique_lock<std::mutex> wrong(other.mMutexMapUpdate);
    VioGeometryCommit::Guard writer(state);
    map.ApplyScaledRotation(update,Sophus::SE3f(),1,true,
        VioMapEventType::InitialInertialAlignment,&map,7,.5);
    const auto journal=map.GetVioMapEvents();
    const auto op=journal.events[journal.count-1].sequence;
    rejects([&]{map.CommitInertialReferenceBoundary(wrong,state,op);});
    rejects([&]{map.CommitInertialReferenceBoundary(update,other.GetGeometryCommitState(),op);});
    rejects([&]{map.CommitInertialReferenceBoundary(update,state,0);});
    map.CommitInertialReferenceBoundary(update,state,op);
    assert(map.GetInertialReferenceBoundary()==std::make_pair(std::uint64_t(1),op));
    rejects([&]{map.CommitInertialReferenceBoundary(update,state,op);});
    writer.Finish();
    rejects([&]{map.CommitInertialReferenceBoundary(update,state,op+1);});
    update.unlock();wrong.unlock();
    { VioGeometryCommit::Guard aborted(state); }
    assert(!state->Stable(state->sequence.load()));
    map.clear();
    assert(map.GetGeometryCommitState()!=state && map.GetInertialReferenceBoundary().first==0);
    update.lock();
    VioGeometryCommit::Guard next(map.GetGeometryCommitState());
    rejects([&]{map.CommitInertialReferenceBoundary(update,state,op);});
    map.ApplyScaledRotation(update,Sophus::SE3f(),1,true,VioMapEventType::MergeAlignment,&map);
    const auto invalid=map.GetVioMapEvents();
    rejects([&]{map.CommitInertialReferenceBoundary(update,map.GetGeometryCommitState(),invalid.events[invalid.count-1].sequence);});
    next.Finish();
}

std::unique_ptr<KeyFrame> observationOwner(Map& map, Pinhole& camera) {
    static ORBVocabulary vocabulary;
    static ORBextractor extractor(200,1.2f,8,20,7);
    cv::Mat image(600,800,CV_8UC1),depth(600,800,CV_32FC1,cv::Scalar(2));
    cv::RNG rng(42);rng.fill(image,cv::RNG::UNIFORM,0,256);
    cv::Mat K=(cv::Mat_<float>(3,3)<<200,0,400,0,200,300,0,0,1);
    cv::Mat distortion=cv::Mat::zeros(4,1,CV_32F);
    Frame f(image,depth,0,&extractor,&vocabulary,K,distortion,20,40,&camera);
    assert(f.N>0);f.SetPose(Sophus::SE3f());f.mvuRight[0]=f.mvKeysUn[0].pt.x-5;
    return std::make_unique<KeyFrame>(f,&map,nullptr);
}

void geometryRegression(bool fisheye) {
    Map map;
    Pinhole pinhole(std::vector<float>{200,200,400,300});
    // Keep all synthetic grid rays in the positive camera hemisphere.
    KannalaBrandt8 left(std::vector<float>{400,400,400,300,.01f,-.005f,.001f,0});
    KannalaBrandt8 right(std::vector<float>{405,399,402,298,-.02f,.003f,0,0});
    auto owner=observationOwner(map,pinhole);
    Frame f;
    Sophus::SE3f Tlr(Eigen::Matrix3f::Identity(),Eigen::Vector3f(.1f,0,0));
    if(fisheye) {
        ORBVocabulary vocabulary; ORBextractor a(200,1.2f,8,20,7),b(200,1.2f,8,20,7);
        cv::Mat image(600,800,CV_8UC1);cv::RNG rng(9);rng.fill(image,cv::RNG::UNIFORM,0,256);
        cv::Mat K=(cv::Mat_<float>(3,3)<<400,0,400,0,400,300,0,0,1),d=cv::Mat::zeros(4,1,CV_32F);
        f=Frame(image,image,1,&a,&b,&vocabulary,K,d,20,40,&left,&right,Tlr);
        f.mvKeys.clear();f.mvKeysUn.clear();f.mvKeysRight.clear();f.mvpMapPoints.clear();
        f.mvbOutlier.clear();f.mvuRight.clear();f.mvLeftToRightMatch.clear();
    }
    f.mnId=42;f.mTimeStamp=1.;f.Nleft=fisheye?72:-1;
    f.mpCamera=fisheye?static_cast<GeometricCamera*>(&left):&pinhole;
    f.mpCamera2=fisheye?&right:nullptr;
    f.SetPose(Sophus::SE3f()); f.mbf=20;f.mvLevelSigma2={1,4};
    f.mnMinX=0;f.mnMaxX=800;f.mnMinY=0;f.mnMaxY=600;
    std::vector<std::unique_ptr<MapPoint>> points;
    for(int i=0;i<72;++i) {
        cv::Point2f pixel(100+200*(i%4)+(i/12),100+200*((i/4)%3));
        Eigen::Vector3f ray=f.mpCamera->unprojectEig(pixel);ray*=(2+.2f*(i%5))/ray.z();
        points.push_back(std::make_unique<MapPoint>(ray,owner.get(),&map));
        points.back()->AddObservation(owner.get(),0);
        assert(points.back()->Observations()>=2);
        f.mvpMapPoints.push_back(points.back().get()); f.mvbOutlier.push_back(false);
        cv::KeyPoint key(pixel,1);key.octave=0;
        if(!fisheye) f.mvKeysUn.push_back(key);
        f.mvKeys.push_back(key);
        f.mvuRight.push_back(pixel.x-f.mbf/ray.z());
        f.mvLeftToRightMatch.push_back(i);
        const Eigen::Vector3f qright=Tlr.inverse()*ray;
        Eigen::Vector2f uv=right.project(qright);
        cv::KeyPoint rk(cv::Point2f(uv.x(),uv.y()),1);rk.octave=1;f.mvKeysRight.push_back(rk);
    }
    auto observe=[&]{ShortRecovery::BeginFrame(f);return ShortRecovery::Observe(f,&map,3,5,"unit");};
    const bool qualified=observe();
    if(!qualified) std::cerr<<"positive fisheye="<<fisheye<<" "<<ShortRecovery::Json()<<std::endl;
    assert(qualified);assert(ShortRecovery::current.stereo==72 && ShortRecovery::current.cells==12);
    assert(ShortRecovery::current.frame==42 && ShortRecovery::current.reset==3 && ShortRecovery::current.atlas==5);
    if(fisheye) {
        assert(f.mvKeysUn.empty()); // Real raw-fisheye storage convention.
        MLPnPsolver solver(f,f.mvpMapPoints);solver.SetRansacParameters(.99,50,300,6,.5,5.991);
        bool noMore=false;std::vector<bool> inliers;int count=0;Eigen::Matrix4f recovered;
        const bool solved=solver.iterate(300,noMore,inliers,count,recovered);
        assert(solved && count>=50 && recovered.allFinite());
        assert((recovered.block<3,1>(0,3).norm()<1e-3));
        assert((recovered.block<3,3>(0,0)-Eigen::Matrix3f::Identity()).norm()<1e-3);
        // Appended right-camera matches cannot be fed into this left-camera solver.
        auto rightOnly=f.mvpMapPoints;rightOnly.insert(rightOnly.end(),f.mvpMapPoints.begin(),f.mvpMapPoints.end());
        for(size_t i=0;i<72;++i) rightOnly[i]=nullptr;
        MLPnPsolver rejected(f,rightOnly);rejected.SetRansacParameters(.99,10,10,6,.5,5.991);
        assert(!rejected.iterate(10,noMore,inliers,count,recovered));
        const auto good=f.mvKeysRight;
        for(size_t i=0;i<points.size();++i) {
            const Eigen::Vector3f q=Tlr*points[i]->GetWorldPos();const Eigen::Vector2f uv=right.project(q);
            f.mvKeysRight[i].pt=cv::Point2f(uv.x(),uv.y());
        }
        assert(!observe() && ShortRecovery::current.stereo==0);f.mvKeysRight=good;
        for(auto& key:f.mvKeysRight) key.pt.x+=20;
        assert(!observe());for(auto& key:f.mvKeysRight) key.pt.x-=20;
    } else {
        for(auto& x:f.mvuRight) x+=20;
        assert(!observe());for(auto& x:f.mvuRight) x-=20;
    }
    f.mvLevelSigma2[0]=0;assert(!observe());f.mvLevelSigma2[0]=1;
    for(auto& point:points) point->SetWorldPos(Eigen::Vector3f(0,0,-2));
    assert(!observe() && ShortRecovery::current.inliers==0);
    auto state=map.GetGeometryCommitState();
    { VioGeometryCommit::Guard writer(state);assert(!observe() && !ShortRecovery::current.available);writer.Finish(); }
    { VioGeometryCommit::Guard aborted(state); }
    assert(!observe() && !ShortRecovery::current.available);
}

int main() {
    _putenv_s("EGOHAND_VIO_SHORT_RECOVERY","1");
    TrackingFrameSnapshot snapshot;
    snapshot.available=true;snapshot.consistent=true;snapshot.timestamp_sec=1;
    assert(!EgohandStableReferenceSnapshot(snapshot,1)); // map transition: null token
    snapshot.geometry_commit_state=std::make_shared<VioGeometryCommit::State>();
    assert(EgohandStableReferenceSnapshot(snapshot,1));
    assert(!EgohandStableReferenceSnapshot(snapshot,2));
    snapshot.inertial_reference_generation=1;
    assert(!EgohandStableReferenceSnapshot(snapshot,1)); // uncommitted generation
    snapshot.inertial_reference_operation=7;assert(EgohandStableReferenceSnapshot(snapshot,1));
    snapshot.consistent=false;assert(!EgohandStableReferenceSnapshot(snapshot,1));snapshot.consistent=true;
    {VioGeometryCommit::Guard writer(snapshot.geometry_commit_state);assert(!EgohandStableReferenceSnapshot(snapshot,1));writer.Finish();}
    assert(!EgohandStableReferenceSnapshot(snapshot,1)); // retired sequence
    snapshot.geometry_commit_sequence=snapshot.geometry_commit_state->sequence.load();
    assert(EgohandStableReferenceSnapshot(snapshot,1));
    {VioGeometryCommit::Guard aborted(snapshot.geometry_commit_state);}
    snapshot.geometry_commit_sequence=snapshot.geometry_commit_state->sequence.load();
    assert(!EgohandStableReferenceSnapshot(snapshot,1));
    predictionReadinessRegression();boundaryRegression();geometryRegression(false);geometryRegression(true);
    std::cout<<"PASS: native reference lifecycle; pinhole/fisheye geometry; inverted extrinsic, disparity, sigma, depth and writer-fence negatives\n";
}
