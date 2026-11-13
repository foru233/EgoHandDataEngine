#include "G2oTypes.h"
#include "CameraModels/Pinhole.h"
#include "VioCheiralityGuard.h"
#include <cassert>
#include <iostream>
#include <limits>

using namespace ORB_SLAM3;

std::unique_ptr<KeyFrame> testKeyFrame(Map& map,Pinhole& camera,bool stereo=false) {
    static ORBVocabulary vocabulary;
    static ORBextractor extractor(200,1.2f,8,20,7);
    cv::Mat image(600,800,CV_8UC1),depth(600,800,CV_32FC1,cv::Scalar(2));
    cv::RNG rng(42);rng.fill(image,cv::RNG::UNIFORM,0,256);
    cv::Mat calibration=(cv::Mat_<float>(3,3)<<200,0,400,0,200,300,0,0,1);
    cv::Mat distortion=cv::Mat::zeros(4,1,CV_32F);
    Frame frame(image,depth,0,&extractor,&vocabulary,calibration,distortion,13,40,&camera);
    assert(frame.N>0);frame.SetPose(Sophus::SE3f());
    frame.mvuRight[0]=stereo ? frame.mvKeysUn[0].pt.x-.5f : -1.f;
    return std::make_unique<KeyFrame>(frame,&map,nullptr);
}

void mapCommitRegression(Pinhole& camera) {
    Map map;
    auto stereoOwner=testKeyFrame(map,camera,true),monoOwner=testKeyFrame(map,camera),farOwner=testKeyFrame(map,camera,true);
    KeyFrame &stereo=*stereoOwner,&mono=*monoOwner,&far=*farOwner;
    MapPoint a(Eigen::Vector3f(0,0,-10),&stereo,&map),
             b(Eigen::Vector3f(0,0,-10),&mono,&map),
             c(Eigen::Vector3f(0,0,1e6f),&far,&map);
    auto connect=[&](MapPoint& point,KeyFrame& keyframe) {
        map.AddMapPoint(&point);point.AddObservation(&keyframe,0);keyframe.AddMapPoint(&point,0);
    };
    connect(a,stereo);connect(b,mono);connect(c,far);
    {
        std::unique_lock<std::mutex> update(map.mMutexMapUpdate);
        assert(VioCheirality::CullCommittedPerspectiveObservations(&map)==2);
    }
    assert(a.isBad() && b.isBad() && !c.isBad());
    assert(stereo.GetMapPoint(0)==nullptr && mono.GetMapPoint(0)==nullptr);
    assert(far.GetMapPoint(0)==&c && c.Observations()==2);

    // One invalid association must not delete a point with enough valid views.
    auto behindOwner=testKeyFrame(map,camera),front1Owner=testKeyFrame(map,camera),
         front2Owner=testKeyFrame(map,camera),front3Owner=testKeyFrame(map,camera);
    KeyFrame &behind=*behindOwner,&front1=*front1Owner,&front2=*front2Owner,&front3=*front3Owner;
    behind.SetPose(Sophus::SE3f(Eigen::Matrix3f::Identity(),Eigen::Vector3f(0,0,-3)));
    MapPoint mixed(Eigen::Vector3f(0,0,1),&behind,&map);
    connect(mixed,behind);connect(mixed,front1);connect(mixed,front2);connect(mixed,front3);
    mixed.UpdateNormalAndDepth();
    assert(std::abs(mixed.GetNormal().z()-.5f)<1e-6f);
    const float oldRange=mixed.GetMaxDistanceInvariance();
    {
        std::unique_lock<std::mutex> update(map.mMutexMapUpdate);
        assert(VioCheirality::CullCommittedPerspectiveObservations(&map)==1);
    }
    assert(!mixed.isBad() && mixed.Observations()==3 && !mixed.IsInKeyFrame(&behind));
    assert(behind.GetMapPoint(0)==nullptr && front1.GetMapPoint(0)==&mixed);
    assert(std::abs(mixed.GetNormal().z()-1.f)<1e-6f);
    assert(!mixed.GetDescriptor().empty() && mixed.GetReferenceKeyFrame()!=&behind);
    assert(mixed.GetMaxDistanceInvariance()<oldRange*.75f);

    // Corrupt geometry is NOT hidden by marking it a geometric outlier.
    c.SetWorldPos(Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN()));
    const auto before=VioNumericalFaultCount();
    {
        std::unique_lock<std::mutex> update(map.mMutexMapUpdate);
        VioCheirality::CullCommittedPerspectiveObservations(&map);
    }
    assert(VioNumericalFaultCount()>before && !c.isBad());
}

int main() {
    Pinhole camera(std::vector<float>{200,200,400,300});
    ImuCamPose transform;
    transform.Rcw={Eigen::Matrix3d::Identity()};
    transform.tcw={Eigen::Vector3d::Zero()};
    transform.pCamera={&camera}; transform.bf=13;
    VertexPose pose;pose.setEstimate(transform);
    g2o::VertexSBAPointXYZ point;
    EdgeStereo edge;
    edge.setVertex(0,&point);edge.setVertex(1,&pose);
    edge.setInformation(Eigen::Matrix3d::Identity());
    // This is the real bug: a behind-camera point can have ZERO pixel error.
    point.setEstimate(Eigen::Vector3d(.3,.2,-1000));
    edge.setMeasurement(transform.ProjectStereo(point.estimate()));
    edge.computeError();
    assert(edge.chi2()<1e-20 && !edge.isDepthPositive());
    assert(VioCheirality::RejectStereo(edge.isDepthPositive()));
    // A physically valid positive-disparity measurement also accepts the
    // wrong hemisphere under a pixel-only chi2 gate: 0.263 < 7.815.
    point.setEstimate(Eigen::Vector3d(0,0,-1000));
    edge.setMeasurement(Eigen::Vector3d(400,300,399.5));edge.computeError();
    assert(edge.chi2()<7.815 && !edge.isDepthPositive());
    EdgeStereoOnlyPose unary(point.estimate().cast<float>());
    unary.setVertex(0,&pose);
    assert(!unary.isDepthPositive());
    // Distance is not a rejection rule. A positive point can be arbitrarily far.
    for (double depth: {1e-6,1.,1e6}) {
        point.setEstimate(Eigen::Vector3d(0,0,depth));
        assert(edge.isDepthPositive());
        assert(!VioCheirality::RejectStereo(edge.isDepthPositive()));
    }
    point.setEstimate(Eigen::Vector3d(0,0,0));assert(!edge.isDepthPositive());
    point.setEstimate(Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),0,1));
    assert(!edge.isDepthPositive());
    // Depth must be checked in the observing camera, not at the world origin.
    transform.tcw[0]=Eigen::Vector3d(0,0,-2);pose.setEstimate(transform);
    point.setEstimate(Eigen::Vector3d(0,0,1));assert(!edge.isDepthPositive());
    mapCommitRegression(camera);
    std::cout << "real stereo edges: zero-error negative depth rejected; positive far/near, zero, NaN and camera transform passed\n";
}
