#include "VioGaugeMath.h"
#include <Eigen/LU>
#include <cassert>
#include <iostream>

using namespace ORB_SLAM3;
Eigen::Matrix3d Rotation(const Eigen::Vector3d& vector) {
    const double angle=vector.norm();
    return angle>0 ? Eigen::AngleAxisd(angle,vector/angle).toRotationMatrix() : Eigen::Matrix3d::Identity();
}
int main() {
    const Eigen::Matrix3d rotation=Rotation({0.31,-0.22,0.67});
    const Eigen::Vector3d position(1.7,-2.1,0.8);
    const auto reference=VioGauge::MakeReference(rotation,position);
    const auto jacobian=VioGauge::Jacobian(rotation,reference);
    assert(VioGauge::ChartValid(rotation,reference));
    assert(jacobian.fullPivLu().rank()==4);
    VioGauge::PoseJacobian numerical;
    const double epsilon=1e-6;
    for (int col=0;col<6;++col) {
        Eigen::Vector3d dr=Eigen::Vector3d::Zero(),dp=Eigen::Vector3d::Zero();
        if(col<3) dr[col]=epsilon; else dp[col-3]=epsilon;
        numerical.col(col)=(VioGauge::Residual(rotation*Rotation(dr),position+rotation*dp,reference)-
                           VioGauge::Residual(rotation*Rotation(-dr),position-rotation*dp,reference))/(2*epsilon);
    }
    assert((jacobian-numerical).cwiseAbs().maxCoeff()<1e-8);
    // The edge has full rank on precisely the four unobservable group directions.
    Eigen::Matrix<double,6,4> group=Eigen::Matrix<double,6,4>::Zero();
    group.block<3,3>(3,0)=rotation.transpose();
    group.block<3,1>(0,3)=rotation.transpose()*Eigen::Vector3d::UnitZ();
    group.block<3,1>(3,3)=rotation.transpose()*Eigen::Vector3d::UnitZ().cross(position);
    assert((jacobian*group).fullPivLu().rank()==4);
    // An arbitrary solved map can be moved along its gauge orbit until edge=0.
    const Eigen::Matrix3d solved=Rotation({-0.12,0.16,0.8});
    const Eigen::Vector3d solvedPosition(-128,153,-196);
    const double yaw=reference.heading-std::atan2(solved(1,reference.bodyAxis),solved(0,reference.bodyAxis));
    const Eigen::Matrix3d gauge=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d shift=reference.position-gauge*solvedPosition;
    assert(VioGauge::Residual(gauge*solved,gauge*solvedPosition+shift,reference).norm()<1e-12);
    // Reprojection and preintegrated inertial residuals stay invariant under xyz/yaw.
    const Eigen::Vector3d p1(1,2,3),p2(1.2,1.9,3.1),v1(0.2,-0.1,0.4),v2(0.3,-0.2,0.1),point(3,5,8),gravity(0,0,-9.81);
    const Eigen::Matrix3d r1=rotation,r2=solved; const double dt=0.04;
    const Eigen::Vector3d visual=r1.transpose()*(point-p1);
    const Eigen::Vector3d visual2=(gauge*r1).transpose()*((gauge*point+shift)-(gauge*p1+shift));
    assert((visual-visual2).norm()<1e-12);
    const Eigen::Vector3d rp=r1.transpose()*(p2-p1-v1*dt-0.5*gravity*dt*dt);
    const Eigen::Vector3d rp2=(gauge*r1).transpose()*((gauge*p2+shift)-(gauge*p1+shift)-(gauge*v1)*dt-0.5*gravity*dt*dt);
    const Eigen::Vector3d rv=r1.transpose()*(v2-v1-gravity*dt);
    const Eigen::Vector3d rv2=(gauge*r1).transpose()*(gauge*v2-gauge*v1-gravity*dt);
    assert((rp-rp2).norm()<1e-12 && (rv-rv2).norm()<1e-12);
    assert((r1.transpose()*r2-(gauge*r1).transpose()*(gauge*r2)).norm()<1e-12);
    std::cout << "gauge constraint: analytic Jacobian, rank4, gauge orbit and visual/IMU invariants passed\n";
}
