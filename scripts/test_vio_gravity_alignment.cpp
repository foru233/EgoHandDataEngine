#include "VioGravityAlignment.h"
#include <cassert>
#include <iostream>
#include <limits>

int main() {
    Eigen::Matrix3f rotation;
    const Eigen::Vector3f gravity(0,0,-1);
    const Eigen::Vector3f directions[] = {
        {0,0,-1}, {0,0,1}, {1e-8f,0,-1}, {0,1e-8f,1}, {0.3f,-0.2f,-0.9f}
    };
    for (const auto& direction: directions) {
        assert(ORB_SLAM3::VioGravityAlignment(direction,rotation));
        assert((rotation*gravity-direction.normalized()).norm()<2e-5f);
        assert((rotation.transpose()*rotation-Eigen::Matrix3f::Identity()).norm()<2e-5f);
        assert(std::abs(rotation.determinant()-1)<2e-5f);
    }
    assert(!ORB_SLAM3::VioGravityAlignment(Eigen::Vector3f::Zero(),rotation));
    assert(!ORB_SLAM3::VioGravityAlignment(Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(),0,1),rotation));
    assert(!ORB_SLAM3::VioGravityAlignment(Eigen::Vector3f(0,std::numeric_limits<float>::infinity(),1),rotation));
    std::cout << "gravity alignment: 5 finite rotations and 3 invalid directions passed\n";
}
