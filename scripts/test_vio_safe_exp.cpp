// Geometry/numerics test for VioSafeExp: identical to SO3::exp for finite inputs,
// and never throws for NaN/inf while returning the untouched base rotation.
// Build: see scripts/build_test_vio_safe_exp.cmd
#include <cmath>
#include <cstdio>
#include <limits>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "VioSafeExp.h"

using ORB_SLAM3::VioSafeExp;

static int failures = 0;
static void expect(bool ok, const char* name) {
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++failures;
}

int main() {
    const Eigen::Matrix3f base = Eigen::AngleAxisf(0.3f, Eigen::Vector3f(1, 2, 3).normalized()).toRotationMatrix();

    // 1. Finite omega must match the Sophus/Eigen exponential exactly.
    const Eigen::Vector3f omega(0.02f, -0.05f, 0.11f);
    const Eigen::Matrix3f reference =
        base * Eigen::Quaternionf(Eigen::AngleAxisf(omega.norm(), omega.normalized())).toRotationMatrix();
    bool ok = false;
    const Eigen::Matrix3f value = VioSafeExp(base, omega, &ok);
    expect(ok, "finite omega reports ok");
    expect((value - reference).cwiseAbs().maxCoeff() < 1e-5f, "finite omega matches SO3::exp");

    // 2. Zero rotation vector is the identity: base must come back unchanged.
    ok = false;
    const Eigen::Matrix3f zero = VioSafeExp(base, Eigen::Vector3f::Zero(), &ok);
    expect(ok, "zero omega reports ok");
    expect((zero - base).cwiseAbs().maxCoeff() < 1e-6f, "zero omega returns base");

    // 3. NaN omega: no throw, base preserved, ok=false.
    Eigen::Vector3f nanOmega = Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN());
    ok = true;
    const Eigen::Matrix3f nanResult = VioSafeExp(base, nanOmega, &ok);
    expect(!ok, "NaN omega reports not-ok");
    expect((nanResult - base).cwiseAbs().maxCoeff() == 0.0f, "NaN omega preserves base");

    // 4. Infinite omega: same contract.
    Eigen::Vector3f infOmega = Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
    ok = true;
    const Eigen::Matrix3f infResult = VioSafeExp(base, infOmega, &ok);
    expect(!ok, "inf omega reports not-ok");
    expect((infResult - base).cwiseAbs().maxCoeff() == 0.0f, "inf omega preserves base");

    // 5. Absurdly large but finite omega must not produce garbage.
    Eigen::Vector3f huge = Eigen::Vector3f::Constant(1e30f);
    ok = true;
    const Eigen::Matrix3f hugeResult = VioSafeExp(base, huge, &ok);
    expect(!ok, "huge omega reports not-ok");
    expect(hugeResult.allFinite(), "huge omega returns finite rotation");

    // 6. NaN injected into the base rotation must also be refused.
    Eigen::Matrix3f badBase = base;
    badBase(0, 0) = std::numeric_limits<float>::quiet_NaN();
    ok = true;
    const Eigen::Matrix3f badBaseResult = VioSafeExp(badBase, omega, &ok);
    expect(!ok, "NaN base reports not-ok");
    expect(!badBaseResult.allFinite(), "NaN base returns base untouched");

    expect(ORB_SLAM3::VioNumericalFaultCount() == 4, "all four rejected corrections set the shared fault");
    const auto faults = ORB_SLAM3::VioNumericalFaultCount();
    VioSafeExp(base, Eigen::Vector3f::Zero());
    expect(ORB_SLAM3::VioNumericalFaultCount() == faults, "valid correction does not clear a prior fault");
    std::printf("%s (%d failures)\n", failures == 0 ? "ALL PASS" : "TESTS FAILED", failures);
    return failures == 0 ? 0 : 1;
}
