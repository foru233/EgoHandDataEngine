// Native check for the world-frame revision criterion (no ORB-SLAM3 runtime needed).
//
// Covers the request in docs/原生世界系修订号_20261111.md:
//   - a rigid whole-map displacement of every keyframe  -> re-anchor (revision +1)
//   - independent per-frame jitter of the same keyframes -> NOT a re-anchor
//   - a large displacement that is not rigid over the same ids -> NOT a re-anchor
//   - a displacement below the threshold -> NOT a re-anchor
//   - too few keyframes -> undecidable, NOT a re-anchor
// The counter semantics (Record/Revision/Reset) are exercised as well, because the
// module is linked into the same translation unit set as the bridge.
#include "VioWorldFrameRevision.h"

#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using ORB_SLAM3::VioWorldFrame::Change;
using ORB_SLAM3::VioWorldFrame::Decision;
using ORB_SLAM3::VioWorldFrame::Fit;
using ORB_SLAM3::VioWorldFrame::PoseSample;
using ORB_SLAM3::VioWorldFrame::Thresholds;
using ORB_SLAM3::VioWorldFrame::Verdict;

namespace {
int g_failures = 0;
int g_checks = 0;

void Expect(bool condition, const std::string& what) {
    ++g_checks;
    if (condition) {
        std::cout << "  ok   " << what << '\n';
        return;
    }
    ++g_failures;
    std::cout << "  FAIL " << what << '\n';
}

Eigen::Matrix3d RotationZ(double degrees) {
    const double radians = degrees * 3.14159265358979323846 / 180.0;
    return Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

// Deterministic keyframe cloud around the origin: a stereo session looks at a table,
// so the positions span metres, not kilometres.
std::vector<Eigen::Vector3d> KeyframeCloud(int count, std::uint32_t seed) {
    std::mt19937 generator(seed);
    std::uniform_real_distribution<double> spread(-3.0, 3.0);
    std::uniform_real_distribution<double> height(-0.5, 1.5);
    std::vector<Eigen::Vector3d> positions;
    positions.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i)
        positions.emplace_back(spread(generator), spread(generator), height(generator));
    return positions;
}

std::vector<PoseSample> ToSamples(const std::vector<Eigen::Vector3d>& positions,
                                  const std::vector<Eigen::Vector3d>& moved) {
    std::vector<PoseSample> samples;
    samples.reserve(positions.size());
    for (std::size_t i = 0; i < positions.size(); ++i) samples.push_back({i + 1, moved[i]});
    return samples;
}

std::vector<PoseSample> Before(const std::vector<Eigen::Vector3d>& positions) {
    std::vector<PoseSample> samples;
    samples.reserve(positions.size());
    for (std::size_t i = 0; i < positions.size(); ++i) samples.push_back({i + 1, positions[i]});
    return samples;
}

std::vector<Eigen::Vector3d> Noise(const std::vector<Eigen::Vector3d>& base, double sigma,
                                   std::uint32_t seed) {
    std::mt19937 generator(seed);
    std::normal_distribution<double> noise(0.0, sigma);
    std::vector<Eigen::Vector3d> moved;
    moved.reserve(base.size());
    for (const Eigen::Vector3d& point : base)
        moved.emplace_back(point.x() + noise(generator), point.y() + noise(generator),
                           point.z() + noise(generator));
    return moved;
}
}  // namespace

int main() {
    const std::vector<Eigen::Vector3d> cloud = KeyframeCloud(75, 20261111u);
    const Eigen::Vector3d shift = Eigen::Vector3d(12.44, -3.20, 18.86);  // event6, |.| = 22.82 m
    std::cout << "keyframes=" << cloud.size() << " shift=" << shift.norm() << " m\n";

    Thresholds thresholds;  // defaults: 0.05 m / 0.05 m / 5% / 5 keyframes

    // 1) Rigid whole-map displacement with 1 cm per-keyframe noise: re-anchor.
    {
        std::cout << "[1] rigid whole-map displacement (22.82 m)\n";
        std::vector<Eigen::Vector3d> moved;
        moved.reserve(cloud.size());
        for (const Eigen::Vector3d& point : cloud) moved.push_back(point + shift);
        const auto jitter = Noise(moved, 0.01, 7u);
        const Fit fit = ORB_SLAM3::VioWorldFrame::FitRigid(Before(cloud), ToSamples(cloud, jitter));
        const Decision decision = ORB_SLAM3::VioWorldFrame::Decide(fit, thresholds);
        Expect(fit.valid && fit.count == 75, "fit valid over all 75 keyframes");
        Expect(std::fabs(fit.translation.norm() - shift.norm()) < 0.05,
               "fitted translation matches 22.82 m");
        Expect(std::fabs(fit.scale - 1.0) < 1e-3, "scale stays 1.0 (rigid, not scale drift)");
        Expect(fit.residual_p95_m < 0.05, "residual p95 under 0.05 m");
        Expect(decision.verdict == Verdict::Reanchor, "verdict is reanchor");
        Expect(decision.reanchored, "counted as a world-frame re-anchor");
    }

    // 2) Independent per-keyframe jitter, no common displacement: not a re-anchor.
    {
        std::cout << "[2] per-keyframe micro adjust (2 cm each, independent)\n";
        const auto jitter = Noise(cloud, 0.02, 11u);
        const Fit fit = ORB_SLAM3::VioWorldFrame::FitRigid(Before(cloud), ToSamples(cloud, jitter));
        const Decision decision = ORB_SLAM3::VioWorldFrame::Decide(fit, thresholds);
        Expect(fit.valid, "fit still valid");
        Expect(fit.translation.norm() < thresholds.min_translation_m,
               "fitted translation stays below 0.05 m");
        Expect(decision.verdict == Verdict::TranslationBelowThreshold, "verdict is below threshold");
        Expect(!decision.reanchored, "NOT counted as a re-anchor");
    }

    // 3) Large but non-rigid: the same ids did not move together, so it is not a re-anchor.
    {
        std::cout << "[3] large non-rigid displacement (10 m per keyframe, independent)\n";
        const auto scattered = Noise(cloud, 10.0, 13u);
        const Fit fit = ORB_SLAM3::VioWorldFrame::FitRigid(Before(cloud), ToSamples(cloud, scattered));
        const Decision decision = ORB_SLAM3::VioWorldFrame::Decide(fit, thresholds);
        Expect(fit.valid, "fit still valid");
        Expect(fit.residual_p95_m > decision.residual_limit_m, "residual p95 exceeds the limit");
        Expect(decision.verdict == Verdict::NonRigid, "verdict is non_rigid");
        Expect(!decision.reanchored, "NOT counted as a re-anchor");
    }

    // 4) A 5 cm rigid displacement with millimetre residuals is still a re-anchor.
    {
        std::cout << "[4] small but rigid displacement (0.10 m, 1 mm noise)\n";
        std::vector<Eigen::Vector3d> moved;
        moved.reserve(cloud.size());
        for (const Eigen::Vector3d& point : cloud) moved.push_back(point + Eigen::Vector3d(0.06, -0.08, 0.0));
        const auto jitter = Noise(moved, 0.001, 17u);
        const Fit fit = ORB_SLAM3::VioWorldFrame::FitRigid(Before(cloud), ToSamples(cloud, jitter));
        const Decision decision = ORB_SLAM3::VioWorldFrame::Decide(fit, thresholds);
        Expect(decision.verdict == Verdict::Reanchor, "verdict is reanchor");
        Expect(decision.reanchored, "counted as a re-anchor");
    }

    // 5) Rotation plus translation: the world frame also turned, still one re-anchor.
    {
        std::cout << "[5] rigid rotation 3 deg + translation 1 m\n";
        const Eigen::Matrix3d rotation = RotationZ(3.0);
        const Eigen::Vector3d translation(0.6, 0.8, 0.0);
        std::vector<Eigen::Vector3d> moved;
        moved.reserve(cloud.size());
        for (const Eigen::Vector3d& point : cloud) moved.push_back(rotation * point + translation);
        const auto jitter = Noise(moved, 0.005, 19u);
        const Fit fit = ORB_SLAM3::VioWorldFrame::FitRigid(Before(cloud), ToSamples(cloud, jitter));
        const Decision decision = ORB_SLAM3::VioWorldFrame::Decide(fit, thresholds);
        Expect(std::fabs(fit.rotation_deg - 3.0) < 0.1, "fitted rotation is 3 deg");
        Expect(decision.verdict == Verdict::Reanchor, "verdict is reanchor");
    }

    // 6) Too few keyframes cannot separate motion from noise: never a re-anchor.
    {
        std::cout << "[6] only 3 keyframes translated 10 m\n";
        const std::vector<Eigen::Vector3d> few = KeyframeCloud(3, 23u);
        std::vector<Eigen::Vector3d> moved;
        for (const Eigen::Vector3d& point : few) moved.push_back(point + Eigen::Vector3d(0, 0, 10));
        const Fit fit = ORB_SLAM3::VioWorldFrame::FitRigid(Before(few), ToSamples(few, moved));
        const Decision decision = ORB_SLAM3::VioWorldFrame::Decide(fit, thresholds);
        Expect(decision.verdict == Verdict::InsufficientSamples, "verdict is insufficient_samples");
        Expect(!decision.reanchored, "NOT counted as a re-anchor");
    }

    // 7) Fit is not fooled by mismatched ids or by a cloud without spatial spread.
    {
        std::cout << "[7] mismatched ids / degenerate cloud\n";
        const auto before = Before(cloud);
        auto reordered = ToSamples(cloud, cloud);
        reordered.front().id = 999999;
        const Fit mismatched = ORB_SLAM3::VioWorldFrame::FitRigid(before, reordered);
        Expect(!mismatched.valid, "mismatched ids are rejected");
        std::vector<Eigen::Vector3d> stacked;
        for (int i = 0; i < 10; ++i) stacked.emplace_back(1.0, 2.0, 3.0);
        const Fit degenerate = ORB_SLAM3::VioWorldFrame::FitRigid(Before(stacked), Before(stacked));
        Expect(!degenerate.valid, "a cloud without spread is rejected instead of guessed");
    }

    // 8) Counter semantics: only a re-anchor bumps the revision.
    {
        std::cout << "[8] Record()/Revision()/Reset()\n";
        ORB_SLAM3::VioWorldFrame::Reset();
        Expect(ORB_SLAM3::VioWorldFrame::Revision() == 0, "reset starts at revision 0");
        std::vector<Eigen::Vector3d> moved;
        for (const Eigen::Vector3d& point : cloud) moved.push_back(point + shift);
        const Fit reanchor = ORB_SLAM3::VioWorldFrame::FitRigid(Before(cloud), ToSamples(cloud, moved));
        Change change;
        Expect(ORB_SLAM3::VioWorldFrame::Record(reanchor, thresholds, 0, 3244.0, 900, &change),
               "rigid displacement recorded as a re-anchor");
        Expect(change.revision == 1 && change.detections == 1 && change.evaluations == 1,
               "revision/detections/evaluations advanced once");
        Expect(std::fabs(change.translation_m - shift.norm()) < 0.05 && std::fabs(change.scale - 1.0) < 1e-6,
               "recorded displacement and scale");
        const Fit jitter = ORB_SLAM3::VioWorldFrame::FitRigid(
            Before(cloud), ToSamples(cloud, Noise(cloud, 0.02, 11u)));
        Expect(!ORB_SLAM3::VioWorldFrame::Record(jitter, thresholds, 0, 3250.0, 950, &change),
               "per-keyframe jitter is not recorded as a re-anchor");
        Expect(change.revision == 1 && change.evaluations == 2,
               "revision unchanged, evaluations advanced");
        const Fit nonrigid = ORB_SLAM3::VioWorldFrame::FitRigid(
            Before(cloud), ToSamples(cloud, Noise(cloud, 10.0, 13u)));
        ORB_SLAM3::VioWorldFrame::Record(nonrigid, thresholds, 0, 3260.0, 990, &change);
        Expect(change.revision == 1 && change.rejected_nonrigid == 1,
               "non-rigid drift is counted but does not bump the revision");
        Expect(ORB_SLAM3::VioWorldFrame::Revision() == 1, "Revision() exposes the same value");
    }

    std::cout << (g_failures == 0 ? "PASS" : "FAIL") << ": " << (g_checks - g_failures) << '/'
              << g_checks << " checks\n";
    return g_failures == 0 ? 0 : 1;
}
