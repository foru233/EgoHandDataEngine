#pragma once
#include <TrackingFrameSnapshot.h>
#include <cmath>

// A map transition can publish an available but inconsistent value snapshot
// with no geometry token. Such a snapshot must never be dereferenced/bound.
inline bool EgohandStableReferenceSnapshot(const ORB_SLAM3::TrackingFrameSnapshot& snapshot,
                                         double expectedTimestamp) {
    return snapshot.available && snapshot.consistent &&
        std::isfinite(snapshot.timestamp_sec) && std::isfinite(expectedTimestamp) &&
        std::abs(snapshot.timestamp_sec-expectedTimestamp)<=1e-6 &&
        ((snapshot.inertial_reference_generation==0)==(snapshot.inertial_reference_operation==0)) &&
        snapshot.geometry_commit_state &&
        snapshot.geometry_commit_state->Stable(snapshot.geometry_commit_sequence);
}
