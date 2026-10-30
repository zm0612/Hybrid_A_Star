/*******************************************************************************
 * Trajectory type used by the post-processing / back-end optimization stage.
 *
 * The front-end HybridAStar::GetPath() returns a flat VectorVec3d (x, y, theta)
 * and discards the per-node FORWARD/BACKWARD direction. For the reverse removal
 * and the Ceres smoother we need to know, for every point, whether the car is
 * moving forward or backward (the "gear"). We re-derive it from the geometry so
 * that we do not have to thread the direction through the Reeds-Shepp shot.
 ******************************************************************************/

#ifndef HYBRID_A_STAR_TRAJECTORY_H
#define HYBRID_A_STAR_TRAJECTORY_H

#include "type.h"

#include <cmath>
#include <vector>

struct TrajPoint {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    int gear = 0; // +1 forward, -1 backward, 0 unknown (used for the very first point)

    TrajPoint() = default;

    TrajPoint(double x_in, double y_in, double theta_in, int gear_in = 0)
            : x(x_in), y(y_in), theta(theta_in), gear(gear_in) {}

    Vec3d ToVec3d() const { return Vec3d(x, y, theta); }
};

typedef std::vector<TrajPoint> Trajectory;

/*!
 * Re-derive the driving direction (gear) for every point of a flat path.
 *
 * For each segment we project the motion vector (p[i+1] - p[i]) onto the heading
 * (cos(theta_i), sin(theta_i)). A positive projection means the rear axle moves
 * roughly in the heading direction => forward, otherwise => backward. The gear of
 * point i is defined by the motion of the segment that *leaves* it; the last point
 * inherits the gear of the segment that enters it.
 */
inline Trajectory InferGears(const VectorVec3d &path) {
    Trajectory traj;
    traj.reserve(path.size());

    for (const auto &p: path) {
        traj.emplace_back(p.x(), p.y(), p.z(), 0);
    }

    if (traj.size() < 2) {
        return traj;
    }

    for (std::size_t i = 0; i + 1 < traj.size(); ++i) {
        const double dx = traj[i + 1].x - traj[i].x;
        const double dy = traj[i + 1].y - traj[i].y;
        const double projection = dx * std::cos(traj[i].theta) + dy * std::sin(traj[i].theta);

        // Degenerate (almost stationary) segment: inherit the previous gear so that
        // numerical noise does not create spurious cusps.
        if (std::abs(projection) < 1e-6 && i > 0) {
            traj[i].gear = traj[i - 1].gear;
        } else {
            traj[i].gear = projection >= 0.0 ? 1 : -1;
        }
    }

    // The last point has no outgoing segment: inherit the incoming gear.
    traj.back().gear = traj[traj.size() - 2].gear;
    // The first point's gear is already set by the loop above from its own outgoing
    // segment. Leave it: a leading reversal (the path *starts* by backing up) must be
    // represented as a cusp at index 1, not silently merged with the next segment.

    return traj;
}

inline VectorVec3d ToVectorVec3d(const Trajectory &traj) {
    VectorVec3d path;
    path.reserve(traj.size());
    for (const auto &p: traj) {
        path.emplace_back(p.x, p.y, p.theta);
    }
    return path;
}

#endif //HYBRID_A_STAR_TRAJECTORY_H
