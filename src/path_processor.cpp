/*******************************************************************************
 * Redundant reversal removal. See path_processor.h for the rationale.
 ******************************************************************************/

#include "hybrid_a_star/path_processor.h"

#include <algorithm>
#include <cmath>

double PathProcessor::NormalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double PathProcessor::AngleDiff(double a, double b) {
    return std::abs(NormalizeAngle(a - b));
}

double PathProcessor::InterpAngle(double a, double b, double t) {
    return NormalizeAngle(a + t * NormalizeAngle(b - a));
}

bool PathProcessor::TryStitchForward(const Trajectory &traj, int anchor, int q_first, int q_last,
                                     double dir_in, double dir_out, int common_gear,
                                     const CollisionChecker &collision_free,
                                     double heading_tol, double step_size, Trajectory *out) {
    const int n = static_cast<int>(traj.size());
    const double ax = traj[anchor].x, ay = traj[anchor].y;
    const double hx = std::cos(dir_in), hy = std::sin(dir_in);
    const double min_progress = std::max(step_size, 0.2);

    // The reversal backtracks and then re-covers ground. Find the first point of the
    // following segment that has made real forward progress past the anchor along
    // dir_in and whose straight connection from the anchor is itself aligned with
    // dir_in. The alignment requirement keeps the shortcut a genuine forward move
    // (the Ceres smoother then rounds the small corner into a steered pass) instead
    // of a near-sideways jog that would be re-labelled as another reversal.
    int q = -1;
    for (int i = q_first; i <= q_last; ++i) {
        const double dx = traj[i].x - ax;
        const double dy = traj[i].y - ay;
        if (dx * hx + dy * hy < min_progress) {
            continue;
        }
        if (AngleDiff(std::atan2(dy, dx), dir_in) <= heading_tol) {
            q = i;
            break;
        }
    }
    if (q < 0) {
        return false; // no aligned forward shortcut exists: keep this reversal
    }

    const double qx = traj[q].x, qy = traj[q].y;
    const double stitch_dist = std::hypot(qx - ax, qy - ay);

    // Body heading at each end (forward/backward aware), swept between them across
    // the stitch for the collision footprint.
    const double theta_a = common_gear >= 0 ? dir_in : NormalizeAngle(dir_in + M_PI);
    const double theta_q = common_gear >= 0 ? dir_out : NormalizeAngle(dir_out + M_PI);

    bool ok = collision_free(ax, ay, theta_a) && collision_free(qx, qy, theta_q);
    Trajectory connection;
    const int steps = std::max(1, static_cast<int>(std::ceil(stitch_dist / step_size)));
    for (int k = 1; ok && k < steps; ++k) {
        const double t = static_cast<double>(k) / steps;
        const double x = ax + t * (qx - ax);
        const double y = ay + t * (qy - ay);
        const double theta = InterpAngle(theta_a, theta_q, t);
        if (!collision_free(x, y, theta)) {
            ok = false;
            break;
        }
        connection.emplace_back(x, y, theta, common_gear);
    }
    if (!ok) {
        return false; // stitch would collide: keep this reversal
    }

    Trajectory result;
    result.reserve(n);
    for (int i = 0; i <= anchor; ++i) {
        result.push_back(traj[i]);
    }
    for (const auto &p: connection) {
        result.push_back(p);
    }
    for (int i = q; i < n; ++i) {
        result.push_back(traj[i]);
    }
    *out = InferGears(ToVectorVec3d(result));
    return true;
}

bool PathProcessor::TryStitchBackward(const Trajectory &traj, int anchor, int q_first, int q_last,
                                      double dir_in, double dir_out, int common_gear,
                                      const CollisionChecker &collision_free,
                                      double heading_tol, double step_size, Trajectory *out) {
    const int n = static_cast<int>(traj.size());
    const double gx = traj[anchor].x, gy = traj[anchor].y;
    const double hx = std::cos(dir_in), hy = std::sin(dir_in);
    const double min_progress = std::max(step_size, 0.2);

    // Mirrored search: walk the preceding segment backwards, looking for the point
    // whose straight connection into the anchor (goal) stays aligned with dir_in and
    // has made real progress toward the goal.
    int q = -1;
    for (int i = q_last; i >= q_first; --i) {
        const double dx = gx - traj[i].x;
        const double dy = gy - traj[i].y;
        if (dx * hx + dy * hy < min_progress) {
            continue;
        }
        if (AngleDiff(std::atan2(dy, dx), dir_in) <= heading_tol) {
            q = i;
            break;
        }
    }
    if (q < 0) {
        return false;
    }

    const double qx = traj[q].x, qy = traj[q].y;
    const double stitch_dist = std::hypot(gx - qx, gy - qy);

    const double theta_q = common_gear >= 0 ? dir_in : NormalizeAngle(dir_in + M_PI);
    const double theta_g = common_gear >= 0 ? dir_out : NormalizeAngle(dir_out + M_PI);

    bool ok = collision_free(qx, qy, theta_q) && collision_free(gx, gy, theta_g);
    Trajectory connection;
    const int steps = std::max(1, static_cast<int>(std::ceil(stitch_dist / step_size)));
    for (int k = 1; ok && k < steps; ++k) {
        const double t = static_cast<double>(k) / steps;
        const double x = qx + t * (gx - qx);
        const double y = qy + t * (gy - qy);
        const double theta = InterpAngle(theta_q, theta_g, t);
        if (!collision_free(x, y, theta)) {
            ok = false;
            break;
        }
        connection.emplace_back(x, y, theta, common_gear);
    }
    if (!ok) {
        return false;
    }

    Trajectory result;
    result.reserve(n);
    for (int i = 0; i <= q; ++i) {
        result.push_back(traj[i]);
    }
    for (const auto &p: connection) {
        result.push_back(p);
    }
    for (int i = anchor; i < n; ++i) {
        result.push_back(traj[i]);
    }
    *out = InferGears(ToVectorVec3d(result));
    return true;
}

double PathProcessor::SegmentLength(const Trajectory &traj, int lo, int hi) {
    double length = 0.0;
    for (int i = lo; i < hi; ++i) {
        const double dx = traj[i + 1].x - traj[i].x;
        const double dy = traj[i + 1].y - traj[i].y;
        length += std::hypot(dx, dy);
    }
    return length;
}

int PathProcessor::CountCusps(const Trajectory &traj) {
    int cusps = 0;
    for (std::size_t i = 1; i + 1 < traj.size(); ++i) {
        if (traj[i].gear != traj[i - 1].gear) {
            ++cusps;
        }
    }
    return cusps;
}

Trajectory PathProcessor::Resample(const Trajectory &traj, double spacing) {
    const int n = static_cast<int>(traj.size());
    if (n < 2 || spacing <= 1e-6) {
        return traj;
    }

    // Segment boundaries at cusps.
    std::vector<int> bound;
    bound.push_back(0);
    for (int i = 1; i < n - 1; ++i) {
        if (traj[i].gear != traj[i - 1].gear) {
            bound.push_back(i);
        }
    }
    bound.push_back(n - 1);

    Trajectory out;
    for (std::size_t k = 0; k + 1 < bound.size(); ++k) {
        const int a = bound[k];
        const int b = bound[k + 1];
        const int gear = traj[a].gear;

        // Cumulative arc length along this segment.
        std::vector<double> cum(b - a + 1, 0.0);
        for (int i = a; i < b; ++i) {
            cum[i - a + 1] = cum[i - a] + std::hypot(traj[i + 1].x - traj[i].x,
                                                     traj[i + 1].y - traj[i].y);
        }
        const double total = cum.back();

        std::vector<std::pair<double, double>> pts; // resampled (x, y)
        if (total < 1e-6) {
            pts.emplace_back(traj[a].x, traj[a].y); // degenerate segment
        } else {
            const int m = std::max(1, static_cast<int>(std::round(total / spacing)));
            int seg = 0;
            for (int j = 0; j <= m; ++j) {
                const double target = total * static_cast<double>(j) / m;
                while (seg < b - a && cum[seg + 1] < target) {
                    ++seg;
                }
                const double seg_len = cum[seg + 1] - cum[seg];
                const double t = seg_len > 1e-9 ? (target - cum[seg]) / seg_len : 0.0;
                const double x = traj[a + seg].x + t * (traj[a + seg + 1].x - traj[a + seg].x);
                const double y = traj[a + seg].y + t * (traj[a + seg + 1].y - traj[a + seg].y);
                pts.emplace_back(x, y);
            }
            // Pin the exact endpoints (start / goal / cusp).
            pts.front() = {traj[a].x, traj[a].y};
            pts.back() = {traj[b].x, traj[b].y};
        }

        // Append (skip the first point if it duplicates the previous segment's cusp).
        //
        // The last point of each segment is pinned to traj[b], which is the *first*
        // point of the next segment (a cusp, or the goal). It must keep ITS OWN gear
        // (traj[b].gear), not the segment's gear, so that the invariant "gear[i] =
        // direction of the segment leaving point i" holds at the reversal. Labelling
        // the cusp with the old gear makes the next pass's optimizer read a backward
        // outgoing edge against a forward gear and flip the heading by pi (the car
        // appears to spin around at the reversal).
        const std::size_t start_idx = out.empty() ? 0u : 1u;
        for (std::size_t i = start_idx; i < pts.size(); ++i) {
            const int pt_gear = (i + 1 == pts.size()) ? traj[b].gear : gear;
            out.emplace_back(pts[i].first, pts[i].second, 0.0, pt_gear);
        }
    }

    // Recompute headings from the resampled geometry (preserve endpoint yaw). At a
    // segment's last point (a cusp) the outgoing edge points into the reversed next
    // segment, so use the incoming edge there to keep the body heading continuous.
    const int on = static_cast<int>(out.size());
    for (int i = 0; i < on; ++i) {
        const bool last_of_segment = (i == on - 1) || (out[i + 1].gear != out[i].gear);
        double motion_dir;
        if (!last_of_segment) {
            motion_dir = std::atan2(out[i + 1].y - out[i].y, out[i + 1].x - out[i].x);
        } else if (i > 0) {
            motion_dir = std::atan2(out[i].y - out[i - 1].y, out[i].x - out[i - 1].x);
        } else {
            motion_dir = out[i].theta;
        }
        out[i].theta = out[i].gear >= 0 ? motion_dir : NormalizeAngle(motion_dir + M_PI);
    }
    if (on > 0) {
        out.front().theta = traj.front().theta;
        out.back().theta = traj.back().theta;
    }

    return out;
}

Trajectory PathProcessor::RemoveRedundantReversals(const Trajectory &traj,
                                                   const CollisionChecker &collision_free) const {
    Trajectory current = traj;

    // Iterate until a full pass removes nothing. The collapse of one wiggle may
    // merge neighbours and expose a new short segment, so we keep looping.
    bool changed = true;
    int guard = 0;
    while (changed && guard++ < 1000) {
        current = CollapseOnce(current, collision_free, &changed);
    }

    return current;
}

Trajectory PathProcessor::CollapseOnce(const Trajectory &traj,
                                       const CollisionChecker &collision_free,
                                       bool *changed) const {
    *changed = false;
    const int n = static_cast<int>(traj.size());
    if (n < 3) {
        return traj;
    }

    // Build group boundaries at cusps (direction reversals).
    std::vector<int> bound;
    bound.push_back(0);
    for (int i = 1; i < n - 1; ++i) {
        if (traj[i].gear != traj[i - 1].gear) {
            bound.push_back(i);
        }
    }
    bound.push_back(n - 1);

    const int num_groups = static_cast<int>(bound.size()) - 1;

    // (a) Interior wiggle: a short reversal group sandwiched between two same-
    //     direction groups, with a small *net* heading change (a large change means
    //     a genuine re-orientation, e.g. a three-point turn, which we must keep).
    //     Try the shortest candidates first.
    if (num_groups >= 3) {
        std::vector<std::pair<double, int>> candidates; // (length, group index)
        for (int s = 1; s < num_groups - 1; ++s) {
            const int a = bound[s];
            const int b = bound[s + 1];
            const double length = SegmentLength(traj, a, b);
            if (length >= min_segment_length_) {
                continue;
            }
            const double dir_in = std::atan2(traj[a].y - traj[a - 1].y, traj[a].x - traj[a - 1].x);
            const double dir_out = std::atan2(traj[b + 1].y - traj[b].y, traj[b + 1].x - traj[b].x);
            if (AngleDiff(dir_in, dir_out) > heading_tol_) {
                continue;
            }
            candidates.emplace_back(length, s);
        }
        std::sort(candidates.begin(), candidates.end());

        for (const auto &cand: candidates) {
            const int s = cand.second;
            const int a = bound[s];     // start of the wiggle (a cusp)
            const int b = bound[s + 1]; // first point of the following group
            const double dir_in = std::atan2(traj[a].y - traj[a - 1].y, traj[a].x - traj[a - 1].x);
            const double dir_out = std::atan2(traj[b + 1].y - traj[b].y, traj[b + 1].x - traj[b].x);
            Trajectory result;
            if (TryStitchForward(traj, a, b, bound[s + 2], dir_in, dir_out,
                                 traj[a - 1].gear, collision_free,
                                 heading_tol_, step_size_, &result)) {
                *changed = true;
                return result;
            }
        }
    }

    // (b) Leading short reversal: the path *starts* by reversing a short distance
    //     to adjust the heading, then goes forward. If the reversal's net heading
    //     change is small it is redundant (the car can simply steer forward from the
    //     start), so stitch forward from the fixed start into the following group.
    if (num_groups >= 2 && traj[0].gear < 0 &&
        SegmentLength(traj, 0, bound[1]) < min_segment_length_) {
        const double dir_in = traj[0].theta; // fixed start body heading = forward dir
        const int q_first = bound[1];
        const int q_last = (num_groups >= 3) ? bound[2] : (n - 1);
        const double dir_out = std::atan2(traj[q_first + 1].y - traj[q_first].y,
                                          traj[q_first + 1].x - traj[q_first].x);
        if (AngleDiff(dir_in, dir_out) <= heading_tol_) {
            Trajectory result;
            if (TryStitchForward(traj, 0, q_first, q_last, dir_in, dir_out,
                                 1, collision_free, heading_tol_, step_size_, &result)) {
                *changed = true;
                return result;
            }
        }
    }

    // (c) Trailing short reversal: the path *ends* by reversing a short distance
    //     into the goal. Symmetric to (b): stitch from the preceding group straight
    //     into the fixed goal.
    if (num_groups >= 2 && traj[n - 1].gear < 0 &&
        SegmentLength(traj, bound[num_groups - 1], n - 1) < min_segment_length_) {
        const int goal = n - 1;
        const int cusp = bound[num_groups - 1];      // start of the trailing group
        const int common_gear = traj[cusp - 1].gear; // gear of the preceding group
        const double dir_out = traj[goal].theta;     // fixed goal body heading
        const double dir_in = std::atan2(traj[cusp].y - traj[cusp - 1].y,
                                         traj[cusp].x - traj[cusp - 1].x);
        if (AngleDiff(dir_in, dir_out) <= heading_tol_) {
            Trajectory result;
            if (TryStitchBackward(traj, goal, bound[num_groups - 2], cusp - 1,
                                  dir_in, dir_out, common_gear, collision_free,
                                  heading_tol_, step_size_, &result)) {
                *changed = true;
                return result;
            }
        }
    }

    return traj;
}