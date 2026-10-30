/*******************************************************************************
 * Ceres-based back-end trajectory optimizer. See trajectory_optimizer.h.
 ******************************************************************************/

#include "hybrid_a_star/trajectory_optimizer.h"

#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace {

typedef ceres::Grid2D<double, 1> Grid;
typedef ceres::BiCubicInterpolator<Grid> Interpolator;

// --- Smoothness: penalize the second difference of three consecutive points. ---
struct SmoothnessCost {
    explicit SmoothnessCost(double weight) : weight_(weight) {}

    template<typename T>
    bool operator()(const T *a, const T *b, const T *c, T *residual) const {
        residual[0] = T(weight_) * (a[0] - T(2.0) * b[0] + c[0]);
        residual[1] = T(weight_) * (a[1] - T(2.0) * b[1] + c[1]);
        return true;
    }

    static ceres::CostFunction *Create(double weight) {
        return new ceres::AutoDiffCostFunction<SmoothnessCost, 2, 2, 2, 2>(
                new SmoothnessCost(weight));
    }

    double weight_;
};

// --- Length: penalize the distance between consecutive points so the optimizer
//     shortens the path. The smoothness (second-difference) term is a curvature
//     penalty: it removes sharp wiggles but is nearly blind to long-wavelength
//     S-curves, whose per-point curvature is tiny. This term gives every interior
//     point a direct incentive to straighten out, and the obstacle barrier keeps it
//     from cutting corners. Weight should be small relative to w_smooth because
//     the residual scales with ds (not ds^2).
struct LengthCost {
    explicit LengthCost(double weight) : weight_(weight) {}

    template<typename T>
    bool operator()(const T *a, const T *b, T *residual) const {
        const T dx = b[0] - a[0];
        const T dy = b[1] - a[1];
        residual[0] = T(weight_) * ceres::sqrt(dx * dx + dy * dy + T(1e-12));
        return true;
    }

    static ceres::CostFunction *Create(double weight) {
        return new ceres::AutoDiffCostFunction<LengthCost, 1, 2, 2>(
                new LengthCost(weight));
    }

    double weight_;
};

// --- Reference: keep a point near its front-end location. ---
struct ReferenceCost {
    ReferenceCost(double ref_x, double ref_y, double weight)
            : ref_x_(ref_x), ref_y_(ref_y), weight_(weight) {}

    template<typename T>
    bool operator()(const T *p, T *residual) const {
        residual[0] = T(weight_) * (p[0] - T(ref_x_));
        residual[1] = T(weight_) * (p[1] - T(ref_y_));
        return true;
    }

    static ceres::CostFunction *Create(double ref_x, double ref_y, double weight) {
        return new ceres::AutoDiffCostFunction<ReferenceCost, 2, 2>(
                new ReferenceCost(ref_x, ref_y, weight));
    }

    double ref_x_, ref_y_, weight_;
};

// --- Obstacle: reciprocal *barrier* clearance penalty for one covering circle,
//     oriented by the LIVE heading (from the neighbouring points).
//
// The circle offset along the vehicle body is rotated by the heading derived from
// (next - prev), so the footprint follows the ACTUAL optimized geometry rather than
// a stale reference heading. Without this, strong smoothing rotates the real body
// while the guard circles stay put, and the car slides into the obstacle.
//
// Barrier shape (dist d, onset d_on, hard floor d0, d0 < d_on):
//     d >= d_on            : 0
//     d0 + eps < d < d_on  : weight * ( 1/(d - d0) - 1/(d_on - d0) )   (-> inf as d->d0)
//     d <= d0 + eps        : quadratic continuation (C1 at d0+eps); its gradient GROWS
//                            with penetration depth, so no finite smoothness weight can
//                            drive the footprint through the obstacle.
// d_on/d0 are capped at the reference clearance so the barrier never demands MORE room
// than the (collision-free) front-end already had -> no kinks where the car must hug a
// wall.
struct ObstacleCost {
    ObstacleCost(const Interpolator *interp, double origin_x, double origin_y, double resolution,
                 double offset_long, double gear_sign, double d_on, double d0, double eps, double weight)
            : interp_(interp), origin_x_(origin_x), origin_y_(origin_y), resolution_(resolution),
              offset_long_(offset_long), gear_sign_(gear_sign),
              d_on_(d_on), d0_(d0), eps_(eps), weight_(weight) {}

    template<typename T>
    bool operator()(const T *prev, const T *cur, const T *next, T *residual) const {
        // Live body heading from the neighbours (flipped for a backward segment, so the
        // asymmetric front/rear footprint stays attached to the body, not the motion).
        T dx = next[0] - prev[0];
        T dy = next[1] - prev[1];
        const T len = ceres::sqrt(dx * dx + dy * dy + T(1e-9));
        T ux = dx / len;
        T uy = dy / len;
        if (gear_sign_ < 0.0) {
            ux = -ux;
            uy = -uy;
        }

        // Covering-circle center in world coordinates, then -> continuous grid coord.
        const T cx = cur[0] + T(offset_long_) * ux;
        const T cy = cur[1] + T(offset_long_) * uy;
        const T col = (cx - T(origin_x_)) / T(resolution_);
        const T row = (cy - T(origin_y_)) / T(resolution_);

        T d;
        interp_->Evaluate(row, col, &d);

        const double inv_on = 1.0 / (d_on_ - d0_);
        if (d >= T(d_on_)) {
            residual[0] = T(0.0);
        } else if (d > T(d0_ + eps_)) {
            residual[0] = T(weight_) * (T(1.0) / (d - T(d0_)) - T(inv_on));
        } else {
            const double r_cap = weight_ * (1.0 / eps_ - inv_on);
            const double slope = weight_ / (eps_ * eps_);
            const double quad = weight_ / (eps_ * eps_);
            const T delta = T(d0_ + eps_) - d;
            residual[0] = T(r_cap) + T(slope) * delta + T(quad) * delta * delta;
        }
        return true;
    }

    static ceres::CostFunction *Create(const Interpolator *interp, double origin_x, double origin_y,
                                       double resolution, double offset_long, double gear_sign,
                                       double d_on, double d0, double eps, double weight) {
        return new ceres::AutoDiffCostFunction<ObstacleCost, 1, 2, 2, 2>(
                new ObstacleCost(interp, origin_x, origin_y, resolution,
                                 offset_long, gear_sign, d_on, d0, eps, weight));
    }

    const Interpolator *interp_;
    double origin_x_, origin_y_, resolution_;
    double offset_long_, gear_sign_, d_on_, d0_, eps_, weight_;
};

// --- Curvature: soft limit |dphi| <= kappa_max * ds (bounded turning radius). ---
struct CurvatureCost {
    CurvatureCost(double max_curvature, double weight)
            : max_curvature_(max_curvature), weight_(weight) {}

    template<typename T>
    bool operator()(const T *a, const T *b, const T *c, T *residual) const {
        const T d1x = b[0] - a[0];
        const T d1y = b[1] - a[1];
        const T d2x = c[0] - b[0];
        const T d2y = c[1] - b[1];

        const T len1 = ceres::sqrt(d1x * d1x + d1y * d1y + T(1e-9));
        const T len2 = ceres::sqrt(d2x * d2x + d2y * d2y + T(1e-9));

        const T h1 = ceres::atan2(d1y, d1x);
        const T h2 = ceres::atan2(d2y, d2x);

        // Wrapped heading change in (-pi, pi], differentiable.
        const T dh = ceres::atan2(ceres::sin(h2 - h1), ceres::cos(h2 - h1));
        const T abs_dh = dh < T(0.0) ? -dh : dh;

        const T ds = (len1 + len2) * T(0.5);
        const T excess = abs_dh - T(max_curvature_) * ds;

        residual[0] = excess > T(0.0) ? T(weight_) * excess : T(0.0);
        return true;
    }

    static ceres::CostFunction *Create(double max_curvature, double weight) {
        return new ceres::AutoDiffCostFunction<CurvatureCost, 1, 2, 2, 2>(
                new CurvatureCost(max_curvature, weight));
    }

    double max_curvature_, weight_;
};

double NormalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

} // namespace

Trajectory TrajectoryOptimizer::Optimize(const Trajectory &init) const {
    const int N = static_cast<int>(init.size());
    if (esdf_ == nullptr || !esdf_->IsValid() || N < 5) {
        return init;
    }

    // Decision variables: (x, y) per point. Reserve so addresses stay stable.
    std::vector<std::array<double, 2>> pts(N);
    for (int i = 0; i < N; ++i) {
        pts[i] = {init[i].x, init[i].y};
    }

    // Fix start, goal, and every cusp (gear-shift) point.
    std::vector<char> fixed(N, 0);
    fixed[0] = 1;
    fixed[N - 1] = 1;
    for (int i = 1; i < N - 1; ++i) {
        if (init[i].gear != init[i - 1].gear) {
            fixed[i] = 1;
        }
    }

    // Arc length and arc-length distance from each point to the nearest cusp, used
    // to taper the smoothness weight near reversal points.
    std::vector<double> arc_len(N, 0.0);
    for (int i = 1; i < N; ++i) {
        arc_len[i] = arc_len[i - 1] + std::hypot(init[i].x - init[i - 1].x,
                                                 init[i].y - init[i - 1].y);
    }
    std::vector<double> cusp_arc;
    for (int i = 1; i < N - 1; ++i) {
        if (init[i].gear != init[i - 1].gear) {
            cusp_arc.push_back(arc_len[i]);
        }
    }
    auto SmoothRatio = [&](int i) -> double {
        if (cusp_arc.empty() || config_.cusp_taper_dist <= 1e-6) {
            return 1.0;
        }
        double nearest = std::numeric_limits<double>::max();
        for (double c: cusp_arc) {
            nearest = std::min(nearest, std::abs(arc_len[i] - c));
        }
        const double t = std::min(1.0, nearest / config_.cusp_taper_dist);
        return config_.cusp_taper_min_ratio + (1.0 - config_.cusp_taper_min_ratio) * t;
    };

    // Freeze whole segments that are too short to smooth without flattening. A short
    // maneuver bounded by two anchors (two cusps, or a cusp and an endpoint) is held
    // rigid at its -- already feasible, collision-free -- front-end shape, instead of
    // being pulled straight between the two fixed anchors. Longer segments are still
    // optimized (with the near-cusp smoothness taper above). This uses the base
    // anchors (start/goal/cusps) only, so it must run before the extra hard-fix zones.
    if (config_.min_opt_segment_length > 1e-6) {
        std::vector<int> anchors;
        for (int i = 0; i < N; ++i) {
            if (fixed[i]) anchors.push_back(i);
        }
        for (std::size_t k = 0; k + 1 < anchors.size(); ++k) {
            const int lo = anchors[k];
            const int hi = anchors[k + 1];
            if (arc_len[hi] - arc_len[lo] < config_.min_opt_segment_length) {
                for (int i = lo + 1; i < hi; ++i) {
                    fixed[i] = 1;
                }
            }
        }
    }

    // Hard-fix zone around each reversal point: freeze every point within
    // cusp_fix_dist (arc length) of a cusp, so the trajectory just before and after a
    // reversal is preserved exactly instead of being optimized/flattened.
    if (config_.cusp_fix_dist > 1e-6 && !cusp_arc.empty()) {
        for (int i = 0; i < N; ++i) {
            for (double c: cusp_arc) {
                if (std::abs(arc_len[i] - c) <= config_.cusp_fix_dist) {
                    fixed[i] = 1;
                    break;
                }
            }
        }
    }

    // Lock the start and goal direction: fixing the endpoint alone pins its position,
    // but the neighbour point could still move and bend the initial/final tangent.
    // Fixing the first two and last two points keeps the start/goal heading intact.
    if (N >= 4) {
        fixed[1] = 1;
        fixed[N - 2] = 1;
    }

    // Optional larger hard-fix zone around the start / goal.
    if (config_.endpoint_fix_dist > 1e-6) {
        const double total = arc_len[N - 1];
        for (int i = 0; i < N; ++i) {
            if (arc_len[i] <= config_.endpoint_fix_dist ||
                total - arc_len[i] <= config_.endpoint_fix_dist) {
                fixed[i] = 1;
            }
        }
    }

    // Distance field wrapper for Ceres (built locally; solved before returning).
    Grid grid(esdf_->data().data(), 0, esdf_->height(), 0, esdf_->width());
    Interpolator interpolator(grid);

    // Vehicle footprint -> covering circles along the longitudinal axis.
    const double half_length = config_.vehicle_length * 0.5;
    const double front = config_.vehicle_length - config_.rear_axle_dist; // ahead of rear axle
    const double rear = -config_.rear_axle_dist;                           // behind rear axle
    const int num_circles = std::max(1, config_.num_circles);
    std::vector<double> long_offsets(num_circles);
    for (int k = 0; k < num_circles; ++k) {
        const double t = num_circles == 1 ? 0.5 : static_cast<double>(k) / (num_circles - 1);
        long_offsets[k] = rear + t * (front - rear);
    }
    const double spacing = num_circles > 1 ? (front - rear) / (num_circles - 1) : config_.vehicle_length;
    const double circle_radius = std::hypot(spacing * 0.5, config_.vehicle_width * 0.5);
    (void) half_length;

    ceres::Problem problem;
    for (int i = 0; i < N; ++i) {
        problem.AddParameterBlock(pts[i].data(), 2);
    }

    for (int i = 0; i < N; ++i) {
        if (fixed[i]) {
            continue;
        }

        // Reference term.
        problem.AddResidualBlock(
                ReferenceCost::Create(init[i].x, init[i].y, config_.w_reference),
                nullptr, pts[i].data());

        // Obstacle term: one reciprocal-barrier residual per covering circle. The
        // circle is oriented by the LIVE heading (neighbours i-1, i+1), so the guard
        // footprint follows the actual optimized geometry. For each circle:
        //   d0   (hard floor)  = min(circle_radius, d_ref): the barrier diverges here,
        //        so the footprint can never be pushed past it -- never worse than the
        //        (collision-free) front-end, and never into actual circle collision.
        //   d_on (onset)       = min(clearance, d_ref), kept at least eps above d0:
        //        where the penalty starts; capped at the reference clearance so we
        //        don't demand extra room (and kink) where the car must hug a wall.
        if (i >= 1 && i + 1 < N) {
            const double ct = std::cos(init[i].theta);
            const double st = std::sin(init[i].theta);
            const double gear_sign = init[i].gear >= 0 ? 1.0 : -1.0;
            const double eps = std::max(1e-3, config_.obstacle_eps);
            for (double d: long_offsets) {
                const double d_ref = esdf_->Distance(init[i].x + d * ct, init[i].y + d * st);
                // Hard floor = circle_radius + safety_margin: when a circle center is
                // pinned at the floor, the covered footprint still clears the obstacle
                // by safety_margin (a floor of just circle_radius would let the body
                // *touch*). Capped at the reference clearance so we never demand more
                // room than the collision-free front-end had (no kinks near walls).
                const double floor_abs = circle_radius + config_.safety_margin;
                double d0 = std::min(floor_abs, d_ref);
                if (d0 < 0.0) d0 = 0.0;
                double d_on = std::min(d_ref, d0 + 0.3); // soft onset band above the floor
                if (d_on < d0 + eps) d_on = d0 + eps;    // guarantee a barrier band
                problem.AddResidualBlock(
                        ObstacleCost::Create(&interpolator, esdf_->origin_x(), esdf_->origin_y(),
                                             esdf_->resolution(), d, gear_sign, d_on, d0, eps,
                                             config_.w_obstacle),
                        nullptr, pts[i - 1].data(), pts[i].data(), pts[i + 1].data());
            }
        }
    }

    // Smoothness + curvature over triples that stay within one monotonic segment.
    // The smoothness weight is tapered down near cusps so the turn-in geometry of a
    // reversal is preserved rather than flattened; curvature (feasibility) keeps full
    // weight everywhere.
    for (int i = 1; i < N - 1; ++i) {
        if (init[i - 1].gear != init[i].gear) {
            continue; // the triple straddles a cusp -> skip
        }
        problem.AddResidualBlock(SmoothnessCost::Create(config_.w_smooth * SmoothRatio(i)),
                                 nullptr, pts[i - 1].data(), pts[i].data(), pts[i + 1].data());
        problem.AddResidualBlock(CurvatureCost::Create(config_.max_curvature, config_.w_curvature),
                                 nullptr, pts[i - 1].data(), pts[i].data(), pts[i + 1].data());
    }

    // Length over consecutive pairs within one monotonic segment. This is the term
    // that removes long-wavelength meanders (see LengthCost): every free point is
    // pulled toward a shorter, straighter path, while the obstacle barrier forbids
    // cutting corners. Skipped across a cusp and for fully-frozen pairs (constant
    // residual, no effect).
    for (int i = 0; i + 1 < N; ++i) {
        if (init[i].gear != init[i + 1].gear || (fixed[i] && fixed[i + 1])) {
            continue;
        }
        problem.AddResidualBlock(LengthCost::Create(config_.w_length),
                                 nullptr, pts[i].data(), pts[i + 1].data());
    }

    for (int i = 0; i < N; ++i) {
        if (fixed[i]) {
            problem.SetParameterBlockConstant(pts[i].data());
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = config_.max_iterations;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = 4;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << "[TrajectoryOptimizer] " << summary.BriefReport() << std::endl;

    // Rebuild the trajectory: keep the gear structure, recompute headings from the
    // optimized geometry, and preserve the commanded start/goal orientations.
    Trajectory out(N);
    for (int i = 0; i < N; ++i) {
        out[i].x = pts[i][0];
        out[i].y = pts[i][1];
        out[i].gear = init[i].gear;
    }

    for (int i = 0; i < N; ++i) {
        double motion_dir;
        if (i < N - 1) {
            motion_dir = std::atan2(out[i + 1].y - out[i].y, out[i + 1].x - out[i].x);
        } else {
            motion_dir = std::atan2(out[i].y - out[i - 1].y, out[i].x - out[i - 1].x);
        }
        out[i].theta = out[i].gear >= 0 ? motion_dir : NormalizeAngle(motion_dir + M_PI);
    }
    // Endpoints keep their exact commanded orientation.
    out.front().theta = init.front().theta;
    out.back().theta = init.back().theta;

    return out;
}
