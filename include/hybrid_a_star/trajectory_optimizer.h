/*******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022 Zhang Zhimeng
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef HYBRID_A_STAR_TRAJECTORY_OPTIMIZER_H
#define HYBRID_A_STAR_TRAJECTORY_OPTIMIZER_H

#include "trajectory.h"
#include "esdf.h"

/*!
 * Back-end trajectory optimization using the Ceres non-linear least squares
 * solver. Given the (reversal-cleaned) front-end trajectory it refines the (x, y)
 * of every interior point while holding the start, the goal and every gear-shift
 * (cusp) point fixed. The cost combines:
 *
 *   - smoothness   : second difference of consecutive points (small curvature)
 *   - reference     : stay close to the front-end path (well-posed, stable)
 *   - obstacle      : keep a safety clearance using the ESDF (differentiable)
 *   - curvature     : soft limit on path curvature => bounded turning radius,
 *                     i.e. the non-holonomic / kinematic feasibility of the car
 *   - length        : total arc length; shortens unnecessary meanders that the
 *                     local (second-difference) smoothness term barely touches
 *
 * All multi-point terms are added only inside a monotonic-direction segment, so
 * the optimizer never smooths across a reversal.
 *
 * Ceres headers are intentionally kept out of this file so that translation units
 * that merely need the public interface do not have to compile against Ceres.
 */
class TrajectoryOptimizer {
public:
    struct Config {
        // Residual weights.
        double w_smooth = 2.0;
        double w_reference = 0.1;
        double w_obstacle = 1.0;
        double w_curvature = 2.0;
        // Total-length penalty. A second-difference (curvature) smoothness term
        // flattens sharp wiggles but barely affects long-wavelength S-curves
        // (their per-point curvature is tiny), so a single solve leaves a slow
        // meander behind. This term gives every interior point a direct incentive
        // to straighten out; keep it small relative to w_smooth (its residual is
        // ~ds, not ~ds^2). The obstacle barrier still forbids cutting corners.
        double w_length = 0.1;

        // Obstacle clearance model. More circles -> smaller radius -> the clearance
        // requirement approaches the true half-width, so the term behaves as a
        // collision barrier instead of over-pushing the path away from walls
        // (which, against fixed cusp points, would create kinks in tight spaces).
        double safety_margin = 0.1;   // extra clearance beyond the covering circles (m)
        int num_circles = 5;          // vehicle footprint approximated by this many circles
        // Reciprocal-barrier floor band (m): how close to the hard floor the penalty
        // is allowed to grow before switching to a (steep) linear continuation.
        // Smaller -> stiffer barrier (harder for smoothness to overpower).
        double obstacle_eps = 0.05;

        // Kinematics.
        double max_curvature = 0.2;   // 1/m, = tan(max_steer) / wheel_base

        // Smoothness taper near reversal (cusp) points. Strong smoothing flattens
        // the natural turn-in geometry around a gear change, so the smoothness weight
        // is scaled down the closer a point is to a cusp: full weight beyond
        // cusp_taper_dist, down to (w_smooth * cusp_taper_min_ratio) right next to the
        // cusp. The reference and curvature terms still apply there, so the maneuver
        // stays anchored to the (feasible) front-end shape instead of being pulled
        // straight.
        double cusp_taper_dist = 2.0;      // arc length (m) over which the taper acts
        double cusp_taper_min_ratio = 0.1; // smoothness weight ratio at the cusp

        // Segments (between two cusps, or a cusp and an endpoint) shorter than this
        // are frozen entirely: their points are held at the front-end geometry rather
        // than optimized, so a tight maneuver between two nearby reversal points is
        // not pulled straight. 0 disables this.
        double min_opt_segment_length = 2.0;

        // Hard-fix (no optimization at all) zones. Unlike the smoothness taper, every
        // point inside these zones is held at the front-end geometry.
        //   cusp_fix_dist     : freeze all points within this arc length (m) of a
        //                       reversal point -> the trajectory just before and after
        //                       a cusp is preserved exactly.
        //   endpoint_fix_dist : freeze all points within this arc length (m) of the
        //                       start / goal (in addition to always locking the start
        //                       and goal position + heading).
        double cusp_fix_dist = 0.0;
        double endpoint_fix_dist = 0.0;

        // Vehicle shape (matches HybridAStar::SetVehicleShape).
        double vehicle_length = 4.7;
        double vehicle_width = 2.0;
        double rear_axle_dist = 1.3;

        int max_iterations = 150;
    };

    TrajectoryOptimizer() = default;

    explicit TrajectoryOptimizer(const Config &config) : config_(config) {}

    //! Set (or replace) the distance field used by the obstacle term. The ESDF
    //! must outlive every subsequent Optimize() call.
    void SetEsdf(const ESDF *esdf) { esdf_ = esdf; }

    //! Derive the curvature limit from the car's wheel base and max steer angle.
    void SetKinematics(double wheel_base, double max_steering_radian) {
        if (wheel_base > 1e-6) {
            config_.max_curvature = std::tan(std::abs(max_steering_radian)) / wheel_base;
        }
    }

    void SetVehicleShape(double length, double width, double rear_axle_dist) {
        config_.vehicle_length = length;
        config_.vehicle_width = width;
        config_.rear_axle_dist = rear_axle_dist;
    }

    Config &config() { return config_; }

    /*!
     * Optimize a trajectory. Gears/cusps are preserved; (x, y) of interior points
     * are refined and headings are recomputed from the optimized geometry. If no
     * ESDF is set or the trajectory is too short the input is returned unchanged.
     */
    Trajectory Optimize(const Trajectory &init) const;

private:
    Config config_;
    const ESDF *esdf_ = nullptr;
};

#endif //HYBRID_A_STAR_TRAJECTORY_OPTIMIZER_H
