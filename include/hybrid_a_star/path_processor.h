/*******************************************************************************
 * Trajectory post-processing: removal of redundant reversals ("磨来磨去").
 *
 * Hybrid A* frequently produces tiny forward/backward wiggles near the start and
 * goal (sequences of short gear changes that make almost no net progress). Those
 * cusps are visually ugly and awkward to execute, yet they are *not* genuine
 * maneuvers such as a three-point turn. This class detects such short reversal
 * segments and collapses them, while preserving reversals that are actually
 * required to re-orient the vehicle.
 ******************************************************************************/

#ifndef HYBRID_A_STAR_PATH_PROCESSOR_H
#define HYBRID_A_STAR_PATH_PROCESSOR_H

#include "trajectory.h"

#include <functional>

class PathProcessor {
public:
    //! Returns true when the vehicle at (x, y, theta) is collision free.
    typedef std::function<bool(double x, double y, double theta)> CollisionChecker;

    PathProcessor() = default;

    PathProcessor(double min_segment_length, double heading_tol, double step_size)
            : min_segment_length_(min_segment_length),
              heading_tol_(heading_tol),
              step_size_(step_size) {}

    /*!
     * Collapse short, redundant reversal segments.
     * @param traj           input trajectory with gears (see InferGears)
     * @param collision_free collision predicate used to validate each stitch
     * @return the cleaned trajectory with recomputed gears
     */
    Trajectory RemoveRedundantReversals(const Trajectory &traj,
                                        const CollisionChecker &collision_free) const;

    //! Number of reversal points (cusps) in a trajectory, for logging.
    static int CountCusps(const Trajectory &traj);

    /*!
     * Resample each monotonic-direction segment to (approximately) uniform
     * arc-length spacing. The Ceres obstacle term can bunch points together near
     * walls; uniform spacing produces clean vehicle visualization, sane curvature,
     * and an execution-friendly trajectory. Gear-shift (cusp) points, the start and
     * the goal are preserved exactly.
     */
    static Trajectory Resample(const Trajectory &traj, double spacing);

    void set_min_segment_length(double v) { min_segment_length_ = v; }
    void set_heading_tol(double v) { heading_tol_ = v; }
    void set_step_size(double v) { step_size_ = v; }

private:
    //! One pass of the collapse loop. Sets *changed when a wiggle was removed.
    Trajectory CollapseOnce(const Trajectory &traj,
                            const CollisionChecker &collision_free,
                            bool *changed) const;

    static double SegmentLength(const Trajectory &traj, int lo, int hi);

    static double NormalizeAngle(double angle);

    //! Smallest absolute difference between two headings, in [0, pi].
    static double AngleDiff(double a, double b);

    //! Shorter-arc interpolation between two headings (t in [0, 1]).
    static double InterpAngle(double a, double b, double t);

    //! Stitch traj[anchor] -> traj[q] (q searched in [q_first, q_last]) as a straight
    //! common_gear move along dir_in. The body heading sweeps from dir_in to dir_out
    //! across the stitch for the collision footprint. On success writes the stitched
    //! trajectory to *out (gears re-inferred) and returns true.
    static bool TryStitchForward(const Trajectory &traj, int anchor, int q_first, int q_last,
                                 double dir_in, double dir_out, int common_gear,
                                 const CollisionChecker &collision_free,
                                 double heading_tol, double step_size, Trajectory *out);

    //! Mirrored variant for a trailing reversal: stitch traj[q] -> traj[anchor] where
    //! the anchor is the fixed goal and q is searched backwards in [q_last, q_first].
    static bool TryStitchBackward(const Trajectory &traj, int anchor, int q_first, int q_last,
                                  double dir_in, double dir_out, int common_gear,
                                  const CollisionChecker &collision_free,
                                  double heading_tol, double step_size, Trajectory *out);

    double min_segment_length_ = 3.0; // meters; segments shorter than this are collapse candidates
    double heading_tol_ = 0.6;        // rad; max net heading change for a wiggle to be "redundant"
    double step_size_ = 0.2;          // meters; resampling step of the stitched straight segment
};

#endif //HYBRID_A_STAR_PATH_PROCESSOR_H
