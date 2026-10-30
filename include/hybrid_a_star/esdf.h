/*******************************************************************************
 * Euclidean Signed Distance Field (ESDF) built from an occupancy grid.
 *
 * The back-end Ceres optimizer needs a smooth, differentiable notion of
 * "distance to the nearest obstacle". The grid-based rectangle collision check
 * in HybridAStar is binary and non-differentiable, so we build a dense distance
 * field once per map and query it (bilinearly for the reverse-removal check, and
 * through a ceres::BiCubicInterpolator inside the cost functors).
 *
 * The distance transform follows:
 *   P. Felzenszwalb and D. Huttenlocher,
 *   "Distance Transforms of Sampled Functions", Theory of Computing, 2012.
 * which computes the exact Euclidean distance transform in O(N) per row/column.
 ******************************************************************************/

#ifndef HYBRID_A_STAR_ESDF_H
#define HYBRID_A_STAR_ESDF_H

#include <cstdint>
#include <vector>

class ESDF {
public:
    ESDF() = default;

    /*!
     * Build the distance field from an occupancy grid.
     * @param occupancy row-major grid, size width * height. A cell is treated as
     *        an obstacle when occupancy[row * width + col] >= occupied_threshold.
     * @param width     number of columns
     * @param height    number of rows
     * @param resolution meters per cell
     * @param origin_x  world x of the grid origin (lower-left corner)
     * @param origin_y  world y of the grid origin (lower-left corner)
     * @param occupied_threshold cells >= this value are obstacles (default 1)
     */
    void Build(const std::vector<uint8_t> &occupancy, int width, int height,
               double resolution, double origin_x, double origin_y,
               uint8_t occupied_threshold = 1u);

    bool IsValid() const { return !distance_.empty(); }

    //! Signed distance (meters) at world coordinate (x, y), bilinearly interpolated:
    //! positive outside obstacles, negative inside. Points outside the grid clamp to
    //! the border.
    double Distance(double x, double y) const;

    //! Raw distance buffer (meters), row-major: distance_[row * width + col].
    const std::vector<double> &data() const { return distance_; }

    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }
    double origin_x() const { return origin_x_; }
    double origin_y() const { return origin_y_; }

    //! World -> continuous grid coordinate helpers (also used by the cost functor).
    double ToCol(double x) const { return (x - origin_x_) / resolution_; }
    double ToRow(double y) const { return (y - origin_y_) / resolution_; }

private:
    //! 1-D squared distance transform of a single row/column (Felzenszwalb).
    static void DistanceTransform1D(const std::vector<double> &f, std::vector<double> &d);

    //! Full 2-D squared distance transform (in place), cell units.
    static void SquaredDistanceTransform2D(std::vector<double> &grid, int width, int height);

    std::vector<double> distance_; // meters, row-major
    int width_ = 0;
    int height_ = 0;
    double resolution_ = 1.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
};

#endif //HYBRID_A_STAR_ESDF_H
