/*******************************************************************************
 * Euclidean Signed Distance Field implementation. See esdf.h for references.
 ******************************************************************************/

#include "hybrid_a_star/esdf.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
const double kInf = std::numeric_limits<double>::max() / 4.0;
}

void ESDF::DistanceTransform1D(const std::vector<double> &f, std::vector<double> &d) {
    const int n = static_cast<int>(f.size());
    d.assign(n, 0.0);

    std::vector<int> v(n, 0);       // locations of parabolas in the lower envelope
    std::vector<double> z(n + 1, 0.0); // boundaries between parabolas

    int k = 0;
    v[0] = 0;
    z[0] = -kInf;
    z[1] = +kInf;

    for (int q = 1; q < n; ++q) {
        double s = ((f[q] + static_cast<double>(q) * q) - (f[v[k]] + static_cast<double>(v[k]) * v[k]))
                   / (2.0 * q - 2.0 * v[k]);
        while (s <= z[k]) {
            --k;
            s = ((f[q] + static_cast<double>(q) * q) - (f[v[k]] + static_cast<double>(v[k]) * v[k]))
                / (2.0 * q - 2.0 * v[k]);
        }
        ++k;
        v[k] = q;
        z[k] = s;
        z[k + 1] = +kInf;
    }

    k = 0;
    for (int q = 0; q < n; ++q) {
        while (z[k + 1] < q) {
            ++k;
        }
        const double dq = static_cast<double>(q) - v[k];
        d[q] = dq * dq + f[v[k]];
    }
}

void ESDF::Build(const std::vector<uint8_t> &occupancy, int width, int height,
                 double resolution, double origin_x, double origin_y,
                 uint8_t occupied_threshold) {
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;

    const int n = width * height;

    // Two unsigned distance transforms give a SIGNED field:
    //   d_out = distance to the nearest obstacle (0 on obstacle cells)
    //   d_in  = distance to the nearest free cell (0 on free cells)
    //   signed distance = d_out - d_in  (>0 outside, <0 inside obstacles)
    // The signed field is essential: an unsigned field is flat (0) everywhere inside
    // an obstacle, so a point pushed inside would get zero gradient and never be
    // driven back out. With the signed field the magnitude grows toward the interior,
    // giving the obstacle barrier a restoring gradient everywhere.
    std::vector<double> grid_out(n, kInf), grid_in(n, kInf);
    for (int i = 0; i < n && i < static_cast<int>(occupancy.size()); ++i) {
        if (occupancy[i] >= occupied_threshold) {
            grid_out[i] = 0.0; // obstacle cell -> seed for d_out
        } else {
            grid_in[i] = 0.0;  // free cell     -> seed for d_in
        }
    }

    SquaredDistanceTransform2D(grid_out, width, height);
    SquaredDistanceTransform2D(grid_in, width, height);

    distance_.resize(n);
    for (int i = 0; i < n; ++i) {
        distance_[i] = (std::sqrt(grid_out[i]) - std::sqrt(grid_in[i])) * resolution_;
    }
}

void ESDF::SquaredDistanceTransform2D(std::vector<double> &grid, int width, int height) {
    std::vector<double> f, d;

    // Transform along columns (fixed col, vary row).
    f.resize(height);
    for (int col = 0; col < width; ++col) {
        for (int row = 0; row < height; ++row) {
            f[row] = grid[row * width + col];
        }
        DistanceTransform1D(f, d);
        for (int row = 0; row < height; ++row) {
            grid[row * width + col] = d[row];
        }
    }

    // Transform along rows (fixed row, vary col).
    f.resize(width);
    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            f[col] = grid[row * width + col];
        }
        DistanceTransform1D(f, d);
        for (int col = 0; col < width; ++col) {
            grid[row * width + col] = d[col];
        }
    }
}

double ESDF::Distance(double x, double y) const {
    if (distance_.empty()) {
        return 0.0;
    }

    double c = ToCol(x);
    double r = ToRow(y);

    // Clamp to the valid interpolation range [0, size - 1].
    c = std::min(std::max(c, 0.0), static_cast<double>(width_ - 1));
    r = std::min(std::max(r, 0.0), static_cast<double>(height_ - 1));

    const int c0 = static_cast<int>(std::floor(c));
    const int r0 = static_cast<int>(std::floor(r));
    const int c1 = std::min(c0 + 1, width_ - 1);
    const int r1 = std::min(r0 + 1, height_ - 1);

    const double tc = c - c0;
    const double tr = r - r0;

    const double d00 = distance_[r0 * width_ + c0];
    const double d01 = distance_[r0 * width_ + c1];
    const double d10 = distance_[r1 * width_ + c0];
    const double d11 = distance_[r1 * width_ + c1];

    const double d0 = d00 * (1.0 - tc) + d01 * tc;
    const double d1 = d10 * (1.0 - tc) + d11 * tc;

    return d0 * (1.0 - tr) + d1 * tr;
}
