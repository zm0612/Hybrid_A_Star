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

#include "type.h"

#include <cmath>

/*!
 * Trajectory optimization function
 *
 * Note: According to hyrbid a star's paper, I have not reproduced very good results.
 *       However, I have obtained a good trajectory in the front-end trajectory search stage,
 *       so even if I don't use back-end optimization,
 *       the trajectory can basically meet the usage requirements.
 */
class TrajectoryOptimizer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TrajectoryOptimizer() = default;

    template<typename CheckCollisionFunction, typename NearestObstacleFunction>
    VectorVec3d Optimize(CheckCollisionFunction CheckCollision, NearestObstacleFunction NearestObstacle,
                         const VectorVec3d &path) {
        const unsigned int N = path.size();
        VecXd grad_f;
        grad_f.resize(2 * N);
        grad_f.setZero();

        VecXd path_matrix(N * 2);
        for (unsigned int i = 0; i < N; ++i) {
            path_matrix.block(i * 2, 0, 2, 1) = path[i].head(2);
        }

        VecXd r, prev_d, prev_r;

        unsigned int max_iteration = 100u;
        for (unsigned int i = 0; i < max_iteration; ++i) {
            grad_f.setZero();

            for (unsigned int j = 2; j < N - 2; ++j) {
                const Vec2d &x_2_j = path_matrix.block((j - 2) * 2, 0, 2, 1);
                const Vec2d &x_1_j = path_matrix.block((j - 1) * 2, 0, 2, 1);
                const Vec2d &x_j = path_matrix.block(j * 2, 0, 2, 1);
                const Vec2d &x_j_1 = path_matrix.block((j + 1) * 2, 0, 2, 1);
                const Vec2d &x_j_2 = path_matrix.block((j + 2) * 2, 0, 2, 1);

                //non holonomic constraints
                grad_f.block(2 * j, 0, 2, 1) +=
                        w_k * CurvatureTerm(x_1_j, x_j, x_j_1);

                // obstacle constraints
                grad_f.block(2 * j, 0, 2, 1) +=
                        w_o * ObstacleTerm(x_j, NearestObstacle(x_j.x(), x_j.y()));

                // smooth constraints
                grad_f.block(2 * j, 0, 2, 1) +=
                        w_s * SmoothTerm(x_2_j, x_1_j, x_j, x_j_1, x_j_2);
            }

            prev_d = -grad_f;
            path_matrix = path_matrix + alpha_ * prev_d * (w_k + w_s + w_o);
        }

        VectorVec3d optimized_path;
        for (unsigned int i = 0; i < N - 1; ++i) {
            Vec2d delta = path_matrix.block(2 * (i + 1), 0, 2, 1)
                          - path_matrix.block(2 * i, 0, 2, 1);
            double theta = std::atan2(delta.y(), delta.x());

            optimized_path.emplace_back(Vec3d(path_matrix(2 * i, 0),
                                              path_matrix(2 * i + 1, 0), theta));
        }

        return optimized_path;
    }

private:

    template<typename T>
    T Clamp(const T value, T bound1, T bound2) {
        if (bound1 > bound2) {
            std::swap(bound1, bound2);
        }

        if (value < bound1) {
            return bound1;
        } else if (value > bound2) {
            return bound2;
        }
        return value;
    }

    inline Vec2d ObstacleTerm(const Vec2d &x_i, const Vec2d &obstacle_coord) const {
        Vec2d gradient;
        gradient.setZero();

        double obstacle_distance = (x_i - obstacle_coord).norm();
        if (obstacle_distance <= d_max) {
            gradient = 2 * (obstacle_distance - d_max) * (x_i - obstacle_coord).normalized();
        }

        return gradient;
    }

    static inline Vec2d SmoothTerm(const Vec2d &x_2_j, const Vec2d &x_1_j, const Vec2d &x_j,
                                   const Vec2d &x_j_1, const Vec2d &x_j_2) {
        return 2.0 * (x_2_j - 4.0 * x_1_j + 6 * x_j - 4 * x_j_1 + x_j_2);
    }

    inline Vec2d CurvatureTerm(const Vec2d &x_1_j,
                               const Vec2d &x_j,
                               const Vec2d &x_j_1) {
        Vec2d delta_x_j = x_j - x_1_j;
        Vec2d delta_x_j_1 = x_j_1 - x_j;

        double delta_x_j_norm = delta_x_j.norm();
        double delta_x_j_1_norm = delta_x_j_1.norm();

        if (delta_x_j_norm > 0.0 && delta_x_j_1_norm > 0.0) {
            double delta_phi = DeltaPhi(x_1_j, x_j, x_j_1);
            if (delta_phi / delta_x_j_norm <= k_max) {
                return Vec2d::Zero();
            } else {
                double temp_1 = 1.0 / delta_x_j_norm;
                double temp_2 = -1.0 / std::sqrt(1 - std::pow(std::cos(delta_phi), 2));
                double temp_4 = delta_phi / delta_x_j_norm / delta_x_j_norm;

                Vec2d p1 = OrthogonalComplement(delta_x_j, -delta_x_j_1) / delta_x_j_1.norm() / delta_x_j.norm();
                Vec2d p2 = OrthogonalComplement(-delta_x_j_1, delta_x_j) / delta_x_j_1.norm() / delta_x_j.norm();
                Vec2d temp_3 = -p1 - p2;

                Vec2d J_1_j = 2.0 * (delta_phi / delta_x_j_norm - k_max) *
                              (temp_1 * temp_2 * p2 + temp_4 * delta_x_j.normalized());
                Vec2d J_j = 2.0 * (delta_phi / delta_x_j_norm - k_max) *
                            (temp_1 * temp_2 * temp_3 - temp_4 * delta_x_j.normalized());
                Vec2d J_j_1 = 2.0 * (delta_phi / delta_x_j_norm - k_max) * (temp_1 * temp_2 * p1);

                Vec2d grad = 0.25 * J_1_j + 0.5 * J_j + 0.25 * J_j_1;

                if (std::isnan(grad.x()) || std::isnan(grad.y())) {
                    return Vec2d::Zero();
                } else {
                    return grad;
                }
            }
        }

        return Vec2d::Zero();
    }

    inline double DeltaPhi(const Vec2d &x0, const Vec2d &x1, const Vec2d &x2) {
        Vec2d delta_x0 = x1 - x0;
        Vec2d delta_x1 = x2 - x1;
        double project_length = delta_x0.transpose() * delta_x1;

        return std::acos(Clamp<double>(project_length / (delta_x0.norm() * delta_x1.norm()), -1, 1));
    }

    static inline Vec2d OrthogonalComplement(const Vec2d &a, const Vec2d &b) {
        return a - a.transpose() * b.normalized() * b.normalized();
    }

private:
    double alpha_ = 0.1;
    double w_o = 0.05;
    double w_k = 0.01;
    double w_s = 0.2;

    double k_max = 0.01;
    double d_max = 3.0;
};

#endif //HYBRID_A_STAR_TRAJECTORY_OPTIMIZER_H