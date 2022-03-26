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

#ifndef HYBRID_A_STAR_RS_PATH_H
#define HYBRID_A_STAR_RS_PATH_H

#include "type.h"

#include <cmath>
#include <limits>

/*!
 * Refer to this paper:
 *   1990 Optimal paths for a car that goes both forwards and backwards. J. A. Reeds, L. A. Shepp. Pacific J.
 *
 * Notes: There are many notation errors in this paper, and there are many formula derivation errors.
 *        When reading the paper, please note that the errors! Errors mainly concentrated in Section 8.
 */
class RSPath {
public:
    RSPath() = delete;

    explicit RSPath(double turning_radius = 1.0);

    enum RSPathSegmentType {
        N = 0, L = 1, S = 2, R = 3
    };
    static const RSPathSegmentType RS_path_segment_type[18][5];

    struct RSPathData {
    public:
        explicit RSPathData(const RSPathSegmentType *type = RS_path_segment_type[0],
                            double t = std::numeric_limits<double>::max(),
                            double u = 0.0, double v = 0.0, double w = 0.0, double x = 0.0) : type_(type) {
            length_[0] = t;
            length_[1] = u;
            length_[2] = v;
            length_[3] = w;
            length_[4] = x;
            total_length_ = std::fabs(length_[0]) + std::fabs(length_[1]) + std::fabs(length_[2])
                            + std::fabs(length_[3]) + std::fabs(length_[4]);
        }

        double Length() const {
            return total_length_;
        }

    public:
        double length_[5]{};
        const RSPathSegmentType *type_;

    private:
        double total_length_;
    };

    /*!
     * Calculate the actual distance from (x_0, y_0, yaw_0) to (x_1, y_1, yaw_1)
     * @param x_0
     * @param y_0
     * @param yaw_0
     * @param x_1
     * @param y_1
     * @param yaw_1
     * @return
     */
    double Distance(double x_0, double y_0, double yaw_0,
                    double x_1, double y_1, double yaw_1);

    /*!
     * Get a reed shepp path
     * @param start_state start state including position and yaw
     * @param goal_state goal state including position and yaw
     * @param step_size actual step size for collision detection
     * @return Discrete points of reed shepp path, including position and yaw
     */
    TypeVectorVecd<3> GetRSPath(const Vec3d &start_state, const Vec3d &goal_state, double step_size, double &length);

    RSPathData GetRSPath(double x_0, double y_0, double yaw_0,
                         double x_1, double y_1, double yaw_1);

    RSPathData GetRSPath(double x, double y, double phi);

private:

    static void CSC(double x, double y, double phi, RSPathData &path);

    void CCC(double x, double y, double phi, RSPathData &path);

    static void CCCC(double x, double y, double phi, RSPathData &path);

    static void CCSC(double x, double y, double phi, RSPathData &path);

    static void CCSCC(double x, double y, double phi, RSPathData &path);

    static inline double Mod2Pi(double x);

    static inline void Polar(double x, double y, double &r, double &theta);

    static inline void TauOmega(double u, double v, double xi, double eta, double phi, double &tau, double &omega);

    // Paper P390, formula 8.1
    static inline bool LpSpLp(double x, double y, double phi, double &t, double &u, double &v);

    // Paper P390, formula 8.2
    static inline bool LpSpRp(double x, double y, double phi, double &t, double &u, double &v);

    // Paper P390 formula 8.3 and 8.4.
    // There is an error in the deduction of the formula in the paper.
    // It can be deduced by itself according to the inscribed circle.
    static inline bool LpRmL(double x, double y, double phi, double &t, double &u, double &v);

    // Paper P391 formula 8.7
    static inline bool LpRupLumRm(double x, double y, double phi, double &t, double &u, double &v);

    // Paper P391 formula 8.8
    static inline bool LpRumLumRp(double x, double y, double phi, double &t, double &u, double &v);

    // Paper P391 formula 8.9
    static inline bool LpRmSmLm(double x, double y, double phi, double &t, double &u, double &v);

    // Paper P391 formula 8.10
    static inline bool LpRmSmRm(double x, double y, double phi, double &t, double &u, double &v);

    // Paper P391 formula 8.11
    // There is an error in the deduction of the formula in the paper.
    // It can be deduced by itself according to the inscribed circle.
    static inline bool LpRmSLmRp(double x, double y, double phi, double &t, double &u, double &v);

private:
    double turning_radius_ = 1.0;
};

#endif //HYBRID_A_STAR_RS_PATH_H
