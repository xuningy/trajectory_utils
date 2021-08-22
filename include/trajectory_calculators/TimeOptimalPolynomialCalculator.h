/*
time_optimal_polynomial_calculator
Copyright (C) 2020 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <array>
#include <vector>
#include <Eigen/Core>

/* Implementation of time optimal polynomial for x-y-z from:
A Computationally Efficient Motion Primitive for Quadrocopter Trajectory Generation
https://ieeexplore.ieee.org/document/7299672
from Mark W. Mueller, Markus Hehn, Raffaello D'Andrea.
*/
namespace planner {

class TimeOptimalPolynomialCalculator {
  public:
    TimeOptimalPolynomialCalculator() {};

    // Specify the final end point constraint for your trajectory.
    enum Constraint {POS_VEL_ACC, POS_VEL, POS_ACC, VEL_ACC, POS, VEL, ACC};

    Eigen::Vector3d perAxisTraj(const Eigen::Vector3d& abc, double t, double p0, double v0, double a0);

    Eigen::Vector3d alphaBetaGamma(double T, double p0, double v0, double a0, double pf, double vf, double af, TimeOptimalPolynomialCalculator::Constraint c);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
