/*
constrainedPolynomial.
Copyright (C) 2021 Xuning Yang

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
#include <Eigen/LU>

#include <trajectory_calculators/PolynomialCalculator.h>
#include <planning_representations/FlatState.h>

// typedef std::array<double, 4> end_point_t;
// typedef Eigen::Matrix<double, 8, 1> Vec8_t;
// typedef Eigen::Matrix<double, 9, 1> Vec9_t;

// inputs are arrays of 4: x, y, z, and yaw.
// Fits a 7th order polynomial so that pos, vel, acc, and jerk match at the endpoints.


namespace planner {


inline std::vector<Vec8_t, Eigen::aligned_allocator<Vec8_t>> constrainedPolynomial(planner::FlatState start_state, planner::FlatState end_state, double T)
{
  PolynomialCalculator poly_calc;

  end_point_t start[4] =
      { {start_state.pos(0), start_state.vel(0), 0, 0 },
        {start_state.pos(1), start_state.vel(1), 0, 0 },
        {start_state.pos(2), start_state.vel(2), 0, 0 },
        {start_state.yaw, start_state.dyaw, 0, 0 } };

  end_point_t end[4] =
      { {end_state.pos(0), end_state.vel(0), 0, 0 },
        {end_state.pos(1), end_state.vel(1), 0, 0 },
        {end_state.pos(2), end_state.vel(2), 0, 0 },
        {end_state.yaw, end_state.dyaw, 0, 0 } };

  return poly_calc.get_poly4(start, end, T);
}

}
