/*
time_optimal_polynomial_calculator
Copyright (C) 2020 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <trajectory_calculators/TimeOptimalPolynomialCalculator.h>

/* Implementation of time optimal polynomial for x-y-z from:
A Computationally Efficient Motion Primitive for Quadrocopter Trajectory Generation
https://ieeexplore.ieee.org/document/7299672
from Mark W. Mueller, Markus Hehn, Raffaello D'Andrea.
*/
namespace planner {

Eigen::Vector3d TimeOptimalPolynomialCalculator::perAxisTraj(const Eigen::Vector3d& abc, double t, double p0, double v0, double a0)
{
  double alpha = abc(0);
  double beta = abc(1);
  double gamma = abc(2);

  double pos = alpha/120*t*t*t*t*t + beta/24*t*t*t*t + gamma/6*t*t*t + a0/2*t*t + v0*t + p0;
  double vel = alpha/24*t*t*t*t + beta/6*t*t*t+ gamma/2*t*t + a0*t + v0;
  double acc = alpha/6*t*t*t + beta/2*t*t + gamma*t + a0;

  Eigen::Vector3d pos_vel_acc(pos, vel, acc);
  return pos_vel_acc;
}

Eigen::Vector3d TimeOptimalPolynomialCalculator::alphaBetaGamma(double T, double p0, double v0, double a0, double pf, double vf, double af, TimeOptimalPolynomialCalculator::Constraint c)
{
  double dp = pf - p0 - v0*T - 0.5*a0*T*T;
  double dv = vf - v0 - a0*T;
  double da = af - a0;

  Eigen::Vector3d abc;

  switch (c)
  {
    case POS_VEL_ACC :
    {
      Eigen::Matrix3d M;
      M <<  720,       -260*T,     60*T*T,
            -360*T,    168*T*T,     -24*T*T*T,
            60*T*T,    -24*T*T*T,    3*T*T*T*T;
      Eigen::Vector3d d(dp, dv, da);
      abc = 1/(T*T*T*T*T) * M * d;
      break;
    }
    case POS_VEL :
    {
      Eigen::Matrix<double, 3, 2> M;
      M <<  320,     -120*T,
            -200*T,   72*T*T,
             40*T*T,   -12*T*T*T;
      Eigen::Vector2d d(dp, dv);
      abc = 1/(T*T*T*T*T) * M * d;
      break;
    }
    case POS_ACC :
    {
      Eigen::Matrix<double, 3, 2> M;
      M <<  90,     -15*T*T,
            -90*T,   15*T*T*T,
             30*T*T,   -3*T*T*T*T;
      Eigen::Vector2d d(dp, da);
      abc = 1/(2*T*T*T*T*T)* M * d;
      break;
    }
    case VEL_ACC :
    {
      Eigen::Matrix<double, 3, 2> M;
      M <<  0,     0,
             -12,   6*T,
             6*T,   -2*T*T;
      Eigen::Vector2d d(dv, da);
      abc =  1/(T*T*T) * M * d;
      break;
    }
    case POS :
    {
      Eigen::Vector3d M;
      M <<  20, -20*T, 10*T*T;
      abc =  1/(T*T*T*T*T) * M * dp;
      break;
    }
    case VEL :
    {
      Eigen::Vector3d M;
      M <<  0, -3, 3*T;
      abc =  1/(T*T*T) * M * dv;
      break;
    }
    case ACC :
    {
      Eigen::Vector3d M;
      M <<  0, 0, 1;
      abc =  1/T * M * da;
      break;
    }
  }

  return abc;
}

}
