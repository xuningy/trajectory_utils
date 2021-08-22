/*
Line Utils
Copyright (C) 2021 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <stdexcept>
#include <vector>
#include <Eigen/Eigen>

namespace line_utils {

// https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
inline Eigen::Vector3d closestPointOnALine(const std::vector<Eigen::Vector3d>& line, const Eigen::Vector3d& point)
{

  double t = -((line[0] - point).dot(line[1] - line[0]))/(line[1] - line[0]).squaredNorm();



  // get the point by using t
  Eigen::Vector3d closest_point = line[0] + (line[1]-line[0])*t;

  // check that t is between 0 and 1
  if (t < 0 || t > 1) {
    // std::cout << "scaling factor t " << t << " is less than 0 or greater than 1! Point is outside line segment range" << std::endl;
  }

  return closest_point;
}

// https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
inline std::tuple<Eigen::Vector3d, double> closestPointOnALine2(const std::vector<Eigen::Vector3d>& line, const Eigen::Vector3d& point)
{

  double t = -((line[0] - point).dot(line[1] - line[0]))/(line[1] - line[0]).squaredNorm();



  // get the point by using t
  Eigen::Vector3d closest_point = line[0] + (line[1]-line[0])*t;

  // check that t is between 0 and 1
  if (t < 0 || t > 1) {
    // std::cout << "scaling factor t " << t << " is less than 0 or greater than 1! Point is outside line segment range" << std::endl;
  }

  return { closest_point, t };
}

// https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
inline double shortestDistanceToLine(const std::vector<Eigen::Vector3d>& line, const Eigen::Vector3d& point)
{

  double d = ((point-line[0]).cross(point - line[1])).norm()/(line[1] - line[0]).norm();

  return d;
}

} // end namespace
