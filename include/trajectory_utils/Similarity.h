/*
Similarity.
Copyright (C) 2021 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <limits>
#include <string>
#include <vector>
#include <stdexcept>

#include <Eigen/Eigen>

#include <planning_representations/FlatState.h>

namespace similarity
{
  // Dynamic Time Warping: A metric for computing similarity between two temporal sequences: https://en.wikipedia.org/wiki/Dynamic_time_warping
  // This function computes the similarity between the euclidean distance of the positions of states.

  inline double DynamicTimeWarpingDistance(const std::vector<planner::FlatState>& traj1, const std::vector<planner::FlatState>& traj2)
  {
    // initialize with zero;
    Eigen::MatrixXd dtw = Eigen::MatrixXd::Constant(traj1.size(), traj2.size(), Eigen::Infinity);
    dtw(0, 0) = 0.0;

    for (size_t i = 1; i < traj1.size(); i++)
    {
      for (size_t j = 1; j < traj2.size(); j++)
      {
        double cost = (traj1[i].pos - traj2[j].pos).norm();
        dtw(i, j) = cost + std::min({dtw(i-1, j),       // insertion
                                     dtw(i, j-1),       // deletion
                                     dtw(i-1, j-1)      // match
                                   });
      }
    }

    return dtw(traj1.size()-1, traj2.size()-1);

  }


  // Discrete Frechet Distance: a measure of the similarity between two discrete curves in metric spaces: http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.90.937&rep=rep1&type=pdf
  // The two vectors must be of the same distance.

  inline double DiscreteFrechetDistance(const std::vector<planner::FlatState>& traj1, const std::vector<planner::FlatState>& traj2)
  {
    if (traj1.size() != traj2.size())
    {
      std::cout << "[similarity::DiscreteFrechetDistance] traj1.size(): " << traj1.size() << " traj2.size(): " << traj2.size() << std::endl;
      throw std::invalid_argument("[similarity::DiscreteFrechetDistance] Provided trajectories does not have the same length!");
    }
      // throw std::invalid_argument(std::string("[similarity::DiscreteFrechetDistance] Provided trajectories does not have the same length! traj1.size(): %d, traj2.size(): %d", traj1.size(), traj2.size()));

    if (traj1.empty())
      throw std::invalid_argument("[similarity::DiscreteFrechetDistance] trajectories does not contain any points!");

    Eigen::MatrixXd CA = Eigen::MatrixXd::Constant(traj1.size(), traj2.size(), -1.0);

    // coupling measure search function; recursive.
    std::function<double(int, int)> C;
    C = [&](int i, int j) mutable
    {
      double dist = (traj1[i].pos - traj2[j].pos).norm();
      if (std::isnan(dist))
      {
        std::cout << "[similarity::DiscreteFrechetDistance] DIST IS NAN" << std::endl;
        return std::numeric_limits<double>::infinity();
      }
      if (CA(i, j) > -0.9)
      {
        return CA(i, j);
      }
      else if (i == 0 && j == 0)
      {
        // std::cout << " case 2" << std::endl;

        CA(i, j) = (traj1[0].pos - traj2[0].pos).norm();
      }
      else if (i > 0 && j == 0)
      {
        double dist = (traj1[i].pos - traj2[0].pos).norm();
        CA(i, j) = std::max(C(i-1, 0), (traj1[i].pos - traj2[0].pos).norm());
      }
      else if (i == 0 && j > 0 )
      {
        CA(i, j) = std::max(C(0, j-1), (traj1[0].pos - traj2[j].pos).norm());
      }
      else if (i > 0 && j > 0)
      {
        CA(i, j) = std::max(
                          std::min({C(i-1, j), C(i-1, j-1), C(i, j-1)}),
                          (traj1[i].pos - traj2[j].pos).norm()
                        );
      }
      else
      {
        std::cout << "[similarity::DiscreteFrechetDistance] you shouldnt be here!" << std::endl;
      }
      // std::cout << "CA(" << i << ", " << j << "): " << CA(i, j) << std::endl;
      return CA(i, j);
    };

    double cm = C(traj1.size()-1, traj2.size()-1);
    return cm;
  }


  inline double DiscreteFrechetDistance(const std::vector<Eigen::Vector3d>& traj1, const std::vector<Eigen::Vector3d>& traj2)
  {
    if (traj1.size() != traj2.size())
    {
      std::cout << "[similarity::DiscreteFrechetDistance] traj1.size(): " << traj1.size() << " traj2.size(): " << traj2.size() << std::endl;
      throw std::invalid_argument("[similarity::DiscreteFrechetDistance] Provided trajectories does not have the same length!");
    }
      // throw std::invalid_argument(std::string("[similarity::DiscreteFrechetDistance] Provided trajectories does not have the same length! traj1.size(): %d, traj2.size(): %d", traj1.size(), traj2.size()));

    if (traj1.empty())
      throw std::invalid_argument("[similarity::DiscreteFrechetDistance] trajectories does not contain any points!");

    Eigen::MatrixXd CA = Eigen::MatrixXd::Constant(traj1.size(), traj2.size(), -1.0);

    // coupling measure search function; recursive.
    std::function<double(int, int)> C;
    C = [&](int i, int j) mutable
    {
      double dist = (traj1[i] - traj2[j]).norm();
      if (std::isnan(dist))
      {
        std::cout << "[similarity::DiscreteFrechetDistance] DIST IS NAN" << std::endl;
        return std::numeric_limits<double>::infinity();
      }
      if (CA(i, j) > -0.9)
      {
        return CA(i, j);
      }
      else if (i == 0 && j == 0)
      {
        // std::cout << " case 2" << std::endl;

        CA(i, j) = (traj1[0] - traj2[0]).norm();
      }
      else if (i > 0 && j == 0)
      {
        double dist = (traj1[i] - traj2[0]).norm();
        CA(i, j) = std::max(C(i-1, 0), (traj1[i] - traj2[0]).norm());
      }
      else if (i == 0 && j > 0 )
      {
        CA(i, j) = std::max(C(0, j-1), (traj1[0] - traj2[j]).norm());
      }
      else if (i > 0 && j > 0)
      {
        CA(i, j) = std::max(
                          std::min({C(i-1, j), C(i-1, j-1), C(i, j-1)}),
                          (traj1[i] - traj2[j]).norm()
                        );
      }
      else
      {
        std::cout << "[similarity::DiscreteFrechetDistance] you shouldnt be here!" << std::endl;
      }
      // std::cout << "CA(" << i << ", " << j << "): " << CA(i, j) << std::endl;
      return CA(i, j);
    };

    double cm = C(traj1.size()-1, traj2.size()-1);
    return cm;
  }


  inline void testDFD()
  {
    std::vector<planner::FlatState> traj1;
    std::vector<planner::FlatState> traj2;
    for (int i = 0; i < 10; i++)
    {
      planner::FlatState pos1;
      planner::FlatState pos2;
      pos1.pos = Eigen::Vector3d(1.0*i, 2.0, 3.0);
      pos2.pos = Eigen::Vector3d(1.0, 2.0*i, 3.0);
      traj1.push_back(pos1);
      traj2.push_back(pos2);
    }
    double cm = DiscreteFrechetDistance(traj1, traj2);
    std::cout << "DISCRETE FRECHET DISANCE: " << cm << std::endl;

    return;
  }

}
