#pragma once

#include <limits>
#include <string>
#include <vector>
#include <stdexcept>

#include <Eigen/Eigen>

#include <control_arch/utils/state_t.h>

namespace similarity
{
  // Dynamic Time Warping: A metric for computing similarity between two temporal sequences: https://en.wikipedia.org/wiki/Dynamic_time_warping
  // This function computes the similarity between the euclidean distance of the positions of states.
  inline double DynamicTimeWarpingDistance(const std::vector<state_t>& traj1, const std::vector<state_t>& traj2)
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
  inline double DiscreteFrechetDistance(const std::vector<state_t>& traj1, const std::vector<state_t>& traj2)
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
      // std::cout << "dist " << dist;
      if (std::isnan(dist))
      {
        std::cout << "DIST IS NAN" << std::endl;
        return std::numeric_limits<double>::infinity();
      }
      if (CA(i, j) > -0.9)
      {
        // std::cout << " case 1" << std::endl;
        return CA(i, j);
      }
      else if (i == 0 && j == 0)
      {
        // std::cout << " case 2" << std::endl;

        CA(i, j) = (traj1[0].pos - traj2[0].pos).norm();
      }
      else if (i > 0 && j == 0)
      {
        // std::cout << " case 3" << std::endl;

        double dist = (traj1[i].pos - traj2[0].pos).norm();
        std::cout << dist << std::endl;
        CA(i, j) = std::max(C(i-1, 0), (traj1[i].pos - traj2[0].pos).norm());
      }
      else if (i == 0 && j > 0 )
      {
        // std::cout << " case 4" << std::endl;

        CA(i, j) = std::max(C(0, j-1), (traj1[0].pos - traj2[j].pos).norm());
      }
      else if (i > 0 && j > 0)
      {
        // std::cout << " case 5" << std::endl;

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


  inline void testDFD()
  {
    std::vector<state_t> traj1;
    std::vector<state_t> traj2;
    for (int i = 0; i < 10; i++)
    {
      state_t pos1;
      state_t pos2;
      pos1.pos = geometry_utils::Vec3(1.0*i, 2.0, 3.0);
      pos2.pos = geometry_utils::Vec3(1.0, 2.0*i, 3.0);
      traj1.push_back(pos1);
      traj2.push_back(pos2);
    }
    double cm = DiscreteFrechetDistance(traj1, traj2);
    std::cout << "DISCRETE FRECHET DISANCE: " << cm << std::endl;

    return;
  }

}
