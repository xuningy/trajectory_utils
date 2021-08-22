#pragma once

#include <array>
#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>

constexpr int poly_len = 8;
typedef std::array<double, 4> end_point_t;
typedef Eigen::Matrix<double, poly_len, 1> Vec8_t;
typedef Eigen::Matrix<double, poly_len+1, 1> Vec9_t;

// inputs are arrays of 4: x, y, z, and yaw.
// Fits a 7th order polynomial so that pos, vel, acc, and jerk match at the endpoints.
namespace planner {

class PolynomialCalculator {

  public:
    PolynomialCalculator();
    Eigen::Matrix<double, poly_len, 1> get_poly(const end_point_t &start, const end_point_t &end, double duration);
    std::vector<Vec8_t, Eigen::aligned_allocator<Vec8_t>> get_poly4(const end_point_t start[], const end_point_t end[], const double duration);
    void get_poly4(const end_point_t start[], const end_point_t end[], const double duration, std::vector<Vec8_t, Eigen::aligned_allocator<Vec8_t>> & ret);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    Eigen::Matrix<double, poly_len, poly_len> poly_mat_inv;
};

} // namespace planner
