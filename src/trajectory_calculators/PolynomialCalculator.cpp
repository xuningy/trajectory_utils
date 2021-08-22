#include <trajectory_calculators/PolynomialCalculator.h>

#include <cmath>
#include <Eigen/LU>

namespace planner {

PolynomialCalculator::PolynomialCalculator() {
  Eigen::Matrix<double, poly_len, poly_len> poly_mat;
  poly_mat <<  1, 0, 0, 0, 0, 0, 0, 0,
               0, 1, 0, 0, 0, 0, 0, 0,
               0, 0, 2, 0, 0, 0, 0, 0,
               0, 0, 0, 6, 0, 0, 0, 0,
               1, 1, 1, 1, 1, 1, 1, 1,
               0, 1, 2, 3, 4, 5, 6, 7,
               0, 0, 2, 6, 12, 20, 30, 42,
               0, 0, 0, 6, 24, 60, 120, 210;
  poly_mat_inv = poly_mat.inverse();
}

Eigen::Matrix<double, poly_len, 1> PolynomialCalculator::get_poly(const end_point_t &start, const end_point_t &end, const double duration) {
  Eigen::Matrix<double, poly_len, 1> poly_b;
  // We need to make sure to rescale the desired higher derivatives to account for retiming.
  poly_b << start[0], duration * start[1], std::pow(duration, 2) * start[2], std::pow(duration, 3) * start[3],
              end[0], duration *   end[1], std::pow(duration, 2) *   end[2], std::pow(duration, 3) *   end[3];

  Eigen::Matrix<double, poly_len, 1> poly_coeffs(poly_mat_inv * poly_b);

  // Resulting polynomial is well defined from 0 to 1.
  // Retime so it is well defined from 0 to duration.
  double divider = 1.0;
  for (int i = 0; i < poly_len; i++) {
    poly_coeffs[i] /= divider;
    divider *= duration;
  }

  return poly_coeffs;
}

std::vector<Vec8_t, Eigen::aligned_allocator<Vec8_t>> PolynomialCalculator::get_poly4(const end_point_t start[], const end_point_t end[], const double duration)
{

  std::vector<Vec8_t, Eigen::aligned_allocator<Vec8_t>> poly4_coeffs;
  poly4_coeffs.emplace_back(get_poly(start[0], end[0], duration));
  poly4_coeffs.emplace_back(get_poly(start[1], end[1], duration));
  poly4_coeffs.emplace_back(get_poly(start[2], end[2], duration));
  poly4_coeffs.emplace_back(get_poly(start[3], end[3], duration));
  return poly4_coeffs;
}

 void PolynomialCalculator::get_poly4(const end_point_t start[], const end_point_t end[], const double duration, std::vector<Vec8_t, Eigen::aligned_allocator<Vec8_t>> &ret)
{

  ret.emplace_back(get_poly(start[0], end[0], duration));
  ret.emplace_back(get_poly(start[1], end[1], duration));
  ret.emplace_back(get_poly(start[2], end[2], duration));
  ret.emplace_back(get_poly(start[3], end[3], duration));
}

}
