#include "filter_utils.h"

namespace dairlib::systems::filter_utils {

using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::VectorXd;

DiscreteSISOButterworthFilter butter(int order, double w_c) {
  DRAKE_DEMAND(order % 2 == 0);
  DRAKE_DEMAND(order > 0);
  DRAKE_DEMAND(0 < w_c && w_c < 1);
  std::vector<Matrix2d> A;
  std::vector<Matrix2d> B;

  // sequentially generate second order
  // filter sections using evenly spaced complex conjugate pairs
  double dtheta = M_PI / order;
  double start = M_PI_2 + dtheta / 2;
  for (int k = 0; k < order / 2; k++) {
    std::complex<double> p_i(cos(start + dtheta * k),
                             sin(start + dtheta * k));
    p_i *= w_c;
    auto p_i_z = exp(p_i);
    double a1 = -2 * p_i_z.real();
    double a2 = norm(p_i_z);
    Matrix2d a;
    Matrix2d b = Matrix2d::Zero();
    b(1, 1) = 1 + a1 + a2;
    a << 0, 1, -a2, -a1;
    A.push_back(a);
    B.push_back(b);
  }

  // Assemble the cascade of second order filters as a single state space system
  MatrixXd BigA = MatrixXd::Zero(order, order);
  VectorXd BigB = VectorXd::Zero(order);
  BigA.topLeftCorner<2, 2>() = A.front();
  BigB.head<2>() = B.front().rightCols<1>();
  for (int i = 1; i < order / 2; i++) {
    BigB.segment<2>(2 * i) = B.at(i) * BigB.segment<2>(2 * (i - 1));
    BigA.middleRows<2>(2 * i) = BigA.middleRows<2>(2 * (i - 1));
    BigA.block<2, 2>(2 * i, 2 * i) = A.at(i);
    for (int j = 0; j < i; j++) {
      BigA.block<2, 2>(2 * i, 2 * j) = B.at(i) * BigA.block<2, 2>(2 * i, 2 * j);
    }
  }
  DiscreteSISOButterworthFilter::ValidateA(BigA);
  return {BigA, BigB};
}
}