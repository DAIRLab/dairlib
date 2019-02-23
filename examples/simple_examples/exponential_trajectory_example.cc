// This is an example code for ExponentialPlusPiecewisePolynomial

#include <iostream>

#include "systems/framework/output_vector.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"

#include <math.h>

using Eigen::MatrixXd;

using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;

int main() {
  // y(t) = K * exp(A * (t - t_j)) * alpha.col(j) + piecewise_polynomial_part(t)

  // Here is a piece of code where the evaluation happens:
  //       template <typename T>
  //       MatrixX<T> ExponentialPlusPiecewisePolynomial<T>::value(double t) const {
  //         int segment_index = this->get_segment_index(t);
  //         MatrixX<T> ret = piecewise_polynomial_part_.value(t);
  //         double tj = this->start_time(segment_index);
  //         auto exponential = (A_ * (t - tj)).eval().exp().eval();
  //         ret.noalias() += K_ * exponential * alpha_.col(segment_index);
  //         return ret;
  //       }


  // exponential + zero polynomial /////////////////////////////////////////////
  std::cout << "exponential + one-segment zero polynomial\n";
  double startTimeOfExp = 0;
  std::vector<double> T1_breakpoint = {startTimeOfExp, 1};
  std::vector<MatrixXd> Y1(T1_breakpoint.size(), MatrixXd::Zero(1, 1));
  Y1[0](0, 0) = 0;
  Y1[1](0, 0) = 0;
  PiecewisePolynomial<double> pp_part =
    PiecewisePolynomial<double>::Pchip(T1_breakpoint, Y1, true);

  MatrixXd K = MatrixXd::Zero(1, 1);
  MatrixXd A = MatrixXd::Zero(1, 1);
  MatrixXd alpha = MatrixXd::Zero(1, 1);
  K(0, 0) = 1;
  A(0, 0) = 1;
  alpha(0, 0) = 1;

  auto exp_traj =
    ExponentialPlusPiecewisePolynomial<double>(K, A, alpha, pp_part);

  for (double d = 0; d <= 2; d += 0.1) {
    std::cout << "exp(" << d << ")= " << exp_traj.value(d) << "\n";
  }

  // exponential + piecewise polynomial ////////////////////////////////////////
  std::cout << "\nexponential + two-segment polynomial \n";
  std::vector<double> T2_breakpoint = {0, 1, 2};
  std::vector<MatrixXd> Y2(T2_breakpoint.size(), MatrixXd::Zero(1, 1));
  Y2[0](0, 0) = 1;
  Y2[1](0, 0) = 2;
  Y2[2](0, 0) = 3;
  std::vector<MatrixXd> Y2_dot(T2_breakpoint.size(), MatrixXd::Zero(1, 1));
  Y2_dot[0](0, 0) = 0;
  Y2_dot[1](0, 0) = 2;
  Y2_dot[2](0, 0) = 0;
  PiecewisePolynomial<double> pp_part2 =
    PiecewisePolynomial<double>::Cubic(T2_breakpoint, Y2, Y2_dot);

  auto exp_traj2 =
    ExponentialPlusPiecewisePolynomial<double>(K, A, alpha, pp_part2);

  for (double d = 0; d <= 2; d += 0.1) {
    std::cout << exp_traj2.value(d) << " ";
  }
  std::cout << std::endl;
  std::cout << "In this example, the exponential is only added on to the "
               "first segment of ploynomial, since alpha is 1x1 matrix \n";

  // sum of two exponential ////////////////////////////////////////////////////
  std::cout << "\nsum of two exponential + one-segment zero polynomial\n";
  MatrixXd K2 = MatrixXd::Zero(1, 2);
  MatrixXd A2 = MatrixXd::Zero(2, 2);
  MatrixXd alpha2 = MatrixXd::Zero(2, 1);
  K2<< 3, 2;
  A2<< 6, 0,
       0, -0.5;
  alpha2<< 1, 1;

  auto exp_traj3 =
    ExponentialPlusPiecewisePolynomial<double>(K2, A2, alpha2, pp_part);

  std::cout << "(from traj, from math.h)\n";
  for (double d = 0; d <= 1; d += 0.1) {
    double fromMath = K2(0) * exp(A2(0, 0) * d) + K2(1) * exp(A2(1, 1) * d);
    std::cout << "t=" << d << ": (" << exp_traj3.value(d) << ", " <<
      fromMath <<")\n";
  }

  // sum of two exponential (another implementation) ///////////////////////////
  std::cout << "\nsum of two exponential + one-segment zero polynomial\n";
  MatrixXd K2_2nd = MatrixXd::Zero(1, 2);
  MatrixXd A2_2nd = MatrixXd::Zero(2, 1);
  MatrixXd alpha2_2nd = MatrixXd::Zero(1, 1);
  K2_2nd<< 3, 2;
  A2_2nd<< 6, -0.5;
  alpha2_2nd<< 1;

  auto exp_traj3_2nd = ExponentialPlusPiecewisePolynomial<double>(
                         K2_2nd, A2_2nd, alpha2_2nd, pp_part);

  std::cout << "(from traj, from math.h)\n";
  for (double d = 0; d <= 1; d += 0.1) {
    double fromMath =
    K2_2nd(0) * exp(A2_2nd(0) * d) + K2_2nd(1) * exp(A2_2nd(1) * d);
    std::cout << "t=" << d << ": (" << exp_traj3_2nd.value(d)
    << ", " << fromMath << ")\n";
  }
  std::cout << "This doesn't work. I guess exp() doesn't operate element-wise, "
               "so A has to be square.\n";

  // 2D - sum of two exponential ///////////////////////////////////////////////
  // create a 2D pp_part
  std::vector<MatrixXd> Y3(T1_breakpoint.size(), MatrixXd::Zero(2, 1));
  Y3[0](0, 0) = 0;
  Y3[1](0, 0) = 0;
  Y3[0](1, 0) = 0;
  Y3[1](1, 0) = 0;
  PiecewisePolynomial<double> pp_part3 =
    PiecewisePolynomial<double>::Pchip(T1_breakpoint, Y3, true);

  std::cout << "\n(2 dimension) sum of two exponential + "
               "one-segment zero polynomial\n";
  MatrixXd K3 = MatrixXd::Zero(2, 4);
  MatrixXd A3 = MatrixXd::Zero(4, 4);
  MatrixXd alpha3 = MatrixXd::Zero(4, 1);
  K3<< 3, 2, 0, 0,
       0, 0, 1.2, -5;
  A3<< 6, 0,    0,   0,
       0, -0.5, 0,   0,
       0, 0,    10,  0,
       0, 0,    0,  -10;
  alpha3<< 1, 1, 1, 1;

  auto exp_traj4 =
  ExponentialPlusPiecewisePolynomial<double>(K3, A3, alpha3, pp_part3);

  std::cout << "First dimension:\n  (from traj, from math.h)\n";
  for (double d = 0; d <= 1; d += 0.1) {
    double fromMath =
                K3(0, 0) * exp(A3(0, 0) * d) + K3(0, 1) * exp(A3(1, 1) * d);
    std::cout << " t=" << d << ": (" << (exp_traj4.value(d))(0)
      << ", " << fromMath << ")\n";
  }
  std::cout << "Second dimension:\n  (from traj, from math.h)\n";
  for (double d = 0; d <= 1; d += 0.1) {
    double fromMath =
                K3(1, 2) * exp(A3(2, 2) * d) + K3(1, 3) * exp(A3(3, 3) * d);
    std::cout << " t=" << d << ": (" << (exp_traj4.value(d))(1)
      << ", " << fromMath << ")\n";
  }
  std::cout << std::endl;

  return 0;
}