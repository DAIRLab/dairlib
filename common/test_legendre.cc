#include "legendre.h"
#include <iostream>
#include "boost/math/special_functions.hpp"

namespace dairlib::polynomials {

int DoMain(int argc, char** argv) {

  Eigen::MatrixXd B = MakeChangeOfBasisOperatorFromLegendreToMonomials(5);
  Eigen::MatrixXi D = LegendreBasisDerivativeOperator(5);

  std::cout << "B:\n" << B << std::endl;
  std::cout << "\nD:\n" << D << std::endl;

  Eigen::MatrixXd D_monomial = B * D.cast<double>() * B.inverse();
  std::cout << "\nD (monomial basis):\n" << D_monomial << std::endl;

  Eigen::VectorXd boost_legendre_derivs = Eigen::VectorXd::Zero(5);
  for (int i = 0; i < 5; ++i) {
    boost_legendre_derivs(i) = boost::math::legendre_p_prime(i, 0.5);
  }
  std::cout << "Derivative Error: " <<
      (EvalLegendreBasisDerivative(5, 1, 0.5) - boost_legendre_derivs).transpose() << std::endl;

  return 0;
}

}

int main(int argc, char** argv) {
  return dairlib::polynomials::DoMain(argc, argv);
}