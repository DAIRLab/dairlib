#include "legendre.h"
#include <iostream>

namespace dairlib::polynomials {

int DoMain(int argc, char** argv) {

  Eigen::MatrixXd B = MakeChangeOfBasisOperatorFromLegendreToMonomials(5);
  Eigen::MatrixXi D = LegendreBasisDerivativeOperator(5);

  std::cout << "B:\n" << B << std::endl;
  std::cout << "\nD:\n" << D << std::endl;

  Eigen::MatrixXd DB = D.cast<double>() * B;
  Eigen::MatrixXd D_monomial = B.completeOrthogonalDecomposition().solve(DB);
  std::cout << "\nD (monomial basis):\n" << D_monomial << std::endl;

  return 0;
}

}

int main(int argc, char** argv) {
  return dairlib::polynomials::DoMain(argc, argv);
}