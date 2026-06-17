#include "math.h"

#include <iostream>

#include "drake/math/autodiff_gradient.h"

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::math::DiscardGradient;
using drake::math::ExtractGradient;
using drake::math::ExtractValue;
using drake::math::InitializeAutoDiff;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

int main() {
  int x_dim = 2;
  int y_dim = 3;
  int z_dim = 1;

  //////////////////////////////////////////////////////////
  // Declare autoDiff x
  VectorXd x_val(x_dim);
  x_val << M_PI / 2, 3;
  AutoDiffVecXd x = InitializeAutoDiff(x_val);

  // Compute jacobian of x
  // Default wrt itself since it's not derived from other autoDiff
  MatrixXd dxdSTH = ExtractGradient(x);
  cout << "dxdSTH = \n" << dxdSTH << "\n\n";

  //////////////////////////////////////////////////////////
  // Declare autoDiff y = x1 + x2^3
  AutoDiffVecXd y(y_dim);
  y << x(0) + x(1) * x(1) * x(1), x(0) * x(1), cos(x(0)) + sqrt(x(1));

  // Get the value of y
  VectorXd y_val = ExtractValue(y);
  cout << "y_val = \n" << y_val << "\n\n";

  // Compute jacobian of y wrt x
  MatrixXd dydx = ExtractGradient(y);
  cout << "dydx = \n" << dydx << "\n\n";

  //////////////////////////////////////////////////////////
  // Declare autoDiff z = y1 + y2
  AutoDiffVecXd z(z_dim);
  z << y(0) + y(1);

  // Get the value of z
  VectorXd z_val = ExtractValue(z);
  cout << "z_val = \n" << z_val << "\n\n";

  // Compute jacobian of z wrt x
  // Note that it's wrt x instead of wrt y
  MatrixXd dzdSTH = ExtractGradient(z);
  cout << "dzdSTH = \n" << dzdSTH << "\n\n";

  return 0;
}
