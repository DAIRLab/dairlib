#include "drake/common/trajectories/piecewise_polynomial.h"
#include "systems/goldilocks_models/file_utils.h"

#include <Eigen/Dense>
#include <iostream>
#include <string>

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using drake::trajectories::PiecewisePolynomial;
using std::vector;
using std::shared_ptr;
using std::cout;
using std::endl;
using std::string;
using std::to_string;

namespace dairlib {
namespace goldilocks_models  {

// Create cubic splines from s and sdot
PiecewisePolynomial<double> createCubicSplineGivenSAndSdot(
  const vector<VectorXd> & h_vec,
  const vector<VectorXd> & s_vec, const vector<VectorXd> & ds_vec);

// Store splines in csv file for plotting
// The first row is time, and the rest rows are s
void storeSplineOfS(const vector<VectorXd> & h_vec,
                    const PiecewisePolynomial<double> & s_spline,
                    const string & directory, const string & prefix);

// Check whether your cubic spline implemented in dynamics constriant is correct
void checkSplineOfS(const vector<VectorXd> & h_vec,
                    const vector<VectorXd> & dds_vec,
                    const PiecewisePolynomial<double> & s_spline);

void storeTau(const vector<VectorXd> & h_vec,
              const vector<VectorXd> & tau_vec,
              const string & directory,
              const string & prefix);

VectorXd createPrimeNumbers(int num_prime);

}  // namespace goldilocks_models
} // dairlib

