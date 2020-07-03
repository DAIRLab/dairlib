#pragma once

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

class InnerLoopSetting {
 public:
  InnerLoopSetting() {};

  int n_node;
  double Q_double;
  double R_double;
  double eps_reg;
  double all_cost_scale;
  bool is_add_tau_in_cost;
  bool is_zero_touchdown_impact;

  int max_iter;
  double major_optimality_tol;
  double major_feasibility_tol;
  bool snopt_scaling;

  string directory;
  string prefix;
  string init_file;
};

// SubQpData stores all the data about the QPs in the SQP algorithm
class SubQpData {
 public:
  // Constructor
  SubQpData(int N_sample);

  // Vectors/Matrices for the outer loop
  vector<std::shared_ptr<VectorXd>> w_sol_vec;
  vector<std::shared_ptr<MatrixXd>> H_vec;
  vector<std::shared_ptr<VectorXd>> b_vec;
  vector<std::shared_ptr<VectorXd>> c_vec;
  vector<std::shared_ptr<MatrixXd>> A_vec;
  vector<std::shared_ptr<VectorXd>> lb_vec;
  vector<std::shared_ptr<VectorXd>> ub_vec;
  vector<std::shared_ptr<VectorXd>> y_vec;
  vector<std::shared_ptr<MatrixXd>> B_vec;
  vector<std::shared_ptr<int>> is_success_vec;

  // Vectors/Matrices for the outer loop (when cost descent is successful)
  vector<std::shared_ptr<MatrixXd>> A_active_vec;
  vector<std::shared_ptr<MatrixXd>> B_active_vec;
  vector<std::shared_ptr<int>> nw_vec;  // size of traj opt dec var
  vector<std::shared_ptr<int>> nl_vec;  // # of active constraints
  vector<std::shared_ptr<MatrixXd>> P_vec;  // w = P_i * theta + q_i
  vector<std::shared_ptr<VectorXd>> q_vec;  // w = P_i * theta + q_i
};

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

bool file_exist (const std::string & name);
bool folder_exist (const std::string & pathname_string);
// return false when the user want to stop
bool CreateFolderIfNotExist(const string& dir, bool ask_for_permission = true);

vector<std::string> ParseCsvToStringVec(const std::string& file_name);
void SaveStringVecToCsv(vector<std::string> strings,
                        const std::string& file_name);

}  // namespace goldilocks_models
} // dairlib

