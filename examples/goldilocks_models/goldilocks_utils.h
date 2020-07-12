#pragma once

#include "common/file_utils.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

#include <iostream>
#include <string>
#include <Eigen/Dense>

namespace dairlib {
namespace goldilocks_models {

class InnerLoopSetting {
 public:
  InnerLoopSetting(){};

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

  std::string directory;
  std::string prefix;
  std::string init_file;
};

// SubQpData stores all the data about the QPs in the SQP algorithm
class SubQpData {
 public:
  // Constructor
  SubQpData(int N_sample);

  // Vectors/Matrices for the outer loop
  std::vector<std::shared_ptr<Eigen::VectorXd>> w_sol_vec;
  std::vector<std::shared_ptr<Eigen::MatrixXd>> H_vec;
  std::vector<std::shared_ptr<Eigen::VectorXd>> b_vec;
  std::vector<std::shared_ptr<Eigen::VectorXd>> c_vec;
  std::vector<std::shared_ptr<Eigen::MatrixXd>> A_vec;
  std::vector<std::shared_ptr<Eigen::VectorXd>> lb_vec;
  std::vector<std::shared_ptr<Eigen::VectorXd>> ub_vec;
  std::vector<std::shared_ptr<Eigen::VectorXd>> y_vec;
  std::vector<std::shared_ptr<Eigen::MatrixXd>> B_vec;
  std::vector<std::shared_ptr<int>> is_success_vec;

  // Vectors/Matrices for the outer loop (when cost descent is successful)
  std::vector<std::shared_ptr<Eigen::MatrixXd>> A_active_vec;
  std::vector<std::shared_ptr<Eigen::MatrixXd>> B_active_vec;
  std::vector<std::shared_ptr<int>> nw_vec;  // size of traj opt dec var
  std::vector<std::shared_ptr<int>> nl_vec;  // # of active constraints
  std::vector<std::shared_ptr<Eigen::MatrixXd>> P_vec;  // w = P_i * theta + q_i
  std::vector<std::shared_ptr<Eigen::VectorXd>> q_vec;  // w = P_i * theta + q_i
};

// Create reduced order model
std::unique_ptr<ReducedOrderModel> CreateRom(
    int rom_option, int robot_option,
    const drake::multibody::MultibodyPlant<double>& plant,
    bool print_info = true);

// Create cubic splines from s and sdot
drake::trajectories::PiecewisePolynomial<double> createCubicSplineGivenSAndSdot(
    const std::vector<Eigen::VectorXd>& h_vec,
    const std::vector<Eigen::VectorXd>& s_vec,
    const std::vector<Eigen::VectorXd>& ds_vec);

// Store splines in csv file for plotting
// The first row is time, and the rest rows are s
void storeSplineOfS(
    const std::vector<Eigen::VectorXd>& h_vec,
    const drake::trajectories::PiecewisePolynomial<double>& s_spline,
    const std::string& directory, const std::string& prefix);

// Check whether your cubic spline implemented in dynamics constriant is correct
void checkSplineOfS(
    const std::vector<Eigen::VectorXd>& h_vec,
    const std::vector<Eigen::VectorXd>& dds_vec,
    const drake::trajectories::PiecewisePolynomial<double>& s_spline);

void storeTau(const std::vector<Eigen::VectorXd>& h_vec,
              const std::vector<Eigen::VectorXd>& tau_vec,
              const std::string& directory, const std::string& prefix);

Eigen::VectorXd createPrimeNumbers(int num_prime);

bool file_exist(const std::string& name);
bool folder_exist(const std::string& pathname_string);
// return false when the user want to stop
bool CreateFolderIfNotExist(const std::string& dir,
                            bool ask_for_permission = true);

std::vector<std::string> ParseCsvToStringVec(const std::string& file_name,
                                             bool is_row_vector = true);
void SaveStringVecToCsv(const std::vector<std::string>& strings,
                        const std::string& file_name);

}  // namespace goldilocks_models
}  // namespace dairlib
