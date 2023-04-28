#pragma once

#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "common/file_utils.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "examples/goldilocks_models/rom_walking_gains.h"

#include "drake/common/trajectories/piecewise_polynomial.h"

namespace dairlib {
namespace goldilocks_models {

using BodyPoint =
    std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>;

namespace SAMPLE_STATUS_CODE {
const double SUCCESS = 1;
const double ITERATION_LIMIT = 0.5;
const double FAIL = 0;
}  // namespace SAMPLE_STATUS_CODE

class InnerLoopSetting {
 public:
  InnerLoopSetting(){};

  int n_node;
  double Q_double;
  double R_double;
  double w_joint_accel;
  double eps_reg;
  double all_cost_scale;
  bool is_add_tau_in_cost;
  bool is_zero_touchdown_impact;
  double mu;

  int max_iter;
  double major_optimality_tol;
  double major_feasibility_tol;
  bool snopt_log;
  bool snopt_scaling;
  bool use_ipopt;

  std::string directory;
  std::string prefix;
  std::string init_file;

  bool com_accel_constraint;

  // For testing
  bool cubic_spline_in_rom_constraint;
  bool swing_foot_cublic_spline_constraint;
  bool zero_ending_pelvis_angular_vel;
  bool com_at_center_of_support_polygon;
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
  // b_main is for the gradient of cost that we only consider optimizing (e.g.
  // excluding all the regularization cost)
  std::vector<std::shared_ptr<Eigen::VectorXd>> b_main_vec;
  std::vector<std::shared_ptr<Eigen::VectorXd>> c_vec;
  std::vector<std::shared_ptr<Eigen::MatrixXd>> A_vec;
  std::vector<std::shared_ptr<Eigen::VectorXd>> lb_vec;
  std::vector<std::shared_ptr<Eigen::VectorXd>> ub_vec;
  std::vector<std::shared_ptr<Eigen::VectorXd>> y_vec;
  std::vector<std::shared_ptr<Eigen::MatrixXd>> B_vec;
  std::vector<std::shared_ptr<double>> is_success_vec;

  // Vectors/Matrices for the outer loop (when cost descent is successful)
  std::vector<std::shared_ptr<Eigen::MatrixXd>> A_active_vec;
  std::vector<std::shared_ptr<Eigen::MatrixXd>> B_active_vec;
  std::vector<std::shared_ptr<int>> nw_vec;  // size of traj opt dec var
  std::vector<std::shared_ptr<int>> nl_vec;  // # of active constraints
  std::vector<std::shared_ptr<Eigen::MatrixXd>> P_vec;  // w = P_i * theta + q_i
  std::vector<std::shared_ptr<Eigen::VectorXd>> q_vec;  // w = P_i * theta + q_i

  // Testing
  std::vector<std::shared_ptr<Eigen::VectorXd>> cost_grad_by_envelope_thm_vec;
};

// Create MultibodyPlant
void CreateMBP(drake::multibody::MultibodyPlant<double>* plant,
               int robot_option, bool heavy_leg=false);

// Create MultibodyPlant for visualization
void CreateMBPForVisualization(drake::multibody::MultibodyPlant<double>* plant,
                               drake::geometry::SceneGraph<double>* scene_graph,
                               Eigen::Vector3d ground_normal, int robot_option);

// Create reduced order model
// For GIP:
// * Couldn’t optimize well even after I reduced step size and turned off
// momentum. (and after I turned back the constraint scaling)
// * I found that the linear solve is not very accurate
std::unique_ptr<ReducedOrderModel> CreateRom(
    int rom_option, int robot_option,
    const drake::multibody::MultibodyPlant<double>& plant,
    bool print_info = true);

// Read in model parameters from files
// `rom` is the ReducedOrderModel class which we want to write the params into.
// `dir` is the path where the model data is stored
// `model_iter` is the optimization iteration # of the model with we want to use
void ReadModelParameters(ReducedOrderModel* rom, const std::string& dir,
                         int model_iter);

// Functions for constructing goldilocks::StateMirror
std::map<int, int> MirrorPosIndexMap(
    const drake::multibody::MultibodyPlant<double>& plant, int robot_option);
std::set<int> MirrorPosSignChangeSet(
    const drake::multibody::MultibodyPlant<double>& plant, int robot_option);
std::map<int, int> MirrorVelIndexMap(
    const drake::multibody::MultibodyPlant<double>& plant, int robot_option);
std::set<int> MirrorVelSignChangeSet(
    const drake::multibody::MultibodyPlant<double>& plant, int robot_option);

// Create cubic splines from s and sdot
drake::trajectories::PiecewisePolynomial<double> CreateCubicSplineGivenYAndYdot(
    const std::vector<Eigen::VectorXd>& h_vec,
    const std::vector<Eigen::VectorXd>& s_vec,
    const std::vector<Eigen::VectorXd>& ds_vec);

// Store splines in csv file for plotting
// The first row is time, and the rest rows are s
void StoreSplineOfY(
    const std::vector<Eigen::VectorXd>& h_vec,
    const drake::trajectories::PiecewisePolynomial<double>& y_spline,
    const std::string& directory, const std::string& prefix);

// Check whether your cubic spline implemented in dynamics constriant is correct
void CheckSplineOfY(
    const std::vector<Eigen::VectorXd>& h_vec,
    const std::vector<Eigen::VectorXd>& yddot_vec,
    const drake::trajectories::PiecewisePolynomial<double>& y_spline);

void storeTau(const std::vector<Eigen::VectorXd>& h_vec,
              const std::vector<Eigen::VectorXd>& tau_vec,
              const std::string& directory, const std::string& prefix);

Eigen::VectorXd createPrimeNumbers(int num_prime);

bool file_exist(const std::string& name);
bool folder_exist(const std::string& pathname_string, bool print = true);
// return false when the user want to stop
bool CreateFolderIfNotExist(const std::string& dir,
                            bool ask_for_permission = true);

std::vector<std::string> ParseCsvToStringVec(const std::string& file_name,
                                             bool is_column_vector = true);
void SaveStringVecToCsv(const std::vector<std::string>& strings,
                        const std::string& file_name);

// Five link robot's left/right leg
BodyPoint FiveLinkRobotLeftContact(
    const drake::multibody::MultibodyPlant<double>& plant);
BodyPoint FiveLinkRobotRightContact(
    const drake::multibody::MultibodyPlant<double>& plant);

// Parameters for planner
struct PlannerSetting {
  int rom_option;
  int iter;
  int sample;  // solution to use for initial guess and cost regularization

  int n_step;
  int knots_per_mode;

  bool zero_touchdown_impact;
  bool use_double_contact_points;

  bool equalize_timestep_size;
  bool fix_duration;

  double feas_tol;
  double opt_tol;
  int max_iter;

  bool use_ipopt;
  bool switch_to_snopt_after_first_loop;
  bool log_solver_info;
  double time_limit;
  double realtime_rate_for_time_limit;

  // gains includes cost weights
  RomWalkingGains gains;

  // Files parameters
  std::string dir_model;  // location of the model files
  std::string dir_data;   // location to store the opt result
  std::string init_file;
  std::string dir_and_prefix_FOM;

  // Testing
  int n_step_lipm;

  // RL
  bool is_RL_training;
  bool get_RL_gradient_offline;
  double min_mpc_thread_loop_duration;
  std::string path_model_params;
  std::string path_var;

  // Debugging
  int solve_idx_for_read_from_file;
  void PrintAll() const;

  bool unit_testing;
};

template<typename K, typename V>
std::map<V, K> reverse_map(const std::map<K, V>& m) {
  std::map<V, K> r;
  for (const auto& kv : m)
    r[kv.second] = kv.first;
  return r;
}

}  // namespace goldilocks_models
}  // namespace dairlib
