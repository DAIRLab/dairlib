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

using BodyPoint =
    std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>;

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

// Create MultibodyPlant
void CreateMBP(drake::multibody::MultibodyPlant<double>* plant,
               int robot_option);

// Create MultibodyPlant for visualization
void CreateMBPForVisualization(drake::multibody::MultibodyPlant<double>* plant,
                               drake::geometry::SceneGraph<double>* scene_graph,
                               Eigen::Vector3d ground_normal, int robot_option);

// Create reduced order model
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
class StateMirror {
 public:
  StateMirror(std::map<int, int> mirror_pos_index_map,
              std::set<int> mirror_pos_sign_change_set,
              std::map<int, int> mirror_vel_index_map,
              std::set<int> mirror_vel_sign_change_set);

  drake::VectorX<double> MirrorPos(const drake::VectorX<double>& q) const;
  drake::VectorX<double> MirrorVel(const drake::VectorX<double>& v) const;

  StateMirror() : StateMirror({}, {}, {}, {}){};

 private:
  std::map<int, int> mirror_pos_index_map_;
  std::set<int> mirror_pos_sign_change_set_;
  std::map<int, int> mirror_vel_index_map_;
  std::set<int> mirror_vel_sign_change_set_;
};
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
bool folder_exist(const std::string& pathname_string);
// return false when the user want to stop
bool CreateFolderIfNotExist(const std::string& dir,
                            bool ask_for_permission = true);

std::vector<std::string> ParseCsvToStringVec(const std::string& file_name,
                                             bool is_row_vector = true);
void SaveStringVecToCsv(const std::vector<std::string>& strings,
                        const std::string& file_name);

// Five link robot's left/right leg
BodyPoint FiveLinkRobotLeftContact(
    const drake::multibody::MultibodyPlant<double>& plant);
BodyPoint FiveLinkRobotRightContact(
    const drake::multibody::MultibodyPlant<double>& plant);

}  // namespace goldilocks_models
}  // namespace dairlib
