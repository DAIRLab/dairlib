#pragma once

#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "common/file_utils.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_saved_traj.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/goldilocks_models/reduced_order_models.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::TriggerTypeSet;

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
  double eps_reg;
  double all_cost_scale;
  bool is_add_tau_in_cost;
  bool is_zero_touchdown_impact;

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
  std::vector<std::shared_ptr<double>> is_success_vec;

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

// Draw a diagram and save to a file
void CreateDiagramFigure(const drake::systems::Diagram<double>& diagram);

template <typename MessageType>
void NetworkPublisher(const std::string& channel_in,
                      const std::string& channel_out, int n_publishes) {
  // Parameters
  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");

  // Build the diagram
  drake::systems::DiagramBuilder<double> builder;
  auto name_pub = builder.AddSystem(LcmPublisherSystem::Make<MessageType>(
      channel_out, &lcm_network, TriggerTypeSet({TriggerType::kForced})));
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("network_publisher_for_" + channel_out));

  // Create simulator
  drake::systems::Diagram<double>* diagram_ptr = owned_diagram.get();
  drake::systems::Simulator<double> simulator(std::move(owned_diagram));
  auto& diagram_context = simulator.get_mutable_context();

  // Create subscriber for lcm driven loop
  drake::lcm::Subscriber<MessageType> input_sub(&lcm_local, channel_in);
  drake::lcm::Subscriber<dairlib::lcmt_saved_traj> mpc_sub(&lcm_local,
                                                           "MPC_OUTPUT");

  // Wait for the first message and initialize the context time..
  drake::log()->info("Waiting for first lcm input message");
  LcmHandleSubscriptionsUntil(&lcm_local,
                              [&]() { return input_sub.count() > 0; });
  drake::log()->info(diagram_ptr->get_name() + " started");

  while (true) {
    // Wait for input message.
    input_sub.clear();
    LcmHandleSubscriptionsUntil(&lcm_local,
                                [&]() { return input_sub.count() > 0; });
    // Pass output message
    MessageType msg;
    msg = input_sub.message();
    name_pub->get_input_port().FixValue(
        &(diagram_ptr->GetMutableSubsystemContext(*name_pub, &diagram_context)),
        msg);
    // Force-publish via the diagram
    diagram_ptr->Publish(diagram_context);

    // Once we have the first mpc message, enter this endless while loop
    if (mpc_sub.count() > 0) {
      while (true) {
        // Wait for input message.
        input_sub.clear();
        LcmHandleSubscriptionsUntil(&lcm_local,
                                    [&]() { return input_sub.count() > 0; });
        mpc_sub.clear();
        LcmHandleSubscriptionsUntil(&lcm_local,
                                    [&]() { return mpc_sub.count() > 0; });
        input_sub.clear();
        LcmHandleSubscriptionsUntil(&lcm_local,
                                    [&]() { return input_sub.count() > 0; });

        int pub_count = 0;
        while (pub_count < n_publishes) {
          // Get message time from the input channel
          // double t_current = input_sub.message().utime * 1e-6;
          // std::cout << "publish at t = " << t_current << std::endl;

          // Pass output message
          MessageType msg;
          msg = input_sub.message();
          name_pub->get_input_port().FixValue(
              &(diagram_ptr->GetMutableSubsystemContext(*name_pub,
                                                        &diagram_context)),
              msg);

          // Force-publish via the diagram
          diagram_ptr->Publish(diagram_context);

          pub_count++;
        }
      }
    }
  }
}

}  // namespace goldilocks_models
}  // namespace dairlib
