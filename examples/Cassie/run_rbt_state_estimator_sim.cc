#include <memory>
#include <string>

#include <gflags/gflags.h>
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "examples/Cassie/cassie_rbt_state_estimator.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/networking/cassie_output_receiver.h"

namespace dairlib {

using std::make_unique;
using std::move;
using std::pow;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Quaternion;

using drake::math::RotationMatrix;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::geometry::SceneGraph;
using drake::systems::Simulator;

using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::CassieOutputReceiver;
using dairlib::systems::CassieRbtStateEstimator;

int doMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  RigidBodyTree<double> tree;

  buildCassieTree(tree, "examples/Cassie/urdf/cassie_v2.urdf",
                  drake::multibody::joints::kQuaternion);
  const double terrain_size = 100;
  const double terrain_depth = 0.2;
  drake::multibody::AddFlatTerrainToWorld(&tree, terrain_size, terrain_depth);

  DiagramBuilder<double> builder;
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  const std::string channel_x = "CASSIE_STATE";
  const std::string channel_output = "CASSIE_OUTPUT";
  const std::string channel_estimator = "CASSIE_ESTIMATOR";
  const int num_contacts = 4;

  // State Receiver
  auto state_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(channel_x, lcm));
  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(tree);
  builder.Connect(state_sub->get_output_port(),
                  state_receiver->get_input_port(0));

  // Cassie out receiver
  auto cassie_output_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(channel_output, lcm));
  auto cassie_output_receiver = builder.AddSystem<CassieOutputReceiver>();
  builder.Connect(cassie_output_sub->get_output_port(),
                  cassie_output_receiver->get_input_port(0));

  // Estimator
  // Initial state and bias values
  // MatrixXd R_init = MatrixXd::Identity(3, 3);
  // VectorXd v_init = VectorXd::Zero(3);
  // VectorXd p_init = VectorXd::Zero(3);
  // MatrixXd d_init = MatrixXd::Zero(3, 4);

  // VectorXd ekf_x_init = VectorXd::Zero(27);
  // for (int i = 0; i < 3; ++i) {
  //  ekf_x_init.segment(i * 3, 3) = R_init.row(i);
  //}
  // ekf_x_init.segment(9, 3) = v_init;
  // ekf_x_init.segment(12, 3) = p_init;
  // for (int i = 0; i < num_contacts; ++i) {
  //  ekf_x_init.segment(15 + i * 3, 3) = d_init.col(i);
  //}

  VectorXd q_init_true(tree.get_num_positions());
  q_init_true << 0.009396, -2.544e-4, 1.02419, 0.990074, 4.19486627e-4,
      9.921595425e-4, 8.38942768e-4, -5.888e-4, -4.515e-4, -5.783e-4,
      -0.0028665, 0.49619, 0.4960, -1.1493, -1.15012, -1.35571, -1.012425,
      1.35291, 1.35386, 0.0, 0.0, -1.57, -1.57;
  VectorXd x_init_true =
      VectorXd::Zero(tree.get_num_positions() + tree.get_num_velocities());
  x_init_true.head(tree.get_num_positions()) = q_init_true;
  Quaternion<double> quat(q_init_true(3), q_init_true(4), q_init_true(5), q_init_true(6));
  RotationMatrix<double> rotmat(quat);
  MatrixXd R = rotmat.matrix();

  // Initial X state
  MatrixXd X_init = MatrixXd::Identity(9, 9);
  X_init.block(0, 0, 3, 3) = R;
  X_init.block(0, 3, 3, 1) = VectorXd::Zero(3);
  X_init.block(0, 4, 3, 1) = q_init_true.head(3);

  VectorXd ekf_bias_init = VectorXd::Zero(6);
  VectorXd gyro_noise_std = 0.002 * VectorXd::Ones(3);
  VectorXd accel_noise_std = 0.04 * VectorXd::Ones(3);
  VectorXd contact_noise_std = 0.05 * VectorXd::Ones(3);
  VectorXd gyro_bias_noise_std = 0.001 * VectorXd::Ones(3);
  VectorXd accel_bias_noise_std = 0.001 * VectorXd::Ones(3);
  VectorXd joints_noise_std = ((M_PI * 0.5) / 180.0) * VectorXd::Ones(16);
  VectorXd prior_base_pose_std = 0.01 * VectorXd::Ones(6);
  VectorXd prior_base_velocity_std = 0.1 * VectorXd::Ones(3);
  VectorXd prior_contact_position_std = 1.0 * VectorXd::Ones(3);
  VectorXd prior_gyro_bias_std = 0.01 * VectorXd::Ones(3);
  VectorXd prior_accel_bias_std = 0.1 * VectorXd::Ones(3);
  VectorXd prior_forward_kinematics_std = 0.03 * VectorXd::Ones(3);

  MatrixXd N_prior = MatrixXd::Identity(3, 3);
  for (int i = 0; i < 3; ++i) {
    N_prior(i, i) = pow(prior_forward_kinematics_std(i), 2);
  }
  MatrixXd Q_prior_base_position = MatrixXd::Zero(3, 3);
  MatrixXd Q_prior_base_velocity = MatrixXd::Zero(3, 3);
  MatrixXd Q_prior_base_orientation = MatrixXd::Zero(3, 3);
  MatrixXd Q_prior_contact = MatrixXd::Zero(3, 3);
  MatrixXd Q_prior_gyro_bias = MatrixXd::Zero(3, 3);
  MatrixXd Q_prior_accel_bias = MatrixXd::Zero(3, 3);
  for (int i = 0; i < 3; ++i) {
    Q_prior_base_position(i, i) = pow(prior_base_pose_std(i), 2);
    Q_prior_base_velocity(i, i) = pow(prior_base_velocity_std(i), 2);
    Q_prior_base_orientation(i, i) = pow(prior_base_pose_std(i + 3), 2);
    Q_prior_contact(i, i) = pow(prior_contact_position_std(i), 2);
    Q_prior_gyro_bias(i, i) = pow(prior_gyro_bias_std(i), 2);
    Q_prior_accel_bias(i, i) = pow(prior_accel_bias_std(i), 2);
  }
  MatrixXd P_init = MatrixXd::Zero(27, 27);
  P_init.block(0, 0, 3, 3) = Q_prior_base_position;
  P_init.block(3, 3, 3, 3) = Q_prior_base_velocity;
  P_init.block(6, 6, 3, 3) = Q_prior_base_orientation;
  P_init.block(9, 9, 3, 3) = Q_prior_contact;
  P_init.block(12, 12, 3, 3) = Q_prior_contact;
  P_init.block(15, 15, 3, 3) = Q_prior_contact;
  P_init.block(18, 18, 3, 3) = Q_prior_contact;
  P_init.block(21, 21, 3, 3) = Q_prior_gyro_bias;
  P_init.block(24, 24, 3, 3) = Q_prior_accel_bias;

  auto estimator = builder.AddSystem<CassieRbtStateEstimator>(
      tree, X_init, ekf_bias_init, true, P_init, N_prior, gyro_noise_std,
      accel_noise_std, contact_noise_std, gyro_bias_noise_std,
      accel_bias_noise_std, joints_noise_std);

  // Connecting the estimator
  builder.Connect(cassie_output_receiver->get_output_port(0),
                  estimator->get_input_port(0));

  // Publishing the estimator state
  auto passthrough = builder.AddSystem<dairlib::systems::SubvectorPassThrough>(
      estimator->get_output_port(0).size(), 0,
      tree.get_num_positions() + tree.get_num_velocities());
  auto estimator_sender =
      builder.AddSystem<dairlib::systems::RobotOutputSender>(tree);
  auto estimator_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          channel_estimator, lcm, 1.0 / 30.0));
  builder.Connect(estimator->get_output_port(0), passthrough->get_input_port());
  builder.Connect(passthrough->get_output_port(),
                  estimator_sender->get_input_port(0));
  builder.Connect(estimator_sender->get_output_port(0),
                  estimator_publisher->get_input_port());

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  auto stepper = make_unique<Simulator<double>>(*diagram, move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(1.0);
  stepper->Initialize();

  stepper->StepTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::doMain(argc, argv); }

