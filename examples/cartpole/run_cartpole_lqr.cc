#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_output.hpp"

#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/constant_vector_source.h"


#include "systems/robot_lcm_systems.h"
#include "common/find_resource.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/controllers/linear_controller.h"
#include "examples/cartpole/lqr.h"

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::controllers::LinearQuadraticRegulator;
using drake::systems::controllers::LinearQuadraticRegulatorResult;
using drake::systems::DiagramBuilder;
using drake::systems::Diagram;
using drake::systems::Simulator;
using drake::systems::Context;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::ConstantVectorSource;


using dairlib::systems::LcmDrivenLoop;
using dairlib::systems::LQR;
using dairlib::FindResourceOrThrow;
using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::RobotCommandSender;
using dairlib::systems::SubvectorPassThrough;

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace dairlib::examples::cartpole {

DEFINE_string(channel_x, "CARTPOLE_STATE", "Channel to recieve state");
DEFINE_string(channel_u, "CARTPOLE_INPUT", "channel to publish input");

int controller_main(int argc, char* argv[]) {
  DiagramBuilder<double> builder;

  MultibodyPlant<double> plant(0.0);
  Parser(&plant).AddModelFromFile(
      FindResourceOrThrow(
          "examples/cartpole/urdf/cartpole.urdf"));
  plant.Finalize();
  auto plant_context  = plant.CreateDefaultContext();

  Eigen::MatrixXd Q = 10 * MatrixXd::Identity(4,4);
  Eigen::MatrixXd R = MatrixXd::Identity(1,1);
  Eigen::MatrixXd A = MatrixXd::Zero(4, 4);
  Eigen::MatrixXd B = MatrixXd::Zero(4, 1);

  A.block(0, 2, 2, 2) = MatrixXd::Identity(2, 2);
  A(2, 1) = 3.51;
  A(3, 1) = 22.2;
  B(2, 0) = 1.02;
  B(3, 0) = 1.7;

  LinearQuadraticRegulatorResult lqr_gains =
      LinearQuadraticRegulator(A, B, Q, R);
  MatrixXd K = lqr_gains.K;
  std::cout << K << std::endl;
  auto lqr = builder.AddSystem<LQR>(
      plant.num_positions(),
      plant.num_velocities(),
      plant.num_actuators(),
      VectorXd::Zero(4),
      K);

  // LCM
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");
  auto input_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm));

  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(plant);
  auto input_sender = builder.AddSystem<RobotCommandSender>(plant);


  builder.Connect(state_receiver->get_output_port(),
      lqr->get_input_port_output());
  builder.Connect(*lqr, *input_sender);
  builder.Connect(*input_sender, *input_publisher);

  // Build diagram
  auto diagram = builder.Build();
  diagram->set_name("cartpole_lqr_controller");

  LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm,
      std::move(diagram),
      state_receiver,
      FLAGS_channel_x,
      true);
  loop.Simulate();

  return 0;
}
}

int main(int argc, char* argv[]) {
  return dairlib::examples::cartpole::controller_main(argc, argv);
}