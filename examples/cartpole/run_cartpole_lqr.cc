#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "darilib/lcmt_robot_input.hpp"

#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/diagram.h"

#include "systems/robot_lcm_systems.h"
#include "common/find_resource.h"
#include "systems/framework/lcm_driven_loop.h"

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::controllers::LinearQuadraticRegulator;
using drake::systems::DiagramBuilder;
using drake::systems::Diagram;
using drake::systems::Simulator;
using drake::systems::Context;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;

using dairlib::systems::LcmDrivenLoop;
using dairlib::FindResourceOrThrow;
using dairlib::systems::RobotOutputReceiver;
using dairlib::systems::RobotCommandSender;

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

  MatrixXd Q = 10 * MatrixXd::Identity(4,4);
  MatrixXd R = MatrixXd::Identity(1,1);


  auto lqr = LinearQuadraticRegulator(
      plant, *plant_context, Q, R, MatrixXd::Zero(0,0),
      plant.get_actuation_input_port().get_index());
  builder.AddSystem(std::move(lqr));

  // LCM
  drake::lcm::DrakeLcm lcm;
  auto input_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm));

  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(plant);
  auto input_sender = builder.AddSystem<RobotCommandSender>(plant);

  // Wire diagram
  builder.Connect(*state_receiver, *lqr);
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
}
}

int main(int argc, char* argv[]) {
  return dairlib::examples::cartpole::controller_main(argc, argv);
}