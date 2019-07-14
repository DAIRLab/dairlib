#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/analysis/simulator.h"

#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_pd_config.hpp"
#include "systems/robot_lcm_systems.h"
#include "systems/controllers/linear_controller.h"
#include "systems/controllers/pd_config_lcm.h"
#include "examples/Cassie/cassie_utils.h"

namespace dairlib {

using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::DiagramBuilder;


int DoMain() {
  DiagramBuilder<double> builder;

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(
               "udpm://239.255.76.67:7667?ttl=0");

  RigidBodyTree<double> tree_with_springs;
  RigidBodyTree<double> tree_without_springs;
  buildCassieTree(tree_with_springs, "examples/Cassie/urdf/cassie_v2.urdf",
                  drake::multibody::joints::kQuaternion);
  buildCassieTree(tree_without_springs, "examples/Cassie/urdf/cassie_v2.urdf",
                  drake::multibody::joints::kQuaternion);
  const double terrain_size = 100;
  const double terrain_depth = 0.20;
  drake::multibody::AddFlatTerrainToWorld(&tree_with_springs,
                                          terrain_size, terrain_depth);
  drake::multibody::AddFlatTerrainToWorld(&tree_without_springs,
                                          terrain_size, terrain_depth);

  const std::string channel_x = "CASSIE_STATE";
  const std::string channel_u = "CASSIE_INPUT";

  // Create state receiver.
  auto state_sub = builder.AddSystem(
                     LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
                       channel_x, lcm));
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(
                          tree_with_springs);
  builder.Connect(state_sub->get_output_port(),
                  state_receiver->get_input_port(0));

  // Create command sender.
  auto command_pub = builder.AddSystem(
                       LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
                         channel_u, lcm, 1.0 / 1000.0));
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(
                          tree_with_springs);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

  // Get body indices for cassie with springs
  int left_toe_ind = GetBodyIndexFromName(tree_with_springs, "toe_left");
  int right_toe_ind = GetBodyIndexFromName(tree_with_springs, "toe_right");
  int left_thigh_ind = GetBodyIndexFromName(tree_with_springs, "thigh_left");
  int right_thigh_ind = GetBodyIndexFromName(tree_with_springs, "thigh_right");
  int left_heel_spring_ind = GetBodyIndexFromName(tree_with_springs,
                             "heel_spring_left");
  int right_heel_spring_ind = GetBodyIndexFromName(tree_with_springs,
                              "heel_spring_right");
  DRAKE_DEMAND(left_toe_ind != -1 && right_toe_ind != -1 &&
               left_thigh_ind != -1 && right_thigh_ind != -1 &&
               left_heel_spring_ind != -1 && right_heel_spring_ind != -1);

  // Create finite state machine
  int left_stance_state = 2;
  int right_stance_state = 3;
  int initial_state_number = 2;
  double duration_per_state = 0.35;
  double time_shift = 0;
  auto fsm = builder.AddSystem<systems::controllers::TimeBasedFiniteStateMachine>(
               &tree_with_springs,
               left_stance_state, right_stance_state, initial_state_number,
               duration_per_state, time_shift);
  builder.Connect(state_receiver->get_output_port(0),
                  fsm->get_input_port_state());

  // Create CoM trajectory generator
  double desired_com_height = 0.89;
  auto lipm_traj_generator =
    builder.AddSystem<systems::controllers::LIPMTrajGenerator>(&tree_with_springs,
        desired_com_height,
        duration_per_state,
        left_stance_state,
        right_stance_state,
        left_toe_ind,
        Eigen::VectorXd::Zero(3),
        right_toe_ind,
        Eigen::VectorXd::Zero(3));
  builder.Connect(fsm->get_output_port(0),
                  lipm_traj_generator->get_input_port_fsm());
  builder.Connect(state_receiver->get_output_port(0),
                  lipm_traj_generator->get_input_port_state());

  // Create swing leg trajectory generator (capture point)


  // Create Operational space control
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
               OscTrackingDataSet * tracking_data_set,
               &tree_with_springs,
               &tree_without_springs);
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_input_port_output());



  builder.Connect(osc->get_output_port(0),
                  command_sender->get_input_port(0));

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  /// Set realtime rate. Otherwise, runs as fast as possible
  auto stepper = std::make_unique<drake::systems::Simulator<double>>(*diagram,
                 std::move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(1.0);
  stepper->Initialize();


  drake::log()->info("controller started");

  stepper->StepTo(std::numeric_limits<double>::infinity());

  return 0;
}


}

int main() { return dairlib::DoMain(); }
