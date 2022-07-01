//
// Created by Zach on 7/1/22.
//

#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_target_standing_height.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/standing_com_traj.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/controllers/standing_pelvis_pd.h"


#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace dairlib {

using systems::controllers::StandingPelvisPD;

using std::cout;
using std::endl;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::TriggerTypeSet;


DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the channel which publishes command");
DEFINE_string(
    cassie_out_channel, "CASSIE_OUTPUT_ECHO",
    "The name of the channel to receive the cassie out structure from.");
DEFINE_bool(print_osc, false, "whether to print the osc debug message or not");
DEFINE_double(cost_weight_multiplier, 1.0,
              "A cosntant times with cost weight of OSC traj tracking");
DEFINE_double(height, .8, "The initial COM height (m)");
DEFINE_string(gains_filename, "examples/Cassie/osc/osc_standing_gains.yaml",
              "Filepath containing gains");
DEFINE_bool(use_radio, false, "use the radio to set height or not");
DEFINE_double(qp_time_limit, 0, "maximum qp solve time");

// Currently the controller runs at the rate between 500 Hz and 200 Hz, so the
// publish rate of the robot state needs to be less than 500 Hz. Otherwise, the
// performance seems to degrade due to this. (Recommended publish rate: 200 Hz)
// Maybe we need to update the lcm driven loop to clear the queue of lcm message
// if it's more than one message?

int DoMain(int argc, char* argv[]) {
  std::cout << "here 0" << std::endl;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // Build fix-spring Cassie MBP
  drake::multibody::MultibodyPlant<double> plant(0.0);
  AddCassieMultibody(&plant, nullptr, true,
                     "examples/Cassie/urdf/cassie_v2.urdf",
                     true,true);
  plant.Finalize();
  std::cout << "here 1" << std::endl;
  auto plant_context = plant.CreateDefaultContext();

  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);
  auto right_toe = RightToeFront(plant);
  auto right_heel = RightToeRear(plant);

  // Build the controller diagram
  DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm lcm_local;

  MatrixXd K_p = Eigen::MatrixXd::Identity(6,6);
  K_p.block(3,3, 3, 3) *= 50;
  MatrixXd K_d = Eigen::MatrixXd::Identity(6,6);
  K_d.block(3,3,3,3) *= 5;
  double k_cp = 1;


  // Create Lcm subsriber for lcmt_target_standing_height
  auto target_height_receiver = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_target_standing_height>(
          "TARGET_HEIGHT", &lcm_local));

  // Create state receiver.
  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant);

//  auto cassie_out_receiver =
//      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
//          FLAGS_cassie_out_channel, &lcm_local));
//  auto cassie_out_to_radio =
//      builder.AddSystem<systems::CassieOutToRadio>();
//  builder.Connect(*cassie_out_receiver, *cassie_out_to_radio);

  // Create command sender.
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm_local, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

//  // Create desired center of mass traj
//  std::vector<std::pair<const Vector3d, const drake::multibody::Frame<double>&>>
//      feet_contact_points = {left_toe, left_heel, right_toe, right_heel};


  // Add PD Controller
  multibody::KinematicEvaluatorSet<double> evaluators(plant);
  auto left_loop = LeftLoopClosureEvaluator(plant);
  auto right_loop = RightLoopClosureEvaluator(plant);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  std::cout << "here 2" << std::endl;
  // Create pelvis pd controller
  auto pelvis_pd = builder.AddSystem<StandingPelvisPD>(
        plant, plant_context.get(), &evaluators, K_p, K_d, k_cp);

  std::cout << "here 3" << std::endl;
  VectorXd P_des = VectorXd::Zero(7);
  P_des(2) = FLAGS_height;

  auto des_traj =
      builder.AddSystem<drake::systems::ConstantVectorSource>(P_des);
  std::cout << "here 4" << std::endl;

  // Connect ports
  builder.Connect(state_receiver->get_output_port(0),
                  pelvis_pd->get_input_port_state_actual());
  builder.Connect(pelvis_pd->get_output_port_torques(),
                  command_sender->get_input_port(0));
  builder.Connect(des_traj->get_output_port(),
                  pelvis_pd->get_input_port_state_desired());

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("pelvis pd standing controller"));

  // Build lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x,
      true);

  loop.Simulate();
  return 0;
}
}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
