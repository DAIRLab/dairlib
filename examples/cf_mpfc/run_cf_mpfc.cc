#include "gflags/gflags.h"

// dairlib
#include "cassie_acom_function.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/high_level_command.h"
#include "examples/Cassie/systems/cassie_out_to_radio.h"
#include "systems/robot_lcm_systems.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/controllers/footstep_planning/cf_mpfc_system.h"
#include "systems/system_utils.h"


//drake
#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_value_source.h"

#include <iostream>

namespace dairlib {

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::VectorXd;

using systems::controllers::CFMPFCSystem;
using systems::controllers::alip_utils::PointOnFramed;

using drake::multibody::SpatialInertia;
using drake::multibody::RotationalInertia;
using drake::multibody::Frame;
using drake::systems::TriggerType;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerTypeSet;
using drake::systems::ConstantValueSource;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");

DEFINE_string(channel_mpc_output, "ALIP_MPC", "LCM channel for mpc output");

DEFINE_string(cassie_out_channel, "CASSIE_OUTPUT_ECHO",
              "The name of the channel to receive the cassie "
              "out structure from.");

int DoMain(int argc, char** argv) {

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant(0.0);
  auto instance = AddCassieMultibody(
      &plant,
      nullptr,
      true,
      "examples/Cassie/urdf/cassie_v2.urdf",
      true,
      false
  );
  plant.Finalize();

  std::unique_ptr<drake::systems::Context<double>> plant_context =
      plant.CreateDefaultContext();

  auto test_gait = systems::controllers::alip_utils::AlipGaitParams {
      0.85,
      32.0,
      0.3,
      0.1,
      0.3,
      Vector2d::UnitX(),
      systems::controllers::alip_utils::Stance::kLeft,
      systems::controllers::alip_utils::ResetDiscretization::kZOH
  };

  systems::controllers::cf_mpfc_params params;

  params.gait_params = test_gait;
  params.gait_params = test_gait;
  params.nmodes = 3;
  params.nknots = 4;
  params.contacts_in_stance_frame = {0.09 * Vector3d::UnitX(), -0.09 * Vector3d::UnitX()};
  params.soft_constraint_cost = 1000;
  params.com_pos_bound = Eigen::Vector2d::Ones();
  params.com_vel_bound = 2.0 * Eigen::Vector2d::Ones();
  params.Q = Eigen::Matrix4d::Identity();
  params.R = Eigen::Matrix3d::Identity();
  params.Qf = Eigen::Matrix4d::Identity();
  params.solver_options.SetOption(
      drake::solvers::GurobiSolver::id(), "Presolve", 1);
  params.solver_options.SetOption(
      drake::solvers::GurobiSolver::id(), "LogToConsole", 0);
  params.mu = 1;

  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);

  // Create finite state machine
  int left_stance_state = 0;
  int right_stance_state = 1;
  int post_left_double_support_state = 3;
  int post_right_double_support_state = 4;

  std::vector<int> left_right_fsm_states;
  std::vector<int> post_left_right_fsm_states;
  std::vector<double> state_durations;

  left_right_fsm_states = {left_stance_state, right_stance_state};
  post_left_right_fsm_states = {post_right_double_support_state,
                                post_left_double_support_state};

  Vector3d mid_contact_point = (left_toe.first + left_heel.first) / 2;
  auto left_toe_mid = PointOnFramed(mid_contact_point, plant.GetFrameByName("toe_left"));
  auto right_toe_mid = PointOnFramed(mid_contact_point, plant.GetFrameByName("toe_right"));
  std::vector<PointOnFramed> left_right_toe = {left_toe_mid, right_toe_mid};

  auto foot_placement_controller = builder.AddSystem<CFMPFCSystem>(
      plant,
      plant_context.get(),
      instance,
      left_right_fsm_states,
      post_left_right_fsm_states,
      left_right_toe,
      params,
      systems::controllers::CalcCassieAcomOrientationInWorld
  );

  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant);

  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<lcmt_cassie_out>(
          FLAGS_cassie_out_channel, &lcm_local));

  auto cassie_out_to_radio =
      builder.AddSystem<systems::CassieOutToRadio>();

  auto high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
      plant,
      plant_context.get(),
      2.0,  // rotational velocity command scaling
      1.0,  // sagittal velocity command scaling
      -0.5, // lateral vel scaling
      0.5   // stick filter time constant
  );

  auto mpc_output_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_alip_mpc_output>(
          FLAGS_channel_mpc_output, &lcm_local,
          TriggerTypeSet({TriggerType::kForced})));


  // --- Connect the rest of the diagram --- //
  // State Reciever connections
  builder.Connect(state_receiver->get_output_port(0),
                  high_level_command->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(0),
                  foot_placement_controller->get_input_port_state());

  // planner ports
  builder.Connect(high_level_command->get_output_port_xy(),
                  foot_placement_controller->get_input_port_vdes());
  builder.Connect(
      foot_placement_controller->get_output_port_mpc_output(),
      mpc_output_pub->get_input_port());

  // misc
  builder.Connect(*cassie_out_receiver, *cassie_out_to_radio);
  builder.Connect(cassie_out_to_radio->get_output_port(),
                  high_level_command->get_input_port_radio());

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name("cf_mpfc_diagram");
  DrawAndSaveDiagramGraph(*owned_diagram, "../cf_mpfc_diagram");

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x,
      true);

  loop.Simulate();
  return 0;



}
}

int main(int argc, char** argv) {
  return dairlib::DoMain(argc, argv);
}