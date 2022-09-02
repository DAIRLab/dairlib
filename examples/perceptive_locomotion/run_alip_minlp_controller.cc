#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/high_level_command.h"
#include "examples/Cassie/systems/cassie_out_to_radio.h"
#include "multibody/multibody_utils.h"

#include "systems/controllers/footstep_planning/alip_minlp_footstep_controller.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "geometry/convex_foothold.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/lcm/lcm_scope_system.h"

namespace dairlib {

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using geometry::ConvexFoothold;

using systems::controllers::alip_utils::PointOnFramed;
using systems::controllers::AlipMINLPGains;
using systems::controllers::AlipMINLPFootstepController;

using drake::multibody::Frame;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::ConstantValueSource;
using drake::trajectories::PiecewisePolynomial;

DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");

DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the channel which publishes command");

DEFINE_string(
    cassie_out_channel, "CASSIE_OUTPUT_ECHO",
    "The name of the channel to receive the cassie out structure from.");

DEFINE_bool(is_two_phase, true,
            "true: only right/left single support"
            "false: both double and single support");

DEFINE_bool(spring_model, true, "");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_w_spr(0.0);
  AddCassieMultibody(&plant_w_spr, nullptr, true ,
                     "examples/Cassie/urdf/cassie_v2.urdf",
                     true, false);
  plant_w_spr.Finalize();
  auto context_w_spr = plant_w_spr.CreateDefaultContext();

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  auto left_toe = LeftToeFront(plant_w_spr);
  auto left_heel = LeftToeRear(plant_w_spr);
  auto right_toe = RightToeFront(plant_w_spr);
  auto right_heel = RightToeRear(plant_w_spr);

  // Create finite state machine
  int left_stance_state = 0;
  int right_stance_state = 1;
  double single_support_duration = 0.35;
  std::vector<int> fsm_states;
  std::vector<double> state_durations;

  fsm_states = {left_stance_state, right_stance_state};
  state_durations = {single_support_duration, single_support_duration};

  Vector3d mid_contact_point = (left_toe.first + left_heel.first) / 2;
  auto left_toe_mid = PointOnFramed(mid_contact_point, plant_w_spr.GetFrameByName("toe_left"));
  auto right_toe_mid = PointOnFramed(mid_contact_point, plant_w_spr.GetFrameByName("toe_right"));
  std::vector<PointOnFramed> left_right_toe = {left_toe_mid, right_toe_mid};

  auto gains = AlipMINLPGains{
    0.1, 0.85, 0.1, 3, 15,
    Matrix4d::Identity(), 10 * MatrixXd::Ones(1,1)};
  auto foot_placement_controller =
      builder.AddSystem<AlipMINLPFootstepController>(
          plant_w_spr, context_w_spr.get(), fsm_states, state_durations,
          left_right_toe, gains);

  ConvexFoothold big_square;
  big_square.SetContactPlane(Vector3d::UnitZ(), Vector3d::Zero()); // Flat Ground
  big_square.AddFace(Vector3d::UnitX(), 100 * Vector3d::UnitX());
  big_square.AddFace(Vector3d::UnitY(), 100 * Vector3d::UnitY());
  big_square.AddFace(-Vector3d::UnitX(), -100 * Vector3d::UnitX());
  big_square.AddFace(-Vector3d::UnitY(), -100 * Vector3d::UnitY());
  std::vector<ConvexFoothold> footholds = {big_square};

  auto foothold_oracle =
      builder.AddSystem<ConstantValueSource<double>>(
          drake::Value<std::vector<ConvexFoothold>>(footholds));

  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_spr);

  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
          FLAGS_cassie_out_channel, &lcm_local));
  auto cassie_out_to_radio =
      builder.AddSystem<systems::CassieOutToRadio>();

  auto high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
      plant_w_spr, context_w_spr.get(), 0.5, 1.0, 0.5);

  auto [footstep_scope, footstep_scope_pub] =
      drake::systems::lcm::LcmScopeSystem::AddToBuilder(
          &builder,
          &lcm_local,
          foot_placement_controller->get_output_port_footstep_target(),
          "FOOTSTEP_SCOPE",
          0);

  // --- Connect the diagram --- //
  // State Reciever connections
  builder.Connect(state_receiver->get_output_port(0),
                  high_level_command->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  foot_placement_controller->get_input_port_state());

  // planner ports
  builder.Connect(high_level_command->get_xy_output_port(),
                  foot_placement_controller->get_input_port_vdes());
  builder.Connect(foothold_oracle->get_output_port(),
                  foot_placement_controller->get_input_port_footholds());

  // misc
  builder.Connect(*cassie_out_receiver, *cassie_out_to_radio);
  builder.Connect(cassie_out_to_radio->get_output_port(),
                  high_level_command->get_radio_port());

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name("AlipMINLP foot placement controller");
//  DrawAndSaveDiagramGraph(*owned_diagram, "planner");

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x,
      false);
  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
