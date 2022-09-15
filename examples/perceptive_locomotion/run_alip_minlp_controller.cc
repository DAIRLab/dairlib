#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_footstep_target.hpp"
#include "dairlib/lcmt_fsm_info.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/high_level_command.h"
#include "examples/Cassie/systems/cassie_out_to_radio.h"
#include "multibody/multibody_utils.h"

#include "systems/controllers/footstep_planning/alip_minlp_footstep_controller.h"
#include "systems/controllers/footstep_planning/flat_terrain_foothold_source.h"
#include "systems/controllers/footstep_planning/footstep_lcm_systems.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/primitives/fsm_lcm_systems.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "examples/perceptive_locomotion/gains/alip_minlp_gains.h"

#include "geometry/convex_foothold.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/constant_value_source.h"

namespace dairlib {

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using geometry::ConvexFoothold;

using systems::controllers::AlipMINLPFootstepController;
using systems::controllers::alip_utils::PointOnFramed;
using systems::controllers::AlipMINLPGains;
using systems::controllers::FootstepSender;
using systems::FlatTerrainFootholdSource;
using systems::FsmSender;

using drake::multibody::Frame;
using drake::systems::TriggerType;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerTypeSet;
using drake::systems::ConstantValueSource;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::trajectories::PiecewisePolynomial;

DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");

DEFINE_string(channel_foot, "FOOTSTEP_TARGET",
              "LCM channel for footstep target");

DEFINE_string(channel_com, "ALIP_COM_TRAJ",
              "LCM channel for center of mass trajectory");

DEFINE_string(fsm_channel, "FSM", "lcm channel for fsm");

DEFINE_string(cassie_out_channel, "CASSIE_OUTPUT_ECHO",
              "The name of the channel to receive the cassie "
              "out structure from.");

DEFINE_string(minlp_gains_filename,
              "examples/perceptive_locomotion/gains/alip_minlp_gains.yaml",
              "Filepath to alip minlp gains");

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
  int post_left_double_support_state = 3;
  int post_right_double_support_state = 4;
  double single_support_duration = 0.4;
  double double_support_duration = 0.05;


  std::vector<int> left_right_fsm_states;
  std::vector<int> post_left_right_fsm_states;
  std::vector<double> state_durations;

  left_right_fsm_states = {left_stance_state, right_stance_state};
  post_left_right_fsm_states = {post_right_double_support_state,
                                post_left_double_support_state};
  state_durations = {single_support_duration, single_support_duration};

  Vector3d mid_contact_point = (left_toe.first + left_heel.first) / 2;
  auto left_toe_mid = PointOnFramed(mid_contact_point, plant_w_spr.GetFrameByName("toe_left"));
  auto right_toe_mid = PointOnFramed(mid_contact_point, plant_w_spr.GetFrameByName("toe_right"));
  std::vector<PointOnFramed> left_right_toe = {left_toe_mid, right_toe_mid};

  auto gains_mpc =
      drake::yaml::LoadYamlFile<AlipMINLPGainsImport>(FLAGS_minlp_gains_filename);
  gains_mpc.SetFilterData(
      plant_w_spr.CalcTotalMass(*context_w_spr), gains_mpc.h_des);

  auto foot_placement_controller =
      builder.AddSystem<AlipMINLPFootstepController>(
          plant_w_spr, context_w_spr.get(), left_right_fsm_states,
          post_left_right_fsm_states, state_durations, double_support_duration,
          left_right_toe, gains_mpc.gains);

  auto foothold_oracle =
      builder.AddSystem<FlatTerrainFootholdSource>(
          plant_w_spr, context_w_spr.get(), left_right_toe);

  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_spr);

  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<lcmt_cassie_out>(
          FLAGS_cassie_out_channel, &lcm_local));

  auto cassie_out_to_radio =
      builder.AddSystem<systems::CassieOutToRadio>();

  auto high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
      plant_w_spr, context_w_spr.get(), 0.5, 1.0, 0.5);

  auto footstep_sender = builder.AddSystem<FootstepSender>();
  auto footstep_pub_ptr = LcmPublisherSystem::Make<lcmt_footstep_target>(FLAGS_channel_foot, &lcm_local);
  auto footstep_pub = builder.AddSystem(std::move(footstep_pub_ptr));

  auto fsm_sender = builder.AddSystem<FsmSender>(plant_w_spr);
  auto fsm_pub_ptr = LcmPublisherSystem::Make<lcmt_fsm_info>(FLAGS_fsm_channel, &lcm_local);
  auto fsm_pub = builder.AddSystem(std::move(fsm_pub_ptr));

  auto com_traj_pub_ptr = LcmPublisherSystem::Make<lcmt_saved_traj>(FLAGS_channel_com, &lcm_local);
  auto com_traj_pub = builder.AddSystem(std::move(com_traj_pub_ptr));

  auto mpc_debug_pub_ptr = LcmPublisherSystem::Make<lcmt_mpc_debug>(
      "ALIP_MINLP_DEBUG", &lcm_local, TriggerTypeSet({TriggerType::kForced}));
  auto mpc_debug_pub = builder.AddSystem(std::move(mpc_debug_pub_ptr));

  // --- Connect the diagram --- //
  // State Reciever connections
  builder.Connect(state_receiver->get_output_port(0),
                  high_level_command->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  foot_placement_controller->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(0),
                  fsm_sender->get_input_port_state());
  builder.Connect(*state_receiver, *foothold_oracle);

  // planner ports
  builder.Connect(high_level_command->get_xy_output_port(),
                  foot_placement_controller->get_input_port_vdes());
  builder.Connect(foothold_oracle->get_output_port(),
                  foot_placement_controller->get_input_port_footholds());

  // planner out ports
  builder.Connect(foot_placement_controller->get_output_port_fsm(),
                  fsm_sender->get_input_port_fsm());
  builder.Connect(foot_placement_controller->get_output_port_prev_impact_time(),
                  fsm_sender->get_input_port_prev_switch_time());
  builder.Connect(foot_placement_controller->get_output_port_next_impact_time(),
                  fsm_sender->get_input_port_next_switch_time());
  builder.Connect(foot_placement_controller->get_output_port_footstep_target(),
                  footstep_sender->get_input_port());
  builder.Connect(foot_placement_controller->get_output_port_com_traj(),
                  com_traj_pub->get_input_port());
  builder.Connect(foot_placement_controller->get_output_port_mpc_debug(),
                  mpc_debug_pub->get_input_port());

  // misc
  builder.Connect(*cassie_out_receiver, *cassie_out_to_radio);
  builder.Connect(cassie_out_to_radio->get_output_port(),
                  high_level_command->get_radio_port());
  builder.Connect(fsm_sender->get_output_port_fsm_info(),
                  fsm_pub->get_input_port());
  builder.Connect(*footstep_sender, *footstep_pub);

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name("AlipMINLP foot placement controller");
  DrawAndSaveDiagramGraph(*owned_diagram, "../planner");

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x,
      false);
  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
