#include <iostream>
#include <signal.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_output.hpp"

#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/high_level_command.h"
#include "examples/Cassie/systems/cassie_out_to_radio.h"
#include "multibody/multibody_utils.h"
#include "multibody/stepping_stone_utils.h"
#include "solvers/solver_options_io.h"
#include "systems/filters/floating_base_velocity_filter.h"
#include "systems/controllers/footstep_planning/alip_mpfc_s2s_system.h"
#include "systems/controllers/footstep_planning/flat_terrain_foothold_source.h"
#include "systems/controllers/footstep_planning/footstep_lcm_systems.h"
#include "systems/primitives/fsm_lcm_systems.h"
#include "systems/perception/grid_map_lcm_systems.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "geometry/convex_polygon_set.h"
#include "geometry/convex_polygon_lcm_systems.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace dairlib {

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using geometry::ConvexPolygon;
using geometry::ConvexPolygonSet;

using perception::GridMapReceiver;

using systems::controllers::Alips2sMPFCSystem;
using systems::controllers::alip_utils::PointOnFramed;
using systems::controllers::FootstepSender;
using systems::FlatTerrainFootholdSource;
using systems::FsmSender;

using drake::multibody::SpatialInertia;
using drake::multibody::RotationalInertia;
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

DEFINE_string(channel_mpc_output, "ALIP_MPC", "LCM channel for mpc output");

DEFINE_string(cassie_out_channel, "CASSIE_OUTPUT_ECHO",
              "The name of the channel to receive the cassie "
              "out structure from.");

DEFINE_string(mpfc_gains_filename,
              "examples/perceptive_locomotion/gains/alip_s2s_mpfc_gains.yaml",
              "Filepath to alip mpfc gains");

DEFINE_string(solver_options_filename,
              "examples/perceptive_locomotion/gains/gurobi_options_planner.yaml",
              "Filepath to the gurobi solver options");

DEFINE_string(foothold_yaml, "", "yaml file with footholds for simulation");

DEFINE_string(channel_terrain, "FOOTHOLDS_PROCESSED",
              "lcm channel containing the footholds");

DEFINE_string(channel_elevation, "CASSIE_ELEVATION_MAP",
              "elevation map lcm channel");

DEFINE_bool(spring_model, true, "");

DEFINE_bool(use_perception, false, "get footholds from perception system");

DEFINE_bool(plan_offboard, false,
            "Sets the planner lcm TTL to be 1. "
            "Set to true to run planner on cassie-laptop");

DEFINE_bool(add_camera_inertia, true,
            "adds inertia from the realsense mount to the plant model");

DEFINE_double(sim_delay, 0.0, "> 0 adds delay to mimic planning offboard");


int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);



  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_w_spr(0.0);
  auto instance_w_spr = AddCassieMultibody(
      &plant_w_spr,
      nullptr,
      true,
      "examples/Cassie/urdf/cassie_v2.urdf",
      true,
      false
  );
  if (FLAGS_add_camera_inertia) {
    auto camera_inertia_about_com =
        RotationalInertia<double>::MakeFromMomentsAndProductsOfInertia(
            0.04, 0.04, 0.04, 0, 0, 0);
    auto camera_inertia = SpatialInertia<double>::MakeFromCentralInertia(
        1.06, Vector3d(0.07, 0.0, 0.17), camera_inertia_about_com);
    plant_w_spr.AddRigidBody("camera_inertia", instance_w_spr, camera_inertia);
    plant_w_spr.WeldFrames(
        plant_w_spr.GetBodyByName("pelvis").body_frame(),
        plant_w_spr.GetBodyByName("camera_inertia").body_frame()
    );
  }
  plant_w_spr.Finalize();
  auto context_w_spr = plant_w_spr.CreateDefaultContext();

  auto gains_mpc = systems::controllers::MakeAlipS2SMPFCParamsFromYaml(
      FLAGS_mpfc_gains_filename, FLAGS_solver_options_filename,
      plant_w_spr, *context_w_spr
  );


  std::vector<ConvexPolygon> footholds;
  if ( !FLAGS_foothold_yaml.empty() ) {
    footholds =
        multibody::LoadSteppingStonesFromYaml(FLAGS_foothold_yaml).footholds;
  }
  auto foothold_source =
  std::make_unique<ConstantValueSource<double>>(
      drake::Value<ConvexPolygonSet>(footholds));

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");

  auto left_toe = LeftToeFront(plant_w_spr);
  auto left_heel = LeftToeRear(plant_w_spr);

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
  auto left_toe_mid = PointOnFramed(mid_contact_point, plant_w_spr.GetFrameByName("toe_left"));
  auto right_toe_mid = PointOnFramed(mid_contact_point, plant_w_spr.GetFrameByName("toe_right"));
  std::vector<PointOnFramed> left_right_toe = {left_toe_mid, right_toe_mid};

  auto foot_placement_controller = builder.AddSystem<Alips2sMPFCSystem>(
      plant_w_spr,
      context_w_spr.get(),
      left_right_fsm_states,
      post_left_right_fsm_states,
      left_right_toe,
      gains_mpc
  );

  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_spr);

  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<lcmt_cassie_out>(
          FLAGS_cassie_out_channel, &lcm_local));

  auto cassie_out_to_radio =
      builder.AddSystem<systems::CassieOutToRadio>();

  auto high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
      plant_w_spr,
      context_w_spr.get(),
      2.0,  // rotational velocity command scaling
      1.5,  // sagittal velocity command scaling
      -0.5, // lateral vel scaling
      0.5   // stick filter time constant
  );

  auto mpc_output_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_alip_mpc_output>(
          FLAGS_channel_mpc_output,
          FLAGS_plan_offboard? &lcm_network : &lcm_local,
          TriggerTypeSet({TriggerType::kForced})));

  auto mpc_debug_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_alip_s2s_mpfc_debug>(
          "ALIP_S2S_MPFC_DEBUG", &lcm_local,
          TriggerTypeSet({TriggerType::kForced})));

  // --- Add and connect the source of the foothold information --- //
  if ( !FLAGS_foothold_yaml.empty() ) {
    DRAKE_ASSERT(FLAGS_channel_x == "CASSIE_STATE_SIMULATION");
    auto foothold_oracle = builder.AddSystem(std::move(foothold_source));
    builder.Connect(foothold_oracle->get_output_port(),
                    foot_placement_controller->get_input_port_footholds());
  } else {
    if (FLAGS_use_perception) {
      auto plane_subscriber = builder.AddSystem(
          LcmSubscriberSystem::Make<lcmt_foothold_set>(
              FLAGS_channel_terrain, &lcm_local));
      auto plane_receiver =
          builder.AddSystem<geometry::ConvexPolygonReceiver>();
      auto map_subscriber = builder.AddSystem(
          LcmSubscriberSystem::Make<lcmt_grid_map>(
              FLAGS_channel_elevation, &lcm_local));
      auto map_receiver = builder.AddSystem<GridMapReceiver>();
      builder.Connect(*plane_subscriber, *plane_receiver);
      builder.Connect(plane_receiver->get_output_port(),
                      foot_placement_controller->get_input_port_footholds());
      builder.Connect(*map_subscriber, *map_receiver);
      builder.Connect(map_receiver->get_output_port(),
                      foot_placement_controller->get_input_port_elevation());
    } else {
      auto foothold_oracle =
          builder.AddSystem<FlatTerrainFootholdSource>(
              plant_w_spr, context_w_spr.get(), left_right_toe);
      builder.Connect(*state_receiver, *foothold_oracle);
      builder.Connect(foothold_oracle->get_output_port(),
                      foot_placement_controller->get_input_port_footholds());
    }
  }

  // --- Connect the rest of the diagram --- //
  // State Reciever connections
  builder.Connect(state_receiver->get_output_port(0),
                  high_level_command->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(0),
                  foot_placement_controller->get_input_port_state());

  // planner ports
  builder.Connect(high_level_command->get_output_port_xy(),
                  foot_placement_controller->get_input_port_vdes());

  // planner out ports
  builder.Connect(foot_placement_controller->get_output_port_mpc_debug(),
                  mpc_debug_pub->get_input_port());

  // misc
  builder.Connect(*cassie_out_receiver, *cassie_out_to_radio);
  builder.Connect(cassie_out_to_radio->get_output_port(),
                  high_level_command->get_input_port_radio());


  if (FLAGS_sim_delay > 0) {
    auto zoh = builder.AddSystem<drake::systems::ZeroOrderHold<double>>(
        FLAGS_sim_delay, drake::Value<dairlib::lcmt_alip_mpc_output>());
    builder.Connect(foot_placement_controller->get_output_port_mpc_output(),
                    zoh->get_input_port());
    builder.Connect(*zoh, *mpc_output_pub);
  } else {
    builder.Connect(foot_placement_controller->get_output_port_mpc_output(),
                    mpc_output_pub->get_input_port());
  }

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name("alip_mpcf_diagram");
  DrawAndSaveDiagramGraph(*owned_diagram, "../alip_mpfc");

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x,
      true);

  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }