#include "gflags/gflags.h"

// dairlib
#include "cassie_acom_function.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/high_level_command.h"
#include "examples/Cassie/systems/cassie_out_to_radio.h"
#include "geometry/convex_polygon_set.h"
#include "geometry/convex_polygon_lcm_systems.h"
#include "multibody/stepping_stone_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/perception/grid_map_lcm_systems.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/controllers/footstep_planning/flat_terrain_foothold_source.h"
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

using geometry::ConvexPolygon;
using geometry::ConvexPolygonSet;
using perception::GridMapReceiver;

using systems::controllers::CFMPFCSystem;
using systems::controllers::alip_utils::PointOnFramed;
using systems::controllers::cf_mpfc_params_io;
using systems::FlatTerrainFootholdSource;

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

DEFINE_string(channel_mpc_output, "CF_MPFC", "LCM channel for mpc output");

DEFINE_string(channel_mpc_debug, "CF_MPFC_DEBUG", "channel for debugging");

DEFINE_string(cassie_out_channel, "CASSIE_OUTPUT_ECHO",
              "The name of the channel to receive the cassie "
              "out structure from.");

DEFINE_string(gains_yaml, "examples/cf_mpfc/gains/mpfc_gains.yaml",
              "mpc gains yaml");

DEFINE_string(solver_options_yaml,
              "examples/cf_mpfc/gains/gurobi_options_planner.yaml",
              "solver options yaml");

DEFINE_string(foothold_yaml, "", "yaml file with footholds for simulation");

DEFINE_bool(use_perception, false, "get footholds from perception system");

DEFINE_string(channel_terrain, "FOOTHOLDS_PROCESSED",
              "lcm channel containing the footholds");

DEFINE_string(channel_elevation, "CASSIE_ELEVATION_MAP",
              "elevation map lcm channel");

int DoMain(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

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

  auto params = cf_mpfc_params_io::get_params_from_yaml(
      FLAGS_gains_yaml, FLAGS_solver_options_yaml, plant, *plant_context);

  std::vector<ConvexPolygon> footholds;
  if ( !FLAGS_foothold_yaml.empty() ) {
    footholds =
        multibody::LoadSteppingStonesFromYaml(FLAGS_foothold_yaml).footholds;
  }
  auto foothold_source =
      std::make_unique<ConstantValueSource<double>>(
          drake::Value<ConvexPolygonSet>(footholds));

  auto foot_placement_controller = builder.AddSystem<CFMPFCSystem>(
      plant,
      plant_context.get(),
      instance,
      left_right_fsm_states,
      post_left_right_fsm_states,
      left_right_toe,
      params
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
      0.8,  // sagittal velocity command scaling
      -0.5, // lateral vel scaling
      0.1   // stick filter time constant
  );

  auto mpc_output_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_cf_mpfc_output>(
          FLAGS_channel_mpc_output, &lcm_local,
          TriggerTypeSet({TriggerType::kForced})));

  auto mpc_debug_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_cf_mpfc_solution>(
          FLAGS_channel_mpc_debug, &lcm_local,
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
              plant, plant_context.get(), left_right_toe);
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
  builder.Connect(
      foot_placement_controller->get_output_port_mpc_output(),
      mpc_output_pub->get_input_port());
  builder.Connect(
      foot_placement_controller->get_output_port_mpc_debug(),
      mpc_debug_pub->get_input_port());

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