#include <iostream>

#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_footstep_target.hpp"
#include "dairlib/lcmt_fsm_info.hpp"

#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/high_level_command.h"
#include "examples/Cassie/systems/cassie_out_to_radio.h"
#include "multibody/multibody_utils.h"
#include "multibody/stepping_stone_utils.h"

#include "systems/controllers/footstep_planning/alip_deadbeat_footstep_controller.h"
#include "systems/controllers/footstep_planning/flat_terrain_foothold_source.h"
#include "systems/controllers/footstep_planning/footstep_lcm_systems.h"
#include "systems/primitives/fsm_lcm_systems.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "examples/perceptive_locomotion/gains/alip_minlp_gains.h"

#include "geometry/convex_foothold_set.h"

#ifdef DAIR_ROS_ON
#include "geometry/convex_foothold_receiver.h"
#include "systems/ros/ros_subscriber_system.h"
#endif

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

using geometry::ConvexFoothold;
using geometry::ConvexFootholdSet;

using systems::controllers::AlipDeadbeatFootstepController;
using systems::controllers::alip_utils::PointOnFramed;
using systems::controllers::AlipMINLPGains;
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

DEFINE_string(foothold_yaml, "", "yaml file with footholds from simulation");

DEFINE_string(foothold_topic, "", "ros topic containing the footholds");

DEFINE_bool(spring_model, true, "");

DEFINE_bool(use_perception, false, "get footholds from percption system");

DEFINE_bool(add_camera_inertia, true,
            "whether to add the camera inertia to the plant model");

DEFINE_bool(plan_offboard, false,
            "Sets the planner lcm TTL to be 1. "
            "Set to true to run planner on cassie-laptop");

DEFINE_double(sim_delay, 0.0, "> 0 adds delay to mimic planning offboard");


int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

#ifdef DAIR_ROS_ON
  ros::init(argc, argv, "alip_minlp_controller");
  ros::NodeHandle node_handle;
#else
  if (FLAGS_use_perception) {
    throw std::runtime_error(
        "You cannot use perception without building against ROS.");
  }
#endif

  auto gains_mpc =
      drake::yaml::LoadYamlFile<AlipMINLPGainsImport>(FLAGS_minlp_gains_filename);

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

  gains_mpc.SetFilterData(
      plant_w_spr.CalcTotalMass(*context_w_spr), gains_mpc.h_des);

  std::vector<ConvexFoothold> footholds;
  if ( !FLAGS_foothold_yaml.empty() ) {
    footholds =
        multibody::LoadSteppingStonesFromYaml(FLAGS_foothold_yaml).footholds;
  }
  auto foothold_source =
      std::make_unique<ConstantValueSource<double>>(
      drake::Value<ConvexFootholdSet>(footholds));

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");

  auto left_toe = LeftToeFront(plant_w_spr);
  auto left_heel = LeftToeRear(plant_w_spr);
  auto right_toe = RightToeFront(plant_w_spr);
  auto right_heel = RightToeRear(plant_w_spr);

  // Create finite state machine
  int left_stance_state = 0;
  int right_stance_state = 1;
  int post_left_double_support_state = 3;
  int post_right_double_support_state = 4;
  double single_support_duration = gains_mpc.ss_time;
  double double_support_duration = gains_mpc.ds_time;


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

  auto foot_placement_controller =
      builder.AddSystem<AlipDeadbeatFootstepController>(
          plant_w_spr, context_w_spr.get(), left_right_fsm_states,
          post_left_right_fsm_states, state_durations, double_support_duration,
          left_right_toe, gains_mpc.gains);

  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_spr);

  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<lcmt_cassie_out>(
          FLAGS_cassie_out_channel, &lcm_local));

  auto cassie_out_to_radio =
      builder.AddSystem<systems::CassieOutToRadio>();

  auto high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
      plant_w_spr, context_w_spr.get(), 2.0, 1.5, -0.5, 0.5);

  auto footstep_sender = builder.AddSystem<FootstepSender>();

  std::unique_ptr<LcmPublisherSystem> footstep_pub_ptr;
  std::unique_ptr<LcmPublisherSystem> fsm_pub_ptr;
  if (FLAGS_plan_offboard) {
    footstep_pub_ptr = LcmPublisherSystem::Make<lcmt_footstep_target>(FLAGS_channel_foot, &lcm_network);
    fsm_pub_ptr = LcmPublisherSystem::Make<lcmt_fsm_info>(FLAGS_fsm_channel, &lcm_network);
  } else {
    footstep_pub_ptr = LcmPublisherSystem::Make<lcmt_footstep_target>(FLAGS_channel_foot, &lcm_local);
    fsm_pub_ptr = LcmPublisherSystem::Make<lcmt_fsm_info>(FLAGS_fsm_channel, &lcm_local);
  }

  auto footstep_pub = builder.AddSystem(std::move(footstep_pub_ptr));
  auto fsm_sender = builder.AddSystem<FsmSender>(plant_w_spr);
  auto fsm_pub = builder.AddSystem(std::move(fsm_pub_ptr));


  // --- Add and connect the source of the foothold information --- //
  if ( !FLAGS_foothold_yaml.empty() ) {
    DRAKE_ASSERT(FLAGS_channel_x == "CASSIE_STATE_SIMULATION");
    auto foothold_oracle = builder.AddSystem(std::move(foothold_source));
    builder.Connect(foothold_oracle->get_output_port(),
                    foot_placement_controller->get_input_port_footholds());
  } else {

    if (FLAGS_use_perception) {
#ifdef DAIR_ROS_ON
      auto plane_subscriber = builder.AddSystem(
          systems::RosSubscriberSystem<
              convex_plane_decomposition_msgs::PlanarTerrain>::Make(
                  FLAGS_foothold_topic, &node_handle));
      auto plane_receiver = builder.AddSystem<geometry::ConvexFootholdReceiver>();
      builder.Connect(*plane_subscriber, *plane_receiver);
      builder.Connect(plane_receiver->get_output_port(),
                      foot_placement_controller->get_input_port_footholds());
#endif
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
                  high_level_command->get_state_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  foot_placement_controller->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(0),
                  fsm_sender->get_input_port_state());

  // planner ports
  builder.Connect(high_level_command->get_xy_output_port(),
                  foot_placement_controller->get_input_port_vdes());

  // planner out ports
  builder.Connect(foot_placement_controller->get_output_port_fsm(),
                  fsm_sender->get_input_port_fsm());
  builder.Connect(foot_placement_controller->get_output_port_prev_impact_time(),
                  fsm_sender->get_input_port_prev_switch_time());
  builder.Connect(foot_placement_controller->get_output_port_next_impact_time(),
                  fsm_sender->get_input_port_next_switch_time());
  builder.Connect(foot_placement_controller->get_output_port_footstep_target(),
                  footstep_sender->get_input_port());

  // misc
  builder.Connect(*cassie_out_receiver, *cassie_out_to_radio);
  builder.Connect(cassie_out_to_radio->get_output_port(),
                  high_level_command->get_radio_port());


  if (FLAGS_sim_delay > 0) {
    auto fzoh = builder.AddSystem<drake::systems::ZeroOrderHold<double>>(
        FLAGS_sim_delay, drake::Value<dairlib::lcmt_footstep_target>());
    builder.Connect(*footstep_sender, *fzoh);
    builder.Connect(*fzoh, *footstep_pub);
    auto fsmzoh = builder.AddSystem<drake::systems::ZeroOrderHold<double>>(
        FLAGS_sim_delay, drake::Value<dairlib::lcmt_fsm_info>());
    builder.Connect(fsm_sender->get_output_port_fsm_info(),
                    fsmzoh->get_input_port());
    builder.Connect(*fsmzoh, *fsm_pub);

  } else {
    builder.Connect(*footstep_sender, *footstep_pub);
    builder.Connect(fsm_sender->get_output_port_fsm_info(),
                    fsm_pub->get_input_port());
  }




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