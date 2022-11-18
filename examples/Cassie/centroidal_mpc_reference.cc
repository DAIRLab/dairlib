#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/kinematic_centroidal_planner/contact_scheduler.h"
#include "examples/Cassie/kinematic_centroidal_planner/kinematic_trajectory_generator.h"
#include "examples/Cassie/osc/high_level_command.h"
#include "examples/Cassie/systems/cassie_out_to_radio.h"
#include "lcm/lcm_trajectory.h"
#include "systems/controllers/controller_failure_aggregator.h"
#include "systems/controllers/kinematic_centroidal_mpc/kinematic_centroidal_mpc.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/primitives/trajectory_passthrough.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/kinematic_centroidal_planner/kinematic_centroidal_gains.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"

namespace dairlib {

using std::map;
using std::pair;
using std::string;
using std::vector;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::geometry::SceneGraph;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::trajectories::PiecewisePolynomial;
using systems::LcmTrajectoryReceiver;
using systems::TrajectoryPassthrough;

using multibody::CreateWithSpringsToWithoutSpringsMapPos;
using multibody::CreateWithSpringsToWithoutSpringsMapVel;

namespace examples {

DEFINE_string(
    channel_x, "CASSIE_STATE_SIMULATION",
    "The name of the channel where state estimation is published. Set to "
    "CASSIE_STATE_DISPATCHER for use on hardware with the state estimator");
DEFINE_string(planner_parameters,
              "examples/Cassie/kinematic_centroidal_planner/"
              "kinematic_centroidal_mpc_gains.yaml",
              "planner parameters containing initial states and other "
              "regularization parameters");
DEFINE_string(channel_reference, "KCMPC_OUTPUT",
              "The name of the channel where the reference trajectories from "
              "MPC are published");
DEFINE_string(
    channel_cassie_out, "CASSIE_OUTPUT_ECHO",
    "The name of the channel where cassie_out_t messages are published");
DEFINE_string(
    motion,
    "examples/Cassie/kinematic_centroidal_planner/motions/motion_test.yaml",
    "YAML file that contains trajectory parameters such as speed, gait "
    "sequence "
    "target_com_height");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  /**** Initialize all the leaf systems ****/
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  // Build the controller diagram
  DiagramBuilder<double> builder;

  // Built the Cassie MBPs
  drake::multibody::MultibodyPlant<double> plant_w_spr(0.0);
  AddCassieMultibody(&plant_w_spr, nullptr, true,
                     "examples/Cassie/urdf/cassie_v2_conservative.urdf",
                     false /*spring model*/, false /*loop closure*/);
  plant_w_spr.Finalize();
  auto context_w_spr = plant_w_spr.CreateDefaultContext();
  drake::multibody::MultibodyPlant<double> plant_wo_spr(0.0);
  AddCassieMultibody(
      &plant_wo_spr, nullptr, true,
      "examples/Cassie/urdf/cassie_fixed_springs_conservative.urdf", false,
      false);
  plant_wo_spr.Finalize();
  auto context_wo_spr = plant_wo_spr.CreateDefaultContext();

  auto map_position_from_spring_to_no_spring =
      CreateWithSpringsToWithoutSpringsMapPos(plant_w_spr, plant_wo_spr);
  auto map_velocity_from_spring_to_no_spring =
      CreateWithSpringsToWithoutSpringsMapVel(plant_w_spr, plant_wo_spr);

  auto mpc_gains = drake::yaml::LoadYamlFile<KinematicCentroidalGains>(
      FLAGS_planner_parameters);
  auto motion = drake::yaml::LoadYamlFile<TrajectoryParameters>(FLAGS_motion);

  /****** Leaf Systems ******/
  auto kcmpc = builder.AddSystem<KinematicCentroidalMPC>(
      plant_w_spr, plant_wo_spr, context_wo_spr.get(), motion, mpc_gains);
  kcmpc->SetSpringMaps(map_position_from_spring_to_no_spring,
                       map_velocity_from_spring_to_no_spring);
  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_w_spr);
  auto reference_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          FLAGS_channel_reference, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto cassie_out_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_cassie_out>(
          FLAGS_channel_cassie_out, &lcm));
  auto high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
      plant_w_spr, context_w_spr.get(), 1.0, 1.0, 1.0);
  auto cassie_out_to_radio = builder.AddSystem<systems::CassieOutToRadio>();

  builder.Connect(state_receiver->get_output_port(),
                  kcmpc->get_state_input_port());
  builder.Connect(kcmpc->get_output_port(), reference_pub->get_input_port());
  builder.Connect(high_level_command->get_xy_output_port(),
                  kcmpc->get_input_port_target_vel());
  builder.Connect(cassie_out_receiver->get_output_port(),
                  cassie_out_to_radio->get_input_port());
  builder.Connect(cassie_out_to_radio->get_output_port(),
                  high_level_command->get_input_port_radio());

  // Run lcm-driven simulation
  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("centroidal_mpc_reference"));

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), state_receiver, FLAGS_channel_x, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());

  loop.Simulate();

  return 0;
}
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::DoMain(argc, argv);
}
