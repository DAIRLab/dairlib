
#include <dairlib/lcmt_radio_out.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "examples/franka/parameters/franka_c3_controller_params.h"
#include "examples/franka/parameters/franka_lcm_channels.h"
#include "examples/franka/systems/c3_state_sender.h"
#include "examples/franka/systems/c3_trajectory_generator.h"
#include "examples/franka/systems/franka_kinematics.h"
#include "examples/franka/systems/plate_balancing_target.h"
#include "multibody/multibody_utils.h"
#include "solvers/lcs_factory.h"
#include "systems/controllers/c3/lcs_factory_system.h"
#include "systems/controllers/c3_controller.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/c3_output_systems.h"
#include "systems/primitives/radio_parser.h"

namespace dairlib {

using dairlib::solvers::LCSFactory;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::MatrixXd;

using Eigen::Vector3d;
using Eigen::VectorXd;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;

DEFINE_string(controller_settings,
              "examples/franka/parameters/franka_c3_controller_params.yaml",
              "Controller settings such as channels. Attempting to minimize "
              "number of gflags");
DEFINE_string(lcm_channels,
              "examples/franka/parameters/lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  // load parameters
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  FrankaC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<FrankaC3ControllerParams>(
          FLAGS_controller_settings);
  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>(FLAGS_lcm_channels);
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(controller_params.osqp_settings_file))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  DiagramBuilder<double> plant_builder;

  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka, nullptr);
  parser_franka.AddModels(
      drake::FindResourceOrThrow(controller_params.franka_model));
  drake::multibody::ModelInstanceIndex end_effector_index =
      parser_franka.AddModels(
          FindResourceOrThrow(controller_params.end_effector_model))[0];

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant_franka.WeldFrames(plant_franka.world_frame(),
                          plant_franka.GetFrameByName("panda_link0"), X_WI);

  RigidTransform<double> T_EE_W =
      RigidTransform<double>(drake::math::RotationMatrix<double>(),
                             controller_params.tool_attachment_frame);
  plant_franka.WeldFrames(
      plant_franka.GetFrameByName("panda_link7"),
      plant_franka.GetFrameByName("plate", end_effector_index), T_EE_W);

  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();

  ///
  MultibodyPlant<double> plant_tray(0.0);
  Parser parser_tray(&plant_tray, nullptr);
  parser_tray.AddModels(controller_params.tray_model);
  plant_tray.Finalize();
  auto tray_context = plant_tray.CreateDefaultContext();

  DiagramBuilder<double> builder;

  auto tray_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.tray_state_channel, &lcm));
  auto franka_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_franka);
  auto tray_state_receiver =
      builder.AddSystem<systems::ObjectStateReceiver>(plant_tray);
  auto reduced_order_model_receiver =
      builder.AddSystem<systems::FrankaKinematics>(
          plant_franka, franka_context.get(), plant_tray, tray_context.get(),
          controller_params.end_effector_name, "tray",
          controller_params.include_end_effector_orientation);
  auto c3_actual_state_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_actual_state_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  std::vector<std::string> state_names = {
      "end_effector_x",  "end_effector_y", "end_effector_z",  "tray_qw",
      "tray_qx",         "tray_qy",        "tray_qz",         "tray_x",
      "tray_y",          "tray_z",         "end_effector_vx", "end_effector_vy",
      "end_effector_vz", "tray_wx",        "tray_wy",         "tray_wz",
      "tray_vz",         "tray_vz",        "tray_vz",
  };
  auto c3_state_sender =
      builder.AddSystem<systems::C3StateSender>(3 + 7 + 3 + 6, state_names);

  builder.Connect(tray_state_sub->get_output_port(),
                  tray_state_receiver->get_input_port());
  builder.Connect(franka_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_franka_state());
  builder.Connect(tray_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_object_state());
  builder.Connect(reduced_order_model_receiver->get_output_port_lcs_state(),
                  c3_state_sender->get_input_port_actual_state());
  builder.Connect(c3_state_sender->get_output_port_actual_c3_state(),
                  c3_actual_state_publisher->get_input_port());
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("franka_c3_controller"));

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), franka_state_receiver,
      lcm_channel_params.franka_state_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());

  LcmHandleSubscriptionsUntil(
      &lcm, [&]() { return tray_state_sub->GetInternalMessageCount() > 1; });
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }