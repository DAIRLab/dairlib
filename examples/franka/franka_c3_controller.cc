
#include <dairlib/lcmt_radio_out.hpp>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "examples/franka/parameters/franka_c3_controller_params.h"
#include "examples/franka/parameters/franka_lcm_channels.h"
#include "examples/franka/systems/c3_trajectory_generator.h"
#include "examples/franka/systems/end_effector_trajectory.h"
#include "examples/franka/systems/franka_kinematics.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "solvers/lcs_factory.h"
#include "systems/controllers/c3_controller.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/c3_output_systems.h"

#include "drake/common/find_resource.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

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
  C3Options c3_options =
      drake::yaml::LoadYamlFile<C3Options>(controller_params.c3_options_file);
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(controller_params.osqp_settings_file))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  DiagramBuilder<double> plant_builder;

  ///

  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka, nullptr);
  parser_franka.package_map().Add("franka_urdfs", "examples/franka/urdf");
  parser_franka.AddModels(
      drake::FindResourceOrThrow(controller_params.franka_model));
  drake::multibody::ModelInstanceIndex end_effector_index =
      parser_franka.AddModels(
          FindResourceOrThrow(controller_params.end_effector_model))[0];

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant_franka.WeldFrames(plant_franka.world_frame(),
                          plant_franka.GetFrameByName("panda_link0"), X_WI);
  Vector3d tool_attachment_frame =
      StdVectorToVectorXd(controller_params.tool_attachment_frame);

  RigidTransform<double> T_EE_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), tool_attachment_frame);
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

  ///
  auto [plant_plate, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_builder, 0.0);
  Parser parser_plate(&plant_plate);
  parser_plate.AddModels(controller_params.plate_model);
  parser_plate.AddModels(controller_params.tray_model);

  plant_plate.WeldFrames(plant_plate.world_frame(),
                         plant_plate.GetFrameByName("base_link"), X_WI);
  plant_plate.Finalize();
  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_plate_ad =
      drake::systems::System<double>::ToAutoDiffXd(plant_plate);

  auto plant_diagram = plant_builder.Build();
  std::unique_ptr<drake::systems::Context<double>> diagram_context =
      plant_diagram->CreateDefaultContext();
  auto& plate_context = plant_diagram->GetMutableSubsystemContext(
      plant_plate, diagram_context.get());
  auto plate_context_ad = plant_plate_ad->CreateDefaultContext();

  ///
  std::vector<drake::geometry::GeometryId> plate_contact_points =
      plant_plate.GetCollisionGeometriesForBody(
          plant_plate.GetBodyByName("plate"));
  std::vector<drake::geometry::GeometryId> tray_geoms =
      plant_plate.GetCollisionGeometriesForBody(
          plant_plate.GetBodyByName("tray"));
  std::unordered_map<std::string, std::vector<drake::geometry::GeometryId>>
      contact_geoms;
  contact_geoms["PLATE"] = plate_contact_points;
  contact_geoms["TRAY"] = tray_geoms;

  std::vector<SortedPair<GeometryId>> contact_pairs;
  for (auto geom_id : contact_geoms["PLATE"]) {
    contact_pairs.emplace_back(geom_id, contact_geoms["TRAY"][0]);
  }

  VectorXd x_des = VectorXd::Zero(plant_plate.num_positions() +
                                  plant_plate.num_velocities());
  x_des << c3_options.q_des, c3_options.v_des;

  DiagramBuilder<double> builder;

  auto tray_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.tray_state_channel, &lcm));
  auto franka_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_franka);
  auto tray_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_tray);
  auto reduced_order_model_receiver =
      builder.AddSystem<systems::FrankaKinematics>(
          plant_franka, franka_context.get(), plant_tray, tray_context.get(),
          controller_params.end_effector_name, "tray",
          controller_params.include_end_effector_orientation);

  auto actor_trajectory_sender = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_actor_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  auto object_trajectory_sender = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_object_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  auto c3_output_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_output>(
          lcm_channel_params.c3_debug_output_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));

  auto controller = builder.AddSystem<systems::C3Controller>(
      plant_plate, &plate_context, *plant_plate_ad, plate_context_ad.get(),
      contact_pairs, c3_options);
  auto c3_trajectory_generator =
      builder.AddSystem<systems::C3TrajectoryGenerator>(
          plant_plate, &plate_context, c3_options);
  auto c3_output_sender = builder.AddSystem<systems::C3OutputSender>();
  controller->SetOsqpSolverOptions(solver_options);
  builder.Connect(franka_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_franka_state());
  builder.Connect(tray_state_sub->get_output_port(),
                  tray_state_receiver->get_input_port());
  builder.Connect(tray_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_object_state());
  builder.Connect(reduced_order_model_receiver->get_output_port(),
                  controller->get_input_port_state());
  builder.Connect(radio_sub->get_output_port(),
                  controller->get_input_port_radio());
  builder.Connect(controller->get_output_port_c3_solution(),
                  c3_trajectory_generator->get_input_port_c3_solution());
  builder.Connect(c3_trajectory_generator->get_output_port_actor_trajectory(),
                  actor_trajectory_sender->get_input_port());
  builder.Connect(c3_trajectory_generator->get_output_port_object_trajectory(),
                  object_trajectory_sender->get_input_port());

  builder.Connect(controller->get_output_port_c3_solution(),
                  c3_output_sender->get_input_port_c3_solution());
  builder.Connect(controller->get_output_port_c3_intermediates(),
                  c3_output_sender->get_input_port_c3_intermediates());
  builder.Connect(c3_output_sender->get_output_port_c3_debug(),
                  c3_output_publisher->get_input_port());

  controller->SetPublishEndEffectorOrientation(
      controller_params.include_end_effector_orientation);

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("franka_c3_controller"));
  plant_diagram->set_name(("franka_c3_plant"));
  DrawAndSaveDiagramGraph(*plant_diagram);

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), franka_state_receiver,
      lcm_channel_params.franka_state_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());
  auto& controller_context = loop.get_diagram()->GetMutableSubsystemContext(
      *controller, &loop.get_diagram_mutable_context());
  controller->get_input_port_target().FixValue(&controller_context, x_des);
  LcmHandleSubscriptionsUntil(
      &lcm, [&]() { return tray_state_sub->GetInternalMessageCount() > 1; });
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }