
#include <dairlib/lcmt_radio_out.hpp>
#include <gflags/gflags.h>

#include "examples/franka/franka_c3_controller_params.h"
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

DEFINE_string(controller_settings, "",
              "Controller settings such as channels. Attempting to minimize "
              "number of gflags");
DEFINE_string(osqp_settings,
              "examples/Cassie/osc_run/osc_running_qp_settings.yaml",
              "Filepath containing qp settings");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  // load parameters
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  FrankaC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<FrankaC3ControllerParams>(
          "examples/franka/franka_c3_controller_params.yaml");
  C3Options c3_options =
      drake::yaml::LoadYamlFile<C3Options>(controller_params.c3_options_file);
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(FLAGS_osqp_settings))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  DiagramBuilder<double> plant_builder;

  ///

  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka, nullptr);
  parser_franka.package_map().Add("franka_urdfs", "examples/franka/urdf");
  parser_franka.AddModelFromFile(controller_params.franka_model);
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant_franka.WeldFrames(plant_franka.world_frame(),
                          plant_franka.GetFrameByName("panda_link0"), X_WI);
  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();

  ///
  MultibodyPlant<double> plant_tray(0.0);
  Parser parser_tray(&plant_tray, nullptr);
  parser_tray.AddModelFromFile(controller_params.tray_model);
  plant_tray.Finalize();
  auto tray_context = plant_tray.CreateDefaultContext();

  ///
  auto [plant_plate, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_builder, 0.0);
  Parser parser_plate(&plant_plate);
  parser_plate.AddModelFromFile(controller_params.plate_model);
  parser_plate.AddModelFromFile(controller_params.tray_model);

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

  ///

  VectorXd x_des = VectorXd::Zero(plant_plate.num_positions() + plant_plate.num_velocities());
  x_des << c3_options.q_des, c3_options.v_des;
//  x_des
  /// default position
//  x_des[2] = 0.45 - 0.02 + radio_out->channel[2] * 0.2;
//  x_des(10) += radio_out->channel[0] * 0.2;
//  x_des(11) += radio_out->channel[1] * 0.2;
//  x_des(12) += radio_out->channel[2] * 0.2;

  DiagramBuilder<double> builder;

  auto tray_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          "TRAY_OUTPUT", &lcm));
  auto franka_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_franka);
  auto tray_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_tray);
  auto reduced_order_model_receiver =
      builder.AddSystem<systems::FrankaKinematics>(
          plant_franka, franka_context.get(), plant_tray, tray_context.get(),
          "paddle", "tray", controller_params.include_end_effector_orientation);

  auto actor_trajectory_sender = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          controller_params.c3_channel_actor, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto object_trajectory_sender = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          controller_params.c3_channel_object, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          controller_params.radio_channel, &lcm));

  auto controller = builder.AddSystem<systems::C3Controller>(
      plant_plate, &plate_context, *plant_plate_ad, plate_context_ad.get(),
      contact_pairs, c3_options);
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
  builder.Connect(controller->get_output_port_actor_trajectory(),
                  actor_trajectory_sender->get_input_port());
  builder.Connect(controller->get_output_port_object_trajectory(),
                  object_trajectory_sender->get_input_port());



  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("franka_c3_controller"));
  plant_diagram->set_name(("franka_c3_plant"));
  DrawAndSaveDiagramGraph(*plant_diagram);

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), franka_state_receiver,
      controller_params.state_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());
  auto& controller_context = loop.get_diagram()->GetMutableSubsystemContext(
      *controller,
      &loop.get_diagram_mutable_context());
//      loop.get_diagram_mutable_context()
  controller->get_input_port_target().FixValue(&controller_context,
                                               x_des);
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }