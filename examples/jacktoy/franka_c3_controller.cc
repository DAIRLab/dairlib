
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
#include "examples/jacktoy/parameters/franka_c3_controller_params.h"
#include "examples/jacktoy/parameters/sampling_based_c3_controller_params.h"
#include "examples/jacktoy/parameters/franka_lcm_channels.h"
#include "examples/jacktoy/systems/c3_state_sender.h"
#include "examples/jacktoy/systems/c3_trajectory_generator.h"
#include "examples/jacktoy/systems/franka_kinematics.h"
#include "examples/jacktoy/systems/control_target_generator.h"
#include "multibody/multibody_utils.h"
#include "solvers/lcs_factory.h"
#include "solvers/lcs_factory_preprocessor.h"
#include "systems/controllers/c3/lcs_factory_system.h"
#include "systems/controllers/c3_controller.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/c3_output_systems.h"

namespace dairlib {

using dairlib::solvers::LCSFactory;
using dairlib::solvers::LCSFactoryPreProcessor;
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
              "examples/jacktoy/parameters/franka_c3_controller_params.yaml",
              "Controller settings such as channels. Attempting to minimize "
              "number of gflags");
DEFINE_string(sampling_controller_settings,
              "examples/jacktoy/parameters/sampling_based_c3_controller_params.yaml",
              "Sampling controller settings such as number of samples and trajectory type. Attempting to minimize"
              "number of gflags");
DEFINE_string(lcm_channels,
              "examples/jacktoy/parameters/lcm_channels_simulation.yaml",
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
  SamplingC3ControllerParams sampling_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          FLAGS_sampling_controller_settings);
  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>(FLAGS_lcm_channels);
  C3Options c3_options = drake::yaml::LoadYamlFile<C3Options>(
      controller_params.c3_options_file[controller_params.scene_index]);
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(controller_params.osqp_settings_file))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  DiagramBuilder<double> plant_builder;
  
  // Loading the full franka model that will go into franka kinematics system
  // This needs to load the full franka and full end effector model. Connections made around line 184 to FrankaKinematics module.   
  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka, nullptr);
  parser_franka.AddModels(
      drake::FindResourceOrThrow(controller_params.franka_model));
  parser_franka.AddModels(drake::FindResourceOrThrow(controller_params.ground_model));
  drake::multibody::ModelInstanceIndex end_effector_index =
      parser_franka.AddModels(
          FindResourceOrThrow(controller_params.end_effector_model))[0];

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant_franka.WeldFrames(plant_franka.world_frame(),
                          plant_franka.GetFrameByName("panda_link0"), X_WI);

  RigidTransform<double> T_EE_W =
      RigidTransform<double>(drake::math::RotationMatrix<double>(),
                             controller_params.tool_attachment_frame);
  RigidTransform<double> X_F_G_franka =
      RigidTransform<double>(drake::math::RotationMatrix<double>(),
                             controller_params.ground_franka_frame);
  plant_franka.WeldFrames(plant_franka.GetFrameByName("panda_link7"), 
                          plant_franka.GetFrameByName("end_effector_base"), T_EE_W);
  plant_franka.WeldFrames(plant_franka.GetFrameByName("panda_link0"), 
                          plant_franka.GetFrameByName("ground"), X_F_G_franka);

  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();

  /// adding the jack model (TODO: Change to object instead of jack)
  MultibodyPlant<double> plant_jack(0.0);
  Parser parser_jack(&plant_jack, nullptr);
  parser_jack.AddModels(controller_params.jack_model);
  plant_jack.Finalize();
  auto jack_context = plant_jack.CreateDefaultContext();

  /// Creating the plant for lcs which will contain only end effector and jack
  auto [plant_for_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_builder, 0.0);
  
  Parser parser_for_lcs(&plant_for_lcs);
  parser_for_lcs.SetAutoRenaming(true);
  /// Loading simple model of end effector (just a sphere) for the lcs plant
  parser_for_lcs.AddModels(controller_params.end_effector_simple_model);

//   drake::multibody::ModelInstanceIndex left_support_index;
//   drake::multibody::ModelInstanceIndex right_support_index;
//   if (controller_params.scene_index > 0) {
//     left_support_index = parser_plate.AddModels(
//         FindResourceOrThrow(controller_params.left_support_model))[0];
//     right_support_index = parser_plate.AddModels(
//         FindResourceOrThrow(controller_params.right_support_model))[0];
//     RigidTransform<double> T_S1_W =
//         RigidTransform<double>(drake::math::RollPitchYaw<double>(controller_params.left_support_orientation),
//                                controller_params.left_support_position);
//     RigidTransform<double> T_S2_W =
//         RigidTransform<double>(drake::math::RollPitchYaw<double>(controller_params.right_support_orientation),
//                                controller_params.right_support_position);
//     plant_for_lcs.WeldFrames(
//         plant_for_lcs.world_frame(),
//         plant_for_lcs.GetFrameByName("support", left_support_index), T_S1_W);
//     plant_for_lcs.WeldFrames(
//         plant_for_lcs.world_frame(),
//         plant_for_lcs.GetFrameByName("support", right_support_index), T_S2_W);
//   }
  parser_for_lcs.AddModels(controller_params.jack_model);
  parser_for_lcs.AddModels(controller_params.ground_model);

// TO DO: The base link may change to the simple end effector model link name or might just be removed entirely.
  /// TODO: @Bibit/Will Please check if the weld frames are correct
  plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
                           plant_for_lcs.GetFrameByName("base_link"), X_WI);
  plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(), 
                           plant_for_lcs.GetFrameByName("ground"), X_F_G_franka);
  plant_for_lcs.Finalize();
  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_for_lcs_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(plant_for_lcs);

  auto plant_diagram = plant_builder.Build();
  std::unique_ptr<drake::systems::Context<double>> diagram_context =
      plant_diagram->CreateDefaultContext();
  auto& plant_for_lcs_context = plant_diagram->GetMutableSubsystemContext(
      plant_for_lcs, diagram_context.get());
  auto plate_context_ad = plant_for_lcs_autodiff->CreateDefaultContext();

  /// TO DO: Maybe just this changes to the simple end effector link
  drake::geometry::GeometryId ee_contact_points =
      plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("end_effector_simple"))[0];
//   if (controller_params.scene_index > 0) {
//     std::vector<drake::geometry::GeometryId> left_support_contact_points =
//         plant_for_lcs.GetCollisionGeometriesForBody(
//             plant_for_lcs.GetBodyByName("support", left_support_index));
//     std::vector<drake::geometry::GeometryId> right_support_contact_points =
//         plant_for_lcs.GetCollisionGeometriesForBody(
//             plant_for_lcs.GetBodyByName("support", right_support_index));
//     plate_contact_points.insert(plate_contact_points.end(),
//                                 left_support_contact_points.begin(),
//                                 left_support_contact_points.end());
//     plate_contact_points.insert(plate_contact_points.end(),
//                                 right_support_contact_points.begin(),
//                                 right_support_contact_points.end());
//   }
//  TODO: This becomes jack geoms but the individual capsule code needs to be ported over
  drake::geometry::GeometryId capsule1_geoms =
      plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("capsule_1"))[0];
  drake::geometry::GeometryId capsule2_geoms =
      plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("capsule_2"))[0];
  drake::geometry::GeometryId capsule3_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("capsule_3"))[0];
  drake::geometry::GeometryId ground_geoms =
      plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("ground"))[0];

  //   Creating a map of contact geoms
  std::unordered_map<std::string, drake::geometry::GeometryId> contact_geoms;
  contact_geoms["EE"] = ee_contact_points;
  contact_geoms["CAPSULE_1"] = capsule1_geoms;
  contact_geoms["CAPSULE_2"] = capsule2_geoms;
  contact_geoms["CAPSULE_3"] = capsule3_geoms;
  contact_geoms["GROUND"] = ground_geoms;

  std::vector<SortedPair<GeometryId>> ee_contact_pairs;

  //   Creating a list of contact pairs for the end effector and the jack to hand over to lcs factory in the controller to resolve
  ee_contact_pairs.push_back(SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_1"]));
  ee_contact_pairs.push_back(SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_2"]));
  ee_contact_pairs.push_back(SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_3"]));
  //   Creating a list of contact pairs for the jack and the ground
  std::vector<SortedPair<GeometryId>> ground_contact1 {SortedPair(contact_geoms["CAPSULE_1"], contact_geoms["GROUND"])};
  std::vector<SortedPair<GeometryId>> ground_contact2 {SortedPair(contact_geoms["CAPSULE_2"], contact_geoms["GROUND"])};
  std::vector<SortedPair<GeometryId>> ground_contact3 {SortedPair(contact_geoms["CAPSULE_3"], contact_geoms["GROUND"])};

  std::vector<std::vector<SortedPair<GeometryId>>> contact_pairs;  // will have [[(ee,cap1), (ee,cap2), (ee_cap3)], [(ground,cap1)], [(ground,cap2)], [(ground,cap3)]]
  contact_pairs.push_back(ee_contact_pairs);
  contact_pairs.push_back(ground_contact1);
  contact_pairs.push_back(ground_contact2);
  contact_pairs.push_back(ground_contact3);

// Removed this next block because we are not passing contact pairs to the controller. Only contact geoms.
// TODO: Change the interface of Will’s C3_controller to take contact_geoms list instead of contact pairs.
//   std::vector<SortedPair<GeometryId>> contact_pairs;
//   for (auto geom_id : contact_geoms["PLATE"]) {
//     contact_pairs.emplace_back(geom_id, contact_geoms["TRAY"][0]);
//   }

  DiagramBuilder<double> builder;

//   TODO @Sharanya Feb 9th: Decide how to generate object target and make changes to plate_balancing_target.h/.cc accordingly with port names and what it does. 

  auto object_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.object_state_channel, &lcm));
  auto franka_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_franka);
  auto object_state_receiver =
      builder.AddSystem<systems::ObjectStateReceiver>(plant_jack);
  //  TODO: The end_effector_name was changed in the yaml file. Check if this next line still works fine.
  auto reduced_order_model_receiver =
      builder.AddSystem<systems::FrankaKinematics>(
          plant_franka, franka_context.get(), plant_jack, jack_context.get(),
          controller_params.end_effector_name, "jack",                         
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
  auto c3_target_state_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_target_state_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_actual_state_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_actual_state_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_forces_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_forces>(
          lcm_channel_params.c3_force_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));

  auto control_target =
      builder.AddSystem<systems::TargetGenerator>(plant_jack);                                   // This system generates the target for the end effector and the object.
  control_target->SetRemoteControlParameters(
      sampling_params.trajectory_type, sampling_params.traj_radius, sampling_params.x_c, sampling_params.y_c, sampling_params.lead_angle, 
      sampling_params.fixed_goal_x, sampling_params.fixed_goal_y, sampling_params.step_size, 
      sampling_params.start_point_x, sampling_params.start_point_y, sampling_params.end_point_x, sampling_params.end_point_y, 
      sampling_params.lookahead_step_size, sampling_params.max_step_size, sampling_params.ee_goal_height, sampling_params.object_half_width);
  std::vector<int> input_sizes = {3, 7, 3, 6};
  auto target_state_mux =
      builder.AddSystem<drake::systems::Multiplexer>(input_sizes);
  auto end_effector_zero_velocity_source =
      builder.AddSystem<drake::systems::ConstantVectorSource>(
          VectorXd::Zero(3));
  auto object_zero_velocity_source =
      builder.AddSystem<drake::systems::ConstantVectorSource>(
          VectorXd::Zero(6));
  builder.Connect(control_target->get_output_port_end_effector_target(),
                  target_state_mux->get_input_port(0));
  builder.Connect(control_target->get_output_port_object_target(),
                  target_state_mux->get_input_port(1));
  builder.Connect(end_effector_zero_velocity_source->get_output_port(),
                  target_state_mux->get_input_port(2));
  builder.Connect(object_zero_velocity_source->get_output_port(),
                  target_state_mux->get_input_port(3));
  // Preprocessing the contact pairs to resolve the contact pairs   
  std::vector<drake::SortedPair<drake::geometry::GeometryId>> resolved_contact_pairs = 
                                                            LCSFactoryPreProcessor::PreProcessor(plant_for_lcs, plant_for_lcs_context, 
                                                            contact_pairs, c3_options.num_friction_directions);
  
  auto lcs_factory = builder.AddSystem<systems::LCSFactorySystem>(
      plant_for_lcs, &plant_for_lcs_context, *plant_for_lcs_autodiff,
      plate_context_ad.get(), resolved_contact_pairs, c3_options);
  auto controller = builder.AddSystem<systems::C3Controller>(
      plant_for_lcs, &plant_for_lcs_context, c3_options);
  auto c3_trajectory_generator =
      builder.AddSystem<systems::C3TrajectoryGenerator>(plant_for_lcs,
                                                        c3_options);
  std::vector<std::string> state_names = {
      "end_effector_x",  "end_effector_y", "end_effector_z",  "object_qw",
      "object_qx",         "object_qy",        "object_qz",         "object_x",
      "object_y",          "object_z",         "end_effector_vx", "end_effector_vy",
      "end_effector_vz", "object_wx",        "object_wy",         "object_wz",
      "object_vz",         "object_vz",        "object_vz",
  };
  auto c3_state_sender =
      builder.AddSystem<systems::C3StateSender>(3 + 7 + 3 + 6, state_names);
  c3_trajectory_generator->SetPublishEndEffectorOrientation(
      controller_params.include_end_effector_orientation);
  auto c3_output_sender = builder.AddSystem<systems::C3OutputSender>();
  controller->SetOsqpSolverOptions(solver_options);
  builder.Connect(franka_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_franka_state());
  builder.Connect(target_state_mux->get_output_port(),
                  controller->get_input_port_target());
  builder.Connect(lcs_factory->get_output_port_lcs(),
                  controller->get_input_port_lcs());
  builder.Connect(object_state_sub->get_output_port(),
                  object_state_receiver->get_input_port());
  builder.Connect(object_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_object_state());
  builder.Connect(object_state_receiver->get_output_port(),
                  control_target->get_input_port_object_state());
  builder.Connect(reduced_order_model_receiver->get_output_port(),
                  controller->get_input_port_lcs_state());
  builder.Connect(radio_sub->get_output_port(),
                  controller->get_input_port_radio());
  builder.Connect(reduced_order_model_receiver->get_output_port(),
                  lcs_factory->get_input_port_lcs_state());
  builder.Connect(radio_sub->get_output_port(),
                  control_target->get_input_port_radio());
  builder.Connect(controller->get_output_port_c3_solution(),
                  c3_trajectory_generator->get_input_port_c3_solution());
  builder.Connect(lcs_factory->get_output_port_lcs_contact_jacobian(),
                  c3_output_sender->get_input_port_lcs_contact_info());
  builder.Connect(c3_trajectory_generator->get_output_port_actor_trajectory(),
                  actor_trajectory_sender->get_input_port());
  builder.Connect(c3_trajectory_generator->get_output_port_object_trajectory(),
                  object_trajectory_sender->get_input_port());
  builder.Connect(target_state_mux->get_output_port(),
                  c3_state_sender->get_input_port_target_state());
  builder.Connect(reduced_order_model_receiver->get_output_port_lcs_state(),
                  c3_state_sender->get_input_port_actual_state());
  builder.Connect(c3_state_sender->get_output_port_target_c3_state(),
                  c3_target_state_publisher->get_input_port());
  builder.Connect(c3_state_sender->get_output_port_actual_c3_state(),
                  c3_actual_state_publisher->get_input_port());
  builder.Connect(controller->get_output_port_c3_solution(),
                  c3_output_sender->get_input_port_c3_solution());
  builder.Connect(controller->get_output_port_c3_intermediates(),
                  c3_output_sender->get_input_port_c3_intermediates());
  builder.Connect(c3_output_sender->get_output_port_c3_debug(),
                  c3_output_publisher->get_input_port());
  builder.Connect(c3_output_sender->get_output_port_c3_force(),
                  c3_forces_publisher->get_input_port());
  //  builder.Connect(c3_output_sender->get_output_port_next_c3_input(),
  //                  lcs_factory->get_input_port_lcs_input());

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("franka_c3_controller"));
  plant_diagram->set_name(("franka_c3_plant"));
//  DrawAndSaveDiagramGraph(*plant_diagram);

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), franka_state_receiver,
      lcm_channel_params.franka_state_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());
  //  auto& controller_context = loop.get_diagram()->GetMutableSubsystemContext(
  //      *controller, &loop.get_diagram_mutable_context());
  //  controller->get_input_port_target().FixValue(&controller_context, x_des);
  LcmHandleSubscriptionsUntil(
      &lcm, [&]() { return object_state_sub->GetInternalMessageCount() > 1; });
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }