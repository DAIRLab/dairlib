#include <math.h>

#include <vector>

#include <dairlib/lcmt_radio_out.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/visualization/visualization_config_functions.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "examples/jacktoy/parameters/franka_c3_controller_params.h"
#include "examples/jacktoy/parameters/sampling_based_c3_controller_params.h"
#include "examples/jacktoy/parameters/franka_lcm_channels.h"
#include "examples/jacktoy/parameters/franka_osc_controller_params.h"
#include "examples/jacktoy/parameters/franka_sim_params.h"
#include "examples/jacktoy/systems/c3_state_sender.h"
#include "examples/jacktoy/systems/c3_trajectory_generator.h"
#include "examples/jacktoy/systems/end_effector_force_trajectory.h"
#include "examples/jacktoy/systems/end_effector_orientation.h"
#include "examples/jacktoy/systems/end_effector_trajectory.h"
#include "examples/jacktoy/systems/franka_kinematics.h"
#include "examples/jacktoy/systems/control_target_generator.h"
#include "multibody/multibody_utils.h"
#include "solvers/lcs_factory.h"
#include "systems/controllers/c3/lcs_factory_system.h"
#include "systems/controllers/c3_controller.h"
#include "systems/controllers/osc/external_force_tracking_data.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/c3_output_systems.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"


namespace dairlib {

using dairlib::systems::SubvectorPassThrough;
using drake::geometry::GeometrySet;
using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::trajectories::PiecewisePolynomial;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;
using systems::AddActuationRecieverAndStateSenderLcm;
using systems::RobotInputReceiver;
using systems::RobotOutputSender;

using drake::math::RigidTransform;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using systems::controllers::ExternalForceTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

using dairlib::solvers::LCSFactory;
using drake::SortedPair;
using drake::geometry::GeometryId;

DEFINE_string(controller_settings,
              "examples/jacktoy/parameters/franka_c3_controller_params.yaml",
              "Controller settings such as channels. Attempting to minimize "
              "number of gflags");
DEFINE_string(sampling_controller_settings,
              "examples/jacktoy/parameters/sampling_based_c3_controller_params.yaml",
              "Sampling controller settings such as number of samples and trajectory type. Attempting to minimize"
              "number of gflags");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>(
          "examples/jacktoy/parameters/lcm_channels_simulation.yaml");
  // load parameters
  FrankaSimParams sim_params = drake::yaml::LoadYamlFile<FrankaSimParams>(
      "examples/jacktoy/parameters/franka_sim_params.yaml");
  FrankaC3ControllerParams c3_controller_params =
      drake::yaml::LoadYamlFile<FrankaC3ControllerParams>(
          "examples/jacktoy/parameters/franka_c3_controller_params.yaml");
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(c3_controller_params.osqp_settings_file))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());
  C3Options c3_options = drake::yaml::LoadYamlFile<C3Options>(
      c3_controller_params.c3_options_file[c3_controller_params.scene_index]);
  OSCGains gains = drake::yaml::LoadYamlFile<OSCGains>(
      FindResourceOrThrow(
          "examples/jacktoy/parameters/franka_osc_controller_params.yaml"),
      {}, {}, yaml_options);
    //   TODO: Is the first controller parameters file still required?
  FrankaControllerParams controller_params =
      drake::yaml::LoadYamlFile<FrankaControllerParams>(
          "examples/jacktoy/parameters/franka_osc_controller_params.yaml");
  SamplingC3ControllerParams sampling_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          FLAGS_sampling_controller_settings);

  DiagramBuilder<double> builder;

  /// OSC
  drake::multibody::MultibodyPlant<double> osc_plant(0.0);
  Parser osc_parser(&osc_plant, nullptr);
  osc_parser.AddModels(
      drake::FindResourceOrThrow(controller_params.franka_model));

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  osc_plant.WeldFrames(osc_plant.world_frame(),
                       osc_plant.GetFrameByName("panda_link0"), X_WI);

  if (!controller_params.end_effector_name.empty()) {
    drake::multibody::ModelInstanceIndex end_effector_index =
        osc_parser.AddModels(
            FindResourceOrThrow(controller_params.end_effector_model))[0];
    RigidTransform<double> T_EE_W =
        RigidTransform<double>(drake::math::RotationMatrix<double>(),
                               controller_params.tool_attachment_frame);
    osc_plant.WeldFrames(
        osc_plant.GetFrameByName("panda_link7"),
        osc_plant.GetFrameByName(controller_params.end_effector_name,
                                 end_effector_index),
        T_EE_W);
  } else {
    std::cout << "OSC osc_plant has been constructed with no end effector."
              << std::endl;
  }
  osc_plant.Finalize();
  auto osc_plant_context = osc_plant.CreateDefaultContext();

  auto end_effector_position_receiver =
      builder.AddSystem<systems::LcmTrajectoryReceiver>(
          "end_effector_position_target");
  auto end_effector_force_receiver =
      builder.AddSystem<systems::LcmTrajectoryReceiver>(
          "end_effector_force_target");
  auto end_effector_orientation_receiver =
      builder.AddSystem<systems::LcmOrientationTrajectoryReceiver>(
          "end_effector_orientation_target");
  auto osc_command_sender =
      builder.AddSystem<systems::RobotCommandSender>(osc_plant);
  auto end_effector_trajectory =
      builder.AddSystem<EndEffectorTrajectoryGenerator>(
          osc_plant, osc_plant_context.get());
  auto end_effector_orientation_trajectory =
      builder.AddSystem<EndEffectorOrientationGenerator>(
          osc_plant, osc_plant_context.get());
  end_effector_orientation_trajectory->SetTrackOrientation(
      controller_params.track_end_effector_orientation);
  auto end_effector_force_trajectory =
      builder.AddSystem<EndEffectorForceTrajectoryGenerator>(
          osc_plant, osc_plant_context.get());
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      osc_plant, osc_plant, osc_plant_context.get(), osc_plant_context.get(),
      false);
  auto end_effector_position_tracking_data =
      std::make_unique<TransTaskSpaceTrackingData>(
          "end_effector_target", controller_params.K_p_end_effector,
          controller_params.K_d_end_effector, controller_params.W_end_effector,
          osc_plant, osc_plant);
  auto franka_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.franka_input_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  end_effector_position_tracking_data->AddPointToTrack(
      controller_params.end_effector_name);
  const VectorXd& end_effector_acceleration_limits =
      controller_params.end_effector_acceleration * Vector3d::Ones();
  end_effector_position_tracking_data->SetCmdAccelerationBounds(
      -end_effector_acceleration_limits, end_effector_acceleration_limits);
  auto mid_link_position_tracking_data_for_rel =
      std::make_unique<JointSpaceTrackingData>(
          "panda_joint2_target", controller_params.K_p_mid_link,
          controller_params.K_d_mid_link, controller_params.W_mid_link,
          osc_plant, osc_plant);
  mid_link_position_tracking_data_for_rel->AddJointToTrack("panda_joint2",
                                                           "panda_joint2dot");

  auto end_effector_force_tracking_data =
      std::make_unique<ExternalForceTrackingData>(
          "end_effector_force", controller_params.W_ee_lambda, osc_plant,
          osc_plant, controller_params.end_effector_name, Vector3d::Zero());

  auto end_effector_orientation_tracking_data =
      std::make_unique<RotTaskSpaceTrackingData>(
          "end_effector_orientation_target",
          controller_params.K_p_end_effector_rot,
          controller_params.K_d_end_effector_rot,
          controller_params.W_end_effector_rot, osc_plant, osc_plant);
  end_effector_orientation_tracking_data->AddFrameToTrack(
      controller_params.end_effector_name);
  Eigen::VectorXd orientation_target = Eigen::VectorXd::Zero(4);
  orientation_target(0) = 1;
  osc->AddTrackingData(std::move(end_effector_position_tracking_data));
  osc->AddConstTrackingData(std::move(mid_link_position_tracking_data_for_rel),
                            1.6 * VectorXd::Ones(1));
  osc->AddTrackingData(std::move(end_effector_orientation_tracking_data));
  osc->AddForceTrackingData(std::move(end_effector_force_tracking_data));
  osc->SetAccelerationCostWeights(gains.W_acceleration);
  osc->SetInputCostWeights(gains.W_input_regularization);
  osc->SetInputSmoothingCostWeights(gains.W_input_smoothing_regularization);
  osc->SetAccelerationConstraints(
      controller_params.enforce_acceleration_constraints);

  osc->SetContactFriction(controller_params.mu);
  drake::solvers::SolverOptions osc_solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(
              "examples/jacktoy/parameters/franka_osc_qp_settings.yaml"))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());
  osc->SetOsqpSolverOptions(osc_solver_options);

  osc->Build();
  /// end of OSC

  /// C3 plant
  DiagramBuilder<double> plant_builder;

  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka, nullptr);
  parser_franka.AddModels(
      drake::FindResourceOrThrow(controller_params.franka_model));
// TODO: Find out if we need ground model here?
//   parser_franka.AddModels(drake::FindResourceOrThrow(c3_controller_params.ground_model));
  drake::multibody::ModelInstanceIndex end_effector_index =
      parser_franka.AddModels(
          FindResourceOrThrow(controller_params.end_effector_model))[0];

  plant_franka.WeldFrames(plant_franka.world_frame(),
                          plant_franka.GetFrameByName("panda_link0"), X_WI);

  RigidTransform<double> T_EE_W =
      RigidTransform<double>(drake::math::RotationMatrix<double>(),
                             controller_params.tool_attachment_frame);
//   RigidTransform<double> X_F_G_franka =
//       RigidTransform<double>(drake::math::RotationMatrix<double>(),
//                              c3_controller_params.ground_franka_frame);
  plant_franka.WeldFrames(
      plant_franka.GetFrameByName("panda_link7"),
      plant_franka.GetFrameByName("end_effector_base", end_effector_index), T_EE_W);
//   plant_franka.WeldFrames(plant_franka.GetFrameByName("panda_link0"), 
//                           plant_franka.GetFrameByName("ground"), X_F_G_franka);

  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();
  MultibodyPlant<double> plant_jack(0.0);
  Parser parser_jack(&plant_jack, nullptr);
  parser_jack.AddModels(c3_controller_params.jack_model);
  plant_jack.Finalize();
  auto jack_context = plant_jack.CreateDefaultContext();

  auto [plant_for_lcs, lcs_scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_builder, 0.0);
  Parser parser_for_lcs(&plant_for_lcs);
  parser_for_lcs.SetAutoRenaming(true);
  parser_for_lcs.AddModels(c3_controller_params.end_effector_simple_model);

//   drake::multibody::ModelInstanceIndex left_support_index;
//   drake::multibody::ModelInstanceIndex right_support_index;
//   if (c3_controller_params.scene_index > 0) {
//     left_support_index = parser_plate.AddModels(
//         FindResourceOrThrow(c3_controller_params.left_support_model))[0];
//     right_support_index = parser_plate.AddModels(
//         FindResourceOrThrow(c3_controller_params.right_support_model))[0];
//     RigidTransform<double> T_S1_W = RigidTransform<double>(
//         drake::math::RollPitchYaw<double>(
//             c3_controller_params.left_support_orientation),
//         c3_controller_params.left_support_position);
//     RigidTransform<double> T_S2_W = RigidTransform<double>(
//         drake::math::RollPitchYaw<double>(
//             c3_controller_params.right_support_orientation),
//         c3_controller_params.right_support_position);
//     plant_for_lcs.WeldFrames(
//         plant_for_lcs.world_frame(),
//         plant_for_lcs.GetFrameByName("support", left_support_index), T_S1_W);
//     plant_for_lcs.WeldFrames(
//         plant_for_lcs.world_frame(),
//         plant_for_lcs.GetFrameByName("support", right_support_index), T_S2_W);
//   }
  parser_for_lcs.AddModels(c3_controller_params.jack_model);
  parser_for_lcs.AddModels(c3_controller_params.ground_model);

  plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
                           plant_for_lcs.GetFrameByName("base_link"), X_WI);
  plant_for_lcs.Finalize();
  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_for_lcs_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(plant_for_lcs);

  auto plant_diagram = plant_builder.Build();
  std::unique_ptr<drake::systems::Context<double>> diagram_context =
      plant_diagram->CreateDefaultContext();
  auto& plant_for_lcs_context = plant_diagram->GetMutableSubsystemContext(
      plant_for_lcs, diagram_context.get());
  auto plate_context_ad = plant_for_lcs_autodiff->CreateDefaultContext();
  std::vector<drake::geometry::GeometryId> plate_contact_points =
      plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("end_effector_simple"));
//   if (c3_controller_params.scene_index > 0) {
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
//   std::vector<drake::geometry::GeometryId> tray_geoms =
//       plant_for_lcs.GetCollisionGeometriesForBody(
//           plant_for_lcs.GetBodyByName("tray"));
//   std::unordered_map<std::string, std::vector<drake::geometry::GeometryId>>
//       contact_geoms;
//   contact_geoms["PLATE"] = plate_contact_points;
//   contact_geoms["TRAY"] = tray_geoms;

//   std::vector<SortedPair<GeometryId>> contact_pairs;
//   for (auto geom_id : contact_geoms["PLATE"]) {
//     contact_pairs.emplace_back(geom_id, contact_geoms["TRAY"][0]);
//   }

  std::vector<drake::geometry::GeometryId> capsule1_geoms =
      plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("capsule_1"));
  std::vector<drake::geometry::GeometryId> capsule2_geoms =
      plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("capsule_2"));
  std::vector<drake::geometry::GeometryId> capsule3_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("capsule_3"));
  std::vector<drake::geometry::GeometryId> ground_geoms =
      plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("ground"));

  std::unordered_map<std::string, std::vector<drake::geometry::GeometryId>>
      contact_geoms;
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

  auto franka_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_franka);
  auto object_state_receiver =
      builder.AddSystem<systems::ObjectStateReceiver>(plant_jack);
  auto reduced_order_model_receiver =
      builder.AddSystem<systems::FrankaKinematics>(
          plant_franka, franka_context.get(), plant_jack, jack_context.get(),
          controller_params.end_effector_name, "jack",
          c3_controller_params.include_end_effector_orientation);

//   TODO: When coming back to fix this file, make sure to change names to object/jack wherever necessary
  auto control_target =
      builder.AddSystem<systems::TargetGenerator>(
          plant_jack);
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
  auto lcs_factory = builder.AddSystem<systems::LCSFactorySystem>(
      plant_for_lcs, &plant_for_lcs_context, *plant_for_lcs_autodiff,
      plate_context_ad.get(), contact_pairs, c3_options);
  auto c3_controller = builder.AddSystem<systems::C3Controller>(
      plant_for_lcs, &plant_for_lcs_context, c3_options);
  auto c3_trajectory_generator =
      builder.AddSystem<systems::C3TrajectoryGenerator>(plant_for_lcs,
                                                        c3_options);
  std::vector<std::string> state_names = {
      "end_effector_x",  "end_effector_y", "end_effector_z",  "tray_qw",
      "tray_qx",         "tray_qy",        "tray_qz",         "tray_x",
      "tray_y",          "tray_z",         "end_effector_vx", "end_effector_vy",
      "end_effector_vz", "tray_wx",        "tray_wy",         "tray_wz",
      "tray_vz",         "tray_vz",        "tray_vz",
  };
  auto c3_state_sender =
      builder.AddSystem<systems::C3StateSender>(3 + 7 + 3 + 6, state_names);
  c3_trajectory_generator->SetPublishEndEffectorOrientation(
      c3_controller_params.include_end_effector_orientation);
  auto c3_output_sender = builder.AddSystem<systems::C3OutputSender>();
  auto c3_target_state_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_target_state_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_actual_state_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_actual_state_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
//  auto c3_output_publisher =
//      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_output>(
//          lcm_channel_params.c3_debug_output_channel, &lcm,
//          TriggerTypeSet({TriggerType::kForced})));
  /// C3

  /// Sim Start
  double sim_dt = sim_params.dt;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);

  Parser parser(&plant);
  parser.SetAutoRenaming(true);
  drake::multibody::ModelInstanceIndex franka_index =
      parser.AddModels(drake::FindResourceOrThrow(sim_params.franka_model))[0];
  drake::multibody::ModelInstanceIndex c3_end_effector_index =
      parser.AddModels(FindResourceOrThrow(sim_params.end_effector_model))[0];
  drake::multibody::ModelInstanceIndex jack_index =
      parser.AddModels(FindResourceOrThrow(sim_params.jack_model))[0];
  multibody::AddFlatTerrain(&plant, &scene_graph, 1.0, 1.0);

  Vector3d franka_origin = Eigen::VectorXd::Zero(3);

  RigidTransform<double> T_X_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), franka_origin);

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   T_X_W);
  plant.WeldFrames(plant.GetFrameByName("panda_link7"),
                   plant.GetFrameByName("plate", c3_end_effector_index),
                   T_EE_W);

//   if (sim_params.scene_index > 0) {
//     drake::multibody::ModelInstanceIndex left_support_index =
//         parser.AddModels(FindResourceOrThrow(sim_params.left_support_model))[0];
//     drake::multibody::ModelInstanceIndex right_support_index = parser.AddModels(
//         FindResourceOrThrow(sim_params.right_support_model))[0];
//     RigidTransform<double> T_S1_W = RigidTransform<double>(
//         drake::math::RollPitchYaw<double>(sim_params.left_support_orientation),
//         sim_params.left_support_position);
//     RigidTransform<double> T_S2_W = RigidTransform<double>(
//         drake::math::RollPitchYaw<double>(sim_params.right_support_orientation),
//         sim_params.right_support_position);
//     plant.WeldFrames(plant.world_frame(),
//                      plant.GetFrameByName("support", left_support_index),
//                      T_S1_W);
//     plant.WeldFrames(plant.world_frame(),
//                      plant.GetFrameByName("support", right_support_index),
//                      T_S2_W);
//     const drake::geometry::GeometrySet& support_geom_set =
//         plant.CollectRegisteredGeometries({
//             &plant.GetBodyByName("support", left_support_index),
//             &plant.GetBodyByName("support", right_support_index),
//         });
//     // we WANT to model collisions between link5 and the supports
//     const drake::geometry::GeometrySet& paddle_geom_set =
//         plant.CollectRegisteredGeometries(
//             {&plant.GetBodyByName("panda_link2"),
//              &plant.GetBodyByName("panda_link3"),
//              &plant.GetBodyByName("panda_link4"),
//              &plant.GetBodyByName("panda_link6"),
//              &plant.GetBodyByName("panda_link7"), &plant.GetBodyByName("plate"),
//              &plant.GetBodyByName("panda_link8")});

//     plant.ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
//         {"paddle", support_geom_set}, {"tray", paddle_geom_set});
//   }

//   const drake::geometry::GeometrySet& paddle_geom_set =
//       plant.CollectRegisteredGeometries({
//           &plant.GetBodyByName("panda_link2"),
//           &plant.GetBodyByName("panda_link3"),
//           &plant.GetBodyByName("panda_link4"),
//           &plant.GetBodyByName("panda_link5"),
//           &plant.GetBodyByName("panda_link6"),
//           &plant.GetBodyByName("panda_link8"),
//       });
//   auto tray_collision_set = GeometrySet(
//       plant.GetCollisionGeometriesForBody(plant.GetBodyByName("tray")));
//   plant.ExcludeCollisionGeometriesWithCollisionFilterGroupPair(
//       {"paddle", paddle_geom_set}, {"tray", tray_collision_set});

  plant.Finalize();
  /* -------------------------------------------------------------------------------------------*/
  auto input_receiver = builder.AddSystem<RobotInputReceiver>(plant);
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      input_receiver->get_output_port(0).size(), 0,
      plant.get_actuation_input_port().size());
  builder.Connect(*input_receiver, *passthrough);
  auto object_state_sender =
      builder.AddSystem<systems::ObjectStateSender>(plant, jack_index);
  auto franka_state_sender =
      builder.AddSystem<RobotOutputSender>(plant, franka_index, false);
  auto state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.franka_state_channel, &lcm, 1.0 / sim_params.franka_publish_rate));
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));
  std::cout << lcm_channel_params.radio_channel << std::endl;
  auto osc_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(osc_plant);

  //// OSC connections
  builder.Connect(osc_state_receiver->get_output_port(0),
                  end_effector_trajectory->get_input_port_state());
  builder.Connect(radio_sub->get_output_port(0),
                  end_effector_trajectory->get_input_port_radio());
  builder.Connect(osc_state_receiver->get_output_port(0),
                  end_effector_orientation_trajectory->get_input_port_state());
  builder.Connect(radio_sub->get_output_port(0),
                  end_effector_orientation_trajectory->get_input_port_radio());
  builder.Connect(osc_state_receiver->get_output_port(0),
                  end_effector_force_trajectory->get_input_port_state());
  builder.Connect(radio_sub->get_output_port(0),
                  end_effector_force_trajectory->get_input_port_radio());
  builder.Connect(osc->get_output_port_osc_command(),
                  osc_command_sender->get_input_port(0));

  builder.Connect(osc_state_receiver->get_output_port(0),
                  osc->get_input_port_robot_output());
  builder.Connect(end_effector_position_receiver->get_output_port(0),
                  end_effector_trajectory->get_input_port_trajectory());
  builder.Connect(
      end_effector_orientation_receiver->get_output_port(0),
      end_effector_orientation_trajectory->get_input_port_trajectory());
  builder.Connect(end_effector_trajectory->get_output_port(0),
                  osc->get_input_port_tracking_data("end_effector_target"));
  builder.Connect(
      end_effector_orientation_trajectory->get_output_port(0),
      osc->get_input_port_tracking_data("end_effector_orientation_target"));
  builder.Connect(end_effector_force_receiver->get_output_port(0),
                  end_effector_force_trajectory->get_input_port_trajectory());
  builder.Connect(end_effector_force_trajectory->get_output_port(0),
                  osc->get_input_port_tracking_data("end_effector_force"));
  builder.Connect(osc_command_sender->get_output_port(),
                  franka_command_pub->get_input_port());
  builder.Connect(plant.get_state_output_port(jack_index),
                  object_state_sender->get_input_port_state());
  c3_controller->SetOsqpSolverOptions(solver_options);
  builder.Connect(franka_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_franka_state());
  builder.Connect(target_state_mux->get_output_port(),
                  c3_controller->get_input_port_target());
  builder.Connect(lcs_factory->get_output_port_lcs(),
                  c3_controller->get_input_port_lcs());
  builder.Connect(object_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_object_state());
  builder.Connect(object_state_receiver->get_output_port(),
                  control_target->get_input_port_object_state());
  builder.Connect(reduced_order_model_receiver->get_output_port(),
                  c3_controller->get_input_port_lcs_state());
  builder.Connect(radio_sub->get_output_port(),
                  c3_controller->get_input_port_radio());
  builder.Connect(reduced_order_model_receiver->get_output_port(),
                  lcs_factory->get_input_port_lcs_state());
//   builder.Connect(radio_sub->get_output_port(),
//                   control_target->get_input_port_radio());
  builder.Connect(c3_controller->get_output_port_c3_solution(),
                  c3_trajectory_generator->get_input_port_c3_solution());
  builder.Connect(lcs_factory->get_output_port_lcs_contact_jacobian(),
                  c3_output_sender->get_input_port_lcs_contact_info());
  builder.Connect(c3_state_sender->get_output_port_target_c3_state(),
                  c3_target_state_publisher->get_input_port());
  builder.Connect(c3_state_sender->get_output_port_actual_c3_state(),
                  c3_actual_state_publisher->get_input_port());
  builder.Connect(target_state_mux->get_output_port(),
                  c3_state_sender->get_input_port_target_state());
  builder.Connect(reduced_order_model_receiver->get_output_port_lcs_state(),
                  c3_state_sender->get_input_port_actual_state());
//  builder.Connect(c3_output_sender->get_output_port_c3_debug(),
//                  c3_output_publisher->get_input_port());
  builder.Connect(c3_controller->get_output_port_c3_solution(),
                  c3_output_sender->get_input_port_c3_solution());
  builder.Connect(c3_controller->get_output_port_c3_intermediates(),
                  c3_output_sender->get_input_port_c3_intermediates());

  // Diagram Connections
  builder.Connect(*object_state_sender, *object_state_receiver);
  builder.Connect(*osc_command_sender, *input_receiver);
  builder.Connect(passthrough->get_output_port(),
                  plant.get_actuation_input_port());
  builder.Connect(c3_trajectory_generator->get_output_port_actor_trajectory(),
                  end_effector_force_receiver->get_input_port_trajectory());
  builder.Connect(c3_trajectory_generator->get_output_port_actor_trajectory(),
                  end_effector_position_receiver->get_input_port_trajectory());
  builder.Connect(
      c3_trajectory_generator->get_output_port_actor_trajectory(),
      end_effector_orientation_receiver->get_input_port_trajectory());
  builder.Connect(*franka_state_sender, *franka_state_receiver);
  builder.Connect(*franka_state_sender, *osc_state_receiver);
  builder.Connect(*franka_state_sender, *state_pub);
  builder.Connect(plant.get_state_output_port(franka_index),
                   franka_state_sender->get_input_port_state());

  int nq = plant.num_positions();
  int nv = plant.num_velocities();

  if (sim_params.visualize_drake_sim) {
    drake::visualization::AddDefaultVisualization(&builder);
  }

  auto diagram = builder.Build();
  diagram->set_name("plate_balancing_full_diagram");
  DrawAndSaveDiagramGraph(*diagram);

  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(true);
  simulator.set_publish_at_initialization(true);
  simulator.set_target_realtime_rate(sim_params.realtime_rate);

  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, &simulator.get_mutable_context());

  VectorXd q = VectorXd::Zero(nq);
  std::map<std::string, int> q_map = MakeNameToPositionsMap(plant);

  q.head(plant.num_positions(franka_index)) = sim_params.q_init_franka;

  q.tail(plant.num_positions(jack_index)) =
      sim_params.q_init_plate[sim_params.scene_index];

  plant.SetPositions(&plant_context, q);

  VectorXd v = VectorXd::Zero(nv);
  plant.SetVelocities(&plant_context, v);

  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }
