#include <math.h>

#include <vector>

#include <drake/lcm/drake_lcm.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_c3.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/franka_ball_rolling/parameters/heuristic_planner_params.h"
#include "examples/franka_ball_rolling/parameters/lcm_channels_params.h"
#include "examples/franka_ball_rolling/parameters/simulate_franka_params.h"
#include "examples/franka_ball_rolling/parameters/state_estimator_params.h"
#include "examples/franka_ball_rolling/parameters/trajectory_params.h"
#include "examples/franka_ball_rolling/systems/c3_state_sender.h"
#include "examples/franka_ball_rolling/systems/control_refine_sender.h"
#include "examples/franka_ball_rolling/systems/franka_kinematics.h"
#include "examples/franka_ball_rolling/systems/heuristic_generator.h"
#include "examples/franka_ball_rolling/systems/track_target_generator.h"
#include "multibody/multibody_utils.h"
#include "solvers/c3_options.h"
#include "systems/controllers/c3/lcs_factory_system.h"
#include "systems/controllers/c3_controller.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/math/autodiff.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

DEFINE_int32(TTL, 0,
             "TTL level for publisher. "
             "Default value is 0.");

namespace dairlib {

using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  /* ------------------------- Load Parameters -----------------------------*/
  SimulateFrankaParams sim_param = drake::yaml::LoadYamlFile<
      SimulateFrankaParams>(
      "examples/franka_ball_rolling/parameters/simulate_franka_params.yaml");
  BallRollingTrajectoryParams traj_param =
      drake::yaml::LoadYamlFile<BallRollingTrajectoryParams>(
          "examples/franka_ball_rolling/parameters/trajectory_params.yaml");
  HeuristicPlannerParams heuristic_param = drake::yaml::LoadYamlFile<
      HeuristicPlannerParams>(
      "examples/franka_ball_rolling/parameters/heuristic_planner_params.yaml");
  StateEstimatorParams estimation_param = drake::yaml::LoadYamlFile<
      StateEstimatorParams>(
      "examples/franka_ball_rolling/parameters/state_estimator_params.yaml");
  C3Options c3_param = drake::yaml::LoadYamlFile<C3Options>(
      "examples/franka_ball_rolling/parameters/c3_options_ball_rolling.yaml");
  BallRollingLcmChannels lcm_channel_param = drake::yaml::LoadYamlFile<
      BallRollingLcmChannels>(
      "examples/franka_ball_rolling/parameters/lcm_channels_sim_params.yaml");

  /* -------------------------- Setup LCM ---------------------------------*/
  drake::lcm::DrakeLcm lcm;
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");

  /* ------------- Create full plant for Forward Kinematics --------------*/
  DiagramBuilder<double> builder_full_model;
  auto [plant_full_model, scene_graph_full_model] =
      AddMultibodyPlantSceneGraph(&builder_full_model, 0.0);
  Parser parser_full_model(&plant_full_model);
  drake::multibody::ModelInstanceIndex franka_index_full =
      parser_full_model.AddModels(sim_param.franka_model)[0];
  drake::multibody::ModelInstanceIndex ground_index_full =
      parser_full_model.AddModels(sim_param.ground_model)[0];
  drake::multibody::ModelInstanceIndex end_effector_index_full =
      parser_full_model.AddModels(sim_param.end_effector_model)[0];
  drake::multibody::ModelInstanceIndex ball_index_full =
      parser_full_model.AddModels(sim_param.ball_model)[0];

  RigidTransform<double> X_WI_full = RigidTransform<double>::Identity();
  RigidTransform<double> X_F_EE_full =
      RigidTransform<double>(sim_param.tool_attachment_frame);
  RigidTransform<double> X_F_G_full =
      RigidTransform<double>(sim_param.ground_offset_frame);

  plant_full_model.WeldFrames(plant_full_model.world_frame(),
                              plant_full_model.GetFrameByName("panda_link0"),
                              X_WI_full);
  plant_full_model.WeldFrames(plant_full_model.GetFrameByName("panda_link7"),
                              plant_full_model.GetFrameByName(
                                  "end_effector_base", end_effector_index_full),
                              X_F_EE_full);
  plant_full_model.WeldFrames(
      plant_full_model.GetFrameByName("panda_link0"),
      plant_full_model.GetFrameByName("ground", ground_index_full), X_F_G_full);
  plant_full_model.Finalize();

  auto diagram_full_model = builder_full_model.Build();
  std::unique_ptr<Context<double>> diagram_context_full_model =
      diagram_full_model->CreateDefaultContext();
  auto& context_full_model = diagram_full_model->GetMutableSubsystemContext(
      plant_full_model, diagram_context_full_model.get());

  GeometryId end_effector_geoms_full =
      plant_full_model.GetCollisionGeometriesForBody(
          plant_full_model.GetBodyByName("end_effector_tip"))[0];
  GeometryId ball_geoms_full = plant_full_model.GetCollisionGeometriesForBody(
      plant_full_model.GetBodyByName("sphere"))[0];
  GeometryId ground_geoms_full = plant_full_model.GetCollisionGeometriesForBody(
      plant_full_model.GetBodyByName("ground"))[0];
  std::vector<GeometryId> contact_geoms_full = {
      end_effector_geoms_full, ball_geoms_full, ground_geoms_full};

  std::vector<SortedPair<GeometryId>> contact_pairs_full;
  contact_pairs_full.push_back(
      SortedPair(contact_geoms_full[0], contact_geoms_full[1]));
  contact_pairs_full.push_back(
      SortedPair(contact_geoms_full[1], contact_geoms_full[2]));

  /* ----------------------- State Subscriber/Receiver --------------------*/
  DiagramBuilder<double> builder;
  auto ball_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_param.estimated_ball_state_channel, &lcm));
  auto franka_state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(
      plant_full_model, franka_index_full);
  auto ball_state_receiver = builder.AddSystem<systems::ObjectStateReceiver>(
      plant_full_model, ball_index_full);
  // franka_state receiver and subsription is declared in lcm driven loop
  builder.Connect(ball_state_sub->get_output_port(),
                  ball_state_receiver->get_input_port());

  /* ----------- Simplified Model/Forward Kinematics Block -----------------*/
  auto simplified_model_generator =
      builder.AddSystem<systems::FrankaKinematics>(
          plant_full_model, context_full_model, franka_index_full,
          ball_index_full, "end_effector_tip", "sphere", false,
          contact_pairs_full, c3_param, sim_param, false);

  /* -------- State Receiver to Forward Kinematics port connection ----------*/
  builder.Connect(franka_state_receiver->get_output_port(),
                  simplified_model_generator->get_input_port_franka_state());
  builder.Connect(ball_state_receiver->get_output_port(),
                  simplified_model_generator->get_input_port_object_state());

  /* ------------- Create LCS plant (for Target Generator, Heuristic Generator
   * and C3 Controller ---------------*/
  DiagramBuilder<double> builder_lcs;
  auto [plant_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&builder_lcs, 0.0);
  Parser parser_lcs(&plant_lcs);
  parser_lcs.AddModels(heuristic_param.end_effector_simple_model);
  parser_lcs.AddModels(heuristic_param.ball_model);
  parser_lcs.AddModels(heuristic_param.ground_model);
  RigidTransform<double> X_WI_lcs = RigidTransform<double>::Identity();
  RigidTransform<double> X_F_G_lcs =
      RigidTransform<double>(sim_param.ground_offset_frame);

  plant_lcs.WeldFrames(plant_lcs.world_frame(),
                       plant_lcs.GetFrameByName("base_link"), X_WI_lcs);
  plant_lcs.WeldFrames(plant_lcs.world_frame(),
                       plant_lcs.GetFrameByName("ground"), X_F_G_lcs);
  plant_lcs.Finalize();

  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_ad_lcs =
      drake::systems::System<double>::ToAutoDiffXd(plant_lcs);
  auto diagram_lcs = builder_lcs.Build();
  std::unique_ptr<Context<double>> diagram_context_lcs =
      diagram_lcs->CreateDefaultContext();
  auto& context_lcs = diagram_lcs->GetMutableSubsystemContext(
      plant_lcs, diagram_context_lcs.get());
  auto context_ad_lcs = plant_ad_lcs->CreateDefaultContext();

  GeometryId end_effector_geoms = plant_lcs.GetCollisionGeometriesForBody(
      plant_lcs.GetBodyByName("end_effector_simple"))[0];
  GeometryId ball_geoms = plant_lcs.GetCollisionGeometriesForBody(
      plant_lcs.GetBodyByName("sphere"))[0];
  GeometryId ground_geoms = plant_lcs.GetCollisionGeometriesForBody(
      plant_lcs.GetBodyByName("ground"))[0];
  std::vector<GeometryId> contact_geoms = {end_effector_geoms, ball_geoms,
                                           ground_geoms};

  std::vector<SortedPair<GeometryId>> contact_pairs;
  contact_pairs.push_back(SortedPair(contact_geoms[0], contact_geoms[1]));
  contact_pairs.push_back(SortedPair(contact_geoms[1], contact_geoms[2]));

  /* --------------- Target and Heuristic Generator Blocks ------------*/
  auto target_generator = builder.AddSystem<systems::TargetGenerator>(
      plant_lcs, sim_param, traj_param);
  auto heuristic_generator = builder.AddSystem<systems::HeuristicGenerator>(
      plant_lcs, sim_param, heuristic_param, traj_param, c3_param);

  /* -------- Generators and Forward Kinematics port connection -------*/
  builder.Connect(simplified_model_generator->get_output_port_lcs_state(),
                  target_generator->get_input_port_state());
  builder.Connect(simplified_model_generator->get_output_port_lcs_state(),
                  heuristic_generator->get_input_port_state());
  builder.Connect(target_generator->get_output_port_target(),
                  heuristic_generator->get_input_port_target());

  /* --------------------- LCS Factory System Block ----------------------*/
  auto lcs_factory_system = builder.AddSystem<systems::LCSFactorySystem>(
      plant_lcs, context_lcs, *plant_ad_lcs, *context_ad_lcs, contact_pairs,
      c3_param);

  /* ------------------- LCS Factory System port connection---------------*/
  builder.Connect(simplified_model_generator->get_output_port_lcs_state(),
                  lcs_factory_system->get_input_port_lcs_state());

  /* ------------------------ C3Controller Block ------------------------*/
  auto c3_controller =
      builder.AddSystem<systems::C3Controller>(plant_lcs, c3_param);

  /* ------------------- C3Controller port connection ---------------------*/
  builder.Connect(heuristic_generator->get_output_port_target(),
                  c3_controller->get_input_port_target());
  builder.Connect(heuristic_generator->get_output_port_cost_matrices(),
                  c3_controller->get_input_port_cost_matrices());
  builder.Connect(simplified_model_generator->get_output_port_lcs_state(),
                  c3_controller->get_input_port_lcs_state());
  builder.Connect(lcs_factory_system->get_output_port_lcs(),
                  c3_controller->get_input_port_lcs());

  /* --------------------- ControlRefinement Block -------------------------*/
  auto control_refinement = builder.AddSystem<systems::ControlRefineSender>(
      plant_lcs, context_lcs, *plant_ad_lcs, *context_ad_lcs, contact_pairs,
      c3_param);

  /* ------------------- ControlRefinement port connection -----------------*/
  builder.Connect(heuristic_generator->get_output_port_orientation(),
                  control_refinement->get_input_port_ee_orientation());
  builder.Connect(c3_controller->get_output_port_c3_solution(),
                  control_refinement->get_input_port_c3_solution());
  builder.Connect(simplified_model_generator->get_output_port_lcs_state(),
                  control_refinement->get_input_port_lcs_state());
  builder.Connect(
      simplified_model_generator->get_output_port_contact_jacobian_full(),
      control_refinement->get_input_port_contact_jacobian());

  // note that for ball rolling planner command, the num_target_state should be
  // 7 (pos) + 6 (vel) and target state is end effector state
  auto planner_command_sender =
      builder.AddSystem<systems::BallRollingCommandSender>(
          7 + 6, plant_full_model.num_actuators(franka_index_full), 1);

  builder.Connect(control_refinement->get_output_port_target(),
                  planner_command_sender->get_input_port_target_state());
  builder.Connect(control_refinement->get_output_port_contact_torque(),
                  planner_command_sender->get_input_port_contact_torque());

  /* ------- Determine if TTL 0 or 1 should be used for publishing --------*/
  drake::lcm::DrakeLcm* pub_lcm;
  if (FLAGS_TTL == 0) {
    std::cout << "Using TTL=0" << std::endl;
    pub_lcm = &lcm;
  } else if (FLAGS_TTL == 1) {
    std::cout << "Using TTL=1" << std::endl;
    pub_lcm = &lcm_network;
  }

  /* ----------------------- Final command publisher ------------------------*/
  auto control_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_ball_rolling_command>(
          lcm_channel_param.impedance_input_channel, pub_lcm,
          {drake::systems::TriggerType::kForced}));
  builder.Connect(planner_command_sender->get_output_port(),
                  control_publisher->get_input_port());

  /* ------ Visualization and Publish of LCS state actual and target  ------*/
  std::vector<std::string> state_names = {
      "end_effector_x",  "end_effector_y", "end_effector_z",  "ball_qw",
      "ball_qx",         "ball_qy",        "ball_qz",         "ball_x",
      "ball_y",          "ball_z",         "end_effector_vx", "end_effector_vy",
      "end_effector_vz", "ball_wx",        "ball_wy",         "ball_wz",
      "ball_vz",         "ball_vz",        "ball_vz",
  };
  auto c3_state_sender =
      builder.AddSystem<systems::C3StateSender>(3 + 7 + 3 + 6, state_names);
  auto c3_actual_state_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_param.c3_actual_state_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_target_state_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_param.c3_target_state_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(simplified_model_generator->get_output_port_lcs_state(),
                  c3_state_sender->get_input_port_actual_state());
  builder.Connect(heuristic_generator->get_output_port_target(),
                  c3_state_sender->get_input_port_target_state());
  builder.Connect(c3_state_sender->get_output_port_actual_c3_state(),
                  c3_actual_state_publisher->get_input_port());
  builder.Connect(c3_state_sender->get_output_port_target_c3_state(),
                  c3_target_state_publisher->get_input_port());

  /* ------- Visualization and Publish of LCS state actual and target  -------*/
  auto diagram = builder.Build();
  diagram->set_name(("Diagram_C3_Ball_Rolling_Planner"));
  DrawAndSaveDiagramGraph(
      *diagram, "examples/franka_ball_rolling/diagram_lcm_c3_ball_rolling");

  /* ------------------------------------------------------------------------------------*/
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(diagram), franka_state_receiver,
      lcm_channel_param.franka_state_channel, true);
  loop.Simulate(std::numeric_limits<double>::infinity());
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }
