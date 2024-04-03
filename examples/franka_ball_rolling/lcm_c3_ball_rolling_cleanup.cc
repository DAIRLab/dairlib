#include <vector>
#include <math.h>
#include <gflags/gflags.h>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/math/rigid_transform.h>
#include "drake/math/autodiff.h"


#include "systems/robot_lcm_systems.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_c3.hpp"
#include "multibody/multibody_utils.h"
#include "systems/system_utils.h"


#include "examples/franka_ball_rolling/parameters/heuristic_gait_params.h"
#include "examples/franka_ball_rolling/parameters/trajectory_params.h"
#include "examples/franka_ball_rolling/parameters/simulate_franka_params.h"
#include "examples/franka_ball_rolling/parameters/c3_state_estimator_params.h"

#include "solvers/c3_options.h"

#include "examples/franka_ball_rolling/systems/franka_kinematics.h"
#include "examples/franka_ball_rolling/systems/track_target_generator.h"
#include "examples/franka_ball_rolling/systems/heuristic_generator.h"
#include "examples/franka_ball_rolling/systems/control_refine_sender.h"
#include "examples/franka_ball_rolling/systems/c3_state_sender.h"

#include "systems/controllers/c3_controller.h"
#include "systems/controllers/c3/lcs_factory_system.h"
#include "systems/framework/lcm_driven_loop.h"

DEFINE_int32(TTL, 0,
              "TTL level for publisher. "
              "Default value is 0.");

namespace dairlib {

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::math::RigidTransform;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::Context;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::multibody::Parser;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;

int DoMain(int argc, char* argv[]){
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  /* -------------------------------- Load Parameters --------------------------------------------*/
  SimulateFrankaParams sim_param = drake::yaml::LoadYamlFile<SimulateFrankaParams>(
          "examples/franka_ball_rolling/parameters/simulate_franka_params.yaml");
  BallRollingTrajectoryParams traj_param = drake::yaml::LoadYamlFile<BallRollingTrajectoryParams>(
          "examples/franka_ball_rolling/parameters/trajectory_params.yaml");
  HeuristicGaitParams heuristic_param = drake::yaml::LoadYamlFile<HeuristicGaitParams>(
            "examples/franka_ball_rolling/parameters/heuristic_gait_params.yaml");
  C3StateEstimatorParams estimation_param = drake::yaml::LoadYamlFile<C3StateEstimatorParams>(
          "examples/franka_ball_rolling/parameters/c3_state_estimator_params.yaml");
  C3Options c3_param = drake::yaml::LoadYamlFile<C3Options>(
          "examples/franka_ball_rolling/parameters/c3_options_ball_rolling.yaml");


  /* -------------------------------- Setup LCM --------------------------------------------*/
  drake::lcm::DrakeLcm lcm;
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");


  /* ----------------------- Create plants for Forward Kinematics  --------------------------*/
  // Forward kinematics need single franka plant and signle object plant
  // Franka plant
  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka, nullptr);
  parser_franka.AddModels(sim_param.franka_model);
  drake::multibody::ModelInstanceIndex end_effector_index =
            parser_franka.AddModels(sim_param.end_effector_model)[0];
  parser_franka.AddModels(sim_param.ground_model);
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  RigidTransform<double> X_F_EE = RigidTransform<double>(sim_param.tool_attachment_frame);
  RigidTransform<double> X_F_G = RigidTransform<double>(sim_param.ground_offset_frame);

  plant_franka.WeldFrames(plant_franka.world_frame(),
                            plant_franka.GetFrameByName("panda_link0"), X_WI);
  plant_franka.WeldFrames(plant_franka.GetFrameByName("panda_link7"),
                          plant_franka.GetFrameByName("end_effector_base"), X_F_EE);
  plant_franka.WeldFrames(plant_franka.GetFrameByName("panda_link0"),
                          plant_franka.GetFrameByName("ground"), X_F_G);
  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();

  // Ball plant
  MultibodyPlant<double> plant_ball(0.0);
  Parser parser_ball(&plant_ball, nullptr);
  parser_ball.AddModels(sim_param.ball_model);
  plant_ball.Finalize();
  auto ball_context = plant_ball.CreateDefaultContext();

  DiagramBuilder<double> builder;
  /* ----------------------------- State Subscriber/Receiver --------------------------------*/
  auto ball_state_sub =
            builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
                    "BALL_STATE_ESTIMATE_NEW", &lcm));
  auto franka_state_receiver =
            builder.AddSystem<systems::RobotOutputReceiver>(plant_franka);
  auto ball_state_receiver =
            builder.AddSystem<systems::ObjectStateReceiver>(plant_ball);
  // franka_state receiver and subsription is declared in lcm driven loop
  builder.Connect(ball_state_sub->get_output_port(),
                    ball_state_receiver->get_input_port());

  /* -------------------------Simplified Model/Forward Kinematics Block --------------------------------*/
  // TODO: think which yaml should put include_end_effector (now hardcoded false)
  auto simplified_model_generator =
            builder.AddSystem<systems::FrankaKinematics>(
                    plant_franka, franka_context.get(), plant_ball, ball_context.get(),
                    "end_effector_tip", "sphere",
                    false, sim_param, false);

  /* ----------------------- State Receiver to Forward Kinematics port connection --------------------------*/
  builder.Connect(franka_state_receiver->get_output_port(),
                  simplified_model_generator->get_input_port_franka_state());
  builder.Connect(ball_state_receiver->get_output_port(),
                  simplified_model_generator->get_input_port_object_state());

  /* ------------- Create LCS plant (for Target Generator, Heuristic Generator and C3 Controller ---------------*/
  DiagramBuilder<double> builder_lcs;
  auto [plant_lcs, scene_graph] = AddMultibodyPlantSceneGraph(&builder_lcs, 0.0);
  Parser parser_lcs(&plant_lcs);
  // TODO: create a high level planning parameter yaml file to save the simplified model urdf path
  // (maybe can integrate with HeuristicGaitParameters)
  parser_lcs.AddModels("examples/franka_ball_rolling/robot_properties_fingers/urdf/end_effector_simple.urdf");
  parser_lcs.AddModels("examples/franka_ball_rolling/robot_properties_fingers/urdf/sphere.urdf");
  parser_lcs.AddModels("examples/franka_ball_rolling/robot_properties_fingers/urdf/ground.urdf");
  RigidTransform<double> X_WI_lcs = RigidTransform<double>::Identity();
  RigidTransform<double> X_F_G_lcs = RigidTransform<double>(sim_param.ground_offset_frame);

  plant_lcs.WeldFrames(plant_lcs.world_frame(), plant_lcs.GetFrameByName("base_link"), X_WI_lcs);
  plant_lcs.WeldFrames(plant_lcs.world_frame(), plant_lcs.GetFrameByName("ground"), X_F_G_lcs);
  plant_lcs.Finalize();

  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_ad_lcs =
    drake::systems::System<double>::ToAutoDiffXd(plant_lcs);
  auto diagram_lcs = builder_lcs.Build();
  std::unique_ptr<Context<double>> diagram_context_lcs = diagram_lcs->CreateDefaultContext();
  auto& context_lcs = diagram_lcs->GetMutableSubsystemContext(plant_lcs, diagram_context_lcs.get());
  auto context_ad_lcs = plant_ad_lcs->CreateDefaultContext();

  GeometryId end_effector_geoms =
          plant_lcs.GetCollisionGeometriesForBody(plant_lcs.GetBodyByName("end_effector_simple"))[0];
  GeometryId ball_geoms =
          plant_lcs.GetCollisionGeometriesForBody(plant_lcs.GetBodyByName("sphere"))[0];
  GeometryId ground_geoms =
          plant_lcs.GetCollisionGeometriesForBody(plant_lcs.GetBodyByName("ground"))[0];
  std::vector<GeometryId> contact_geoms =
    {end_effector_geoms, ball_geoms, ground_geoms};

    std::vector<SortedPair<GeometryId>> contact_pairs;
    contact_pairs.push_back(SortedPair(contact_geoms[0], contact_geoms[1]));
    contact_pairs.push_back(SortedPair(contact_geoms[1], contact_geoms[2]));

  /* --------------------------------- Target and Heuristic Generator Blocks  -----------------------------------------*/
  auto target_generator =
          builder.AddSystem<systems::TargetGenerator>(plant_lcs, sim_param, traj_param);
  auto heuristic_generator =
            builder.AddSystem<systems::HeuristicGenerator>(plant_lcs, sim_param, heuristic_param,
                                                           traj_param, c3_param);

  /* ----------------------- Generators and Forward Kinematics port connection --------------------------*/
  builder.Connect(simplified_model_generator->get_output_port_lcs_state(),
                  target_generator->get_input_port_state());
  builder.Connect(simplified_model_generator->get_output_port_lcs_state(),
                  heuristic_generator->get_input_port_state());
  builder.Connect(target_generator->get_output_port_target(),
                    heuristic_generator->get_input_port_target());

  /* -------------------------------------- LCS Factory System Block  -----------------------------------------*/
  auto lcs_factory_system = builder.AddSystem<systems::LCSFactorySystem>(plant_lcs,context_lcs,
                                                                         *plant_ad_lcs,*context_ad_lcs,
                                                                         contact_pairs,c3_param);

  /* ----------------------------------- LCS Factory System port connection --------------------------------*/
  builder.Connect(simplified_model_generator->get_output_port_lcs_state(),
                  lcs_factory_system->get_input_port_lcs_state());

  /* ------------------------------------- C3Controller Block  -----------------------------------------*/
  auto c3_controller = builder.AddSystem<systems::C3Controller>(plant_lcs, c3_param);

  /* ----------------------------------- C3Controller port connection --------------------------------*/
  builder.Connect(heuristic_generator->get_output_port_target(),
                  c3_controller->get_input_port_target());
  builder.Connect(heuristic_generator->get_output_port_cost_matrices(),
                    c3_controller->get_input_port_cost_matrices());
  builder.Connect(simplified_model_generator->get_output_port_lcs_state(),
                  c3_controller->get_input_port_lcs_state());
  builder.Connect(lcs_factory_system->get_output_port_lcs(),
                    c3_controller->get_input_port_lcs());

  /* ------------------------------------- ControlRefinement Block  -----------------------------------------*/
  auto control_refinement = builder.AddSystem<systems::ControlRefineSender>(plant_lcs,context_lcs,
                                                                            *plant_ad_lcs,*context_ad_lcs,
                                                                            contact_pairs,c3_param);

  /* ----------------------------------- ControlRefinement port connection --------------------------------*/
  builder.Connect(heuristic_generator->get_output_port_orientation(),
                  control_refinement->get_input_port_ee_orientation());
  builder.Connect(c3_controller->get_output_port_c3_solution(),
                    control_refinement->get_input_port_c3_solution());
  builder.Connect(simplified_model_generator->get_output_port_lcs_state(),
                    control_refinement->get_input_port_lcs_state());

  auto state_force_sender = builder.AddSystem<systems::RobotC3Sender>(14, 9, 6, 9);
  builder.Connect(control_refinement->get_output_port_target(),
                  state_force_sender->get_input_port(0));

  /* ----------------------- Determine if TTL 0 or 1 should be used for publishing  --------------------------*/
  drake::lcm::DrakeLcm* pub_lcm;
  if (FLAGS_TTL == 0) {
        std::cout << "Using TTL=0" << std::endl;
        pub_lcm = &lcm;
  }
  else if (FLAGS_TTL == 1) {
        std::cout << "Using TTL=1" << std::endl;
        pub_lcm = &lcm_network;
  }

  /* ----------------------------------- Final command publisher --------------------------------*/
  auto control_publisher = builder.AddSystem(
            LcmPublisherSystem::Make<dairlib::lcmt_c3>(
                    "CONTROLLER_INPUT_NEW", pub_lcm,
                    {drake::systems::TriggerType::kForced}, 0.0));
  builder.Connect(state_force_sender->get_output_port(),
                    control_publisher->get_input_port());


  /* ----------------------- Visualization and Publish of LCS state actual and target  --------------------------*/
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
                    "LCS_STATE", &lcm, TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(simplified_model_generator->get_output_port_lcs_state(),
                  c3_state_sender->get_input_port_actual_state());
  builder.Connect(c3_state_sender->get_output_port_actual_c3_state(),
                    c3_actual_state_publisher->get_input_port());


  /* ----------------------- Visualization and Publish of LCS state actual and target  --------------------------*/
  auto diagram = builder.Build();
  diagram->set_name(("Diagram_C3_Ball_Rolling_Planner"));
  DrawAndSaveDiagramGraph(*diagram, "examples/franka_ball_rolling/diagram_lcm_c3_ball_rolling");

  /* -------------------------------------------------------------------------------------------*/
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
            &lcm, std::move(diagram), franka_state_receiver,
            "FRANKA_STATE_ESTIMATE_NEW", true);
  loop.Simulate(std::numeric_limits<double>::infinity());
  return 0;
}

} // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv);}
