
#include <dairlib/lcmt_radio_out.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "examples/cube_flip/trifinger/parameter_headers/trifinger_controller_params.h"
#include "examples/cube_flip/trifinger/parameter_headers/trifinger_lcm_channels.h"
#include "examples/cube_flip/parameter_headers/iC3_options.h"
#include "examples/cube_flip/systems/iC3_trajectory_generator.h"
#include "examples/cube_flip/systems/c3_goal_generator.h"
#include "examples/cube_flip/systems/c3_trajectory_generator.h"
#include "examples/cube_flip/trifinger/systems/trifinger_kinematics.h"
#include "systems/controllers/c3/ic3_tracking_controller.h"
#include "examples/cube_flip/systems/timed_gate.h"
#include "examples/cube_flip/systems/iC3_timing_system.h"


#include "multibody/multibody_utils.h"
#include "c3/multibody/lcs_factory.h"
#include "c3/core/c3_options.h"
#include "c3/systems/c3_controller_options.h"
#include "c3/core/solver_options_io.h"

#include "systems/framework/lcm_driven_loop.h"
#include "systems/primitives/radio_parser.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/c3_output_systems.h"

namespace dairlib {

using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RigidTransform;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::MatrixXd;

using c3::C3Options;
using c3::systems::C3ControllerOptions;

using Eigen::Vector3d;
using Eigen::VectorXd;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;

DEFINE_string(controller_settings,
              "examples/cube_flip/trifinger/parameters/trifinger_controller_params.yaml",
              "Controller settings such as channels. Attempting to minimize "
              "number of gflags");
DEFINE_string(lcm_channels,
              "examples/cube_flip/trifinger/parameters/trifinger_lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  // load parameters
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;

  TrifingerLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<TrifingerLcmChannels>(FLAGS_lcm_channels);

  TrifingerControllerParams controller_params =
      drake::yaml::LoadYamlFile<TrifingerControllerParams>(
          FLAGS_controller_settings);

  iC3Options ic3_options =
      drake::yaml::LoadYamlFile<iC3Options>(
          controller_params.ic3_options_file);

  C3ControllerOptions c3_controller_options =
      drake::yaml::LoadYamlFile<C3ControllerOptions>(
          controller_params.c3_controller_options_file);

  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<c3::SolverOptionsFromYaml>(
          FindResourceOrThrow(controller_params.osqp_settings_file))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());


  MultibodyPlant<double> plant_trifinger(0.0);
  Parser parser_trifinger(&plant_trifinger);
  parser_trifinger.SetAutoRenaming(true);
  parser_trifinger.package_map().Add(
    "robot_properties_fingers", 
    "examples/cube_flip/trifinger/robot_properties_fingers"
  );
  ModelInstanceIndex trifinger_index = parser_trifinger.AddModels(FindResourceOrThrow(controller_params.trifinger_model))[0];
  parser_trifinger.AddModels(FindResourceOrThrow(controller_params.end_effector_model));

  // HARDECODED tip names
  vector<std::string> trifinger_tip_names = {
    "finger_tip_link_0", "finger_tip_link_120", "finger_tip_link_240"};

  for (int i = 0; i < 3; i++) {
    const auto& trifinger_tip_frame = plant_trifinger.GetFrameByName(trifinger_tip_names[i], trifinger_index);
    const auto& fingertip_frame = plant_trifinger.GetFrameByName(controller_params.end_effector_names[i]);
    plant_trifinger.WeldFrames(trifinger_tip_frame, fingertip_frame, RigidTransform<double>::Identity());
  }

  Eigen::Vector3d base_translation(-0.0 * Vector3d::UnitZ());
  RigidTransformd X_WI(drake::math::RotationMatrix<double>(), base_translation);
  plant_trifinger.WeldFrames(plant_trifinger.world_frame(), 
    plant_trifinger.GetFrameByName("base_link"), X_WI);

  plant_trifinger.Finalize();
  auto trifinger_context = plant_trifinger.CreateDefaultContext();

  // Object plant
  MultibodyPlant<double> plant_object(0.0);
  Parser parser_object(&plant_object, nullptr);
  parser_object.AddModels(controller_params.object_model);
  plant_object.Finalize();
  auto object_context = plant_object.CreateDefaultContext();


  // Plant for c3 mpc
  DiagramBuilder<double> plant_builder;
  auto [plant_for_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_builder, 0.0);
  Parser lcs_parser(&plant_for_lcs);
  lcs_parser.SetAutoRenaming(true);
  lcs_parser.AddModels(controller_params.end_effector_lcs_model);
  lcs_parser.AddModels(controller_params.object_model);
  lcs_parser.AddModels(controller_params.ground_model);
  
  for (auto fingertip : controller_params.ee_base_link_names) {
    plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
                            plant_for_lcs.GetFrameByName(fingertip), RigidTransform<double>::Identity());
  }
  plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
                          plant_for_lcs.GetFrameByName("ground"), RigidTransform<double>::Identity());

  plant_for_lcs.Finalize();


  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_for_lcs_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(plant_for_lcs);

  auto plant_diagram = plant_builder.Build();
  std::unique_ptr<drake::systems::Context<double>> diagram_context =
      plant_diagram->CreateDefaultContext();
  auto& plant_lcs_context = plant_diagram->GetMutableSubsystemContext(
      plant_for_lcs, diagram_context.get());
  auto plant_lcs_context_ad = plant_for_lcs_autodiff->CreateDefaultContext();


  GeometryId ground_collision_geom = 
    plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("ground"))[0];

  std::vector<GeometryId> end_effector_collision_geoms;
  for (auto fingertip_name : controller_params.end_effector_names) {
    end_effector_collision_geoms.push_back(
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName(fingertip_name))[0]);
  }

	std::vector<GeometryId> cube_collision_geoms;
  for (int i = 0; i <= 8; i++) {
      cube_collision_geoms.push_back(
          plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("cube"))[i]);
  }

  std::vector<SortedPair<GeometryId>> contact_pairs;
  // fingertip-cube contact pairs
	for (auto geom_id : end_effector_collision_geoms) {
		contact_pairs.emplace_back(cube_collision_geoms[0], geom_id);
  }
  // fingertip-ground contact pairs
  for (auto geom_id : end_effector_collision_geoms) {
		contact_pairs.emplace_back(geom_id, ground_collision_geom);
  }
  // cube-ground contact pairs
  for (int i = 1; i < cube_collision_geoms.size(); i++) {
		contact_pairs.emplace_back(cube_collision_geoms[i], ground_collision_geom);
  }


  DiagramBuilder<double> builder;
  auto trifinger_state_receiver =
    builder.AddSystem<systems::RobotOutputReceiver>(plant_trifinger);

  // "Neutral config" of fingers
  auto nominal_position =
      builder.AddSystem<drake::systems::ConstantVectorSource<double>>(controller_params.nominal_position);

  auto ic3_x_trajectory_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.ic3_positions_channel, &lcm));

  auto ic3_u_trajectory_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.ic3_inputs_channel, &lcm));

  auto c3_actor_trajectory_sender = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_actor_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));   

  auto lqr_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_lqr_output>(
          "iC3_LQR", &lcm));

  auto object_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.object_state_channel, &lcm));

  auto object_state_receiver =
      builder.AddSystem<systems::ObjectStateReceiver>(plant_object);

  auto c3_goal_generator = 
      builder.AddSystem<C3GoalGenerator>(plant_for_lcs, &plant_lcs_context, *plant_for_lcs_autodiff, 
          plant_lcs_context_ad.get(), ic3_options, c3_controller_options, controller_params.x_target, 1); 

  auto reduced_order_model_receiver =
    builder.AddSystem<TrifingerKinematics>(
        plant_trifinger, trifinger_context.get(), plant_object, object_context.get(),
        controller_params.end_effector_names, "cube");

  auto radio_sub =
        builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
            lcm_channel_params.radio_channel, &lcm));

  auto ic3_timing_system = builder.AddSystem<iC3TimingSystem>(
            ic3_options, controller_params);

  // Set nominal position of plate to init position
  VectorXd xd = controller_params.x_target;

  auto x_desired_source =
    builder.AddSystem<drake::systems::ConstantVectorSource<double>>(xd);    

  auto controller =
      builder.AddSystem<systems::iC3TrackingController>
          (plant_for_lcs, c3_controller_options, ic3_options, 
            controller_params.time_to_wait);

  auto c3_trajectory_generator =
      builder.AddSystem<C3TrajectoryGenerator>(plant_for_lcs, c3_controller_options, 
          controller_params.track_dynamically_feasible, 2); 
  c3_trajectory_generator->SetPublishEndEffectorOrientation(false);
  

  auto timed_gate =
      builder.AddSystem<TimedGate>(ic3_options, 1);    

  builder.Connect(object_state_sub->get_output_port(),
                  object_state_receiver->get_input_port());

  builder.Connect(trifinger_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_trifinger_state());  
  builder.Connect(object_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_object_state());
    
  // Note: nominal position unused here
  builder.Connect(nominal_position->get_output_port(),
                  c3_goal_generator->get_input_port_nominal_position());       
  builder.Connect(reduced_order_model_receiver->get_output_port(),
                  c3_goal_generator->get_input_port_state());    
  builder.Connect(ic3_x_trajectory_sub->get_output_port(),
                  c3_goal_generator->get_input_port_ic3_x()); 
  builder.Connect(ic3_timing_system->get_output_port_timestep(),
                  c3_goal_generator->get_input_port_timestep());                 
                  
  builder.Connect(radio_sub->get_output_port(),
                  ic3_timing_system->get_input_port_radio());  


  builder.Connect(ic3_timing_system->get_output_port_timestep(),
                  controller->get_input_port_timestep());
  builder.Connect(c3_goal_generator->get_output_port_x_curr(),
                  controller->get_input_port_lcs_state());
  builder.Connect(c3_goal_generator->get_output_port_target(),
                  controller->get_input_port_target());
  builder.Connect(c3_goal_generator->get_output_port_lcs(),
                  controller->get_input_port_lcs());
  builder.Connect(lqr_sub->get_output_port(),
                  controller->get_input_port_lqr());    
  builder.Connect(ic3_x_trajectory_sub->get_output_port(),
                  controller->get_input_port_ic3_x());
  builder.Connect(ic3_u_trajectory_sub->get_output_port(),
                  controller->get_input_port_ic3_u());

  // Note: nominal position unused here
  builder.Connect(nominal_position->get_output_port(),
                  c3_trajectory_generator->get_input_port_nominal_position());
  builder.Connect(c3_goal_generator->get_output_port_lcs(),
                  c3_trajectory_generator->get_input_port_lcs());
  builder.Connect(controller->get_output_port_c3_solution(),
                  c3_trajectory_generator->get_input_port_c3_solution());

  builder.Connect(c3_trajectory_generator->get_output_port_actor_trajectory(),
                  timed_gate->get_input_port_c3_actor());
  builder.Connect(nominal_position->get_output_port(),
                  timed_gate->get_input_port_nominal_position());
  builder.Connect(ic3_x_trajectory_sub->get_output_port(),
                  timed_gate->get_input_port_ic3_x());
  builder.Connect(ic3_timing_system->get_output_port_timestep(),
                  timed_gate->get_input_port_timestep());
  builder.Connect(radio_sub->get_output_port(),
                  timed_gate->get_input_port_radio());

  builder.Connect(timed_gate->get_output_port_actor(),
                  c3_actor_trajectory_sender->get_input_port());


  auto owned_diagram = builder.Build();
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  shared_diagram->set_name(("trifinger_controller"));
  DrawAndSaveDiagramGraph(*shared_diagram);

  std::cout << "Before lcm driven loop" << std::endl;
  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, trifinger_state_receiver,
      lcm_channel_params.trifinger_state_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());

  LcmHandleSubscriptionsUntil(
      &lcm, [&]() { return ic3_x_trajectory_sub->GetInternalMessageCount() > 0; });
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }