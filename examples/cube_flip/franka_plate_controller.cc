
#include <dairlib/lcmt_radio_out.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "examples/cube_flip/parameter_headers/franka_plate_controller_params.h"
#include "examples/cube_flip/parameter_headers/franka_plate_lcm_channels.h"
#include "examples/cube_flip/parameter_headers/iC3_options.h"
#include "examples/cube_flip/systems/iC3_trajectory_generator.h"
#include "examples/cube_flip/systems/c3_goal_generator.h"
#include "examples/cube_flip/systems/c3_trajectory_generator.h"
#include "systems/controllers/c3/ic3_tracking_controller.h"
#include "examples/cube_flip/systems/timed_gate.h"
#include "systems/franka_kinematics.h"


#include "multibody/multibody_utils.h"
#include "solvers/lcs_factory.h"
#include "solvers/c3_options.h"

#include "systems/controllers/c3/lcs_factory_system.h"
#include "systems/controllers/c3/c3_controller.h"

#include "systems/framework/lcm_driven_loop.h"
#include "systems/primitives/radio_parser.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/c3_output_systems.h"

namespace dairlib {

using dairlib::solvers::LCSFactory;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RigidTransform;
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

using Eigen::Vector3d;
using Eigen::VectorXd;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;

DEFINE_string(controller_settings,
              "examples/cube_flip/parameters/franka_plate_controller_params.yaml",
              "Controller settings such as channels. Attempting to minimize "
              "number of gflags");
DEFINE_string(lcm_channels,
              "examples/cube_flip/parameters/plate_lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  // load parameters
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;

  FrankaPlateLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaPlateLcmChannels>(FLAGS_lcm_channels);

  FrankaPlateC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<FrankaPlateC3ControllerParams>(
          FLAGS_controller_settings);

  iC3Options ic3_options =
      drake::yaml::LoadYamlFile<iC3Options>(
          controller_params.ic3_options_file);

  C3Options c3_options =
      drake::yaml::LoadYamlFile<C3Options>(
          controller_params.c3_options_file);

  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(controller_params.osqp_settings_file))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());


  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka, nullptr);
  parser_franka.AddModelsFromUrl(controller_params.franka_model);
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

  // Object plant
  MultibodyPlant<double> plant_object(0.0);
  Parser parser_object(&plant_object, nullptr);
  parser_object.AddModels(controller_params.object_model);
  plant_object.Finalize();
  auto object_context = plant_object.CreateDefaultContext();


  // Dummy plant for ic3 
  DiagramBuilder<double> plant_ic3_builder;
  auto [plant_ic3, ic3_scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_ic3_builder, 0.0);
  Parser ic3_parser(&plant_ic3);
  ic3_parser.SetAutoRenaming(true);
  ic3_parser.AddModels(controller_params.end_effector_lcs_model);
  ic3_parser.AddModels(controller_params.object_model);

  // plant_ic3.WeldFrames(plant_ic3.world_frame(),
  //                       plant_ic3.GetFrameByName("plate"), X_WI);

  plant_ic3.Finalize();


  // Plant for c3 mpc
  DiagramBuilder<double> plant_builder;
  auto [plant_for_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_builder, 0.0);
  Parser lcs_parser(&plant_for_lcs);
  lcs_parser.SetAutoRenaming(true);
  lcs_parser.AddModels(controller_params.end_effector_lcs_model);

  lcs_parser.AddModels(controller_params.object_model);
  
  // plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
  //                          plant_for_lcs.GetFrameByName("plate"), X_WI);
  plant_for_lcs.Finalize();


  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_for_lcs_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(plant_for_lcs);

  auto plant_diagram = plant_builder.Build();
  std::unique_ptr<drake::systems::Context<double>> diagram_context =
      plant_diagram->CreateDefaultContext();
  auto& plant_lcs_context = plant_diagram->GetMutableSubsystemContext(
      plant_for_lcs, diagram_context.get());
  auto plant_lcs_context_ad = plant_for_lcs_autodiff->CreateDefaultContext();

  std::vector<drake::geometry::GeometryId> end_effector_contact_points =
      plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("plate"));
  
  std::vector<drake::geometry::GeometryId> cube_geoms_full =
      plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("cube"));

  std::vector<drake::geometry::GeometryId> cube_geoms(cube_geoms_full.begin()+1, cube_geoms_full.end());

  std::unordered_map<std::string, std::vector<drake::geometry::GeometryId>>
      contact_geoms;
  contact_geoms["PLATE"] = end_effector_contact_points;
  contact_geoms["CUBE"] = cube_geoms;

  std::vector<SortedPair<GeometryId>> contact_pairs;
  for (auto geom_id : contact_geoms["CUBE"]) {
    contact_pairs.emplace_back(geom_id, contact_geoms["PLATE"][0]);
  }

  DiagramBuilder<double> builder;
  auto franka_state_receiver =
    builder.AddSystem<systems::RobotOutputReceiver>(plant_franka);

  auto nominal_position =
      builder.AddSystem<drake::systems::ConstantVectorSource<double>>(controller_params.ee_init_position);

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
          
  // auto c3_object_trajectory_sender = builder.AddSystem(
  //     LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
  //         lcm_channel_params.c3_object_channel, &lcm,
  //         TriggerTypeSet({TriggerType::kForced})));   

  if (controller_params.run_open_loop) {
    auto ic3_target_generator =
      builder.AddSystem<iC3TrajectoryGenerator>(plant_for_lcs, ic3_options); 

    builder.Connect(ic3_target_generator->get_output_port_actor_trajectory(),
                    c3_actor_trajectory_sender->get_input_port());  
    // builder.Connect(ic3_target_generator->get_output_port_object_trajectory(),
    //                 c3_object_trajectory_sender->get_input_port()); 

    builder.Connect(nominal_position->get_output_port(),
                    ic3_target_generator->get_input_port_nominal_trajectory());

    builder.Connect(ic3_x_trajectory_sub->get_output_port(),
                    ic3_target_generator->get_input_port_iC3_x_trajectory());
    builder.Connect(ic3_u_trajectory_sub->get_output_port(),
                    ic3_target_generator->get_input_port_iC3_u_trajectory());
  } else {
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
          plant_lcs_context_ad.get(), contact_pairs, c3_options, controller_params.x_target, 0); 

    auto reduced_order_model_receiver =
      builder.AddSystem<systems::FrankaKinematics>(
          plant_franka, franka_context.get(), plant_object, object_context.get(),
          controller_params.end_effector_name, "cube",
          controller_params.include_end_effector_orientation);

    // Set nominal position of plate to init position
    VectorXd xd = controller_params.x_target;
    xd.segment(0, 3) = xd.segment(0, 3) + controller_params.ee_init_position;

    auto x_desired_source =
      builder.AddSystem<drake::systems::ConstantVectorSource<double>>(xd);    

    auto controller =
        builder.AddSystem<systems::iC3TrackingController>
            (plant_for_lcs, c3_options, ic3_options, 
							controller_params.time_to_wait);

    auto c3_trajectory_generator =
        builder.AddSystem<C3TrajectoryGenerator>(plant_for_lcs, c3_options, 
            controller_params.track_dynamically_feasible, 0); 
    c3_trajectory_generator->SetPublishEndEffectorOrientation(true);
    

    auto timed_gate =
      	builder.AddSystem<TimedGate>(controller_params.time_to_wait, ic3_options, 0);    

    builder.Connect(object_state_sub->get_output_port(),
                    object_state_receiver->get_input_port());

    builder.Connect(franka_state_receiver->get_output_port(),
                    reduced_order_model_receiver->get_input_port_franka_state());  
    builder.Connect(object_state_receiver->get_output_port(),
                    reduced_order_model_receiver->get_input_port_object_state());
      
    builder.Connect(nominal_position->get_output_port(),
            				c3_goal_generator->get_input_port_nominal_position());       
    builder.Connect(reduced_order_model_receiver->get_output_port(),
            				c3_goal_generator->get_input_port_state());      

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
		builder.Connect(timed_gate->get_output_port_actor(),
										c3_actor_trajectory_sender->get_input_port());

  }


  auto owned_diagram = builder.Build();
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  shared_diagram->set_name(("franka_plate_controller"));
  DrawAndSaveDiagramGraph(*shared_diagram);

  std::cout << "Before lcm driven loop" << std::endl;
  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, franka_state_receiver,
      lcm_channel_params.franka_state_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());

  LcmHandleSubscriptionsUntil(
      &lcm, [&]() { return ic3_x_trajectory_sub->GetInternalMessageCount() > 0; });
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }