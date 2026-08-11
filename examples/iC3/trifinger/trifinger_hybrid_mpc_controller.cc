
#include <dairlib/lcmt_radio_out.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "examples/iC3/trifinger/parameter_headers/trifinger_controller_params.h"
#include "examples/iC3/trifinger/parameter_headers/trifinger_lcm_channels.h"
#include "examples/iC3/iC3_options.h"
#include "examples/iC3/hybrid_mpc_options.h"
#include "examples/iC3/systems/mpc_trajectory_generator.h"
#include "examples/iC3/trifinger/systems/trifinger_kinematics.h"
#include "systems/controllers/c3/ic3_hybrid_mpc_tracking_controller.h"
#include "examples/iC3/systems/timed_gate.h"
#include "examples/iC3/systems/iC3_timing_system.h"
#include "examples/iC3/systems/perception_noise_filter.h"

#include "multibody/multibody_utils.h"
#include "c3/multibody/lcs_factory.h"

#include "systems/framework/lcm_driven_loop.h"
#include "systems/primitives/radio_parser.h"
#include "systems/robot_lcm_systems.h"
#include "solvers/solver_options_io.h"
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
using c3::multibody::LCSFactory;

using Eigen::Vector3d;
using Eigen::VectorXd;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;

DEFINE_string(lcm_channels,
              "examples/iC3/trifinger/parameters/trifinger_lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

DEFINE_int32(example_idx, 1, "1 = 180 yaw, 2 = 180 pivot");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  // load parameters
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;

  TrifingerLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<TrifingerLcmChannels>(FLAGS_lcm_channels);


  std::string controller_params_file;
  if (FLAGS_example_idx == 1) {
    controller_params_file = "examples/iC3/trifinger/parameters/trifinger_controller_params.yaml";
  } else if (FLAGS_example_idx == 2) {
    controller_params_file = "examples/iC3/trifinger/parameters/trifinger_pivot_controller_params.yaml";
  }

  TrifingerControllerParams controller_params =
      drake::yaml::LoadYamlFile<TrifingerControllerParams>(controller_params_file);

  std::string hybrid_mpc_options_file;
  if (FLAGS_example_idx == 1) {
    hybrid_mpc_options_file = "examples/iC3/trifinger/parameters/trifinger_hybrid_mpc_options_180.yaml";
  } else if (FLAGS_example_idx == 2) {
    hybrid_mpc_options_file = "examples/iC3/trifinger/parameters/trifinger_hybrid_mpc_options_pivot.yaml";
  }

  HybridMpcOptions hybrid_mpc_options =
      drake::yaml::LoadYamlFile<HybridMpcOptions>(hybrid_mpc_options_file);

  iC3Options ic3_options =
      drake::yaml::LoadYamlFile<iC3Options>(
          controller_params.ic3_options_file);

   drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(hybrid_mpc_options.osqp_settings))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  MultibodyPlant<double> plant_trifinger(0.0);
  Parser parser_trifinger(&plant_trifinger);
  parser_trifinger.SetAutoRenaming(true);
  parser_trifinger.package_map().Add(
    "robot_properties_fingers", 
    "examples/iC3/trifinger/robot_properties_fingers"
  );
  ModelInstanceIndex trifinger_index = parser_trifinger.AddModels(FindResourceOrThrow(controller_params.trifinger_model))[0];
  parser_trifinger.AddModels(FindResourceOrThrow(controller_params.end_effector_model));

  // HARDECODED tip names
  vector<std::string> trifinger_tip_names = {
    "finger_tip_link_0", "finger_tip_link_120", "finger_tip_link_240"};

  VectorXd xd = controller_params.x_target;

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
  parser_object.AddModels(hybrid_mpc_options.object_model);
  plant_object.Finalize();
  auto object_context = plant_object.CreateDefaultContext();


  // Plant for lcs
  DiagramBuilder<double> plant_builder;
  auto [plant_for_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_builder, 0.0);
  Parser lcs_parser(&plant_for_lcs);
  lcs_parser.SetAutoRenaming(true);
  lcs_parser.AddModels(controller_params.end_effector_lcs_model);
  lcs_parser.AddModels(hybrid_mpc_options.object_model);
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

  LCSFactory lcs_factory = LCSFactory(plant_for_lcs, plant_lcs_context, *plant_for_lcs_autodiff, 
                                    *plant_lcs_context_ad, hybrid_mpc_options.lcs_factory_options);

  int n_x = plant_for_lcs.num_positions() + plant_for_lcs.num_velocities();
  int n_u = plant_for_lcs.num_actuators();

  MatrixXd A_x(MatrixXd::Zero(n_x, n_x));
  VectorXd lb_x(VectorXd::Zero(n_x));
  VectorXd ub_x(VectorXd::Zero(n_x));

  MatrixXd A_x_mpc(MatrixXd::Zero(n_x, n_x));
  VectorXd lb_x_mpc(VectorXd::Zero(n_x));
  VectorXd ub_x_mpc(VectorXd::Zero(n_x));

  MatrixXd A_u(MatrixXd::Zero(n_u, n_u));
  VectorXd lb_u(VectorXd::Zero(n_u));
  VectorXd ub_u(VectorXd::Zero(n_u));

  if (FLAGS_example_idx == 1) {
    for (int i = 0; i < 3; i++) {
      A_x(3*i, 3*i) = 1;
      A_x(3*i+1, 3*i+1) = 1;
      A_x(3*i+2, 3*i+2) = 1;

      A_x(16 + 3*i, 16 + 3*i) = 1;
      A_x(16 + 3*i + 1, 16 + 3*i+1) = 1;
      A_x(16 + 3*i + 2, 16 + 3*i+2) = 1;

      lb_x(3*i) = xd(3*i) - 0.08;
      lb_x(3*i+1) = xd(3*i+1) - 0.08;
      lb_x(3*i+2) = xd(3*i+2) - 0.01;

      lb_x(16 + 3*i) = -0.08;
      lb_x(16 + 3*i+1) = -0.08;
      lb_x(16 + 3*i+2) = -0.05;

      ub_x(3*i) = xd(3*i) + 0.08;
      ub_x(3*i+1) = xd(3*i+1) + 0.08;
      ub_x(3*i+2) = xd(3*i+2) + 0.01;

      ub_x(16 + 3*i) = 0.08;
      ub_x(16 + 3*i+1) = 0.08;
      ub_x(16 + 3*i+2) = 0.05;


      A_x_mpc(16 + 3*i, 16 + 3*i) = 1;
      A_x_mpc(16 + 3*i + 1, 16 + 3*i+1) = 1;
      A_x_mpc(16 + 3*i + 2, 16 + 3*i+2) = 1;

      lb_x_mpc(16 + 3*i) = -0.08;
      lb_x_mpc(16 + 3*i+1) = -0.08;
      lb_x_mpc(16 + 3*i+2) = -0.05;

      ub_x_mpc(16 + 3*i) = 0.08;
      ub_x_mpc(16 + 3*i+1) = 0.08;
      ub_x_mpc(16 + 3*i+2) = 0.05;
    }

    for (int i = 0; i < 3; i++) {
      A_u(3*i, 3*i) = 1;
      A_u(3*i+1, 3*i+1) = 1;
      A_u(3*i+2, 3*i+2) = 1;
      
      lb_u(3*i) = -0.4;
      lb_u(3*i+1) = -0.4;
      lb_u(3*i+2) = 0.15;

      ub_u(3*i) = 0.4;
      ub_u(3*i+1) = 0.4;
      ub_u(3*i+2) = 0.25;
    }
  } else if (FLAGS_example_idx == 2) {
    for (int i = 0; i < 3; i++) {
      A_x(3*i, 3*i) = 1;
      A_x(3*i+1, 3*i+1) = 1;
      A_x(3*i+2, 3*i+2) = 1;

      A_x(16 + 3*i, 16 + 3*i) = 1;
      A_x(16 + 3*i + 1, 16 + 3*i+1) = 1;
      A_x(16 + 3*i + 2, 16 + 3*i+2) = 1;

      double xy_bound = (i == 0) ? 0.07 : 0.05;

      lb_x(3*i) = xd(3*i) - xy_bound;
      lb_x(3*i+1) = xd(3*i+1) - xy_bound;
      lb_x(3*i+2) = xd(3*i+2) - 0.01;

      lb_x(16 + 3*i) = -0.1;
      lb_x(16 + 3*i+1) = -0.1;
      lb_x(16 + 3*i+2) = -0.1;

      ub_x(3*i) = xd(3*i) + xy_bound;
      ub_x(3*i+1) = xd(3*i+1) + xy_bound;
      ub_x(3*i+2) = xd(3*i+2) + 0.05;

      ub_x(16 + 3*i) = 0.1;
      ub_x(16 + 3*i+1) = 0.1;
      ub_x(16 + 3*i+2) = 0.1;


      A_x_mpc(16 + 3*i, 16 + 3*i) = 1;
      A_x_mpc(16 + 3*i + 1, 16 + 3*i+1) = 1;
      A_x_mpc(16 + 3*i + 2, 16 + 3*i+2) = 1;

      lb_x_mpc(16 + 3*i) = -0.1;
      lb_x_mpc(16 + 3*i+1) = -0.1;
      lb_x_mpc(16 + 3*i+2) = -0.1;

      ub_x_mpc(16 + 3*i) = 0.1;
      ub_x_mpc(16 + 3*i+1) = 0.1;
      ub_x_mpc(16 + 3*i+2) = 0.1;
    }

    for (int i = 0; i < 3; i++) {
      A_u(3*i, 3*i) = 1;
      A_u(3*i+1, 3*i+1) = 1;
      A_u(3*i+2, 3*i+2) = 1;
      
      lb_u(3*i) = -2;
      lb_u(3*i+1) = -2;
      lb_u(3*i+2) = -1;

      ub_u(3*i) = 2;
      ub_u(3*i+1) = 2;
      ub_u(3*i+2) = 1;
    }
  }

  std::cout << "Before builder " << std::endl;
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

  auto ic3_lambda_trajectory_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.ic3_forces_channel, &lcm));


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

  auto perception_noise_filter = 
      builder.AddSystem<PerceptionNoiseFilter>(controller_params.add_noise);

  auto reduced_order_model_receiver =
    builder.AddSystem<TrifingerKinematics>(
        plant_trifinger, trifinger_context.get(), plant_object, object_context.get(),
        controller_params.end_effector_names, "cube");

  auto radio_sub =
        builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
            lcm_channel_params.radio_channel, &lcm));

  auto ic3_timing_system = builder.AddSystem<iC3TimingSystem>(
            ic3_options, controller_params);

  auto x_desired_source =
    builder.AddSystem<drake::systems::ConstantVectorSource<double>>(xd);    

  auto controller =
      builder.AddSystem<systems::iC3HybridMpcTrackingController>
          (plant_for_lcs, lcs_factory, hybrid_mpc_options, solver_options, ic3_options, FLAGS_example_idx,
            A_x_mpc, lb_x_mpc, ub_x_mpc, A_u, lb_u, ub_u);

  auto mpc_trajectory_generator =
      builder.AddSystem<MPCTrajectoryGenerator>(plant_for_lcs, &plant_lcs_context, lcs_factory, 
        ic3_options, hybrid_mpc_options.lcs_factory_options, controller_params.track_dynamically_feasible, FLAGS_example_idx,
        A_x, lb_x, ub_x, A_u, lb_u, ub_u); 

  mpc_trajectory_generator->SetPublishEndEffectorOrientation(false);
  
  auto timed_gate =
      builder.AddSystem<TimedGate>(ic3_options, 1);    

  builder.Connect(object_state_sub->get_output_port(),
                  object_state_receiver->get_input_port());

  builder.Connect(trifinger_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_trifinger_state());  
  builder.Connect(object_state_receiver->get_output_port(),
                  perception_noise_filter->get_input_port_object_state());
  builder.Connect(perception_noise_filter->get_output_port_object_state(),
                  reduced_order_model_receiver->get_input_port_object_state());                    
                  
  builder.Connect(radio_sub->get_output_port(),
                  ic3_timing_system->get_input_port_radio());  

  builder.Connect(nominal_position->get_output_port(),
                  controller->get_input_port_nominal_position());
  builder.Connect(ic3_timing_system->get_output_port_timestep(),
                  controller->get_input_port_timestep());
  builder.Connect(reduced_order_model_receiver->get_output_port(),
                  controller->get_input_port_lcs_state());  
  builder.Connect(ic3_x_trajectory_sub->get_output_port(),
                  controller->get_input_port_ic3_x());
  builder.Connect(ic3_u_trajectory_sub->get_output_port(),
                  controller->get_input_port_ic3_u());
  builder.Connect(ic3_lambda_trajectory_sub->get_output_port(),
                  controller->get_input_port_ic3_lambda());

  builder.Connect(controller->get_output_port_solution(),
                  mpc_trajectory_generator->get_input_port_solution());
  builder.Connect(reduced_order_model_receiver->get_output_port(),
                  mpc_trajectory_generator->get_input_port_x_lcs());   
  builder.Connect(nominal_position->get_output_port(),
                  mpc_trajectory_generator->get_input_port_nominal_position());

  builder.Connect(mpc_trajectory_generator->get_output_port_actor_trajectory(),
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
  shared_diagram->set_name(("trifinger_hybrid_mpc_controller"));
  DrawAndSaveDiagramGraph(*shared_diagram);

  std::cout << "Before lcm driven loop" << std::endl;
  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, trifinger_state_receiver,
      lcm_channel_params.trifinger_state_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());

  LcmHandleSubscriptionsUntil(
      &lcm, [&]() { return (ic3_x_trajectory_sub->GetInternalMessageCount() > 1); });
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
