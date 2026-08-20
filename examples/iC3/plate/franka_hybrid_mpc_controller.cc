
#include <dairlib/lcmt_radio_out.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "examples/iC3/plate/parameter_headers/franka_plate_controller_params.h"
#include "examples/iC3/plate/parameter_headers/franka_plate_lcm_channels.h"
#include "examples/iC3/iC3_options.h"
#include "examples/iC3/hybrid_mpc_options.h"
#include "examples/iC3/systems/mpc_trajectory_generator.h"
#include "systems/franka_kinematics.h"
#include "systems/controllers/c3/ic3_hybrid_mpc_tracking_controller.h"
#include "examples/iC3/systems/timed_gate.h"
#include "examples/iC3/systems/iC3_timing_system.h"
#include "examples/iC3/systems/perception_noise_filter.h"
#include "examples/iC3/systems/state_vector_to_basic_vector.h"

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
              "examples/iC3/plate/parameters/plate_lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");
DEFINE_string(controller_params_file,
              "examples/iC3/plate/parameters/franka_plate_controller_params.yaml",
              "Filepath containing lcm channels");
DEFINE_string(hybrid_mpc_options_file,
              "examples/iC3/plate/parameters/plate_hybrid_mpc_options.yaml",
              "Filepath containing lcm channels");            

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  // load parameters
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;

  FrankaPlateLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaPlateLcmChannels>(FLAGS_lcm_channels);

  FrankaPlateControllerParams controller_params =
      drake::yaml::LoadYamlFile<FrankaPlateControllerParams>(FLAGS_controller_params_file);

  HybridMpcOptions hybrid_mpc_options =
      drake::yaml::LoadYamlFile<HybridMpcOptions>(FLAGS_hybrid_mpc_options_file);

  iC3Options ic3_options =
      drake::yaml::LoadYamlFile<iC3Options>(
          controller_params.ic3_options_file);

   drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(hybrid_mpc_options.osqp_settings))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  VectorXd xd = controller_params.x_target;

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

  MatrixXd A_x_mpc(MatrixXd::Zero(n_x, n_x));
  VectorXd lb_x_mpc(VectorXd::Zero(n_x));
  VectorXd ub_x_mpc(VectorXd::Zero(n_x));

  MatrixXd A_x(MatrixXd::Zero(n_x, n_x));
  VectorXd lb_x(VectorXd::Zero(n_x));
  VectorXd ub_x(VectorXd::Zero(n_x));

  MatrixXd A_u_mpc(MatrixXd::Zero(n_u, n_u));
  VectorXd lb_u_mpc(VectorXd::Zero(n_u));
  VectorXd ub_u_mpc(VectorXd::Zero(n_u));

  MatrixXd A_u(MatrixXd::Zero(n_u, n_u));
  VectorXd lb_u(VectorXd::Zero(n_u));
  VectorXd ub_u(VectorXd::Zero(n_u));

  // A_x_mpc(2, 2) = 1;
  A_x_mpc(3, 3) = 1;
  A_x_mpc(4, 4) = 1;

  A_x(0, 0) = 1;
  A_x(1, 1) = 1;
  A_x(2, 2) = 1;
  A_x(3, 3) = 1;
  A_x(4, 4) = 1;

//   lb_x_mpc(2) = -0.1; 
//   lb_x_mpc(3) = -0.5;
//   lb_x_mpc(4) = -0.5;

//   ub_x_mpc(2) = 0.1;
//   ub_x_mpc(3) = 0.5;
//   ub_x_mpc(4) = 0.5;

  size_t pos = controller_params.end_effector_model.find("offset");
  std::cout << "OFFSET POS " << pos << std::endl;
  double z_offset = (pos == std::string::npos) ? 0 : -0.107;

  lb_x(0) = -0.08;
  lb_x(1) = -0.08;
  lb_x(2) = z_offset - 0.1; 
  lb_x(3) = -0.75;
  lb_x(4) = -0.75;

  ub_x(0) = 0.08;
  ub_x(1) = 0.08;
  ub_x(2) = z_offset + 0.1;
  ub_x(3) = 0.75;
  ub_x(4) = 0.75;

  A_u(0, 0) = 1;
  A_u(1, 1) = 1;
  A_u(2, 2) = 1;
  A_u(3, 3) = 1;
  A_u(4, 4) = 1;

  lb_u(0) = -1;
  lb_u(1) = -1;
  lb_u(2) = 9.81 * 0.85 - 3.5;
  lb_u(3) = -2.4;
  lb_u(4) = -2.4;

  ub_u(0) = 1;
  ub_u(1) = 1;
  ub_u(2) = 9.81 * 0.85 + 3.5;
  ub_u(3) = 2.4;
  ub_u(4) = 2.4;


  std::cout << "Before builder " << std::endl;
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

  auto noisy_object_state_sender =
      builder.AddSystem<systems::ObjectStateSender>(plant_object);

  auto noisy_object_state_vector =
      builder.AddSystem<StateVectorToBasicVector>(plant_object);

  auto noisy_object_state_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.noisy_object_state_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));  

  auto reduced_order_model_receiver =
      builder.AddSystem<systems::FrankaKinematics>(
          plant_franka, franka_context.get(), plant_object, object_context.get(),
          controller_params.end_effector_name, "pancake",
          controller_params.include_end_effector_orientation);

  auto radio_sub =
        builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
            lcm_channel_params.radio_channel, &lcm));

  auto ic3_timing_system = builder.AddSystem<iC3TimingSystem>(
            ic3_options, controller_params);

  auto x_desired_source =
    builder.AddSystem<drake::systems::ConstantVectorSource<double>>(xd);    

  auto controller =
      builder.AddSystem<systems::iC3HybridMpcTrackingController>
          (plant_for_lcs, lcs_factory, hybrid_mpc_options, solver_options, ic3_options, 0,
            A_x_mpc, lb_x_mpc, ub_x_mpc, A_u, lb_u, ub_u);

  auto mpc_trajectory_generator =
      builder.AddSystem<MPCTrajectoryGenerator>(plant_for_lcs, &plant_lcs_context, lcs_factory, ic3_options, 
        hybrid_mpc_options.lcs_factory_options, controller_params.track_dynamically_feasible, 0,
        A_x, lb_x, ub_x, A_u, lb_u, ub_u); 

  mpc_trajectory_generator->SetPublishEndEffectorOrientation(true);
  
  auto timed_gate =
      builder.AddSystem<TimedGate>(ic3_options, 0);    

  builder.Connect(object_state_sub->get_output_port(),
                  object_state_receiver->get_input_port());

  builder.Connect(franka_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_franka_state());  
  builder.Connect(object_state_receiver->get_output_port(),
                  perception_noise_filter->get_input_port_object_state());
  builder.Connect(perception_noise_filter->get_output_port_object_state(),
                  reduced_order_model_receiver->get_input_port_object_state());   
  builder.Connect(perception_noise_filter->get_output_port_object_state(),
                  noisy_object_state_vector->get_input_port_state());
  builder.Connect(noisy_object_state_vector->get_output_port_state(),
                  noisy_object_state_sender->get_input_port());                     
  builder.Connect(noisy_object_state_sender->get_output_port(),
                  noisy_object_state_publisher->get_input_port());         

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
  shared_diagram->set_name(("franka_plate_hybrid_mpc_controller"));
  DrawAndSaveDiagramGraph(*shared_diagram);

  std::cout << "Before lcm driven loop" << std::endl;
  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, franka_state_receiver,
      lcm_channel_params.franka_state_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());

  LcmHandleSubscriptionsUntil(
      &lcm, [&]() { return ic3_x_trajectory_sub->GetInternalMessageCount() > 1 
                            && object_state_sub->GetInternalMessageCount() > 1; });
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
