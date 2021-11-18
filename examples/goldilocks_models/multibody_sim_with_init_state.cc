// This is modified from exmaples/Cassie/multibody_sim

#include <memory>
#include <iostream>
#include <memory>
#include <thread>

#include <gflags/gflags.h>

#include "common/file_utils.h"
#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_solvers.h"
#include "multibody/multibody_utils.h"
#include "multibody/multibody_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {
using dairlib::systems::SubvectorPassThrough;
using drake::geometry::SceneGraph;
using drake::multibody::ContactResultsToLcmSystem;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using drake::math::RotationMatrix;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

// Optimal ROM controller
DEFINE_int32(pause_second, 0, "pause after initialization");

DEFINE_bool(start_with_right_stance, false, "");
DEFINE_bool(simulate_about_one_step_time, false,
            "if true, we run the simulation for about one foot step");

// Simulation parameters.
DEFINE_bool(floating_base, true, "Fixed or floating base model");

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_bool(time_stepping, true,
            "If 'true', the plant is modeled as a "
            "discrete system with periodic updates. "
            "If 'false', the plant is modeled as a continuous system.");
DEFINE_double(dt, 8e-5,
              "The step size to use for time_stepping, ignored for continuous");
DEFINE_double(v_stiction, 1e-3, "Stiction tolernace (m/s)");
DEFINE_double(penetration_allowance, 1e-5,
              "Penetration allowance for the contact model. Nearly equivalent"
              " to (m)");
DEFINE_double(end_time, std::numeric_limits<double>::infinity(),
              "End time for simulator");
DEFINE_double(publish_rate, 1000, "Publish rate for simulator");
DEFINE_double(init_height, .7,
              "Initial starting height of the pelvis above "
              "ground");
DEFINE_bool(spring_model, true, "Use a URDF with or without legs springs");

// Solve for the configuration which respects the fourbar linkage constraint and
// joint limit constraints.
// The reason why we do this here is because the planner's constraint on Cassie
// might be too loose
VectorXd SolveForFeasibleConfiguration(
    const VectorXd& q_goal,
    const drake::multibody::MultibodyPlant<double>& plant,
    bool left_stance = true) {
  multibody::KinematicEvaluatorSet<double> evaluators(plant);
  auto left_loop = LeftLoopClosureEvaluator(plant);
  auto right_loop = RightLoopClosureEvaluator(plant);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);
  auto left_toe = LeftToeFront(plant);
  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant, left_toe.first, left_toe.second, Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d::Zero(), false);
  auto left_heel = LeftToeRear(plant);
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant, left_heel.first, left_heel.second, Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d::Zero(), false);
  auto right_toe = RightToeFront(plant);
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant, right_toe.first, right_toe.second, Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d::Zero(), false);
  auto right_heel = RightToeRear(plant);
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant, right_heel.first, right_heel.second, Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d::Zero(), false);
  if (left_stance) {
    evaluators.add_evaluator(&left_toe_evaluator);
    evaluators.add_evaluator(&left_heel_evaluator);
  } else {
    evaluators.add_evaluator(&right_toe_evaluator);
    evaluators.add_evaluator(&right_heel_evaluator);
  }

  auto program = multibody::MultibodyProgram(plant);
  auto q = program.AddPositionVariables();
  auto kinematic_constraint = program.AddKinematicConstraint(evaluators, q);
  program.AddJointLimitConstraints(q);

  // Soft constraint on the joint positions
  int n_q = plant.num_positions();
  program.AddQuadraticErrorCost(Eigen::MatrixXd::Identity(n_q, n_q), q_goal, q);

  program.SetInitialGuess(q, q_goal);

  std::cout << "Solving...\n";
  std::cout << "Choose the best solver: "
            << drake::solvers::ChooseBestSolver(program).name() << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  const auto result = drake::solvers::Solve(program, program.initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  auto q_sol = result.GetSolution(q);
  std::cout << to_string(result.get_solution_result()) << std::endl;
  std::cout << "Solve time:" << elapsed.count() << std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() << std::endl;
  std::cout << "q sol = " << q_sol.transpose() << "\n\n";

  return q_sol;
}

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Construct plant without springs
  drake::multibody::MultibodyPlant<double> plant_wo_springs(0.0);
  addCassieMultibody(
      &plant_wo_springs, nullptr, FLAGS_floating_base /*floating base*/,
      "examples/Cassie/urdf/cassie_fixed_springs.urdf", false, true);
  plant_wo_springs.Finalize();

  std::string dir = "../dairlib_data/goldilocks_models/planning/robot_1/data/";

  // Read in the initial state of Cassie from the ROM planner
  VectorXd init_robot_state = readCSV(dir + "x0_each_mode.csv").col(0);
  if (FLAGS_start_with_right_stance) {
    using namespace goldilocks_models;
    int robot_option = 1;  // Cassie
    StateMirror state_mirror(
        MirrorPosIndexMap(plant_wo_springs, robot_option),
        MirrorPosSignChangeSet(plant_wo_springs, robot_option),
        MirrorVelIndexMap(plant_wo_springs, robot_option),
        MirrorVelSignChangeSet(plant_wo_springs, robot_option));
    std::cout << "before mirroring, init_robot_state = "
              << init_robot_state.transpose() << std::endl;
    init_robot_state << state_mirror.MirrorPos(
        init_robot_state.head(plant_wo_springs.num_positions())),
        state_mirror.MirrorVel(
            init_robot_state.tail(plant_wo_springs.num_velocities()));
    std::cout << "after mirroring, init_robot_state = "
              << init_robot_state.transpose() << std::endl;
  }

  // Read in the end time of the trajectory
  int knots_per_foot_step = readCSV(dir + "nodes_per_step.csv")(0, 0);
  double end_time_of_first_step =
      readCSV(dir + "time_at_knots.csv")(knots_per_foot_step, 0);
  std::cout << "knots_per_foot_step = " << knots_per_foot_step << std::endl;
  std::cout << "end_time_of_first_step = " << end_time_of_first_step
            << std::endl;

  // Plant/System initialization
  DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double time_step = FLAGS_time_stepping ? FLAGS_dt : 0.0;
  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>(time_step);
  if (FLAGS_floating_base) {
    multibody::addFlatTerrain(&plant, &scene_graph, .8, .8);
  }

  std::string urdf;
  if (FLAGS_spring_model) {
    urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  } else {
    urdf = "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  }

  addCassieMultibody(&plant, &scene_graph, FLAGS_floating_base, urdf,
                     FLAGS_spring_model, true);
  plant.Finalize();

  plant.set_penetration_allowance(FLAGS_penetration_allowance);
  plant.set_stiction_tolerance(FLAGS_v_stiction);

  // Create lcm systems.
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
  auto input_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
          "CASSIE_INPUT", lcm));
  auto input_receiver = builder.AddSystem<systems::RobotInputReceiver>(plant);
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      input_receiver->get_output_port(0).size(), 0,
      plant.get_actuation_input_port().size());
  auto state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          "CASSIE_STATE_SIMULATION", lcm, 1.0 / FLAGS_publish_rate));
  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(plant);

  // Contact Information
  ContactResultsToLcmSystem<double>& contact_viz =
      *builder.template AddSystem<ContactResultsToLcmSystem<double>>(plant);
  contact_viz.set_name("contact_visualization");
  auto& contact_results_publisher = *builder.AddSystem(
      LcmPublisherSystem::Make<drake::lcmt_contact_results_for_viz>(
          "CASSIE_CONTACT_DRAKE", lcm, 1.0 / FLAGS_publish_rate));
  contact_results_publisher.set_name("contact_results_publisher");

  // Sensor aggregator and publisher of lcmt_cassie_out
  const auto& sensor_aggregator =
      AddImuAndAggregator(&builder, plant, passthrough->get_output_port());
  auto sensor_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_cassie_out>(
          "CASSIE_OUTPUT", lcm, 1.0 / FLAGS_publish_rate));

  // connect leaf systems
  builder.Connect(*input_sub, *input_receiver);
  builder.Connect(*input_receiver, *passthrough);
  builder.Connect(passthrough->get_output_port(),
                  plant.get_actuation_input_port());
  builder.Connect(plant.get_state_output_port(),
                  state_sender->get_input_port_state());
  builder.Connect(*state_sender, *state_pub);
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());
  builder.Connect(plant.get_contact_results_output_port(),
                  contact_viz.get_input_port(0));
  builder.Connect(contact_viz.get_output_port(0),
                  contact_results_publisher.get_input_port());
  builder.Connect(sensor_aggregator.get_output_port(0),
                  sensor_pub->get_input_port());

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram_context->EnableCaching();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set initial conditions of the simulation
  /*VectorXd q_init, u_init, lambda_init;
  double mu_fp = 0;
  double min_normal_fp = 70;
  double toe_spread = .2;
  // Create a plant for CassieFixedPointSolver.
  // Note that we cannot use the plant from the above diagram, because after the
  // diagram is built, plant.get_actuation_input_port().HasValue(*context)
  // throws a segfault error
  drake::multibody::MultibodyPlant<double> plant_for_solver(0.0);
  addCassieMultibody(&plant_for_solver, nullptr,
                     FLAGS_floating_base, urdf,
                     FLAGS_spring_model, true);
  plant_for_solver.Finalize();
  if (FLAGS_floating_base) {
    CassieFixedPointSolver(plant_for_solver, FLAGS_init_height, mu_fp,
                           min_normal_fp, true, toe_spread, &q_init, &u_init,
                           &lambda_init);
  } else {
    CassieFixedBaseFixedPointSolver(plant_for_solver, &q_init, &u_init,
                                    &lambda_init);
  }
  plant.SetPositions(&plant_context, q_init);
  plant.SetVelocities(&plant_context, VectorXd::Zero(plant.num_velocities()));*/

  // Map state of plant_wo_springs to plant_w_springs
  Eigen::MatrixXd map_position_from_spring_to_no_spring =
      multibody::CreateWithSpringsToWithoutSpringsMapPos(plant,
                                                            plant_wo_springs);
  Eigen::MatrixXd map_velocity_from_spring_to_no_spring =
      multibody::CreateWithSpringsToWithoutSpringsMapVel(plant,
                                                            plant_wo_springs);
  VectorXd init_robot_state_w_springs(plant.num_positions() +
                                      plant.num_velocities());
  init_robot_state_w_springs
      << map_position_from_spring_to_no_spring.transpose() *
             init_robot_state.head(plant_wo_springs.num_positions()),
      map_velocity_from_spring_to_no_spring.transpose() *
          init_robot_state.tail(plant_wo_springs.num_velocities());
  init_robot_state_w_springs.head(plant.num_positions()) =
      SolveForFeasibleConfiguration(
          init_robot_state_w_springs.head(plant.num_positions()), plant);

  std::cout << "init_robot_state (from planner) = "
            << init_robot_state.transpose() << std::endl;
  std::cout << "init_robot_state_w_springs = "
            << init_robot_state_w_springs.transpose() << std::endl;
  plant.SetPositionsAndVelocities(&plant_context, init_robot_state_w_springs);

  Simulator<double> simulator(*diagram, std::move(diagram_context));

  if (!FLAGS_time_stepping) {
    // simulator.get_mutable_integrator()->set_maximum_step_size(0.01);
    // simulator.get_mutable_integrator()->set_target_accuracy(1e-1);
    // simulator.get_mutable_integrator()->set_fixed_step_mode(true);
    simulator.reset_integrator<drake::systems::RungeKutta2Integrator<double>>(
        FLAGS_dt);
  }

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();

  // pause a second for the planner to plan
  std::this_thread::sleep_for(std::chrono::seconds(FLAGS_pause_second));

  simulator.AdvanceTo(FLAGS_simulate_about_one_step_time
                          ? end_time_of_first_step
                          : std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
