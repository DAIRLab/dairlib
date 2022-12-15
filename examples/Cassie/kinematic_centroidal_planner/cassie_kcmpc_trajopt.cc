#include <unistd.h>

#include <iostream>

#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/scene_graph.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/solve.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/trajectory_source.h>
#include <drake/systems/rendering/multibody_position_to_geometry_pose.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/Cassie/kinematic_centroidal_planner/cassie_kc_utils.h"
#include "examples/Cassie/kinematic_centroidal_planner/cassie_kinematic_centroidal_solver.h"
#include "examples/Cassie/kinematic_centroidal_planner/trajectory_parameters.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/trajectory_optimization/kinematic_centroidal_planner/kcmpc_reference_generator.h"
#include "systems/trajectory_optimization/kinematic_centroidal_planner/kinematic_centroidal_gains.h"
#include "systems/trajectory_optimization/kinematic_centroidal_planner/reference_generation_utils.h"

using drake::geometry::DrakeVisualizer;
using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

DEFINE_string(channel_reference, "KCMPC_REF",
              "The name of the channel where the reference trajectories from "
              "MPC are published");
DEFINE_string(
    trajectory_parameters,
    "examples/Cassie/kinematic_centroidal_planner/motions/motion_walk.yaml",
    "YAML file that contains trajectory parameters such as speed, tolerance, "
    "target_com_height");
DEFINE_string(planner_parameters,
              "examples/Cassie/kinematic_centroidal_planner/"
              "kinematic_centroidal_mpc_gains.yaml",
              "planner parameters containing initial states and other "
              "regularization parameters");
DEFINE_double(knot_points_to_publish, 10, "Number of knot points to publish");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  auto traj_params = drake::yaml::LoadYamlFile<TrajectoryParameters>(
      FLAGS_trajectory_parameters);
  auto gains = drake::yaml::LoadYamlFile<KinematicCentroidalGains>(
      FLAGS_planner_parameters);
  // Create fix-spring Cassie MBP
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant(0.0);
  MultibodyPlant<double> plant_vis(0.0);

  Parser parser(&plant);
  Parser parser_vis(&plant_vis, &scene_graph);

  std::string full_name = dairlib::FindResourceOrThrow(
      "examples/Cassie/urdf/cassie_fixed_springs_conservative.urdf");
  parser.AddModelFromFile(full_name);
  parser_vis.AddModelFromFile(full_name);
  plant.Finalize();
  plant_vis.Finalize();

  auto plant_context = plant.CreateDefaultContext();

  // Create gaits
  auto stand = drake::yaml::LoadYamlFile<Gait>(
      "examples/Cassie/kinematic_centroidal_planner/gaits/stand.yaml");
  auto walk = drake::yaml::LoadYamlFile<Gait>(
      "examples/Cassie/kinematic_centroidal_planner/gaits/walk.yaml");
  auto right_step = drake::yaml::LoadYamlFile<Gait>(
      "examples/Cassie/kinematic_centroidal_planner/gaits/right_step.yaml");
  auto left_step = drake::yaml::LoadYamlFile<Gait>(
      "examples/Cassie/kinematic_centroidal_planner/gaits/left_step.yaml");
  auto jump = drake::yaml::LoadYamlFile<Gait>(
      "examples/Cassie/kinematic_centroidal_planner/gaits/jump.yaml");

  std::unordered_map<std::string, Gait> gait_library;
  gait_library["stand"] = stand;
  gait_library["walk"] = walk;
  gait_library["right_step"] = right_step;
  gait_library["left_step"] = left_step;
  gait_library["jump"] = jump;
  // Create reference
  std::vector<Gait> gait_samples;
  for (auto gait : traj_params.gait_sequence) {
    gait_samples.push_back(gait_library.at(gait));
  }
  DRAKE_DEMAND(gait_samples.size() == traj_params.duration_scaling.size());
  std::vector<double> time_points = KcmpcReferenceGenerator::GenerateTimePoints(
      traj_params.duration_scaling, gait_samples);

  Eigen::VectorXd reference_state = GenerateNominalStand(
      plant, traj_params.target_com_height, traj_params.stance_width, false);

  // Create MPC and set gains
  CassieKinematicCentroidalSolver mpc(
      plant, traj_params.n_knot_points,
      time_points.back() / (traj_params.n_knot_points - 1), 0.4,
      reference_state.head(plant.num_positions()), traj_params.spring_constant,
      traj_params.damping_coefficient,
      sqrt(pow(traj_params.target_com_height, 2) +
           pow(traj_params.stance_width, 2)),
      traj_params.stance_width);
  mpc.SetGains(gains);
  mpc.SetMinimumFootClearance(traj_params.swing_foot_minimum_height);
  mpc.SetMaximumSlipLegLength(traj_params.max_slip_leg_length);

  mpc.SetComplexitySchedule(GenerateComplexitySchedule(
      traj_params.n_knot_points, traj_params.complexity_schedule));

  KcmpcReferenceGenerator generator(plant, plant_context.get(),
                                    CreateContactPoints(plant, 0));

  generator.SetNominalReferenceConfiguration(
      reference_state.head(plant.num_positions()));
  generator.SetComKnotPoints({time_points, traj_params.com_vel_vector});
  generator.SetGaitSequence({time_points, gait_samples});
  generator.Generate();

  // Add reference and mode sequence
  mpc.SetForceTrackingReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          generator.grf_traj_));
  mpc.SetGenPosReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          generator.q_trajectory_));
  mpc.SetGenVelReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          generator.v_trajectory_));
  mpc.SetComReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          generator.com_trajectory_));
  mpc.SetContactTrackingReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          generator.contact_traj_));
  mpc.SetMomentumReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          generator.momentum_trajectory_));
  mpc.SetModeSequence(generator.contact_sequence_);
  //
  //  // Set initial guess
  mpc.AddInitialStateConstraint(reference_state);
  mpc.SetRobotStateGuess(generator.q_trajectory_, generator.v_trajectory_);
  mpc.SetComPositionGuess(generator.com_trajectory_);
  mpc.SetContactGuess(generator.contact_traj_);
  mpc.SetForceGuess(generator.grf_traj_);
  mpc.SetMomentumGuess(generator.momentum_trajectory_);

  {
    drake::solvers::SolverOptions options;
    auto id = drake::solvers::IpoptSolver::id();
    options.SetOption(id, "tol", gains.tol);
    options.SetOption(id, "dual_inf_tol", gains.tol);
    options.SetOption(id, "constr_viol_tol", gains.tol);
    options.SetOption(id, "compl_inf_tol", gains.tol);
    options.SetOption(id, "max_iter", 800);
    options.SetOption(id, "nlp_lower_bound_inf", -1e6);
    options.SetOption(id, "nlp_upper_bound_inf", 1e6);
    options.SetOption(id, "print_timing_statistics", "yes");
    options.SetOption(id, "print_level", 5);

    // Set to ignore overall tolerance/dual infeasibility, but terminate when
    // primal feasible and objective fails to increase over 5 iterations.
    options.SetOption(id, "acceptable_compl_inf_tol", gains.tol);
    options.SetOption(id, "acceptable_constr_viol_tol", gains.tol);
    options.SetOption(id, "acceptable_obj_change_tol", 1e-3);
    options.SetOption(id, "acceptable_tol", 1e2);
    options.SetOption(id, "acceptable_iter", 1);

    options.SetOption(id, "warm_start_init_point", "no");
    mpc.Build(options);
  }

  std::cout << "Solving optimization\n\n";
  auto pp_xtraj = mpc.Solve();

  auto lcm_traj = mpc.GenerateLcmTraj(FLAGS_knot_points_to_publish);
  auto lcm = std::make_unique<drake::lcm::DrakeLcm>();
  dairlib::lcmt_timestamped_saved_traj timestamped_msg;
  timestamped_msg.saved_traj = lcm_traj;
  timestamped_msg.utime = 1;
  drake::lcm::Publish(lcm.get(), FLAGS_channel_reference, timestamped_msg);
  try {
    mpc.SaveSolutionToFile("examples/Cassie/saved_trajectories/kcmpc_solution");
  } catch (...) {
    std::cout << "Unable to save trajectory, try running binary manually "
                 "rather than using bazel run"
              << std::endl;
  }

  auto traj_source =
      builder.AddSystem<drake::systems::TrajectorySource>(pp_xtraj);
  auto passthrough = builder.AddSystem<dairlib::systems::SubvectorPassThrough>(
      mpc.Plant().num_positions() + mpc.Plant().num_velocities(), 0,
      mpc.Plant().num_positions());
  builder.Connect(traj_source->get_output_port(),
                  passthrough->get_input_port());
  auto to_pose = builder.AddSystem<
      drake::systems::rendering::MultibodyPositionToGeometryPose<double>>(
      plant_vis);
  builder.Connect(passthrough->get_output_port(), to_pose->get_input_port());

  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant_vis.get_source_id().value()));

  DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph);
  auto diagram = builder.Build();

  while (true) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(0.25);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
    sleep(2);
  }
}

int main(int argc, char* argv[]) { DoMain(argc, argv); }
