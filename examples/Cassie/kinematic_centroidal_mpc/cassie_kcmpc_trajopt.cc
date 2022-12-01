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
#include "examples/Cassie/kinematic_centroidal_mpc/cassie_kinematic_centroidal_mpc.h"
#include "examples/Cassie/kinematic_centroidal_mpc/cassie_reference_utils.h"
#include "examples/Cassie/kinematic_centroidal_mpc/trajectory_parameters.h"
#include "systems/controllers/kinematic_centroidal_mpc/kcmpc_reference_generator.h"
#include "systems/controllers/kinematic_centroidal_mpc/kinematic_centroidal_gains.h"
#include "systems/primitives/subvector_pass_through.h"

using drake::geometry::DrakeVisualizer;
using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

DEFINE_string(channel_reference, "KCMPC_REF",
              "The name of the channel where the reference trajectories from "
              "MPC are published");
DEFINE_string(
    trajectory_parameters,
    "examples/Cassie/kinematic_centroidal_mpc/trajectory_parameters.yaml",
    "YAML file that contains trajectory parameters such as speed, tolerance, "
    "com_height");
DEFINE_double(knot_points_to_publish, 10, "Number of knot points to publish");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  auto traj_params = drake::yaml::LoadYamlFile<TrajectoryParameters>(
      FLAGS_trajectory_parameters);
  auto gains = drake::yaml::LoadYamlFile<KinematicCentroidalGains>(
      "examples/Cassie/kinematic_centroidal_mpc/"
      "kinematic_centroidal_mpc_gains.yaml");
  // Create fix-spring Cassie MBP
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant(0.0);
  MultibodyPlant<double> plant_vis(0.0);

  Parser parser(&plant);
  Parser parser_vis(&plant_vis, &scene_graph);

  std::string full_name = dairlib::FindResourceOrThrow(
      "examples/Cassie/urdf/cassie_fixed_springs.urdf");
  parser.AddModelFromFile(full_name);
  parser_vis.AddModelFromFile(full_name);
  plant.Finalize();
  plant_vis.Finalize();

  // Create gaits
  auto stand = drake::yaml::LoadYamlFile<Gait>(
      "examples/Cassie/kinematic_centroidal_mpc/gaits/stand.yaml");
  auto walk = drake::yaml::LoadYamlFile<Gait>(
      "examples/Cassie/kinematic_centroidal_mpc/gaits/walk.yaml");
  auto step = drake::yaml::LoadYamlFile<Gait>(
      "examples/Cassie/kinematic_centroidal_mpc/gaits/step.yaml");

  // Create reference
  // TODO(yangwill): move this into the reference generator
  // Specify knot points
  std::vector<Gait> gait_samples = {stand, walk, stand};
  DRAKE_DEMAND(gait_samples.size() == traj_params.duration_scaling.size());
  std::vector<double> durations = std::vector<double>(gait_samples.size());
  for (int i = 0; i < gait_samples.size(); ++i) {
    durations[i] = traj_params.duration_scaling[i] * gait_samples[i].period;
  }
  std::vector<double> time_points;
  double start_time = 0;
  time_points.push_back(start_time);
  for (auto duration : durations) {
    time_points.push_back(time_points.back() + duration);
  }

  Eigen::VectorXd reference_state = GenerateNominalStand(
      plant, traj_params.com_height, traj_params.stance_width, false);

  // Create MPC and set gains
  CassieKinematicCentroidalMPC mpc(
      plant, traj_params.n_knot_points,
      time_points.back() / (traj_params.n_knot_points - 1), 0.4, reference_state.head(plant.num_positions()), 6000, traj_params.com_height, traj_params.stance_width);
  mpc.SetGains(gains);

  std::vector<Complexity> complexity_schedule(traj_params.n_knot_points);
  std::fill(complexity_schedule.begin(), complexity_schedule.end(),Complexity::KINEMATIC_CENTROIDAL);
  for(int i = 15; i <35 ; i++){
    complexity_schedule[i] = Complexity::PLANAR_SLIP;
  }
  mpc.SetComplexitySchedule(complexity_schedule);

  KcmpcReferenceGenerator generator(plant,
                                    reference_state.head(plant.num_positions()),
                                    mpc.CreateContactPoints(plant, 0));

  std::vector<Eigen::Vector3d> com_vel = {
      {{0, 0, 0}, {traj_params.vel, 0, 0}, {0, 0, 0}}};
  generator.SetComKnotPoints({time_points, com_vel});
  generator.SetGaitSequence({time_points, gait_samples});
  generator.Build();

  // Add reference and mode sequence
  mpc.AddForceTrackingReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          generator.grf_traj_));
  mpc.AddGenPosReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          generator.q_trajectory_));
  mpc.AddGenVelReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          generator.v_trajectory_));
  mpc.AddComReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          generator.com_trajectory_));
  mpc.AddContactTrackingReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          generator.contact_traj_));
  mpc.AddMomentumReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          generator.momentum_trajectory_));
  mpc.SetModeSequence(generator.contact_sequence_);

  // Set initial guess
  mpc.AddInitialStateConstraint(reference_state);
  mpc.SetRobotStateGuess(generator.q_trajectory_, generator.v_trajectory_);
  mpc.SetComPositionGuess(generator.com_trajectory_);
  mpc.SetContactGuess(generator.contact_traj_);
  mpc.SetForceGuess(generator.grf_traj_);
  mpc.SetMomentumGuess(generator.momentum_trajectory_);

  {
    drake::solvers::SolverOptions options;
    auto id = drake::solvers::IpoptSolver::id();
    options.SetOption(id, "tol", traj_params.tol);
    options.SetOption(id, "dual_inf_tol", traj_params.tol);
    options.SetOption(id, "constr_viol_tol", traj_params.tol);
    options.SetOption(id, "compl_inf_tol", traj_params.tol);
    options.SetOption(id, "max_iter", 200);
    options.SetOption(id, "nlp_lower_bound_inf", -1e6);
    options.SetOption(id, "nlp_upper_bound_inf", 1e6);
    options.SetOption(id, "print_timing_statistics", "yes");
    options.SetOption(id, "print_level", 5);

    // Set to ignore overall tolerance/dual infeasibility, but terminate when
    // primal feasible and objective fails to increase over 5 iterations.
    options.SetOption(id, "acceptable_compl_inf_tol", traj_params.tol);
    options.SetOption(id, "acceptable_constr_viol_tol", traj_params.tol);
    options.SetOption(id, "acceptable_obj_change_tol", 1e-3);
    options.SetOption(id, "acceptable_tol", 1e2);
    options.SetOption(id, "acceptable_iter", 1);

    options.SetOption(id, "warm_start_init_point", "no");
    mpc.Build(options);
  }

    double alpha = .2;
    mpc.CreateVisualizationCallback(
        "examples/Cassie/urdf/cassie_fixed_springs.urdf", alpha);

  std::cout << "Solving optimization\n\n";
  const auto pp_xtraj = mpc.Solve();
  try {
    mpc.SaveSolutionToFile("examples/Cassie/saved_trajectories/kcmpc_solution");
  } catch (...) {
    std::cout << "Unable to save trajectory, try running binary manually "
                 "rather than using bazel run"
              << std::endl;
  }

  mpc.PublishSolution(FLAGS_channel_reference, FLAGS_knot_points_to_publish);

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
    simulator.set_target_realtime_rate(0.2);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
  }
}

int main(int argc, char* argv[]) {
  //  DoMain(45, 5, 0.8, 0.3, 0.5, 1e-2);
  //  DoMain(45, 0.8, 0.25, 0.0, 1e-2);
  DoMain(argc, argv);
}
