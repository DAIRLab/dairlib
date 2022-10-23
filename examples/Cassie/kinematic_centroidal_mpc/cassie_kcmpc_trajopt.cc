
#include <drake/systems/framework/diagram_builder.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <iostream>
#include <drake/systems/primitives/trajectory_source.h>
#include <drake/systems/rendering/multibody_position_to_geometry_pose.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/solvers/solve.h>
#include <drake/common/yaml/yaml_io.h>
#include "common/find_resource.h"
#include "systems/primitives/subvector_pass_through.h"
#include "examples/Cassie/kinematic_centroidal_mpc/reference_generator.h"
#include "examples/Cassie/kinematic_centroidal_mpc/cassie_kinematic_centroidal_mpc.h"
#include "systems/controllers/kinematic_centroidal_mpc/kinematic_centroidal_gains.h"

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::geometry::DrakeVisualizer;

void DoMain(int n_knot_points, double duration, double com_height, double stance_width, double squat_distance, double tol){
  auto gains = drake::yaml::LoadYamlFile<KinematicCentroidalGains>("examples/Cassie/kinematic_centroidal_mpc/kinematic_centroidal_mpc_gains.yaml");
  // Create fix-spring Cassie MBP
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant(0.0);
  MultibodyPlant<double> plant_vis(0.0);

  Parser parser(&plant);
  Parser parser_vis(&plant_vis, &scene_graph);

  std::string full_name =
      dairlib::FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
  parser.AddModelFromFile(full_name);
  parser_vis.AddModelFromFile(full_name);
  plant.Finalize();
  plant_vis.Finalize();

  std::cout<<"Creating MPC"<<std::endl;
  CassieKinematicCentroidalMPC mpc (plant, n_knot_points, duration/(n_knot_points-1));

  std::cout<<"Setting initial guess"<<std::endl;
  mpc.SetZeroInitialGuess();
  Eigen::VectorXd reference_state = GenerateNominalStand(mpc.Plant(), 1.9, stance_width);
  mpc.SetRobotStateGuess(reference_state);

  double stance_wiggle = 0.01;

  Eigen::Vector3d left_lb(std::numeric_limits<double>::lowest(), stance_width/2-stance_wiggle, std::numeric_limits<double>::lowest());
  Eigen::Vector3d left_ub(std::numeric_limits<double>::max(), stance_width/2+stance_wiggle, std::numeric_limits<double>::max());

  Eigen::Vector3d right_lb(std::numeric_limits<double>::lowest(), -stance_width/2-stance_wiggle, std::numeric_limits<double>::lowest());
  Eigen::Vector3d right_ub(std::numeric_limits<double>::max(), -stance_width/2+stance_wiggle, std::numeric_limits<double>::max());
  mpc.AddContactPointPositionConstraint(0, left_lb, left_ub);
  mpc.AddContactPointPositionConstraint(1, left_lb, left_ub);
  mpc.AddContactPointPositionConstraint(2, right_lb, right_ub);
  mpc.AddContactPointPositionConstraint(3, right_lb, right_ub);

  Eigen::VectorXd ref_force = Eigen::VectorXd::Zero(12);
  ref_force[2] = 33*9.81/4;
  ref_force[5] = 33*9.81/4;
  ref_force[8] = 33*9.81/4;
  ref_force[11] = 33*9.81/4;
  mpc.AddConstantForceTrackingReference(ref_force);
  mpc.AddConstantStateReference(reference_state);

  Eigen::VectorXd reference_com = Eigen::VectorXd::Zero(3);
  reference_com[2] = com_height;
  Eigen::VectorXd reference_com_bottom = Eigen::VectorXd::Zero(3);
  reference_com_bottom[2] = com_height-squat_distance;
  std::vector<double> time_points = {0, duration};
  auto com_reference = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(time_points,
                                                                                               {reference_com,
                                                                                                reference_com_bottom});
  mpc.AddComReference(std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(com_reference));

  mpc.AddContactTrackingReference(std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(Eigen::VectorXd::Zero(
      4 * 6)));
  mpc.AddConstantMomentumReference(Eigen::VectorXd::Zero(6));


  mpc.AddComHeightBoundingConstraint(0.1,2);
  mpc.SetComPositionGuess({0, 0, com_height});

  std::vector<std::vector<bool>> mode_sequence(n_knot_points);
  for(int i = 0; i < n_knot_points/4; i++){
    mode_sequence[i] = {true, true, true, true};
  }
  for(int i = n_knot_points/4; i < 2 * n_knot_points/4; i++){
    mode_sequence[i] = {true, true, false, false};
  }
  for(int i = 2 * n_knot_points/4; i < 3 * n_knot_points/4; i++){
    mode_sequence[i] = {false, false, true, true};
  }
  for(int i = 3 * n_knot_points/4; i < n_knot_points; i++){
    mode_sequence[i] = {true, true, true, true};
  }

  mpc.SetModeSequence(mode_sequence);
  mpc.AddInitialStateConstraint(reference_state);
  mpc.SetGains(gains);
  std::cout<<"Adding solver options"<<std::endl;
  {
    drake::solvers::SolverOptions options;
    auto id = drake::solvers::IpoptSolver::id();
    options.SetOption(id, "tol", tol);
    options.SetOption(id, "dual_inf_tol", tol);
    options.SetOption(id, "constr_viol_tol", tol);
    options.SetOption(id, "compl_inf_tol", tol);
    options.SetOption(id, "max_iter", 2000);
    options.SetOption(id, "nlp_lower_bound_inf", -1e6);
    options.SetOption(id, "nlp_upper_bound_inf", 1e6);
    options.SetOption(id, "print_timing_statistics", "yes");
    options.SetOption(id, "print_level", 5);

    // Set to ignore overall tolerance/dual infeasibility, but terminate when
    // primal feasible and objective fails to increase over 5 iterations.
    options.SetOption(id, "acceptable_compl_inf_tol", tol);
    options.SetOption(id, "acceptable_constr_viol_tol", tol);
    options.SetOption(id, "acceptable_obj_change_tol", 1e-3);
    options.SetOption(id, "acceptable_tol", 1e2);
    options.SetOption(id, "acceptable_iter", 5);
    mpc.Build(options);
  }

  std::cout<<"Adding visualization callback"<<std::endl;
  double alpha = .2;
  mpc.CreateVisualizationCallback(
      "examples/Cassie/urdf/cassie_fixed_springs.urdf", alpha);

  std::cout << "Solving optimization\n\n";
  const auto pp_xtraj = mpc.Solve();
  mpc.SaveSolutionToFile(std::string(getenv("HOME")) + "/workspace/dairlib/examples/Cassie/saved_trajectories/kcmpc_solution");

  auto traj_source =
      builder.AddSystem<drake::systems::TrajectorySource>(pp_xtraj);
  auto passthrough = builder.AddSystem<dairlib::systems::SubvectorPassThrough>(
      mpc.Plant().num_positions() + mpc.Plant().num_velocities(), 0, mpc.Plant().num_positions());
  builder.Connect(traj_source->get_output_port(),
                  passthrough->get_input_port());
  auto to_pose =
      builder.AddSystem<drake::systems::rendering::MultibodyPositionToGeometryPose<double>>(plant_vis);
  builder.Connect(passthrough->get_output_port(), to_pose->get_input_port());

  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant_vis.get_source_id().value()));

  DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph);
  auto diagram = builder.Build();

  while (true) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(.5);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
  }
}

int main(int argc, char* argv[]) {
  // Assuming 2 cycles per second
  DoMain(20, 2, 0.95, 0.2, 0.0, 1e-3);
}
