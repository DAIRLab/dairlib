
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
#include "examples/Cassie/kinematic_centroidal_mpc/cassie_reference_utils.h"
#include "examples/Cassie/kinematic_centroidal_mpc/cassie_kinematic_centroidal_mpc.h"
#include "systems/controllers/kinematic_centroidal_mpc/kinematic_centroidal_gains.h"
#include "systems/controllers/kinematic_centroidal_mpc/kcmpc_reference_generator.h"

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::geometry::DrakeVisualizer;

void DoMain(int n_knot_points, double duration, double base_height, double stance_width, double vel, double tol){
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

  CassieKinematicCentroidalMPC mpc (plant, n_knot_points, duration/(n_knot_points-1), 0.4);
  mpc.SetGains(gains);

  Eigen::VectorXd reference_state = GenerateNominalStand(mpc.Plant(), base_height, stance_width);
  Gait stand;
  stand.period = 1;
  stand.gait_pattern = {{0, 1, drake::Vector<bool, 4>(true, true, true, true)}};

  Gait walk;
  walk.period = 1;
  walk.gait_pattern = {{0, 0.4, drake::Vector<bool, 4>(true, true, false, false)},
                        {0.4, 0.5, drake::Vector<bool, 4>(true, true, true, true)},
                        {0.5, 0.9, drake::Vector<bool, 4>(false, false, true, true)},
                        {0.9, 1.0, drake::Vector<bool, 4>(true, true, true, true)}};

  KcmpcReferenceGenerator generator(plant, reference_state.head(plant.num_positions()), mpc.CreateContactPoints(plant,0));
  std::vector<double> time_points = {0, 0.5, duration};
  std::vector<Eigen::Vector3d> com_vel = {{{0, 0, 0},
                                           {vel, 0, 0},
                                           {vel, 0, 0}}};

  std::vector<Gait> gait_samples = {stand, walk, walk};

  generator.SetComKnotPoints({time_points, com_vel});
  generator.SetGaitSequence({time_points, gait_samples});
  generator.Build();

  mpc.AddForceTrackingReference(std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(generator.grf_traj_));
  mpc.AddGenPosReference(std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(generator.q_trajectory_));
  mpc.AddGenVelReference(std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(generator.v_trajectory_));
  mpc.AddComReference(std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(generator.com_trajectory_));
  mpc.AddContactTrackingReference(std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(generator.contact_traj_));
  mpc.AddConstantMomentumReference(Eigen::VectorXd::Zero(6));
  mpc.SetModeSequence(generator.contact_sequence_);

  mpc.AddInitialStateConstraint(reference_state);
  mpc.SetRobotStateGuess(generator.q_trajectory_, generator.v_trajectory_);
  mpc.SetComPositionGuess(generator.com_trajectory_);
  mpc.SetContactGuess(generator.contact_traj_);
  mpc.SetForceGuess(generator.grf_traj_);

  {
    drake::solvers::SolverOptions options;
    auto id = drake::solvers::IpoptSolver::id();
    options.SetOption(id, "tol", tol);
    options.SetOption(id, "dual_inf_tol", tol);
    options.SetOption(id, "constr_viol_tol", tol);
    options.SetOption(id, "compl_inf_tol", tol);
    options.SetOption(id, "max_iter", 200);
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
    options.SetOption(id, "acceptable_iter", 1);

    options.SetOption(id, "warm_start_init_point", "no");
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
    simulator.set_target_realtime_rate(1.0);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
  }
}

int main(int argc, char* argv[]) {
  DoMain(36, 3, 1.2, 0.2, 0.5, 1e-2);
}
