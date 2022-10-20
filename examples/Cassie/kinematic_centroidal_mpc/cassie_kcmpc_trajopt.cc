
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
#include "common/find_resource.h"
#include "systems/controllers/kinematic_centroidal_mpc/kinematic_centroidal_mpc.h"
#include "examples/Cassie/cassie_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "multibody/kinematic/kinematic_constraints.h"
#include "examples/Cassie/kinematic_centroidal_mpc/reference_generator.h"

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::geometry::DrakeVisualizer;

void DoMain(int n_knot_points, double duration, double com_height, double stance_width, double squat_distance, double tol){
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
  std::map<std::string, int> positions_map = dairlib::multibody::MakeNameToPositionsMap(plant);

  auto left_toe_pair = dairlib::LeftToeFront(plant);
  auto left_heel_pair = dairlib::LeftToeRear(plant);
  auto right_toe_pair = dairlib::RightToeFront(plant);
  auto right_heel_pair = dairlib::RightToeRear(plant);

  std::vector<int> toe_active_inds{0, 1, 2};
  std::vector<int> heel_active_inds{0, 1, 2};

  auto left_toe_eval = dairlib::multibody::WorldPointEvaluator<double>(
      plant, left_toe_pair.first, left_toe_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), toe_active_inds);

  auto left_heel_eval = dairlib::multibody::WorldPointEvaluator<double>(
      plant, left_heel_pair.first, left_heel_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), heel_active_inds);

  auto right_toe_eval = dairlib::multibody::WorldPointEvaluator<double>(
      plant, right_toe_pair.first, right_toe_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), toe_active_inds);

  auto right_heel_eval = dairlib::multibody::WorldPointEvaluator<double>(
      plant, right_heel_pair.first, right_heel_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), heel_active_inds);

  std::cout<<"Creating MPC"<<std::endl;
  KinematicCentroidalMPC mpc (plant, n_knot_points, duration/(n_knot_points-1),
                              {left_toe_eval, left_heel_eval, right_toe_eval, right_heel_eval});

  // create joint/motor names
  std::vector<std::pair<std::string, std::string>> l_r_pairs{
      std::pair<std::string, std::string>("_left", "_right"),
      std::pair<std::string, std::string>("_right", "_left"),
  };
  std::vector<std::string> asy_joint_names{
      "hip_roll",
      "hip_yaw",
  };
  std::vector<std::string> sym_joint_names{"hip_pitch", "knee", "ankle_joint", "toe"};
  std::vector<std::string> joint_names{};
  std::vector<std::string> motor_names{};
  for (auto &l_r_pair : l_r_pairs) {
    for (unsigned int i = 0; i < asy_joint_names.size(); i++) {
      joint_names.push_back(asy_joint_names[i] + l_r_pair.first);
      motor_names.push_back(asy_joint_names[i] + l_r_pair.first + "_motor");
    }
    for (unsigned int i = 0; i < sym_joint_names.size(); i++) {
      joint_names.push_back(sym_joint_names[i] + l_r_pair.first);
      if (sym_joint_names[i].compare("ankle_joint") != 0) {
        motor_names.push_back(sym_joint_names[i] + l_r_pair.first + "_motor");
      }
    }
  }
  mpc.AddPlantJointLimits(joint_names);

  auto l_loop_evaluator = dairlib::LeftLoopClosureEvaluator(plant);
  auto r_loop_evaluator = dairlib::RightLoopClosureEvaluator(plant);
  dairlib::multibody::KinematicEvaluatorSet<double> evaluators(plant);
  evaluators.add_evaluator(&l_loop_evaluator);
  evaluators.add_evaluator(&r_loop_evaluator);

  auto loop_closure =
      std::make_shared<dairlib::multibody::KinematicPositionConstraint<double>>(
          plant,
          evaluators,
          Eigen::VectorXd::Zero(2),
          Eigen::VectorXd::Zero(2));
  for(int knot_point = 0; knot_point < n_knot_points; knot_point ++){
    mpc.AddKinematicConstraint(loop_closure, mpc.state_vars(knot_point).head(plant.num_positions()));
  }

  std::cout<<"Setting initial guess"<<std::endl;
  mpc.SetZeroInitialGuess();
  Eigen::VectorXd reference_state = GenerateNominalStand(plant, 1.9, stance_width);
  mpc.SetRobotStateGuess(reference_state);

  double cost_force = 0.0001;

  double cost_joint_pos = 0.001;
  double cost_joint_vel = 0.002;

  double cost_contact_pos = 0.1;
  double cost_contact_vel = 0.2;

  double cost_com_pos = 20;

  double cost_base_vel = 0.1;
  double cost_orientation = 8;
  double cost_angular_vel = 0.01;

  double cost_lin_mom = 0.0001;
  double cost_ang_mom = 0.0001;

  double stance_wiggle = 0.01;

  Eigen::Vector3d left_lb(std::numeric_limits<double>::lowest(), stance_width/2-stance_wiggle, std::numeric_limits<double>::lowest());
  Eigen::Vector3d left_ub(std::numeric_limits<double>::max(), stance_width/2+stance_wiggle, std::numeric_limits<double>::max());

  Eigen::Vector3d right_lb(std::numeric_limits<double>::lowest(), -stance_width/2-stance_wiggle, std::numeric_limits<double>::lowest());
  Eigen::Vector3d right_ub(std::numeric_limits<double>::max(), -stance_width/2+stance_wiggle, std::numeric_limits<double>::max());
  mpc.AddContactPointPositionConstraint(0, left_lb, left_ub);
  mpc.AddContactPointPositionConstraint(1, left_lb, left_ub);
  mpc.AddContactPointPositionConstraint(2, right_lb, right_ub);
  mpc.AddContactPointPositionConstraint(3, right_lb, right_ub);


  Eigen::VectorXd Q_force = cost_force * Eigen::VectorXd::Ones(12);
  Eigen::VectorXd ref_force = Eigen::VectorXd::Zero(12);
  ref_force[2] = 33*9.81/4;
  ref_force[5] = 33*9.81/4;
  ref_force[8] = 33*9.81/4;
  ref_force[11] = 33*9.81/4;
  mpc.AddConstantForceTrackingReferenceCost(ref_force, Q_force.asDiagonal());


  Eigen::VectorXd Q_state = Eigen::VectorXd::Zero(plant.num_positions() + plant.num_velocities());
  Q_state.head(4) = cost_orientation * Eigen::VectorXd::Ones(4);
  Q_state.segment(7, plant.num_positions()-7) = cost_joint_pos * Eigen::VectorXd::Ones(plant.num_positions()-7);
  Q_state.segment(plant.num_positions(), 3) = cost_angular_vel * Eigen::VectorXd::Ones(3);
  Q_state.segment(plant.num_positions() + 3, 3) = cost_base_vel * Eigen::VectorXd::Ones(3);
  Q_state.tail(plant.num_velocities() - 6) = cost_joint_vel * Eigen::VectorXd::Ones(plant.num_velocities() - 6);
  mpc.AddConstantStateReferenceCost(reference_state, Q_state.asDiagonal());

  Eigen::VectorXd reference_com = Eigen::VectorXd::Zero(3);
  reference_com[2] = com_height;
  Eigen::VectorXd reference_com_bottom = Eigen::VectorXd::Zero(3);
  reference_com_bottom[2] = com_height-squat_distance;
  std::vector<double> time_points = {0, duration};
  auto com_reference = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(time_points,
                                                                                               {reference_com,
                                                                                                reference_com_bottom});
  Eigen::VectorXd Q_com = cost_com_pos * Eigen::VectorXd::Ones(3);
  mpc.AddComReferenceCost(std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(com_reference),
                          Q_com.asDiagonal());

  Eigen::VectorXd Q_contact = cost_contact_pos * Eigen::VectorXd::Ones(4 * 6);
  Q_contact.tail(4 * 3) = cost_contact_vel * Eigen::VectorXd::Ones(4 * 3);
  mpc.AddContactTrackingReferenceCost(std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(Eigen::VectorXd::Zero(
      4 * 6)), Q_contact.asDiagonal());


  Eigen::VectorXd Q_momentum(6);
  Q_momentum.head(3) = cost_ang_mom * Eigen::Vector3d::Ones();
  Q_momentum.tail(3) = cost_lin_mom * Eigen::Vector3d::Ones();
  mpc.AddConstantMomentumReferenceCost(Eigen::VectorXd::Zero(6), Q_momentum.asDiagonal());

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
      plant.num_positions() + plant.num_velocities(), 0, plant.num_positions());
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
