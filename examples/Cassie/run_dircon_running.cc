#include <chrono>
#include <memory>
#include <string>

#include <drake/geometry/drake_visualizer.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/choose_best_solver.h>
#include <drake/solvers/ipopt_solver.h>
#include <drake/solvers/snopt_solver.h>
#include <drake/systems/analysis/simulator.h>
#include <gflags/gflags.h>

#include "examples/Cassie/cassie_utils.h"
#include "lcm/dircon_saved_trajectory.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "solvers/cost_function_utils.h"
#include "systems/trajectory_optimization/dircon/dircon_mode.h"

#include "drake/solvers/solve.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"

using std::cout;
using std::endl;
using std::map;
using std::shared_ptr;
using std::string;
using std::unordered_map;
using std::vector;

using drake::geometry::DrakeVisualizer;
using drake::geometry::SceneGraph;
using drake::multibody::Parser;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::multibody::KinematicEvaluator;
using dairlib::multibody::KinematicEvaluatorSet;
using dairlib::systems::trajectory_optimization::Dircon;
using dairlib::systems::trajectory_optimization::DirconMode;
using dairlib::systems::trajectory_optimization::DirconModeSequence;
using dairlib::systems::trajectory_optimization::PointPositionConstraint;
using drake::math::RotationMatrix;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::MatrixXDecisionVariable;
using drake::solvers::SolutionResult;
using drake::solvers::VectorXDecisionVariable;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::trajectories::PiecewisePolynomial;

DEFINE_int32(knot_points, 16, "Number of knot points per stance mode");
DEFINE_double(stride_length, 0.2, "Stride length.");
DEFINE_double(start_height, 0.90, "Starting pelvis height for the trajectory.");
DEFINE_double(stance_T, 0.2, "Single stance duration (s).");
DEFINE_double(flight_phase_T, 0.1, "Flight phase duration (s).");
DEFINE_int32(scale_option, 2,
             "Scale option of SNOPT"
             "Use 2 if seeing snopta exit 40 in log file");
DEFINE_bool(scale_variables, true, "Set to false if using default scaling");
DEFINE_bool(scale_constraints, true, "Set to false if using default scaling");
DEFINE_double(tol, 1e-7, "Tolerance for constraint violation and dual gap");
DEFINE_string(load_filename, "", "File to load decision vars from.");
DEFINE_string(data_directory, "examples/Cassie/saved_trajectories/",
              "Directory path to save decision vars to.");
DEFINE_string(save_filename, "default_filename",
              "Filename to save decision "
              "vars to.");
DEFINE_bool(ipopt, false, "Set flag to true to use ipopt instead of snopt");
DEFINE_bool(same_knotpoints, false,
            "Set flag to true if seeding with a trajectory with the same "
            "number of knotpoints");

namespace dairlib {

void setKinematicConstraints(Dircon<double>& trajopt,
                             const MultibodyPlant<double>& plant);
void SetInitialGuessFromTrajectory(
    Dircon<double>& trajopt,
    const MultibodyPlant<double>& plant, const string& filepath,
    bool same_knot_points = false);
vector<string> createStateNameVectorFromMap(const map<string, int>& pos_map,
                                            const map<string, int>& vel_map,
                                            const map<string, int>& act_map);
void DoMain() {
  // Drake system initialization stuff
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");
  MultibodyPlant<double> plant(0.0);
  MultibodyPlant<double> plant_vis(0.0);

  string file_name = "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  Parser parser(&plant);
  Parser parser_vis(&plant_vis, &scene_graph);

  parser.AddModels(file_name);
  parser_vis.AddModels(file_name);
  plant.Finalize();
  plant_vis.Finalize();

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_x = n_q + n_v;

  std::cout << "nq: " << n_q << " n_v: " << n_v << " n_x: " << n_x << std::endl;
  // Create maps for joints
  map<string, int> pos_map = multibody::MakeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::MakeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::MakeNameToActuatorsMap(plant);

  // Set up contact/distance constraints
  auto left_toe_pair = LeftToeFront(plant);
  auto left_heel_pair = LeftToeRear(plant);
  auto right_toe_pair = RightToeFront(plant);
  auto right_heel_pair = RightToeRear(plant);

  std::vector<int> toe_active_inds{0, 1, 2};
  std::vector<int> heel_active_inds{1, 2};

  double mu = 1;

  auto left_toe_eval = multibody::WorldPointEvaluator(
      plant, left_toe_pair.first, left_toe_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), toe_active_inds);
  left_toe_eval.set_frictional();
  left_toe_eval.set_mu(mu);
  auto left_heel_eval = multibody::WorldPointEvaluator(
      plant, left_heel_pair.first, left_heel_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), heel_active_inds);
  left_heel_eval.set_frictional();
  left_heel_eval.set_mu(mu);
  auto right_toe_eval = multibody::WorldPointEvaluator(
      plant, right_toe_pair.first, right_toe_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), toe_active_inds);
  right_toe_eval.set_frictional();
  right_toe_eval.set_mu(mu);
  auto right_heel_eval = multibody::WorldPointEvaluator(
      plant, right_heel_pair.first, right_heel_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), heel_active_inds);
  right_heel_eval.set_frictional();
  right_heel_eval.set_mu(mu);

  auto left_loop_eval = LeftLoopClosureEvaluator(plant);
  auto right_loop_eval = RightLoopClosureEvaluator(plant);

  auto left_stance_constraints = KinematicEvaluatorSet<double>(plant);
  int left_toe_eval_ind = left_stance_constraints.add_evaluator(&left_toe_eval);
  int left_heel_eval_ind =
      left_stance_constraints.add_evaluator(&left_heel_eval);
  left_stance_constraints.add_evaluator(&left_loop_eval);
  left_stance_constraints.add_evaluator(&right_loop_eval);

  auto right_stance_constraints = KinematicEvaluatorSet<double>(plant);
  int right_toe_eval_ind =
      right_stance_constraints.add_evaluator(&right_toe_eval);
  int right_heel_eval_ind =
      right_stance_constraints.add_evaluator(&right_heel_eval);
  right_stance_constraints.add_evaluator(&left_loop_eval);
  right_stance_constraints.add_evaluator(&right_loop_eval);

  auto flight_phase_constraints = KinematicEvaluatorSet<double>(plant);
  flight_phase_constraints.add_evaluator(&left_loop_eval);
  flight_phase_constraints.add_evaluator(&right_loop_eval);

  int stance_knotpoints = FLAGS_knot_points;
  int flight_phase_knotpoints = FLAGS_knot_points / 2;
  double flight_phase_T = FLAGS_flight_phase_T;
  double min_T = FLAGS_stance_T;
  double max_T = FLAGS_stance_T;

  auto left_stance = DirconMode<double>(left_stance_constraints,
                                        stance_knotpoints, min_T, max_T);
  auto right_stance = DirconMode<double>(right_stance_constraints,
                                         stance_knotpoints, min_T, max_T);
  left_stance.MakeConstraintRelative(left_toe_eval_ind, 0);     // x
  left_stance.MakeConstraintRelative(left_toe_eval_ind, 1);     // y
  left_stance.MakeConstraintRelative(left_heel_eval_ind, 0);    // x
  left_stance.MakeConstraintRelative(left_heel_eval_ind, 1);    // y
  right_stance.MakeConstraintRelative(right_toe_eval_ind, 0);   // x
  right_stance.MakeConstraintRelative(right_toe_eval_ind, 1);   // y
  right_stance.MakeConstraintRelative(right_heel_eval_ind, 0);  // x
  right_stance.MakeConstraintRelative(right_heel_eval_ind, 1);  // y
  auto flight_phase =
      DirconMode<double>(flight_phase_constraints, flight_phase_knotpoints,
                         flight_phase_T, flight_phase_T);

  auto all_modes = DirconModeSequence<double>(plant);
  all_modes.AddMode(&left_stance);
  all_modes.AddMode(&flight_phase);
  all_modes.AddMode(&right_stance);
  //  all_modes.AddMode(&flight_phase);

  auto trajopt = Dircon<double>(all_modes);
  auto& prog = trajopt.prog();

  double tol = FLAGS_tol;
  if (FLAGS_ipopt) {
    // Ipopt settings adapted from CaSaDi and FROST
    auto id = drake::solvers::IpoptSolver::id();
    prog.SetSolverOption(id, "tol", tol);
    prog.SetSolverOption(id, "dual_inf_tol", tol);
    prog.SetSolverOption(id, "constr_viol_tol", tol);
    prog.SetSolverOption(id, "compl_inf_tol", tol);
    prog.SetSolverOption(id, "max_iter", 1e5);
    prog.SetSolverOption(id, "nlp_lower_bound_inf", -1e6);
    prog.SetSolverOption(id, "nlp_upper_bound_inf", 1e6);
    prog.SetSolverOption(id, "print_timing_statistics", "yes");
    prog.SetSolverOption(id, "print_level", 5);
    prog.SetSolverOption(id, "output_file", "../ipopt.out");

    // Set to ignore overall tolerance/dual infeasibility, but terminate when
    // primal feasible and objective fails to increase over 5 iterations.
    prog.SetSolverOption(id, "acceptable_compl_inf_tol", tol);
    prog.SetSolverOption(id, "acceptable_constr_viol_tol", tol);
    prog.SetSolverOption(id, "acceptable_obj_change_tol", 1e-3);
    prog.SetSolverOption(id, "acceptable_tol", 1e2);
    prog.SetSolverOption(id, "acceptable_iter", 5);
  } else {
    // Snopt settings
    auto id = drake::solvers::SnoptSolver::id();
    prog.SetSolverOption(id, "Print file", "../snopt.out");
    prog.SetSolverOption(id, "Major iterations limit", 1e5);
    prog.SetSolverOption(id, "Iterations limit", 100000);
    prog.SetSolverOption(id, "Verify level", 0);

    // snopt doc said try 2 if seeing snopta exit 40
    prog.SetSolverOption(id, "Scale option", 2);
    prog.SetSolverOption(id, "Solution", "No");

    // target nonlinear constraint violation
    prog.SetSolverOption(id, "Major optimality tolerance", 1e-4);

    // target complementarity gap
    prog.SetSolverOption(id, "Major feasibility tolerance", tol);
  }

  std::cout << "Adding kinematic constraints: " << std::endl;
  setKinematicConstraints(trajopt, plant);
  std::cout << "Setting initial conditions: " << std::endl;

  if (!FLAGS_load_filename.empty()) {
    std::cout << "Loading: " << FLAGS_load_filename << std::endl;
    SetInitialGuessFromTrajectory(trajopt, plant,
                                  FLAGS_data_directory + FLAGS_load_filename,
                                  FLAGS_same_knotpoints);
  } else {
    prog.SetInitialGuessForAllVariables(
        VectorXd::Random(prog.decision_variables().size()));
  }

  //  auto loaded_traj =
  //      DirconTrajectory(plant, FLAGS_data_directory + FLAGS_load_filename);
  //
  //  std::vector<drake::trajectories::PiecewisePolynomial<double>> x_trajs;
  //  x_trajs.push_back(PiecewisePolynomial<double>::CubicHermite(
  //      loaded_traj.GetStateBreaks(0), loaded_traj.GetStateSamples(0),
  //      loaded_traj.GetStateDerivativeSamples(0)));
  //  x_trajs.push_back(PiecewisePolynomial<double>::CubicHermite(
  //      loaded_traj.GetStateBreaks(1), loaded_traj.GetStateSamples(1),
  //      loaded_traj.GetStateDerivativeSamples(1)));
  //  x_trajs.push_back(PiecewisePolynomial<double>::CubicHermite(
  //      loaded_traj.GetStateBreaks(2), loaded_traj.GetStateSamples(2),
  //      loaded_traj.GetStateDerivativeSamples(2)));

  // To avoid NaN quaternions
  for (int i = 0; i < trajopt.N(); i++) {
    auto xi = trajopt.state(i);
    if ((prog.GetInitialGuess(xi.head(4)).norm() == 0) ||
        std::isnan(prog.GetInitialGuess(xi.head(4)).norm())) {
      prog.SetInitialGuess(xi(0), 1);
      prog.SetInitialGuess(xi(1), 0);
      prog.SetInitialGuess(xi(2), 0);
      prog.SetInitialGuess(xi(3), 0);
    }
  }

  double alpha = .2;
  int num_poses = std::min(FLAGS_knot_points, 10);
  trajopt.CreateVisualizationCallback(file_name, num_poses, alpha);

  drake::solvers::SolverId solver_id("");
  if (FLAGS_ipopt) {
    solver_id = drake::solvers::IpoptSolver().id();
    cout << "\nChose manually: " << solver_id.name() << endl;
  } else {
    solver_id = drake::solvers::ChooseBestSolver(prog);
    cout << "\nChose the best solver: " << solver_id.name() << endl;
  }

  cout << "Solving DIRCON\n\n";
  auto start = std::chrono::high_resolution_clock::now();
  const auto result = drake::solvers::Solve(prog, prog.initial_guess());
  //  SolutionResult solution_result = result.get_solution_result();
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  cout << "Solve time:" << elapsed.count() << std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() << std::endl;
  std::cout << "Solve result: " << result.get_solution_result() << std::endl;

  // Save trajectory to file
  DirconTrajectory saved_traj(plant, trajopt, result, "running_trajectory",
                              "Decision variables and state/input trajectories "
                              "for walking");
  saved_traj.WriteToFile(FLAGS_data_directory + FLAGS_save_filename);
  std::cout << "Wrote to file: " << FLAGS_data_directory + FLAGS_save_filename
            << std::endl;
  drake::trajectories::PiecewisePolynomial<double> optimal_traj =
      trajopt.ReconstructStateTrajectory(result);
  multibody::ConnectTrajectoryVisualizer(&plant_vis, &builder, &scene_graph,
                                         optimal_traj);

  DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph);
  auto diagram = builder.Build();

  //  while (true) {
  //    drake::systems::Simulator<double> simulator(*diagram);
  //    simulator.set_target_realtime_rate(0.1);
  //    simulator.Initialize();
  //    simulator.AdvanceTo(optimal_traj.end_time());
  //  }
}

void setKinematicConstraints(Dircon<double>& trajopt,
                             const MultibodyPlant<double>& plant) {
  auto& prog = trajopt.prog();

  // Create maps for joints
  map<string, int> pos_map = multibody::MakeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::MakeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::MakeNameToActuatorsMap(plant);

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();

  // Get the decision variables that will be used
  int N = trajopt.N();
  auto x = trajopt.state();
  auto u = trajopt.input();
  auto x0 = trajopt.initial_state();
  auto x_mid = trajopt.state(N / 2);
  auto xf = trajopt.final_state();
  auto u0 = trajopt.input(0);
  auto uf = trajopt.input(N - 1);

  // Standing constraints
  double start_height = FLAGS_start_height;

  // position constraints
  prog.AddBoundingBoxConstraint(-0.25, 0.25, x0(pos_map.at("base_x")));
  prog.AddLinearConstraint(x0(pos_map.at("base_x")) + FLAGS_stride_length ==
                              xf(pos_map.at("base_x")));
  prog.AddBoundingBoxConstraint(start_height, start_height,
                                   x0(pos_map.at("base_z")));
  //  trajopt.AddConstraintToAllKnotPoints(x(pos_map.at("base_y")) <= 0.05);
  //  trajopt.AddConstraintToAllKnotPoints(x(pos_map.at("base_y")) >= -0.05);
  // initial fb orientation constraint
  VectorXd quat_identity(4);
  quat_identity << 1, 0, 0, 0;
  prog.AddBoundingBoxConstraint(quat_identity, quat_identity, x0.head(4));
  prog.AddBoundingBoxConstraint(quat_identity, quat_identity, x_mid.head(4));
  prog.AddBoundingBoxConstraint(quat_identity, quat_identity, xf.head(4));

  // periodicity constraint
  prog.AddLinearConstraint(x0(pos_map.at("base_y")) ==
                              -xf(pos_map.at("base_y")));
  prog.AddLinearConstraint(x0(pos_map.at("base_z")) ==
                              xf(pos_map.at("base_z")));
  prog.AddLinearConstraint(x0(n_q + vel_map.at("base_wx")) ==
                              xf(n_q + vel_map.at("base_wx")));
  prog.AddLinearConstraint(x0(n_q + vel_map.at("base_wy")) ==
                              -xf(n_q + vel_map.at("base_wy")));
  prog.AddLinearConstraint(x0(n_q + vel_map.at("base_wz")) ==
                              xf(n_q + vel_map.at("base_wz")));
  prog.AddLinearConstraint(x0(n_q + vel_map.at("base_vx")) ==
                              xf(n_q + vel_map.at("base_vx")));
  prog.AddLinearConstraint(x0(n_q + vel_map.at("base_vy")) ==
                              -xf(n_q + vel_map.at("base_vy")));
  prog.AddLinearConstraint(x0(n_q + vel_map.at("base_vz")) ==
                              xf(n_q + vel_map.at("base_vz")));

  // create joint/motor names
  vector<std::pair<string, string>> l_r_pairs{
      std::pair<string, string>("_left", "_right"),
      std::pair<string, string>("_right", "_left"),
  };
  vector<string> asy_joint_names{"hip_roll", "hip_yaw"};
  vector<string> sym_joint_names{"hip_pitch", "knee", "ankle_joint", "toe"};
  vector<string> joint_names{};
  vector<string> motor_names{};
  for (const auto& l_r_pair : l_r_pairs) {
    for (const auto& asy_joint_name : asy_joint_names) {
      joint_names.push_back(asy_joint_name + l_r_pair.first);
      motor_names.push_back(asy_joint_name + l_r_pair.first + "_motor");
    }
    for (const auto& sym_joint_name : sym_joint_names) {
      joint_names.push_back(sym_joint_name + l_r_pair.first);
      if (sym_joint_name != "ankle_joint") {
        motor_names.push_back(sym_joint_name + l_r_pair.first + "_motor");
      }
    }
  }

  for (const auto& l_r_pair : l_r_pairs) {
    // Symmetry constraints
    for (const auto& sym_joint_name : sym_joint_names) {
      prog.AddLinearConstraint(
          x0(pos_map[sym_joint_name + l_r_pair.first]) ==
          xf(pos_map[sym_joint_name + l_r_pair.second]));
      prog.AddLinearConstraint(
          x0(n_q + vel_map.at(sym_joint_name + l_r_pair.first + "dot")) ==
          xf(n_q + vel_map.at(sym_joint_name + l_r_pair.second + "dot")));
      if (sym_joint_name != "ankle_joint") {  // No actuator at ankle
        prog.AddLinearConstraint(
            u0(act_map.at(sym_joint_name + l_r_pair.first + "_motor")) ==
            uf(act_map.at(sym_joint_name + l_r_pair.second + "_motor")));
      }
    }
    // Asymmetry constraints
    for (const auto& asy_joint_name : asy_joint_names) {
      prog.AddLinearConstraint(
          x0(pos_map[asy_joint_name + l_r_pair.first]) ==
          -xf(pos_map[asy_joint_name + l_r_pair.second]));
      prog.AddLinearConstraint(
          x0(n_q + vel_map.at(asy_joint_name + l_r_pair.first + "dot")) ==
          -xf(n_q + vel_map.at(asy_joint_name + l_r_pair.second + "dot")));
      if (asy_joint_name != "ankle_joint") {  // No actuator at ankle
        prog.AddLinearConstraint(
            u0(act_map.at(asy_joint_name + l_r_pair.first + "_motor")) ==
            -uf(act_map.at(asy_joint_name + l_r_pair.second + "_motor")));
      }
    }
  }

  // joint limits
  std::cout << "Joint limit constraints: " << std::endl;
  for (const auto& member : joint_names) {
    trajopt.AddConstraintToAllKnotPoints(
        x(pos_map.at(member)) <=
        plant.GetJointByName(member).position_upper_limits()(0));
    trajopt.AddConstraintToAllKnotPoints(
        x(pos_map.at(member)) >=
        plant.GetJointByName(member).position_lower_limits()(0));
  }

  // actuator limits
  std::cout << "Actuator limit constraints: " << std::endl;
  for (int i = 0; i < trajopt.N(); i++) {
    auto ui = trajopt.input(i);
    prog.AddBoundingBoxConstraint(VectorXd::Constant(n_u, -200),
                                     VectorXd::Constant(n_u, +200), ui);
  }

  std::cout << "Foot placement constraints: " << std::endl;
  // toe position constraint in y direction (avoid leg crossing)
  // tighter constraint than before
  auto left_foot_y_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", Vector3d::Zero(), Eigen::RowVector3d(0, 1, 0),
          0.10 * VectorXd::Ones(1), 0.25 * VectorXd::Ones(1));
  auto right_foot_y_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", Vector3d::Zero(), Eigen::RowVector3d(0, 1, 0),
          -0.25 * VectorXd::Ones(1), -0.1 * VectorXd::Ones(1));

  for (int i = 0; i < N; ++i) {
    auto x_i = trajopt.state(i);
    prog.AddConstraint(left_foot_y_constraint, x_i.head(n_q));
    prog.AddConstraint(right_foot_y_constraint, x_i.head(n_q));
  }

  for (int mode = 0; mode < trajopt.num_modes(); ++mode) {
    for (int index = 0; index < trajopt.mode_length(mode); index++) {
      // Assumes mode_lengths are the same across modes

      auto lambda = trajopt.force_vars(mode, index);
      if (mode == 0 || mode == 2) {
        prog.AddLinearConstraint(lambda(2) >= 10);
        prog.AddLinearConstraint(lambda(5) >= 10);
      }
    }
  }
  std::cout << "Stride length constraints: " << std::endl;
  // stride length constraint
  auto right_foot_x_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", Vector3d::Zero(), Eigen::RowVector3d(1, 0, 0),
          FLAGS_stride_length * VectorXd::Ones(1),
          FLAGS_stride_length * VectorXd::Ones(1));
  auto left_foot_x_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", Vector3d::Zero(), Eigen::RowVector3d(1, 0, 0),
          0 * VectorXd::Ones(1), 0 * VectorXd::Ones(1));
  prog.AddConstraint(left_foot_x_constraint, x0.head(n_q));
  prog.AddConstraint(right_foot_x_constraint, xf.head(n_q));

  std::cout << "Foot clearance constraints: " << std::endl;
  //   Foot clearance constraint
  auto left_foot_z_constraint_clearance =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", Vector3d::Zero(), Eigen::RowVector3d(0, 0, 1),
          0.055 * VectorXd::Ones(1), (0.15) * VectorXd::Ones(1));
  auto right_foot_z_constraint_clearance =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", Vector3d::Zero(), Eigen::RowVector3d(0, 0, 1),
          0.055 * VectorXd::Ones(1), (0.15) * VectorXd::Ones(1));
  for (int i = 0; i < N; i++) {
    auto x_i = trajopt.state(i);
    prog.AddConstraint(left_foot_z_constraint_clearance, x_i.head(n_q));
    prog.AddConstraint(right_foot_z_constraint_clearance, x_i.head(n_q));
    prog.AddBoundingBoxConstraint(-1.8, -1.5, x_i[pos_map["toe_left"]]);
    prog.AddBoundingBoxConstraint(-1.8, -1.5, x_i[pos_map["toe_right"]]);
  }

  std::cout << "Miscellaneous constraints" << std::endl;
  // Miscellaneous constraints
  trajopt.AddConstraintToAllKnotPoints(x(pos_map.at("hip_yaw_left")) >= -0.01);
  trajopt.AddConstraintToAllKnotPoints(x(pos_map.at("hip_yaw_left")) <= 0.01);
  trajopt.AddConstraintToAllKnotPoints(x(pos_map.at("hip_yaw_right")) >= -0.01);
  trajopt.AddConstraintToAllKnotPoints(x(pos_map.at("hip_yaw_right")) <= 0.01);
  // Miscellaneous constraints
  trajopt.AddConstraintToAllKnotPoints(x(pos_map.at("hip_roll_left")) >= 0.0);
  trajopt.AddConstraintToAllKnotPoints(x(pos_map.at("hip_roll_left")) <= 0.10);
  trajopt.AddConstraintToAllKnotPoints(x(pos_map.at("hip_roll_right")) >=
                                       -0.10);
  trajopt.AddConstraintToAllKnotPoints(x(pos_map.at("hip_roll_right")) <= 0.0);
  trajopt.AddConstraintToAllKnotPoints(x(pos_map.at("hip_pitch_left")) >= 0.50);
  trajopt.AddConstraintToAllKnotPoints(x(pos_map.at("hip_pitch_left")) <= 0.90);
  trajopt.AddConstraintToAllKnotPoints(x(pos_map.at("hip_pitch_right")) >=
                                       0.50);
  trajopt.AddConstraintToAllKnotPoints(x(pos_map.at("hip_pitch_right")) <=
                                       0.90);

  std::cout << "Adding costs: " << std::endl;
  double W = 1e-1;
  MatrixXd Q = MatrixXd::Identity(n_v, n_v);
  Q(7, 7) = 10;
  Q(8, 8) = 10;
  //  MatrixXd R = 1e-3 * MatrixXd::Identity(n_u, n_u);
  //  R(8, 8) = 1;
  //  R(9, 9) = 1;
  trajopt.AddRunningCost((x.tail(n_v).transpose() * Q * x.tail(n_v)));
  //  trajopt.AddRunningCost(u.transpose() * R * u);
  solvers::AddPositiveWorkCost(trajopt, plant, W);

  //  MatrixXd S = MatrixXd::Zero(n_u, n_v);
  //  S(0, 6) = 1;
  //  S(1, 7) = 1;
  //  S(2, 8) = 1;
  //  S(3, 9) = 1;
  //  S(4, 10) = 1;
  //  S(5, 11) = 1;
  //  S(6, 12) = 1;
  //  S(7, 13) = 1;
  //  S(8, 16) = 1;
  //  S(9, 17) = 1;
  //  const drake::symbolic::Expression e_max_{max(static_cast<const
  //  AutoDiffScalar<VectorXd>>(
  //              u.transpose() * S * x.tail(n_v) + u.transpose() * R * u),
  //              VectorXd::Zero(1))};
  //  drake::symbolic::max(u.transpose() * S * x.tail(n_v), 0);
  //  trajopt.AddRunningCost(drake::symbolic::max(u.transpose() * S *
  //  x.tail(n_v), 0)); trajopt.AddRunningCost(u.transpose() * S * x.tail(n_v)
  //  + u.transpose() * R * u);
}

void SetInitialGuessFromTrajectory(Dircon<double>& trajopt,
                                   const MultibodyPlant<double>& plant,
                                   const string& filepath,
                                   bool same_knot_points) {
  DirconTrajectory previous_traj = DirconTrajectory(plant, filepath);
  if (same_knot_points) {
    trajopt.prog().SetInitialGuessForAllVariables(
        previous_traj.GetDecisionVariables());
    return;
  }
  auto state_traj = previous_traj.ReconstructStateTrajectory();
  auto input_traj = previous_traj.ReconstructInputTrajectory();
  auto lambda_traj = previous_traj.ReconstructLambdaTrajectory();
  auto lambda_c_traj = previous_traj.ReconstructLambdaCTrajectory();
  auto gamma_traj = previous_traj.ReconstructGammaCTrajectory();

  trajopt.SetInitialTrajectory(input_traj, state_traj);
  for (int mode = 0; mode < trajopt.num_modes() - 1; ++mode) {
    if (trajopt.mode_length(mode) > 1) {
      std::cout << "mode: " << mode << std::endl;
      trajopt.SetInitialForceTrajectory(mode, lambda_traj[mode],
                                        lambda_c_traj[mode], gamma_traj[mode]);
    }
  }
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::DoMain();
}