#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/choose_best_solver.h>
#include <drake/solvers/ipopt_solver.h>
#include <drake/solvers/snopt_solver.h>
#include <drake/systems/analysis/simulator.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "examples/Cassie/cassie_utils.h"
#include "lcm/dircon_saved_trajectory.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "solvers/nonlinear_cost.h"
#include "systems/trajectory_optimization/dircon/dircon.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/parser.h"

using std::cout;
using std::endl;
using std::map;
using std::shared_ptr;
using std::string;
using std::unordered_map;
using std::vector;

using drake::geometry::SceneGraph;
using drake::multibody::Parser;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::multibody::CreateWithSpringsToWithoutSpringsMapPos;
using dairlib::multibody::CreateWithSpringsToWithoutSpringsMapVel;
using dairlib::multibody::KinematicEvaluator;
using dairlib::multibody::KinematicEvaluatorSet;
using dairlib::solvers::NonlinearCost;
using dairlib::systems::trajectory_optimization::Dircon;
using dairlib::systems::trajectory_optimization::DirconMode;
using dairlib::systems::trajectory_optimization::DirconModeSequence;
using dairlib::systems::trajectory_optimization::DirconOptions;
using dairlib::systems::trajectory_optimization::PointPositionConstraint;
using drake::VectorX;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::MatrixXDecisionVariable;
using drake::solvers::SolutionResult;
using drake::solvers::VectorXDecisionVariable;
using drake::trajectories::PiecewisePolynomial;

DEFINE_string(jumping_parameters, "", "Jumping parameters");

namespace dairlib {

struct DirconJumpingParameters {
  // model urdfs
  std::string spring_urdf;
  std::string fixed_spring_urdf;
  // trajectory parameters
  int knot_points;
  double delta_height;
  double start_height;
  double distance;
  double min_stance_duration;
  double max_stance_duration;
  double min_flight_duration;
  double max_flight_duration;
  double actuator_limit;
  double input_delta;
  // solver parameters
  double snopt_scale_option;
  double tol;
  double dual_inf_tol;
  double constr_viol_tol;
  double compl_inf_tol;
  double acceptable_tol;
  int acceptable_iter;
  double cost_scaling;
  bool use_ipopt;
  int ipopt_iter;
  // warmstart settings
  std::string data_directory;
  std::string load_filename;
  std::string save_filename;
  bool use_springs;
  bool convert_to_springs;
  bool same_knotpoints;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(spring_urdf));
    a->Visit(DRAKE_NVP(fixed_spring_urdf));
    a->Visit(DRAKE_NVP(knot_points));
    a->Visit(DRAKE_NVP(delta_height));
    a->Visit(DRAKE_NVP(start_height));
    a->Visit(DRAKE_NVP(distance));
    a->Visit(DRAKE_NVP(min_stance_duration));
    a->Visit(DRAKE_NVP(max_stance_duration));
    a->Visit(DRAKE_NVP(min_flight_duration));
    a->Visit(DRAKE_NVP(max_flight_duration));
    a->Visit(DRAKE_NVP(actuator_limit));
    a->Visit(DRAKE_NVP(input_delta));
    a->Visit(DRAKE_NVP(snopt_scale_option));
    a->Visit(DRAKE_NVP(tol));
    a->Visit(DRAKE_NVP(dual_inf_tol));
    a->Visit(DRAKE_NVP(constr_viol_tol));
    a->Visit(DRAKE_NVP(compl_inf_tol));
    a->Visit(DRAKE_NVP(acceptable_tol));
    a->Visit(DRAKE_NVP(acceptable_iter));
    a->Visit(DRAKE_NVP(cost_scaling));
    a->Visit(DRAKE_NVP(use_ipopt));
    a->Visit(DRAKE_NVP(ipopt_iter));
    a->Visit(DRAKE_NVP(data_directory));
    a->Visit(DRAKE_NVP(load_filename));
    a->Visit(DRAKE_NVP(save_filename));
    a->Visit(DRAKE_NVP(use_springs));
    a->Visit(DRAKE_NVP(convert_to_springs));
    a->Visit(DRAKE_NVP(same_knotpoints));
  }
};

void SetKinematicConstraints(Dircon<double>* trajopt,
                             const MultibodyPlant<double>& plant,
                             DirconJumpingParameters& gait_params);
void AddCosts(Dircon<double>* trajopt, const MultibodyPlant<double>& plant,
              DirconModeSequence<double>*,
              DirconJumpingParameters& gait_params);
void SetInitialGuessFromDirconTrajectory(
    Dircon<double>& trajopt, const MultibodyPlant<double>& plant,
    DirconJumpingParameters& gait_params,
    Eigen::MatrixXd spr_map = Eigen::MatrixXd::Zero(1, 1));
void SetInitialGuessFromKCTrajectory(Dircon<double>& trajopt,
                                     const string& filepath);

void DoMain() {
  // Drake system initialization stuff
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");
  MultibodyPlant<double> plant(0.0);
  MultibodyPlant<double> plant_vis(0.0);
  auto gait_params =
      drake::yaml::LoadYamlFile<DirconJumpingParameters>(FindResourceOrThrow(
          "examples/Cassie/saved_trajectories/gait_parameters/" +
          FLAGS_jumping_parameters));

  string file_name = gait_params.fixed_spring_urdf;
  if (gait_params.use_springs) file_name = gait_params.spring_urdf;

  AddCassieMultibody(&plant, nullptr, true, file_name, gait_params.use_springs,
                     false);

  Parser parser_vis(&plant_vis, &scene_graph);
  parser_vis.AddModels(file_name);

  plant.Finalize();
  plant_vis.Finalize();

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_x = n_q + n_v;

  MatrixXd spr_map = MatrixXd::Zero(n_x, n_x);
  if (gait_params.use_springs && gait_params.convert_to_springs) {
    MultibodyPlant<double> plant_wo_spr(0.0);
    Parser parser(&plant_wo_spr);
    parser.AddModels(
        "examples/Cassie/urdf/cassie_fixed_springs_conservative.urdf");
    plant_wo_spr.Finalize();
    spr_map = CreateWithSpringsToWithoutSpringsMapPos(plant, plant_wo_spr);
  }

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

  auto double_stance_constraints = KinematicEvaluatorSet<double>(plant);
  int left_toe_eval_ind =
      double_stance_constraints.add_evaluator(&left_toe_eval);
  int left_heel_eval_ind =
      double_stance_constraints.add_evaluator(&left_heel_eval);
  int right_toe_eval_ind =
      double_stance_constraints.add_evaluator(&right_toe_eval);
  int right_heel_eval_ind =
      double_stance_constraints.add_evaluator(&right_heel_eval);
  double_stance_constraints.add_evaluator(&left_loop_eval);
  double_stance_constraints.add_evaluator(&right_loop_eval);

  auto flight_phase_constraints = KinematicEvaluatorSet<double>(plant);
  flight_phase_constraints.add_evaluator(&left_loop_eval);
  flight_phase_constraints.add_evaluator(&right_loop_eval);

  int stance_knotpoints = gait_params.knot_points;
  int flight_phase_knotpoints = gait_params.knot_points;

  /****
   * Mode duration constraints
   */
  auto crouch_mode = DirconMode<double>(
      double_stance_constraints, stance_knotpoints,
      gait_params.min_stance_duration, gait_params.max_stance_duration);
  auto flight_mode = DirconMode<double>(
      flight_phase_constraints, flight_phase_knotpoints,
      gait_params.min_flight_duration, gait_params.max_flight_duration);
  auto land_mode = DirconMode<double>(
      double_stance_constraints, stance_knotpoints,
      gait_params.min_stance_duration, gait_params.max_stance_duration);

  crouch_mode.MakeConstraintRelative(left_toe_eval_ind, 0);
  crouch_mode.MakeConstraintRelative(left_toe_eval_ind, 1);
  //  crouch_mode.MakeConstraintRelative(left_heel_eval_ind, 0);
  crouch_mode.MakeConstraintRelative(left_heel_eval_ind, 1);
  crouch_mode.MakeConstraintRelative(right_toe_eval_ind, 0);
  crouch_mode.MakeConstraintRelative(right_toe_eval_ind, 1);
  //  crouch_mode.MakeConstraintRelative(right_heel_eval_ind, 0);
  crouch_mode.MakeConstraintRelative(right_heel_eval_ind, 1);

  land_mode.MakeConstraintRelative(left_toe_eval_ind, 0);
  land_mode.MakeConstraintRelative(left_toe_eval_ind, 1);
  land_mode.MakeConstraintRelative(left_heel_eval_ind, 0);
  land_mode.MakeConstraintRelative(left_heel_eval_ind, 1);
  land_mode.MakeConstraintRelative(right_toe_eval_ind, 0);
  land_mode.MakeConstraintRelative(right_toe_eval_ind, 1);
  land_mode.MakeConstraintRelative(right_heel_eval_ind, 0);
  land_mode.MakeConstraintRelative(right_heel_eval_ind, 1);
  if (gait_params.delta_height != 0) {
    land_mode.MakeConstraintRelative(left_toe_eval_ind, 2);
    land_mode.MakeConstraintRelative(left_heel_eval_ind, 2);
    land_mode.MakeConstraintRelative(right_toe_eval_ind, 2);
    land_mode.MakeConstraintRelative(right_heel_eval_ind, 2);
  }

  auto all_modes = DirconModeSequence<double>(plant);
  all_modes.AddMode(&crouch_mode);
  all_modes.AddMode(&flight_mode);
  all_modes.AddMode(&land_mode);

  auto trajopt = Dircon<double>(all_modes);
  auto& prog = trajopt.prog();

  drake::solvers::SolverOptions solver_options;
  if (gait_params.use_ipopt) {
    // Ipopt settings adapted from CaSaDi and FROST
    auto id = drake::solvers::IpoptSolver::id();
    solver_options.SetOption(id, "tol", gait_params.tol);  // NOLINT
    solver_options.SetOption(id, "dual_inf_tol",
                             gait_params.dual_inf_tol);  // NOLINT
    solver_options.SetOption(id, "constr_viol_tol",
                             gait_params.constr_viol_tol);  // NOLINT
    solver_options.SetOption(id, "compl_inf_tol",
                             gait_params.compl_inf_tol);             // NOLINT
    solver_options.SetOption(id, "nlp_lower_bound_inf", -1e6);       // NOLINT
    solver_options.SetOption(id, "nlp_upper_bound_inf", 1e6);        // NOLINT
    solver_options.SetOption(id, "print_timing_statistics", "yes");  // NOLINT
    solver_options.SetOption(id, "print_level", 5);                  // NOLINT
    solver_options.SetOption(id, "output_file", "../ipopt.out");     // NOLINT
    solver_options.SetOption(id, "acceptable_tol",
                             gait_params.acceptable_tol);  // NOLINT
    solver_options.SetOption(id, "acceptable_iter",
                             gait_params.acceptable_iter);             // NOLINT
    solver_options.SetOption(id, "max_iter", gait_params.ipopt_iter);  // NOLINT

  } else {
    // Snopt settings
    auto id = drake::solvers::SnoptSolver::id();
    solver_options.SetOption(id, "Print file", "../snopt.out");        // NOLINT
    solver_options.SetOption(id, "Major iterations limit", 1e5);       // NOLINT
    solver_options.SetOption(id, "Iterations limit", 100000);          // NOLINT
    solver_options.SetOption(id, "Verify level", 0);                   // NOLINT
    solver_options.SetOption(id, "Major optimality tolerance", 1e-5);  // NOLINT
    solver_options.SetOption(id, "Solution", "No");                    // NOLINT
    solver_options.SetOption(id, "Major feasibility tolerance",
                             gait_params.tol);  // NOLINT
  }

  std::cout << "Adding kinematic constraints: " << std::endl;
  SetKinematicConstraints(&trajopt, plant, gait_params);
  std::cout << "Adding costs: " << std::endl;
  AddCosts(&trajopt, plant, &all_modes, gait_params);
  std::cout << "Setting initial conditions: " << std::endl;
  vector<int> mode_lengths = {gait_params.knot_points, gait_params.knot_points,
                              gait_params.knot_points};

  int num_knot_points = trajopt.N();

  if (!gait_params.load_filename.empty()) {
    std::cout << "Loading: " << gait_params.load_filename << std::endl;
    if (gait_params.load_filename.find("kc") != -1) {
      SetInitialGuessFromKCTrajectory(
          trajopt, gait_params.data_directory + gait_params.load_filename);
    } else {
      SetInitialGuessFromDirconTrajectory(trajopt, plant, gait_params, spr_map);
    }
  }

  double alpha = .2;
  int num_poses =
      std::max((int)(mode_lengths.size() + 1), gait_params.knot_points);
  trajopt.CreateVisualizationCallback(file_name, num_poses, alpha);

  cout << "Solving DIRCON\n\n";
  auto ipopt_solver = drake::solvers::IpoptSolver();
  auto snopt_solver = drake::solvers::SnoptSolver();
  MathematicalProgramResult result;

  auto start = std::chrono::high_resolution_clock::now();
  if (gait_params.use_ipopt) {
    result = ipopt_solver.Solve(prog, prog.initial_guess(), solver_options);
  } else {
    result = snopt_solver.Solve(prog, prog.initial_guess(), solver_options);
  }

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  cout << "Solve time:" << elapsed.count() << std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() << std::endl;
  std::cout << "Solve result: " << result.get_solution_result() << std::endl;

  // Save trajectory to file
  DirconTrajectory saved_traj(plant, trajopt, result, "jumping_trajectory",
                              "Decision variables and state/input trajectories "
                              "for jumping");
  saved_traj.WriteToFile(gait_params.data_directory +
                         gait_params.save_filename);
  std::cout << "Wrote to file: "
            << gait_params.data_directory + gait_params.save_filename
            << std::endl;
  drake::trajectories::PiecewisePolynomial<double> optimal_traj =
      trajopt.ReconstructStateTrajectory(result);
  multibody::ConnectTrajectoryVisualizer(&plant_vis, &builder, &scene_graph,
                                         optimal_traj);

  auto diagram = builder.Build();

  while (true) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(0.5);
    simulator.Initialize();
    simulator.AdvanceTo(optimal_traj.end_time());
  }
}

void SetKinematicConstraints(Dircon<double>* trajopt,
                             const MultibodyPlant<double>& plant,
                             DirconJumpingParameters& gait_params) {
  auto* prog = &trajopt->prog();
  // Create maps for joints
  map<string, int> pos_map = multibody::MakeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::MakeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::MakeNameToActuatorsMap(plant);

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();

  // Get the decision variables that will be used
  int N = trajopt->N();
  std::vector<int> mode_lengths = {gait_params.knot_points,
                                   gait_params.knot_points,
                                   gait_params.knot_points};
  auto x_0 = trajopt->initial_state();
  auto x_top = trajopt->state(N / 2);
  auto x_f = trajopt->final_state();
  auto u = trajopt->input();
  auto u_0 = trajopt->input(0);
  auto u_f = trajopt->input(N - 1);
  auto x = trajopt->state();

  // Standing constraints
  double rest_height = gait_params.start_height;
  double eps = 1e-4;
  double midpoint = 0.045;

  //  bounding box constraints on all decision  variables
  for (int i = 0; i < mode_lengths.size(); ++i) {
    for (int j = 0; j < mode_lengths.at(i); ++j) {
      auto force_vars_ij = trajopt->force_vars(i, j);
      prog->AddBoundingBoxConstraint(
          VectorXd::Constant(force_vars_ij.rows(), -200),
          VectorXd::Constant(force_vars_ij.rows(), 200), force_vars_ij);
    }
  }

  std::cout << "Adding initial orientation constraints " << std::endl;

  // position constraints
  prog->AddBoundingBoxConstraint(0 - midpoint, 0 - midpoint,
                                 x_0(pos_map.at("pelvis_x")));
  prog->AddBoundingBoxConstraint(gait_params.distance + midpoint,
                                 gait_params.distance + midpoint,
                                 x_f(pos_map.at("pelvis_x")));
  prog->AddBoundingBoxConstraint(-eps, eps, x_0(pos_map.at("pelvis_y")));
  prog->AddBoundingBoxConstraint(-eps, eps, x_f(pos_map.at("pelvis_y")));

  // initial fb orientation constraint
  VectorXd quat_identity(4);
  quat_identity << 1, 0, 0, 0;
  prog->AddBoundingBoxConstraint(quat_identity, quat_identity, x_0.head(4));
  prog->AddBoundingBoxConstraint(quat_identity, quat_identity, x_f.head(4));

  // hip yaw and roll constraints
  prog->AddBoundingBoxConstraint(0, 0, x_0(pos_map.at("hip_yaw_left")));
  prog->AddBoundingBoxConstraint(0, 0, x_0(pos_map.at("hip_yaw_right")));

  // hip yaw and roll constraints
  prog->AddBoundingBoxConstraint(0, 0, x_f(pos_map.at("hip_yaw_left")));
  prog->AddBoundingBoxConstraint(0, 0, x_f(pos_map.at("hip_yaw_right")));

  std::cout << "Adding jumping height constraints" << std::endl;

  // Jumping height constraints
  prog->AddBoundingBoxConstraint(rest_height - eps, rest_height + eps,
                                 x_0(pos_map.at("pelvis_z")));
  if (gait_params.delta_height < 0) {
    prog->AddBoundingBoxConstraint(
        gait_params.delta_height + rest_height + eps,
        0.5 * gait_params.delta_height + rest_height - eps,
        x_top(pos_map.at("pelvis_z")));

  } else {
    prog->AddBoundingBoxConstraint(
        0.5 * gait_params.delta_height + rest_height - eps,
        gait_params.delta_height + rest_height + eps,
        x_top(pos_map.at("pelvis_z")));
  }
  prog->AddBoundingBoxConstraint(
      0.8 * gait_params.delta_height + rest_height - eps,
      0.8 * gait_params.delta_height + rest_height + eps,
      x_f(pos_map.at("pelvis_z")));

  // Zero starting and final velocities
  prog->AddLinearConstraint(VectorXd::Zero(n_v) == x_0.tail(n_v));
  prog->AddLinearConstraint(VectorXd::Zero(n_v) == x_f.tail(n_v));

  // create joint/motor names
  vector<std::pair<string, string>> l_r_pairs{
      std::pair<string, string>("_left", "_right"),
      std::pair<string, string>("_right", "_left"),
  };
  vector<string> asy_joint_names{
      "hip_roll",
      "hip_yaw",
  };
  vector<string> spring_joint_names{
      "knee_joint",
      "ankle_spring_joint",
  };
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
    if (gait_params.use_springs) {
      for (const auto& spring_joint_name : spring_joint_names) {
        joint_names.push_back(spring_joint_name + l_r_pair.first);
      }
    }
  }
  l_r_pairs.pop_back();

  // joint limits
  std::cout << "Joint limit constraints: " << std::endl;
  for (const auto& member : joint_names) {
    trajopt->AddConstraintToAllKnotPoints(
        x(pos_map.at(member)) <=
        plant.GetJointByName(member).position_upper_limits()(0));
    trajopt->AddConstraintToAllKnotPoints(
        x(pos_map.at(member)) >=
        plant.GetJointByName(member).position_lower_limits()(0));
  }
  // actuator limits
  std::cout << "Actuator limit constraints: " << std::endl;
  VectorXd u_min(n_u);
  VectorXd u_max(n_u);
  for (drake::multibody::JointActuatorIndex i(0); i < n_u; ++i) {
    u_min[i] = gait_params.actuator_limit *
               -plant.get_joint_actuator(i).effort_limit();
    u_max[i] =
        gait_params.actuator_limit * plant.get_joint_actuator(i).effort_limit();
  }
  for (int i = 0; i < trajopt->N(); i++) {
    auto ui = trajopt->input(i);
    prog->AddBoundingBoxConstraint(u_min, u_max, ui);
    if (i < trajopt->N() - 1) {
      auto uip1 = trajopt->input(i + 1);
      prog->AddConstraint(ui - uip1 <=
                          VectorXd::Constant(n_u, gait_params.input_delta));
      prog->AddConstraint(ui - uip1 >=
                          VectorXd::Constant(n_u, -gait_params.input_delta));
    }
  }

  Vector3d pt_front_contact(-0.0457, 0.112, 0);
  Vector3d pt_rear_contact(0.088, 0, 0);

  // toe position constraint in y direction (avoid leg crossing)
  // tighter constraint than before
  auto left_foot_y_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", Vector3d::Zero(), Eigen::RowVector3d(0, 1, 0),
          0.1 * VectorXd::Ones(1), 0.15 * VectorXd::Ones(1));
  auto right_foot_y_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", Vector3d::Zero(), Eigen::RowVector3d(0, 1, 0),
          -0.15 * VectorXd::Ones(1), -0.1 * VectorXd::Ones(1));
  auto left_foot_z_ground_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", pt_front_contact, Eigen::RowVector3d(0, 0, 1),
          VectorXd::Zero(1), 1.0 * VectorXd::Ones(1));
  auto right_foot_z_ground_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", pt_front_contact, Eigen::RowVector3d(0, 0, 1),
          VectorXd::Zero(1), 1.0 * VectorXd::Ones(1));

  for (int mode : {0, 1, 2}) {
    for (int index = 0; index < mode_lengths[mode]; index++) {
      // Assumes mode_lengths are the same across modes
      auto x_i = trajopt->state((mode_lengths[mode] - 1) * mode + index);
      prog->AddConstraint(left_foot_y_constraint, x_i.head(n_q));
      prog->AddConstraint(right_foot_y_constraint, x_i.head(n_q));
      if (gait_params.delta_height >= 0) {
        prog->AddConstraint(left_foot_z_ground_constraint, x_i.head(n_q));
        prog->AddConstraint(right_foot_z_ground_constraint, x_i.head(n_q));
      }
    }
  }

  // Constraint for platform clearance
  if (gait_params.delta_height > 0.3) {
    auto left_foot_x_box_constraint =
        std::make_shared<PointPositionConstraint<double>>(
            plant, "toe_left", pt_front_contact, Eigen::RowVector3d(1, 0, 0),
            0.2 * (gait_params.distance - eps) * VectorXd::Ones(1),
            0.2 * (gait_params.distance + eps) * VectorXd::Ones(1));
    auto right_foot_x_box_constraint =
        std::make_shared<PointPositionConstraint<double>>(
            plant, "toe_right", pt_front_contact, Eigen::RowVector3d(1, 0, 0),
            0.2 * (gait_params.distance - eps) * VectorXd::Ones(1),
            0.2 * (gait_params.distance + eps) * VectorXd::Ones(1));
    prog->AddConstraint(left_foot_x_box_constraint, x_top.head(n_q));
    prog->AddConstraint(right_foot_x_box_constraint, x_top.head(n_q));
    auto left_foot_z_box_constraint =
        std::make_shared<PointPositionConstraint<double>>(
            plant, "toe_left", Vector3d::Zero(), Eigen::RowVector3d(0, 0, 1),
            (0.1 + gait_params.delta_height - eps) * VectorXd::Ones(1),
            (0.3 + gait_params.delta_height + eps) * VectorXd::Ones(1));
    auto right_foot_z_box_constraint =
        std::make_shared<PointPositionConstraint<double>>(
            plant, "toe_right", Vector3d::Zero(), Eigen::RowVector3d(0, 0, 1),
            (0.1 + gait_params.delta_height - eps) * VectorXd::Ones(1),
            (0.3 + gait_params.delta_height + eps) * VectorXd::Ones(1));
    prog->AddConstraint(left_foot_z_box_constraint, x_top.head(n_q));
    prog->AddConstraint(right_foot_z_box_constraint, x_top.head(n_q));
  }

  // Foot start constraint
  auto left_foot_x_start_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", pt_front_contact, Eigen::RowVector3d(1, 0, 0),
          (0 - eps) * VectorXd::Ones(1), (0 + eps) * VectorXd::Ones(1));
  auto right_foot_x_start_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", pt_front_contact, Eigen::RowVector3d(1, 0, 0),
          (0 - eps) * VectorXd::Ones(1), (0 + eps) * VectorXd::Ones(1));
  prog->AddConstraint(left_foot_x_start_constraint, x_0.head(n_q));
  prog->AddConstraint(right_foot_x_start_constraint, x_0.head(n_q));

  // Jumping distance constraint
  auto left_foot_x_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", pt_rear_contact, Eigen::RowVector3d(1, 0, 0),
          (gait_params.distance - eps) * VectorXd::Ones(1),
          (gait_params.distance + eps) * VectorXd::Ones(1));
  auto right_foot_x_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", pt_rear_contact, Eigen::RowVector3d(1, 0, 0),
          (gait_params.distance - eps) * VectorXd::Ones(1),
          (gait_params.distance + eps) * VectorXd::Ones(1));
  prog->AddConstraint(left_foot_x_constraint, x_f.head(n_q));
  prog->AddConstraint(right_foot_x_constraint, x_f.head(n_q));

  // Foot ground constraint
  auto left_foot_rear_z_final_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", pt_rear_contact, Eigen::RowVector3d(0, 0, 1),
          (gait_params.delta_height - eps) * VectorXd::Ones(1),
          (gait_params.delta_height + eps) * VectorXd::Ones(1));
  auto right_foot_rear_z_final_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", pt_rear_contact, Eigen::RowVector3d(0, 0, 1),
          (gait_params.delta_height - eps) * VectorXd::Ones(1),
          (gait_params.delta_height + eps) * VectorXd::Ones(1));
  auto left_foot_front_z_final_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", pt_front_contact, Eigen::RowVector3d(0, 0, 1),
          (gait_params.delta_height - eps) * VectorXd::Ones(1),
          (gait_params.delta_height + eps) * VectorXd::Ones(1));
  auto right_foot_front_z_final_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", pt_front_contact, Eigen::RowVector3d(0, 0, 1),
          (gait_params.delta_height - eps) * VectorXd::Ones(1),
          (gait_params.delta_height + eps) * VectorXd::Ones(1));
  prog->AddConstraint(left_foot_front_z_final_constraint, x_f.head(n_q));
  prog->AddConstraint(right_foot_front_z_final_constraint, x_f.head(n_q));
  prog->AddConstraint(left_foot_rear_z_final_constraint, x_f.head(n_q));
  prog->AddConstraint(right_foot_rear_z_final_constraint, x_f.head(n_q));

  // Only add constraints of lambdas for stance modes
  // ALL BUT THE LAST SEGMENT (to ensure the feet can leave the ground
  //  for (int index = 0; index < (mode_lengths[0] - 2); index++) {
  //    auto lambda = trajopt->force_vars(0, index);
  //    prog->AddLinearConstraint(lambda(2) >= 60);
  //    prog->AddLinearConstraint(lambda(5) >= 60);
  //    prog->AddLinearConstraint(lambda(8) >= 60);
  //    prog->AddLinearConstraint(lambda(11) >= 60);
  //  }
  // Limit the ground reaction forces in the landing phase
  for (int index = 0; index < mode_lengths[2]; index++) {
    auto lambda = trajopt->force_vars(2, index);
    prog->AddLinearConstraint(lambda(2) <= 325);
    prog->AddLinearConstraint(lambda(5) <= 325);
    prog->AddLinearConstraint(lambda(8) <= 325);
    prog->AddLinearConstraint(lambda(11) <= 325);
    prog->AddLinearConstraint(lambda(2) >= 50);
    prog->AddLinearConstraint(lambda(5) >= 50);
    prog->AddLinearConstraint(lambda(8) >= 50);
    prog->AddLinearConstraint(lambda(11) >= 50);
  }
}

/******
COSTS
******/
void AddCosts(Dircon<double>* trajopt, const MultibodyPlant<double>& plant,
              DirconModeSequence<double>* constraints,
              DirconJumpingParameters& gait_params) {
  auto x = trajopt->state();
  auto u = trajopt->input();

  map<string, int> pos_map = multibody::MakeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::MakeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::MakeNameToActuatorsMap(plant);

  int n_v = plant.num_velocities();
  int n_q = plant.num_positions();
  int n_u = plant.num_actuators();

  // create joint/motor names
  vector<std::pair<string, string>> l_r_pairs{
      std::pair<string, string>("_left", "_right"),
      std::pair<string, string>("_right", "_left"),
  };
  vector<string> asy_joint_names{
      "hip_roll",
      "hip_yaw",
  };
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
  l_r_pairs.pop_back();

  std::cout << "Symmetry costs for symmetric joints: " << std::endl;
  double w_symmetry_pos = gait_params.cost_scaling * 1e5;
  double w_symmetry_vel = gait_params.cost_scaling * 1e2;
  double w_symmetry_u = gait_params.cost_scaling * 1e2;
  for (const auto& l_r_pair : l_r_pairs) {
    for (const auto& sym_joint_name : sym_joint_names) {
      auto pos_diff = x(pos_map.at(sym_joint_name + l_r_pair.first)) -
                      x(pos_map.at(sym_joint_name + l_r_pair.second));
      auto vel_diff = x(vel_map.at(sym_joint_name + l_r_pair.first + "dot")) -
                      x(vel_map.at(sym_joint_name + l_r_pair.second + "dot"));
      trajopt->AddRunningCost(w_symmetry_pos * pos_diff * pos_diff);
      trajopt->AddRunningCost(w_symmetry_vel * vel_diff * vel_diff);
    }
  }

  std::cout << "Symmetry costs for asymmetric joints: " << std::endl;
  for (const auto& l_r_pair : l_r_pairs) {
    for (const auto& asy_joint_name : asy_joint_names) {
      auto pos_diff = x(pos_map.at(asy_joint_name + l_r_pair.first)) +
                      x(pos_map.at(asy_joint_name + l_r_pair.second));
      auto vel_diff = x(vel_map.at(asy_joint_name + l_r_pair.first + "dot")) +
                      x(vel_map.at(asy_joint_name + l_r_pair.second + "dot"));
      trajopt->AddRunningCost(w_symmetry_pos * pos_diff * pos_diff);
      trajopt->AddRunningCost(w_symmetry_vel * vel_diff * vel_diff);
    }
  }

  std::cout << "Running costs: " << std::endl;

  MatrixXd Q = gait_params.cost_scaling * 0.02 * MatrixXd::Identity(n_v, n_v);
  MatrixXd R = gait_params.cost_scaling * 1e-1 * MatrixXd::Identity(n_u, n_u);

  VectorXd q_f = VectorXd::Zero(n_q);
  q_f(pos_map.at("hip_yaw_left")) = 0;
  q_f(pos_map.at("hip_yaw_right")) = 0;
  q_f(pos_map.at("hip_roll_left")) = 0;
  q_f(pos_map.at("hip_roll_right")) = 0;
  q_f(pos_map.at("hip_pitch_left")) = 0.67;
  q_f(pos_map.at("hip_pitch_right")) = 0.67;
  q_f(pos_map.at("pelvis_z")) =
      0.8 * gait_params.delta_height + gait_params.start_height;
  VectorXd q_f_target_left = q_f.segment(6, 4);
  VectorXd q_f_target_right = q_f.segment(13, 3);

  MatrixXd Q_left = gait_params.cost_scaling * MatrixXd::Identity(4, 4);
  MatrixXd Q_right = gait_params.cost_scaling * MatrixXd::Identity(3, 3);
  Q_left(2, 2) = gait_params.cost_scaling * 1e2;
  Q_right(2, 2) = gait_params.cost_scaling * 1e2;

  trajopt->AddRunningCost(x.tail(n_v).transpose() * Q * x.tail(n_v));
  trajopt->AddRunningCost((x.segment(6, 4) - q_f_target_left).transpose() *
                          Q_left * (x.segment(6, 4) - q_f_target_left));
  trajopt->AddRunningCost((x.segment(13, 3) - q_f_target_right).transpose() *
                          Q_right * (x.segment(13, 3) - q_f_target_right));
  trajopt->AddRunningCost(u.transpose() * R * u);
}

void SetInitialGuessFromDirconTrajectory(Dircon<double>& trajopt,
                                         const MultibodyPlant<double>& plant,
                                         DirconJumpingParameters& gait_params,
                                         Eigen::MatrixXd spr_map) {
  DirconTrajectory previous_traj = DirconTrajectory(
      plant, gait_params.data_directory + gait_params.load_filename);
  if (gait_params.same_knotpoints) {
    trajopt.prog().SetInitialGuessForAllVariables(
        previous_traj.GetDecisionVariables());
    return;
  }
  drake::trajectories::PiecewisePolynomial<double> state_traj;
  if (gait_params.use_springs && gait_params.convert_to_springs) {
    std::cout << "Using spring conversion" << std::endl;
    state_traj = previous_traj.ReconstructStateTrajectoryWithSprings(spr_map);
  } else {
    state_traj = previous_traj.ReconstructStateTrajectory();
  }
  auto input_traj = previous_traj.ReconstructInputTrajectory();
  auto lambda_traj = previous_traj.ReconstructLambdaTrajectory();
  auto lambda_c_traj = previous_traj.ReconstructLambdaCTrajectory();
  auto gamma_traj = previous_traj.ReconstructGammaCTrajectory();

  trajopt.SetInitialTrajectory(input_traj, state_traj);
  //  for (int mode = 0; mode < trajopt.num_modes(); ++mode) {
  //    trajopt.SetInitialForceTrajectory(mode, lambda_traj[mode],
  //                                      lambda_c_traj[mode],
  //                                      gamma_traj[mode]);
  //  }
}

void SetInitialGuessFromKCTrajectory(Dircon<double>& trajopt,
                                     const string& filepath) {
  LcmTrajectory previous_traj = LcmTrajectory(filepath);

  const auto state_traj_block = previous_traj.GetTrajectory("state_traj");
  const auto lambda_traj_block =
      previous_traj.GetTrajectory("contact_force_traj");
  auto state_traj = PiecewisePolynomial<double>::FirstOrderHold(
      state_traj_block.time_vector, state_traj_block.datapoints);
  auto input_traj = PiecewisePolynomial<double>::ZeroOrderHold(
      state_traj_block.time_vector,
      MatrixXd::Zero(10, state_traj_block.time_vector.size()));
  auto lambda_traj = PiecewisePolynomial<double>::FirstOrderHold(
      lambda_traj_block.time_vector, lambda_traj_block.datapoints);

  trajopt.SetInitialTrajectory(input_traj, state_traj);
  trajopt.SetInitialForceTrajectory(0, lambda_traj);
  trajopt.SetInitialForceTrajectory(1, lambda_traj);
  trajopt.SetInitialForceTrajectory(2, lambda_traj);
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::DoMain();
}
