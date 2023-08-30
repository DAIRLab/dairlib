#include <chrono>
#include <memory>
#include <string>
#include <iostream>

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

#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/solve.h"
#include "drake/planning/trajectory_optimization/multiple_shooting.h"

using std::cout;
using std::endl;
using std::map;
using std::shared_ptr;
using std::string;
using std::unordered_map;
using std::vector;

using drake::geometry::SceneGraph;
using drake::multibody::Parser;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::multibody::CreateWithSpringsToWithoutSpringsMapPos;
using dairlib::multibody::CreateWithSpringsToWithoutSpringsMapVel;
using dairlib::multibody::KinematicEvaluator;
using dairlib::multibody::KinematicEvaluatorSet;
using dairlib::solvers::NonlinearCost;
using dairlib::systems::trajectory_optimization::Dircon;
using dairlib::systems::trajectory_optimization::DirconDynamicConstraint;
using dairlib::systems::trajectory_optimization::DirconKinConstraintType;
using dairlib::systems::trajectory_optimization::DirconKinematicConstraint;
using dairlib::systems::trajectory_optimization::DirconMode;
using dairlib::systems::trajectory_optimization::DirconModeSequence;
using dairlib::systems::trajectory_optimization::DirconOptions;
using dairlib::systems::trajectory_optimization::HybridDircon;
using dairlib::systems::trajectory_optimization::PointPositionConstraint;
using drake::VectorX;
using drake::math::RotationMatrix;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::MatrixXDecisionVariable;
using drake::solvers::SolutionResult;
using drake::solvers::VectorXDecisionVariable;
using drake::planning::trajectory_optimization::MultipleShooting;
using drake::trajectories::PiecewisePolynomial;

DEFINE_int32(knot_points, 10, "Number of knot points per contact mode");
DEFINE_double(height, 0.2, "Target height for jumping.");
DEFINE_double(start_height, 1.0, "Starting pelvis height for the trajectory.");
DEFINE_double(distance, 0.0, "Target distance (x) from the initial position.");
DEFINE_double(duration, 0.0, "Duration of the total gait");
DEFINE_int32(scale_option, 0,
             "Scale option of SNOPT"
             "Use 2 if seeing snopta exit 40 in log file");
DEFINE_double(tol, 1e-6, "Tolerance for constraint violation and dual gap");
DEFINE_string(load_filename, "", "File to load decision vars from.");
DEFINE_string(data_directory, "examples/Cassie/saved_trajectories/",
              "Directory path to save decision vars to.");
DEFINE_string(save_filename, "default_filename",
              "Filename to save decision "
              "vars to.");
DEFINE_string(traj_name, "", "File to load saved LCM trajs from.");
DEFINE_bool(use_springs, false, "Whether or not to use the spring model");
DEFINE_bool(
    convert_to_springs, false,
    "Whether the loaded trajectory needs to be converted to with springs");
DEFINE_bool(ipopt, false, "Set flag to true to use ipopt instead of snopt");
DEFINE_bool(same_knotpoints, false,
            "Set flag to true if seeding with a trajectory with the same "
            "number of knotpoints");
DEFINE_double(cost_scaling, 1.0, "Common scaling factor for costs.");

namespace dairlib {

HybridDircon<double>* createDircon(MultibodyPlant<double>& plant);

void SetKinematicConstraints(Dircon<double>* trajopt,
                             const MultibodyPlant<double>& plant);
void AddCosts(Dircon<double>* trajopt, const MultibodyPlant<double>& plant,
              DirconModeSequence<double>*);
void AddCostsSprings(Dircon<double>* trajopt,
                     const MultibodyPlant<double>& plant,
                     DirconModeSequence<double>*);
void SetInitialGuessFromTrajectory(
    Dircon<double>& trajopt, const MultibodyPlant<double>& plant,
    const string& filepath, bool same_knot_points = false,
    Eigen::MatrixXd spr_map = Eigen::MatrixXd::Zero(1, 1));

class JointAccelCost : public solvers::NonlinearCost<double> {
 public:
  JointAccelCost(const MatrixXd& W_Q,
                 const drake::multibody::MultibodyPlant<double>& plant,
                 const KinematicEvaluatorSet<double>* constraints,
                 const std::string& description = "")
      : solvers::NonlinearCost<double>(
      plant.num_positions() + plant.num_velocities() +
          plant.num_actuators() + constraints->count_full(),
      description),
        plant_(plant),
        context_(plant_.CreateDefaultContext()),
        constraints_(constraints),
        n_v_(plant.num_velocities()),
        n_x_(plant.num_positions() + plant.num_velocities()),
        n_u_(plant.num_actuators()),
        n_lambda_(constraints->count_full()),
        W_Q_(W_Q){};

 private:
  void EvaluateCost(const Eigen::Ref<const drake::VectorX<double>>& x,
                    drake::VectorX<double>* y) const override {
    DRAKE_ASSERT(x.size() == n_x_ + n_u_ + n_lambda_);

    // Extract our input variables:
    const auto x0 = x.segment(0, n_x_);
    const auto u0 = x.segment(n_x_, n_u_);
    VectorXd l0 = x.segment(n_x_ + n_u_, n_lambda_);

    multibody::SetContext<double>(plant_, x0, u0, context_.get());
    const VectorX<double> xdot0 =
        constraints_->CalcTimeDerivatives(*context_, &l0);

    (*y) = xdot0.tail(n_v_).transpose() * W_Q_ * xdot0.tail(n_v_);
  };

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const KinematicEvaluatorSet<double>* constraints_;

  int n_v_;
  int n_x_;
  int n_u_;
  int n_lambda_;

  MatrixXd W_Q_;
};

void DoMain() {
  // Drake system initialization stuff
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");
  MultibodyPlant<double> plant(0.0);
  MultibodyPlant<double> plant_vis(0.0);

  string file_name =
      "examples/Cassie/urdf/cassie_fixed_springs_conservative.urdf";

  if (FLAGS_use_springs) {
    file_name = "examples/Cassie/urdf/cassie_v2_conservative.urdf";
  }

  AddCassieMultibody(&plant, nullptr, true, file_name, FLAGS_use_springs,
                     false);

  Parser parser_vis(&plant_vis, &scene_graph);
  parser_vis.AddModelFromFile(file_name);

  plant.Finalize();
  plant_vis.Finalize();

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_x = n_q + n_v;

  MatrixXd spr_map = MatrixXd::Zero(n_x, n_x);
  if (FLAGS_use_springs && FLAGS_convert_to_springs) {
    MultibodyPlant<double> plant_wo_spr(0.0);
    Parser parser(&plant_wo_spr);
    parser.AddModelFromFile(
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

  int stance_knotpoints = FLAGS_knot_points;
  int flight_phase_knotpoints = FLAGS_knot_points;

  /****
   * Mode duration constraints
   */
  auto crouch_mode = DirconMode<double>(double_stance_constraints,
                                        stance_knotpoints, 0.5, 0.75);
  auto flight_mode = DirconMode<double>(flight_phase_constraints,
                                        flight_phase_knotpoints, 0.25, 0.5);
  auto land_mode = DirconMode<double>(double_stance_constraints,
                                      stance_knotpoints, 0.4, 0.6);

  crouch_mode.MakeConstraintRelative(left_toe_eval_ind, 0);
  crouch_mode.MakeConstraintRelative(left_toe_eval_ind, 1);
  crouch_mode.MakeConstraintRelative(left_heel_eval_ind, 0);
  crouch_mode.MakeConstraintRelative(left_heel_eval_ind, 1);
  crouch_mode.MakeConstraintRelative(right_toe_eval_ind, 0);
  crouch_mode.MakeConstraintRelative(right_toe_eval_ind, 1);
  crouch_mode.MakeConstraintRelative(right_heel_eval_ind, 0);
  crouch_mode.MakeConstraintRelative(right_heel_eval_ind, 1);

  land_mode.MakeConstraintRelative(left_toe_eval_ind, 0);
  land_mode.MakeConstraintRelative(left_toe_eval_ind, 1);
  land_mode.MakeConstraintRelative(left_toe_eval_ind, 2);
  land_mode.MakeConstraintRelative(left_heel_eval_ind, 0);
  land_mode.MakeConstraintRelative(left_heel_eval_ind, 1);
  land_mode.MakeConstraintRelative(left_heel_eval_ind, 2);
  land_mode.MakeConstraintRelative(right_toe_eval_ind, 0);
  land_mode.MakeConstraintRelative(right_toe_eval_ind, 1);
  land_mode.MakeConstraintRelative(right_toe_eval_ind, 2);
  land_mode.MakeConstraintRelative(right_heel_eval_ind, 0);
  land_mode.MakeConstraintRelative(right_heel_eval_ind, 1);
  land_mode.MakeConstraintRelative(right_heel_eval_ind, 2);

  auto all_modes = DirconModeSequence<double>(plant);
  all_modes.AddMode(&crouch_mode);
  all_modes.AddMode(&flight_mode);
  all_modes.AddMode(&land_mode);

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
    if (FLAGS_use_springs) {
      prog.SetSolverOption(id, "Print file", "../w_springs_snopt.out");
    } else {
      prog.SetSolverOption(id, "Print file", "../snopt.out");
    }
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
  SetKinematicConstraints(&trajopt, plant);
  if (FLAGS_use_springs) {
    AddCostsSprings(&trajopt, plant, &all_modes);
  } else {
    AddCosts(&trajopt, plant, &all_modes);
  }
  std::cout << "Setting initial conditions: " << std::endl;
  vector<int> mode_lengths = {FLAGS_knot_points, FLAGS_knot_points,
                              FLAGS_knot_points};

  int num_knot_points = trajopt.N();
  std::cout << "nq: " << n_q << endl;
  std::cout << "nv: " << n_v << endl;
  std::cout << "nu: " << plant.num_actuators() << endl;
  cout << "N: " << num_knot_points << endl;
  cout << "Num decision vars: " <<
    prog.decision_variables().size() << endl;

  if (!FLAGS_load_filename.empty()) {
    std::cout << "Loading: " << FLAGS_load_filename << std::endl;
    SetInitialGuessFromTrajectory(trajopt, plant,
                                  FLAGS_data_directory + FLAGS_load_filename,
                                  FLAGS_same_knotpoints, spr_map);
    //    trajopt->SetInitialGuessForAllVariables(decisionVars);
  }

  double alpha = .2;
  int num_poses = std::min(FLAGS_knot_points, 5);
  trajopt.CreateVisualizationCallback(file_name, num_poses, alpha);

  cout << "\nChoose the best solver: "
       << drake::solvers::ChooseBestSolver(prog).name() << endl;

  cout << "Solving DIRCON\n\n";
  auto start = std::chrono::high_resolution_clock::now();
  const auto result = drake::solvers::Solve(prog, prog.initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  cout << "Solve time:" << elapsed.count() << std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() << std::endl;
  std::cout << "Solve result: " << result.get_solution_result() << std::endl;

  std::cout << "Lambda sol: " << result.GetSolution(trajopt.impulse_vars(1))
            << std::endl;

  // Save trajectory to file
  DirconTrajectory saved_traj(plant, trajopt, result, "jumping_trajectory",
                              "Decision variables and state/input trajectories "
                              "for jumping");
  saved_traj.WriteToFile(FLAGS_data_directory + FLAGS_save_filename);
  std::cout << "Wrote to file: " << FLAGS_data_directory + FLAGS_save_filename
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
                             const MultibodyPlant<double>& plant) {
  // Create maps for joints
  map<string, int> pos_map = multibody::MakeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::MakeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::MakeNameToActuatorsMap(plant);

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();

  // Get the decision variables that will be used
  int N = trajopt->N();
  std::vector<int> mode_lengths = {FLAGS_knot_points, FLAGS_knot_points,
                                   FLAGS_knot_points};
  auto x_0 = trajopt->initial_state();
  auto x_top = trajopt->state(N / 2);
  auto x_f = trajopt->final_state();
  auto u = trajopt->input();
  auto u_0 = trajopt->input(0);
  auto u_f = trajopt->input(N - 1);
  auto x = trajopt->state();

  // Duration Bounds

  // Standing constraints
  double rest_height = FLAGS_start_height;
  double eps = 1e-6;

  double midpoint = 0.045;

  // get mp
  auto& prog = trajopt->prog();
  // position constraints
  prog.AddBoundingBoxConstraint(0 - midpoint, 0 - midpoint,
                                    x_0(pos_map.at("base_x")));
  prog.AddBoundingBoxConstraint(0, 0, x_0(pos_map.at("base_y")));
  prog.AddBoundingBoxConstraint(0, 0, x_f(pos_map.at("base_y")));

  // initial fb orientation constraint
  VectorXd quat_identity(4);
  quat_identity << 1, 0, 0, 0;
  prog.AddBoundingBoxConstraint(quat_identity, quat_identity, x_0.head(4));
  prog.AddBoundingBoxConstraint(quat_identity, quat_identity, x_f.head(4));

  // hip yaw and roll constraints
  prog.AddBoundingBoxConstraint(0, 0, x_0(pos_map.at("hip_yaw_left")));
  prog.AddBoundingBoxConstraint(0, 0, x_0(pos_map.at("hip_yaw_right")));

  prog.AddBoundingBoxConstraint(0.00, 0.1,
                                    x_0(pos_map.at("hip_roll_left")));
  prog.AddBoundingBoxConstraint(-0.1, -0.00,
                                    x_0(pos_map.at("hip_roll_right")));

  // hip yaw and roll constraints
  prog.AddBoundingBoxConstraint(0, 0, x_f(pos_map.at("hip_yaw_left")));
  prog.AddBoundingBoxConstraint(0, 0, x_f(pos_map.at("hip_yaw_right")));

  prog.AddBoundingBoxConstraint(-2.1, -1.6, x_0(pos_map.at("toe_left")));
  prog.AddBoundingBoxConstraint(-2.1, -1.6, x_0(pos_map.at("toe_right")));
  prog.AddBoundingBoxConstraint(-2.1, -1.6, x_f(pos_map.at("toe_left")));
  prog.AddBoundingBoxConstraint(-2.1, -1.6, x_f(pos_map.at("toe_right")));

  // Jumping height constraints
  prog.AddBoundingBoxConstraint(rest_height - eps, rest_height + eps,
                                    x_0(pos_map.at("base_z")));
  prog.AddBoundingBoxConstraint(0.5 * FLAGS_height + rest_height - eps,
                                    FLAGS_height + rest_height + eps,
                                    x_top(pos_map.at("base_z")));
  prog.AddBoundingBoxConstraint(0.8 * FLAGS_height + rest_height - eps,
                                    0.8 * FLAGS_height + rest_height + eps,
                                    x_f(pos_map.at("base_z")));

  // Zero starting and final velocities
  prog.AddLinearConstraint(VectorXd::Zero(n_v) == x_0.tail(n_v));
  prog.AddLinearConstraint(VectorXd::Zero(n_v) == x_f.tail(n_v));

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
    if (FLAGS_use_springs) {
      for (const auto& spring_joint_name : spring_joint_names) {
        joint_names.push_back(spring_joint_name + l_r_pair.first);
      }
    }
  }
  l_r_pairs.pop_back();

  // Symmetry constraints
  for (const auto& l_r_pair : l_r_pairs) {
    for (const auto& sym_joint_name : sym_joint_names) {
      trajopt->AddConstraintToAllKnotPoints(
          x_0(pos_map[sym_joint_name + l_r_pair.first]) ==
              x_0(pos_map[sym_joint_name + l_r_pair.second]));
      prog.AddLinearConstraint(
          x_f(pos_map[sym_joint_name + l_r_pair.first]) ==
              x_f(pos_map[sym_joint_name + l_r_pair.second]));
      if (sym_joint_name != "ankle_joint") {  // No actuator at ankle
        trajopt->AddConstraintToAllKnotPoints(
            u_0(act_map.at(sym_joint_name + l_r_pair.first + "_motor")) ==
                u_0(act_map.at(sym_joint_name + l_r_pair.second + "_motor")));
        trajopt->AddConstraintToAllKnotPoints(
            u_f(act_map.at(sym_joint_name + l_r_pair.first + "_motor")) ==
                u_f(act_map.at(sym_joint_name + l_r_pair.second + "_motor")));
      }
    }
  }

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
  for (int i = 0; i < trajopt->N(); i++) {
    auto ui = trajopt->input(i);
    prog.AddBoundingBoxConstraint(VectorXd::Constant(n_u, -175),
                                      VectorXd::Constant(n_u, +175), ui);
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
  for (int mode : {0, 1, 2}) {
    for (int index = 0; index < mode_lengths[mode]; index++) {
      // Assumes mode_lengths are the same across modes
      auto x_i = trajopt->state((mode_lengths[mode] - 1) * mode + index);
      prog.AddConstraint(left_foot_y_constraint, x_i.head(n_q));
      prog.AddConstraint(right_foot_y_constraint, x_i.head(n_q));
    }
  }

  // Jumping distance constraint for platform clearance
  auto left_foot_x_start_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", pt_front_contact, Eigen::RowVector3d(1, 0, 0),
          (0 - eps) * VectorXd::Ones(1), (0 + eps) * VectorXd::Ones(1));
  auto right_foot_x_start_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", pt_front_contact, Eigen::RowVector3d(1, 0, 0),
          (0 - eps) * VectorXd::Ones(1), (0 + eps) * VectorXd::Ones(1));
  prog.AddConstraint(left_foot_x_start_constraint, x_0.head(n_q));
  prog.AddConstraint(right_foot_x_start_constraint, x_0.head(n_q));

  // Jumping distance constraint for platform clearance
  auto left_foot_x_platform_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", pt_front_contact, Eigen::RowVector3d(1, 0, 0),
          0.4 * (FLAGS_distance - eps) * VectorXd::Ones(1),
          0.4 * (FLAGS_distance + eps) * VectorXd::Ones(1));
  auto right_foot_x_platform_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", pt_front_contact, Eigen::RowVector3d(1, 0, 0),
          0.4 * (FLAGS_distance - eps) * VectorXd::Ones(1),
          0.4 * (FLAGS_distance + eps) * VectorXd::Ones(1));
  prog.AddConstraint(left_foot_x_platform_constraint, x_top.head(n_q));
  prog.AddConstraint(right_foot_x_platform_constraint, x_top.head(n_q));

  // Jumping distance constraint
  auto left_foot_x_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", pt_rear_contact, Eigen::RowVector3d(1, 0, 0),
          (FLAGS_distance - eps) * VectorXd::Ones(1),
          (FLAGS_distance + eps) * VectorXd::Ones(1));
  auto right_foot_x_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", pt_rear_contact, Eigen::RowVector3d(1, 0, 0),
          (FLAGS_distance - eps) * VectorXd::Ones(1),
          (FLAGS_distance + eps) * VectorXd::Ones(1));
  prog.AddConstraint(left_foot_x_constraint, x_f.head(n_q));
  prog.AddConstraint(right_foot_x_constraint, x_f.head(n_q));

  // Foot clearance constraint
  auto left_foot_z_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", Vector3d::Zero(), Eigen::RowVector3d(0, 0, 1),
          (1.25 * FLAGS_height - eps) * VectorXd::Ones(1),
          (1.25 * FLAGS_height + eps) * VectorXd::Ones(1));
  auto right_foot_z_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", Vector3d::Zero(), Eigen::RowVector3d(0, 0, 1),
          (1.25 * FLAGS_height - eps) * VectorXd::Ones(1),
          (1.25 * FLAGS_height + eps) * VectorXd::Ones(1));
  prog.AddConstraint(left_foot_z_constraint, x_top.head(n_q));
  prog.AddConstraint(right_foot_z_constraint, x_top.head(n_q));

  auto left_foot_rear_z_final_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", pt_rear_contact, Eigen::RowVector3d(0, 0, 1),
          (FLAGS_height - eps) * VectorXd::Ones(1),
          (FLAGS_height + eps) * VectorXd::Ones(1));
  auto right_foot_rear_z_final_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", pt_rear_contact, Eigen::RowVector3d(0, 0, 1),
          (FLAGS_height - eps) * VectorXd::Ones(1),
          (FLAGS_height + eps) * VectorXd::Ones(1));
  prog.AddConstraint(left_foot_rear_z_final_constraint, x_f.head(n_q));
  prog.AddConstraint(right_foot_rear_z_final_constraint, x_f.head(n_q));

  auto left_foot_front_z_final_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", pt_front_contact, Eigen::RowVector3d(0, 0, 1),
          (FLAGS_height - eps) * VectorXd::Ones(1),
          (FLAGS_height + eps) * VectorXd::Ones(1));
  auto right_foot_front_z_final_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", pt_front_contact, Eigen::RowVector3d(0, 0, 1),
          (FLAGS_height - eps) * VectorXd::Ones(1),
          (FLAGS_height + eps) * VectorXd::Ones(1));
  prog.AddConstraint(left_foot_front_z_final_constraint, x_f.head(n_q));
  prog.AddConstraint(right_foot_front_z_final_constraint, x_f.head(n_q));

  // Only add constraints of lambdas for stance modes
  // ALL BUT THE LAST SEGMENT (to ensure the feet can leave the ground
  for (int index = 0; index < (mode_lengths[0] - 2); index++) {
    auto lambda = trajopt->force_vars(0, index);
    prog.AddLinearConstraint(lambda(2) >= 60);
    prog.AddLinearConstraint(lambda(5) >= 60);
    prog.AddLinearConstraint(lambda(8) >= 60);
    prog.AddLinearConstraint(lambda(11) >= 60);
  }
  // Limit the ground reaction forces in the landing phase
  for (int index = 0; index < mode_lengths[2]; index++) {
    auto lambda = trajopt->force_vars(2, index);
    prog.AddLinearConstraint(lambda(2) <= 350);
    prog.AddLinearConstraint(lambda(5) <= 350);
    prog.AddLinearConstraint(lambda(8) <= 350);
    prog.AddLinearConstraint(lambda(11) <= 350);
    prog.AddLinearConstraint(lambda(2) >= 50);
    prog.AddLinearConstraint(lambda(5) >= 50);
    prog.AddLinearConstraint(lambda(8) >= 50);
    prog.AddLinearConstraint(lambda(11) >= 50);
  }
  // Limit the ground impulses when landing
  auto Lambda = trajopt->impulse_vars(1);
  prog.AddLinearConstraint(Lambda(2) <= 2);
  prog.AddLinearConstraint(Lambda(5) <= 2);
  prog.AddLinearConstraint(Lambda(8) <= 2);
  prog.AddLinearConstraint(Lambda(11) <= 2);
}

/******
COSTS
******/
void AddCosts(Dircon<double>* trajopt, const MultibodyPlant<double>& plant,
              DirconModeSequence<double>* constraints) {
  auto x = trajopt->state();
  auto u = trajopt->input();

  map<string, int> pos_map = multibody::MakeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::MakeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::MakeNameToActuatorsMap(plant);

  int n_v = plant.num_velocities();
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

  double w_symmetry_pos = FLAGS_cost_scaling * 1e5;
  double w_symmetry_vel = FLAGS_cost_scaling * 1e2;
  double w_symmetry_u = FLAGS_cost_scaling * 1e2;
  for (const auto& l_r_pair : l_r_pairs) {
    for (const auto& sym_joint_name : sym_joint_names) {
      auto pos_diff = x(pos_map.at(sym_joint_name + l_r_pair.first)) -
          x(pos_map.at(sym_joint_name + l_r_pair.second));
      auto vel_diff = x(vel_map.at(sym_joint_name + l_r_pair.first + "dot")) -
          x(vel_map.at(sym_joint_name + l_r_pair.second + "dot"));
      trajopt->AddRunningCost(w_symmetry_pos * pos_diff * pos_diff);
      trajopt->AddRunningCost(w_symmetry_vel * vel_diff * vel_diff);
      if (sym_joint_name != "ankle_joint") {
        auto act_diff =
            u(act_map.at(sym_joint_name + l_r_pair.first + "_motor")) -
                u(act_map.at(sym_joint_name + l_r_pair.second + "_motor"));
        trajopt->AddRunningCost(w_symmetry_u * act_diff * act_diff);
      }
    }
  }
  for (const auto& l_r_pair : l_r_pairs) {
    for (const auto& asy_joint_name : asy_joint_names) {
      auto pos_diff = x(pos_map.at(asy_joint_name + l_r_pair.first)) +
          x(pos_map.at(asy_joint_name + l_r_pair.second));
      auto vel_diff = x(vel_map.at(asy_joint_name + l_r_pair.first + "dot")) +
          x(vel_map.at(asy_joint_name + l_r_pair.second + "dot"));
      trajopt->AddRunningCost(w_symmetry_pos * pos_diff * pos_diff);
      trajopt->AddRunningCost(w_symmetry_vel * vel_diff * vel_diff);
      if (asy_joint_name != "ankle_joint") {
        auto act_diff =
            u(act_map.at(asy_joint_name + l_r_pair.first + "_motor")) +
                u(act_map.at(asy_joint_name + l_r_pair.second + "_motor"));
        trajopt->AddRunningCost(w_symmetry_u * act_diff * act_diff);
      }
    }
  }

  MatrixXd Q = 0.02 * MatrixXd::Identity(n_v, n_v);
  MatrixXd R = 1e-4 * MatrixXd::Identity(n_u, n_u);
  R(act_map.at("hip_roll_left_motor"), act_map.at("hip_roll_left_motor")) =
      5e-1;
  R(act_map.at("hip_roll_right_motor"), act_map.at("hip_roll_right_motor")) =
      5e-1;
  R(act_map.at("hip_yaw_left_motor"), act_map.at("hip_yaw_left_motor")) = 5e-1;
  R(act_map.at("hip_yaw_right_motor"), act_map.at("hip_yaw_right_motor")) =
      5e-1;
  R(act_map.at("hip_pitch_left_motor"), act_map.at("hip_pitch_left_motor")) =
      5e-5;
  R(act_map.at("hip_pitch_right_motor"), act_map.at("hip_pitch_right_motor")) =
      5e-5;

  Q *= FLAGS_cost_scaling;
  R *= FLAGS_cost_scaling;
  trajopt->AddRunningCost(x.tail(n_v).transpose() * Q * x.tail(n_v));
  trajopt->AddRunningCost(u.transpose() * R * u);

  vector<int> mode_lengths = {FLAGS_knot_points, FLAGS_knot_points,
                              FLAGS_knot_points};
  MatrixXd W = 1e-3 * MatrixXd::Identity(n_v, n_v);
  W(vel_map["hip_pitch_leftdot"], vel_map["hip_pitch_leftdot"]) = 5e-3;
  W(vel_map["hip_pitch_rightdot"], vel_map["hip_pitch_rightdot"]) = 5e-3;
  W(vel_map["hip_roll_leftdot"], vel_map["hip_roll_leftdot"]) = 1e-3;
  W(vel_map["hip_roll_rightdot"], vel_map["hip_roll_rightdot"]) = 1e-3;
  W(vel_map["toe_leftdot"], vel_map["toe_leftdot"]) = 1e-4;
  W(vel_map["toe_rightdot"], vel_map["toe_rightdot"]) = 1e-4;
  W *= 2.0 * FLAGS_cost_scaling;
  vector<std::shared_ptr<JointAccelCost>> joint_accel_costs;
  for (int mode : {0, 1, 2}) {
    joint_accel_costs.push_back(std::make_shared<JointAccelCost>(
        W, plant, &constraints->mode(mode).evaluators()));
    for (int index = 0; index < mode_lengths[mode]; index++) {
      // Assumes mode_lengths are the same across modes
      auto x_i = trajopt->state_vars(mode, index);
      auto u_i = trajopt->input_vars(mode, index);
      auto l_i = trajopt->force_vars(mode, index);
      trajopt->prog().AddCost(joint_accel_costs[mode], {x_i, u_i, l_i});
    }
  }
}

/******
COSTS
******/
void AddCostsSprings(Dircon<double>* trajopt,
                     const MultibodyPlant<double>& plant,
                     DirconModeSequence<double>* constraints) {
  auto x = trajopt->state();
  auto u = trajopt->input();

  map<string, int> pos_map = multibody::MakeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::MakeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::MakeNameToActuatorsMap(plant);

  int n_v = plant.num_velocities();
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

  double w_symmetry_pos = 1e3;
  double w_symmetry_vel = 1e1;
  for (const auto& l_r_pair : l_r_pairs) {
    for (const auto& sym_joint_name : sym_joint_names) {
      auto pos_diff = x(pos_map[sym_joint_name + l_r_pair.first]) -
          x(pos_map[sym_joint_name + l_r_pair.second]);
      auto vel_diff = x(vel_map[sym_joint_name + l_r_pair.first + "dot"]) -
          x(vel_map[sym_joint_name + l_r_pair.second + "dot"]);
      trajopt->AddRunningCost(w_symmetry_pos * pos_diff * pos_diff);
      trajopt->AddRunningCost(w_symmetry_vel * vel_diff * vel_diff);
    }
  }

  std::cout << "n_v_: " << n_v << std::endl;
  MatrixXd Q = 0.01 * MatrixXd::Identity(n_v, n_v);
  MatrixXd R = 1e-4 * MatrixXd::Identity(n_u, n_u);
  Q *= FLAGS_cost_scaling;
  R *= FLAGS_cost_scaling;

  trajopt->AddRunningCost(x.tail(n_v).transpose() * Q * x.tail(n_v));
  trajopt->AddRunningCost(u.transpose() * R * u);

  vector<int> mode_lengths = {FLAGS_knot_points, FLAGS_knot_points,
                              FLAGS_knot_points};
  MatrixXd W = 1e-3 * MatrixXd::Identity(n_v, n_v);
  W(vel_map["hip_pitch_leftdot"], vel_map["hip_pitch_leftdot"]) *= 1e1;
  W(vel_map["hip_pitch_rightdot"], vel_map["hip_pitch_rightdot"]) *= 1e1;
  W(vel_map["hip_roll_leftdot"], vel_map["hip_roll_leftdot"]) *= 1e1;
  W(vel_map["hip_roll_rightdot"], vel_map["hip_roll_rightdot"]) *= 1e1;
  // Add no costs on spring acceleration because we can't control for that
  W(vel_map["knee_joint_leftdot"], vel_map["knee_joint_leftdot"]) = 0.0;
  W(vel_map["knee_joint_rightdot"], vel_map["knee_joint_rightdot"]) = 0.0;
  W(vel_map["ankle_spring_joint_leftdot"],
    vel_map["ankle_spring_joint_leftdot"]) = 0.0;
  W(vel_map["ankle_spring_joint_rightdot"],
    vel_map["ankle_spring_joint_rightdot"]) = 0.0;
  W *= FLAGS_cost_scaling;
  vector<std::shared_ptr<JointAccelCost>> joint_accel_costs;
  for (int mode : {0, 1, 2}) {
    joint_accel_costs.push_back(std::make_shared<JointAccelCost>(
        W, plant, &constraints->mode(mode).evaluators()));
    for (int index = 0; index < mode_lengths[mode]; index++) {
      // Assumes mode_lengths are the same across modes
      auto x_i = trajopt->state_vars(mode, index);
      auto u_i = trajopt->input_vars(mode, index);
      auto l_i = trajopt->force_vars(mode, index);
      trajopt->prog().AddCost(joint_accel_costs[mode], {x_i, u_i, l_i});
    }
  }
}

void SetInitialGuessFromTrajectory(Dircon<double>& trajopt,
                                   const MultibodyPlant<double>& plant,
                                   const string& filepath,
                                   bool same_knot_points,
                                   Eigen::MatrixXd spr_map) {
  DirconTrajectory previous_traj = DirconTrajectory(plant, filepath);
  if (same_knot_points) {
    trajopt.prog().SetInitialGuessForAllVariables(
        previous_traj.GetDecisionVariables());
    return;
  }
  PiecewisePolynomial<double> state_traj;
  if (FLAGS_use_springs && FLAGS_convert_to_springs) {
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
  for (int mode = 0; mode < trajopt.num_modes(); ++mode) {
    trajopt.SetInitialForceTrajectory(mode, lambda_traj[mode],
                                      lambda_c_traj[mode], gamma_traj[mode]);
  }
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::DoMain();
}
