#include <algorithm>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>

#include <gflags/gflags.h>

#include "common/file_utils.h"
#include "common/find_resource.h"
#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "examples/Cassie/cassie_utils.h"
#include "lcm/dircon_saved_trajectory.h"
#include "multibody/com_pose_system.h"
#include "multibody/kinematic/kinematic_constraints.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "solvers/nonlinear_constraint.h"
#include "solvers/optimization_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/trajectory_optimization/dircon/dircon.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

using std::cout;
using std::endl;
using std::map;
using std::shared_ptr;
using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::VectorX;
using drake::geometry::DrakeVisualizer;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::SolutionResult;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using drake::trajectories::PiecewisePolynomial;

using dairlib::systems::SubvectorPassThrough;

DEFINE_string(init_file, "", "the file name of initial guess");
DEFINE_string(data_directory, "../dairlib_data/cassie_trajopt_data/",
              "directory to save/read data");
DEFINE_string(save_filename, "default_filename",
              "Filename to save decision vars to.");
DEFINE_bool(store_data, false, "To store solution or not");
DEFINE_int32(max_iter, 100, "Iteration limit");
DEFINE_int32(N, 20, "Number of knotpoints");
DEFINE_double(duration, 0.4, "Duration of the single support phase (s)");
DEFINE_double(tol, 1e-4, "Tolerance for constraint violation and dual gap");
DEFINE_bool(ipopt, false, "Use IPOPT as solver instead of SNOPT");
DEFINE_bool(playback, true, "Playback the solution");

// Parameters which enable dircon-improving features
DEFINE_bool(scale_constraint, true, "Scale the nonlinear constraint values");
DEFINE_bool(scale_variable, false, "Scale the decision variable");

namespace dairlib {
using systems::trajectory_optimization::Dircon;
using systems::trajectory_optimization::DirconMode;
using systems::trajectory_optimization::KinematicConstraintType;

void DoMain(double duration, int max_iter, const string& data_directory,
            const string& init_file, double tol, bool to_store_data) {
  // Create fix-spring Cassie MBP
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant(0.0);
  MultibodyPlant<double> plant_vis(0.0);

  Parser parser(&plant);
  Parser parser_vis(&plant_vis, &scene_graph);

  string full_name =
      FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
  parser.AddModelFromFile(full_name);
  parser_vis.AddModelFromFile(full_name);
  plant.Finalize();
  plant_vis.Finalize();

  // Create maps for joints
  map<string, int> positions_map = multibody::MakeNameToPositionsMap(plant);
  map<string, int> velocities_map = multibody::MakeNameToVelocitiesMap(plant);
  map<string, int> actuators_map = multibody::MakeNameToActuatorsMap(plant);

  int base_qw_idx = positions_map.at("base_qw");
  int base_qx_idx = positions_map.at("base_qx");
  int base_qy_idx = positions_map.at("base_qy");
  int base_qz_idx = positions_map.at("base_qz");
  int base_x_idx = positions_map.at("base_x");
  int base_y_idx = positions_map.at("base_y");
  int base_z_idx = positions_map.at("base_z");
  int hip_roll_left_idx = positions_map.at("hip_roll_left");
  int hip_roll_right_idx = positions_map.at("hip_roll_right");
  int hip_yaw_left_idx = positions_map.at("hip_yaw_left");
  int hip_yaw_right_idx = positions_map.at("hip_yaw_right");
  int hip_pitch_left_idx = positions_map.at("hip_pitch_left");
  int hip_pitch_right_idx = positions_map.at("hip_pitch_right");
  int knee_left_idx = positions_map.at("knee_left");
  int knee_right_idx = positions_map.at("knee_right");
  int ankle_joint_left_idx = positions_map.at("ankle_joint_left");
  int ankle_joint_right_idx = positions_map.at("ankle_joint_right");
  int toe_left_idx = positions_map.at("toe_left");
  int toe_right_idx = positions_map.at("toe_right");

  int base_wx_idx = velocities_map.at("base_wx");
  int base_wy_idx = velocities_map.at("base_wy");
  int base_wz_idx = velocities_map.at("base_wz");
  int base_vx_idx = velocities_map.at("base_vx");
  int base_vy_idx = velocities_map.at("base_vy");
  int base_vz_idx = velocities_map.at("base_vz");
  int hip_roll_leftdot_idx = velocities_map.at("hip_roll_leftdot");
  int hip_roll_rightdot_idx = velocities_map.at("hip_roll_rightdot");
  int hip_yaw_leftdot_idx = velocities_map.at("hip_yaw_leftdot");
  int hip_yaw_rightdot_idx = velocities_map.at("hip_yaw_rightdot");
  int hip_pitch_leftdot_idx = velocities_map.at("hip_pitch_leftdot");
  int hip_pitch_rightdot_idx = velocities_map.at("hip_pitch_rightdot");
  int knee_leftdot_idx = velocities_map.at("knee_leftdot");
  int knee_rightdot_idx = velocities_map.at("knee_rightdot");
  int ankle_joint_leftdot_idx = velocities_map.at("ankle_joint_leftdot");
  int ankle_joint_rightdot_idx = velocities_map.at("ankle_joint_rightdot");
  int toe_leftdot_idx = velocities_map.at("toe_leftdot");
  int toe_rightdot_idx = velocities_map.at("toe_rightdot");

  int hip_roll_left_motor_idx = actuators_map.at("hip_roll_left_motor");
  int hip_roll_right_motor_idx = actuators_map.at("hip_roll_right_motor");
  int hip_yaw_left_motor_idx = actuators_map.at("hip_yaw_left_motor");
  int hip_yaw_right_motor_idx = actuators_map.at("hip_yaw_right_motor");
  int hip_pitch_left_motor_idx = actuators_map.at("hip_pitch_left_motor");
  int hip_pitch_right_motor_idx = actuators_map.at("hip_pitch_right_motor");
  int knee_left_motor_idx = actuators_map.at("knee_left_motor");
  int knee_right_motor_idx = actuators_map.at("knee_right_motor");
  int toe_left_motor_idx = actuators_map.at("toe_left_motor");
  int toe_right_motor_idx = actuators_map.at("toe_right_motor");

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();
  // int n_x = n_q + n_v;

  // Set up contact/distance evaluators
  auto left_loop_eval = LeftLoopClosureEvaluator(plant);
  auto right_loop_eval = RightLoopClosureEvaluator(plant);

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

  auto evaluators = multibody::KinematicEvaluatorSet<double>(plant);
  int left_toe_eval_ind = evaluators.add_evaluator(&left_toe_eval);
  int left_heel_eval_ind = evaluators.add_evaluator(&left_heel_eval);
  int right_toe_eval_ind = evaluators.add_evaluator(&right_toe_eval);
  int right_heel_eval_ind = evaluators.add_evaluator(&right_heel_eval);
  evaluators.add_evaluator(&left_loop_eval);
  evaluators.add_evaluator(&right_loop_eval);

  int num_knotpoints = FLAGS_N;
  double min_T = .2;
  double max_T = 5;
  auto double_support =
      DirconMode<double>(evaluators, num_knotpoints, min_T, max_T);

  // Set x-y coordinates as relative
  double_support.MakeConstraintRelative(left_toe_eval_ind, 0);    // x
  double_support.MakeConstraintRelative(left_toe_eval_ind, 1);    // y
  double_support.MakeConstraintRelative(left_heel_eval_ind, 0);   // x
  double_support.MakeConstraintRelative(left_heel_eval_ind, 1);   // y
  double_support.MakeConstraintRelative(right_toe_eval_ind, 0);   // x
  double_support.MakeConstraintRelative(right_toe_eval_ind, 1);   // y
  double_support.MakeConstraintRelative(right_heel_eval_ind, 0);  // x
  double_support.MakeConstraintRelative(right_heel_eval_ind, 1);  // y

  // Constraint scaling
  if (FLAGS_scale_constraint) {
    // Dynamic constraints
    double s_dyn_1 = (FLAGS_scale_variable) ? 2.0 : 1.0;
    double s_dyn_2 = (FLAGS_scale_variable) ? 6.0 : 1.0;
    double s_dyn_3 = (FLAGS_scale_variable) ? 85.0 : 1.0;
    double_support.SetDynamicsScale(
        {base_qw_idx, base_qx_idx, base_qy_idx, base_qz_idx, base_x_idx,
         base_y_idx, base_z_idx, hip_roll_left_idx, hip_roll_right_idx,
         hip_yaw_left_idx, hip_yaw_right_idx, hip_pitch_left_idx,
         hip_pitch_right_idx, knee_left_idx, knee_right_idx},
        1.0 / 150.0);
    double_support.SetDynamicsScale(
        {ankle_joint_left_idx, ankle_joint_right_idx},
        1.0 / 150.0 / 3.33 / s_dyn_1);
    double_support.SetDynamicsScale({toe_left_idx, toe_right_idx}, 1.0 / 150.0);
    double_support.SetDynamicsScale(
        {base_wx_idx, base_wy_idx, base_wz_idx, base_vx_idx, base_vy_idx,
         base_vz_idx, hip_roll_leftdot_idx, hip_roll_rightdot_idx},
        1.0 / 150.0 / s_dyn_1);
    double_support.SetDynamicsScale({hip_yaw_leftdot_idx, hip_yaw_rightdot_idx},
                                    1.0 / 150.0 / s_dyn_2);
    double_support.SetDynamicsScale(
        {hip_pitch_leftdot_idx, hip_pitch_rightdot_idx, knee_leftdot_idx,
         knee_rightdot_idx, ankle_joint_leftdot_idx, ankle_joint_rightdot_idx},
        1.0 / 150.0 / 10);
    double_support.SetDynamicsScale({toe_leftdot_idx, toe_rightdot_idx},
                                    1.0 / 150.0 / 15.0 / s_dyn_3);

    // // Kinematic constraints
    double s_kin_vel = 500;
    double s_kin_pos = 1000;
    double s_kin_1 = (FLAGS_scale_variable) ? 10.0 : 1.0;
    double s_kin_2 = (FLAGS_scale_variable) ? 2.0 : 1.0;

    double dist_scale = 2.0 / 55;

    // double_support.SetKinAccelerationScale({4, 5}, {0}, dist_scale /
    // s_kin_1);

    double_support.SetKinVelocityScale(
        {0, 1, 2, 3}, {0, 1, 2}, 1.0 / 500.0 * s_kin_vel * s_kin_2 / s_kin_1);
    double_support.SetKinVelocityScale(
        {4, 5}, {0}, dist_scale * s_kin_vel * s_kin_2 / s_kin_1);

    double_support.SetKinPositionScale(
        {0, 1, 2, 3}, {0, 1, 2}, 1.0 / 500.0 * s_kin_pos * s_kin_2 / s_kin_1);
    double_support.SetKinPositionScale(
        {4, 5}, {0}, dist_scale * s_kin_pos * s_kin_2 / s_kin_1);
  }

  for (int i = 0; i < num_knotpoints; i++) {
    double_support.SkipQuaternionConstraint(i);
  }
  // double_support.set_constraint_type(0, KinematicConstraintType::kAccelOnly);
  // double_support.set_constraint_type(num_knotpoints - 1,
  //                                   KinematicConstraintType::kAccelOnly);

  auto trajopt = Dircon<double>(&double_support);
  auto& prog = trajopt.prog();

  if (FLAGS_ipopt) {
    // Ipopt settings adapted from CaSaDi and FROST
    auto id = drake::solvers::IpoptSolver::id();
    prog.SetSolverOption(id, "tol", tol);
    prog.SetSolverOption(id, "dual_inf_tol", tol);
    prog.SetSolverOption(id, "constr_viol_tol", tol);
    prog.SetSolverOption(id, "compl_inf_tol", tol);
    prog.SetSolverOption(id, "max_iter", max_iter);
    prog.SetSolverOption(id, "nlp_lower_bound_inf", -1e6);
    prog.SetSolverOption(id, "nlp_upper_bound_inf", 1e6);
    prog.SetSolverOption(id, "print_timing_statistics", "yes");
    prog.SetSolverOption(id, "print_level", 5);

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
    // prog.SetSolverOption(id, "Print file", "../snopt.out");
    prog.SetSolverOption(id, "Major iterations limit", max_iter);
    prog.SetSolverOption(id, "Iterations limit", 100000);
    prog.SetSolverOption(id, "Verify level", 0);

    // snopt doc said try 2 if seeing snopta exit 40
    prog.SetSolverOption(id, "Scale option", 0);

    // target nonlinear constraint violation
    prog.SetSolverOption(id, "Major optimality tolerance", tol);

    // target complementarity gap
    prog.SetSolverOption(id, "Major feasibility tolerance", tol);
  }

  // Get the decision variables that will be used
  auto u = trajopt.input();
  auto x = trajopt.state();
  auto x0 = trajopt.initial_state();
  auto xf = trajopt.state_vars(0, num_knotpoints - 1);
  auto xmid = trajopt.state_vars(0, (num_knotpoints - 1) / 2);

  // height constraint
  prog.AddBoundingBoxConstraint(1, 1, x0(positions_map.at("base_z")));
  prog.AddBoundingBoxConstraint(1.1, 1.1, xf(positions_map.at("base_z")));

  // initial pelvis position
  prog.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_x")));
  prog.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_y")));

  // pelvis pose constraints
  for (int i = 0; i < num_knotpoints; i++) {
    auto xi = trajopt.state(i);
    prog.AddBoundingBoxConstraint(1, 1, xi(positions_map.at("base_qw")));
    prog.AddBoundingBoxConstraint(0, 0, xi(positions_map.at("base_qx")));
    prog.AddBoundingBoxConstraint(0, 0, xi(positions_map.at("base_qy")));
    prog.AddBoundingBoxConstraint(0, 0, xi(positions_map.at("base_qz")));
  }

  // start/end velocity constraints
  prog.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v),
                                   x0.tail(n_v));
  prog.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v),
                                   xf.tail(n_v));

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

  // joint limits
  for (const auto& member : joint_names) {
    trajopt.AddConstraintToAllKnotPoints(
        x(positions_map.at(member)) <=
        plant.GetJointByName(member).position_upper_limits()(0));
    trajopt.AddConstraintToAllKnotPoints(
        x(positions_map.at(member)) >=
        plant.GetJointByName(member).position_lower_limits()(0));
  }

  // u limit
  for (int i = 0; i < num_knotpoints; i++) {
    auto ui = trajopt.input_vars(0, i);
    prog.AddBoundingBoxConstraint(VectorXd::Constant(n_u, -300),
                                     VectorXd::Constant(n_u, +300), ui);
  }

  // toe position constraint in y direction (avoid leg crossing)
  std::vector<int> y_active({1});
  auto left_foot_y_eval = multibody::WorldPointEvaluator(
      plant, Eigen::Vector3d::Zero(), left_toe_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), y_active);
  auto right_foot_y_eval = multibody::WorldPointEvaluator(
      plant, Eigen::Vector3d::Zero(), right_toe_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), y_active);
  auto foot_y_evaluators = multibody::KinematicEvaluatorSet<double>(plant);
  foot_y_evaluators.add_evaluator(&left_foot_y_eval);
  foot_y_evaluators.add_evaluator(&right_foot_y_eval);

  auto foot_y_lb =
      Eigen::Vector2d(0.05, -std::numeric_limits<double>::infinity());
  auto foot_y_ub =
      Eigen::Vector2d(std::numeric_limits<double>::infinity(), -0.05);
  auto foot_y_constraint =
      std::make_shared<multibody::KinematicPositionConstraint<double>>(
          plant, foot_y_evaluators, foot_y_lb, foot_y_ub);

  // scaling
  if (FLAGS_scale_constraint) {
    std::unordered_map<int, double> odbp_constraint_scale;
    odbp_constraint_scale.insert(std::pair<int, double>(0, 0.5));
    odbp_constraint_scale.insert(std::pair<int, double>(1, 0.5));
    foot_y_constraint->SetConstraintScaling(odbp_constraint_scale);
  }
  for (int index = 0; index < num_knotpoints; index++) {
    auto x = trajopt.state(index);
    prog.AddConstraint(foot_y_constraint, x.head(n_q));
  }

  // add cost
  const MatrixXd Q = 10 * 12.5 * MatrixXd::Identity(n_v, n_v);
  const MatrixXd R = 12.5 * MatrixXd::Identity(n_u, n_u);
  trajopt.AddRunningCost(x.tail(n_v).transpose() * Q * x.tail(n_v));
  trajopt.AddRunningCost(u.transpose() * R * u);

  // Scale variable
  // Scaling decision variable doesn't seem to help in the task of squatting.
  // One hypothesis is that the initial guess we feed to the solver is very
  // good, so the variable scaling doesn't matter to much.
  if (FLAGS_scale_variable) {
    // time
    trajopt.ScaleTimeVariables(0.015);
    // state
    std::vector<int> idx_list = {
        n_q + base_wx_idx,          n_q + base_wy_idx,
        n_q + base_wz_idx,          n_q + base_vx_idx,
        n_q + base_vy_idx,          n_q + base_vz_idx,
        n_q + hip_roll_leftdot_idx, n_q + hip_roll_rightdot_idx,
        n_q + hip_yaw_leftdot_idx,  n_q + hip_yaw_rightdot_idx};
    trajopt.ScaleStateVariables(idx_list, 6);
    idx_list.clear();
    idx_list = {n_q + hip_pitch_leftdot_idx,   n_q + hip_pitch_rightdot_idx,
                n_q + knee_leftdot_idx,        n_q + knee_rightdot_idx,
                n_q + ankle_joint_leftdot_idx, n_q + ankle_joint_rightdot_idx,
                n_q + toe_leftdot_idx,         n_q + toe_rightdot_idx};
    trajopt.ScaleStateVariables(idx_list, 3);
    // input
    trajopt.ScaleInputVariables(
        {hip_roll_left_motor_idx, hip_roll_right_motor_idx}, 60);
    trajopt.ScaleInputVariables(
        {hip_yaw_left_motor_idx, hip_yaw_right_motor_idx}, 300);  // 300
    trajopt.ScaleInputVariables(
        {hip_pitch_left_motor_idx, hip_pitch_right_motor_idx,
         knee_left_motor_idx, knee_right_motor_idx},
        60);
    trajopt.ScaleInputVariables({toe_left_motor_idx, toe_right_motor_idx},
                                600);  // 600
    // force
    trajopt.ScaleForceVariables(0, {0, 1}, 10);
    trajopt.ScaleForceVariables(0, {2, 2}, 1000);  // 1000
    trajopt.ScaleForceVariables(0, {3, 4}, 10);
    trajopt.ScaleForceVariable(0, 5, 1000);
    trajopt.ScaleForceVariables(0, {6, 7}, 10);
    trajopt.ScaleForceVariable(0, 8, 1000);
    trajopt.ScaleForceVariables(0, {9, 10}, 10);
    trajopt.ScaleForceVariable(0, 11, 1000);
    trajopt.ScaleForceVariables(0, {12, 13}, 600);

    // Print out the scaling factors
    /*for (int i=0; i < trajopt.decision_variables().size() ; i++) {
      cout << trajopt.decision_variable(i) << ", ";
      cout << trajopt.decision_variable(i).get_id() << ", ";
      cout << trajopt.FindDecisionVariableIndex(trajopt.decision_variable(i))
          << ", ";
      auto scale_map = trajopt.GetVariableScaling();
      auto it = scale_map.find(i);
      if (it != scale_map.end()) {
        cout << it->second;
      }
      cout << endl;
    }*/
  }

  // initial guess
  if (!init_file.empty()) {
    MatrixXd z0 = readCSV(data_directory + init_file);
    prog.SetInitialGuessForAllVariables(z0);
  } else {
    // Add random initial guess first (the seed for RNG is fixed)
    prog.SetInitialGuessForAllVariables(
        VectorXd::Random(prog.decision_variables().size()));

    VectorXd q0, qf, u0, uf, lambda0, lambdaf;
    double min_normal_force = 70;
    double toe_spread = .3;
    double init_height = 1.0;
    double final_height = 1.1;
    double init_time = .5;
    CassieFixedPointSolver(plant, init_height, 0, min_normal_force, true,
                           toe_spread, &q0, &u0, &lambda0);
    CassieFixedPointSolver(plant, final_height, 0, min_normal_force, true,
                           toe_spread, &qf, &uf, &lambdaf);
    // Build spline
    VectorXd times(2);
    times << 0, init_time;
    // Use cubic spline for state with zero endpoint derivatives
    MatrixXd state_matrix(plant.num_positions() + plant.num_velocities(), 2);
    VectorXd zero_velocity = VectorXd::Zero(plant.num_velocities());
    state_matrix.col(0) << q0, zero_velocity;
    state_matrix.col(1) << qf, zero_velocity;
    auto state_spline = PiecewisePolynomial<double>::CubicShapePreserving(
        times, state_matrix, true);

    // Use FOH for input and forces
    MatrixXd input_matrix(plant.num_actuators(), 2);
    input_matrix << u0, uf;
    auto input_spline =
        PiecewisePolynomial<double>::FirstOrderHold(times, input_matrix);

    MatrixXd force_matrix(lambda0.size(), 2);
    force_matrix << lambda0, lambdaf;
    auto force_spline =
        PiecewisePolynomial<double>::FirstOrderHold(times, force_matrix);

    trajopt.SetInitialForceTrajectory(0, force_spline);
    trajopt.SetInitialTrajectory(input_spline, state_spline);
  }
  // Careful: MUST set the initial guess for quaternion, since 0-norm quaternion
  // produces NAN value in some calculation.
  for (int i = 0; i < num_knotpoints; i++) {
    auto xi = trajopt.state(i);
    if ((prog.GetInitialGuess(xi.segment<4>(base_qw_idx)).norm() == 0) ||
        std::isnan(
            prog.GetInitialGuess(xi.segment<4>(base_qw_idx)).norm())) {
      prog.SetInitialGuess(xi(base_qw_idx), 1);
      prog.SetInitialGuess(xi(base_qx_idx), 0);
      prog.SetInitialGuess(xi(base_qy_idx), 0);
      prog.SetInitialGuess(xi(base_qz_idx), 0);
    }
  }

  double alpha = .2;
  int num_poses = std::min(num_knotpoints, 5);
  trajopt.CreateVisualizationCallback(
      "examples/Cassie/urdf/cassie_fixed_springs.urdf", num_poses, alpha);

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
  auto solver = drake::solvers::MakeSolver(solver_id);
  drake::solvers::MathematicalProgramResult result;
  solver->Solve(prog, prog.initial_guess(),
                prog.solver_options(),
                &result);
  SolutionResult solution_result = result.get_solution_result();
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  // trajopt.PrintSolution();
  for (int i = 0; i < 100; i++) {
    cout << '\a';
  }  // making noise to notify
  cout << "\n" << to_string(solution_result) << endl;
  cout << "Solve time:" << elapsed.count() << std::endl;
  cout << "Cost:" << result.get_optimal_cost() << std::endl;

  // Save trajectory to file
  if (!FLAGS_save_filename.empty()) {
    DirconTrajectory saved_traj(
        plant, trajopt, result, "walking_trajectory",
        "Decision variables and state/input trajectories "
        "for walking");
    saved_traj.WriteToFile(FLAGS_data_directory + FLAGS_save_filename);
    std::cout << "Wrote to file: " << FLAGS_data_directory + FLAGS_save_filename
              << std::endl;
  }

  // Check which solver was used
  cout << "Solver: " << result.get_solver_id().name() << endl;

  // store the solution of the decision variable
  VectorXd z = result.GetSolution(prog.decision_variables());
  VectorXd constraint_y, constraint_lb, constraint_ub;
  MatrixXd constraint_A;
  solvers::LinearizeConstraints(
      prog, z, &constraint_y,&constraint_A,
      &constraint_lb, &constraint_ub);
  if (to_store_data) {
    writeCSV(data_directory + string("z.csv"), z);
    writeCSV(data_directory + string("A.csv"), constraint_A);
    writeCSV(data_directory + string("y.csv"), constraint_y);
    writeCSV(data_directory + string("lb.csv"), constraint_lb);
    writeCSV(data_directory + string("ub.csv"), constraint_ub);
  }
  // for (int i = 0; i < z.size(); i++) {
  //   cout << trajopt.decision_variables()[i] << ", " << z[i] << endl;
  // }
  // cout << endl;

  // Check if the nonlinear constraints are all satisfied
  // solvers::CheckGenericConstraints(trajopt, result, tol);
  // cout << "constraint_satisfied = " << constraint_satisfied << endl;

  // store the time, state, and input at knot points
  VectorXd time_at_knots = trajopt.GetSampleTimes(result);
  MatrixXd state_at_knots = trajopt.GetStateSamples(result);
  MatrixXd input_at_knots = trajopt.GetInputSamples(result);
  state_at_knots.col(num_knotpoints - 1) = result.GetSolution(xf);

  if (to_store_data) {
    writeCSV(data_directory + string("t_i.csv"), time_at_knots);
    writeCSV(data_directory + string("x_i.csv"), state_at_knots);
    writeCSV(data_directory + string("u_i.csv"), input_at_knots);
  }

  // visualizer
  const PiecewisePolynomial<double> pp_xtraj =
      trajopt.ReconstructStateTrajectory(result);

  auto traj_source =
      builder.AddSystem<drake::systems::TrajectorySource>(pp_xtraj);
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      plant.num_positions() + plant.num_velocities(), 0, plant.num_positions());
  builder.Connect(traj_source->get_output_port(),
                  passthrough->get_input_port());
  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant_vis);
  builder.Connect(passthrough->get_output_port(), to_pose->get_input_port());

  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant_vis.get_source_id().value()));

  // *******Add COM visualization**********
  bool plot_com = true;
  bool com_on_ground = true;
  auto ball_plant = multibody::ConstructBallPlant(&scene_graph);
  if (plot_com) {
    // connect
    auto q_passthrough = builder.AddSystem<SubvectorPassThrough>(
        plant.num_positions() + plant.num_velocities(), 0,
        plant.num_positions());
    builder.Connect(traj_source->get_output_port(),
                    q_passthrough->get_input_port());
    auto rbt_passthrough = builder.AddSystem<multibody::ComPoseSystem>(plant);

    auto ball_to_pose =
        builder.AddSystem<MultibodyPositionToGeometryPose<double>>(*ball_plant);
    builder.Connect(*q_passthrough, *rbt_passthrough);
    if (com_on_ground) {
      builder.Connect(rbt_passthrough->get_xy_com_output_port(),
                      ball_to_pose->get_input_port());
    } else {
      builder.Connect(rbt_passthrough->get_com_output_port(),
                      ball_to_pose->get_input_port());
    }
    builder.Connect(
        ball_to_pose->get_output_port(),
        scene_graph.get_source_pose_port(ball_plant->get_source_id().value()));
  }
  // **************************************

  DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph);
  auto diagram = builder.Build();

  while (FLAGS_playback) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(.1);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
  }
}
}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::DoMain(FLAGS_duration, FLAGS_max_iter, FLAGS_data_directory,
                  FLAGS_init_file, FLAGS_tol, FLAGS_store_data);
}
