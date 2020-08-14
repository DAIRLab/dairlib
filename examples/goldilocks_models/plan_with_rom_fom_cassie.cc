#include <chrono>
#include <gflags/gflags.h>

#include "common/file_utils.h"
#include "common/find_resource.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/planning/rom_traj_opt.h"
#include "examples/goldilocks_models/reduced_order_models.h"

#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

using dairlib::FindResourceOrThrow;
using drake::AutoDiffXd;
using drake::geometry::SceneGraph;
using drake::multibody::Body;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::SolutionResult;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXcd;
using Eigen::VectorXd;
using std::cin;
using std::cout;
using std::endl;
using std::string;
using std::to_string;
using std::vector;

namespace dairlib {
namespace goldilocks_models {

DEFINE_int32(robot_option, 1, "0: plannar robot. 1: cassie_fixed_spring");
DEFINE_int32(rom_option, 4, "See find_goldilocks_models.cc");
DEFINE_int32(iter, 20, "The iteration # of the theta that you use");
DEFINE_int32(sample, 4, "The sample # of the initial condition that you use");

DEFINE_int32(n_step, 3, "Number of foot steps in rom traj opt");
DEFINE_double(final_position, 2, "The final position for the robot");

DEFINE_string(init_file, "", "Initial Guess for Planning Optimization");
DEFINE_int32(knots_per_mode, 24, "Number of knots per mode in rom traj opt");
DEFINE_bool(fix_duration, false, "Fix the total time");
DEFINE_bool(equalize_timestep_size, true, "Make all timesteps the same size");
DEFINE_bool(zero_touchdown_impact, true, "Zero impact at foot touchdown");
DEFINE_double(opt_tol, 1e-4, "");
DEFINE_double(feas_tol, 1e-4, "");

DEFINE_double(init_phase, 0,
              "The phase where the initial FOM pose is throughout the single "
              "support period. This is used to prepare ourselves for MPC");
DEFINE_bool(start_with_left_stance, true,
            "The starting stance of the robot. This is used to prepare "
            "ourselves for MPC");
DEFINE_double(disturbance, 0, "Disturbance to FoM initial state");

DEFINE_bool(log_solver_info, false,
            "Log snopt output to a file or ipopt to terminal");
DEFINE_bool(use_ipopt, false, "use ipopt instead of snopt");
// DEFINE_bool(add_x_pose_in_cost, false, "Add x0 and xf in the cost function");

// Planning with optimal reduced order model and full order model
// (discrete map is from full order model)
int planningWithRomAndFom(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  DRAKE_DEMAND(FLAGS_robot_option == 1);
  DRAKE_DEMAND(0 <= FLAGS_init_phase && FLAGS_init_phase <= 1);

  // Create MBP
  MultibodyPlant<double> plant(0.0);
  CreateMBP(&plant, FLAGS_robot_option);

  // Files parameters
  const string dir = "../dairlib_data/goldilocks_models/planning/robot_" +
                     to_string(FLAGS_robot_option) + "/";
  const string dir_model = dir + "models/";  // location of the model files
  const string dir_data = dir + "data/";     // location to store the opt result
  string init_file = FLAGS_init_file;
  if (!CreateFolderIfNotExist(dir_model)) return 0;
  if (!CreateFolderIfNotExist(dir_data)) return 0;

  // Reduced order model
  std::unique_ptr<ReducedOrderModel> rom =
      CreateRom(FLAGS_rom_option, FLAGS_robot_option, plant, false);
  ReadModelParameters(rom.get(), dir_model, FLAGS_iter);

  // Create mirror maps
  StateMirror state_mirror(MirrorPosIndexMap(plant, FLAGS_robot_option),
                           MirrorPosSignChangeSet(plant, FLAGS_robot_option),
                           MirrorVelIndexMap(plant, FLAGS_robot_option),
                           MirrorVelSignChangeSet(plant, FLAGS_robot_option));

  // Optimization parameters
  int n_y = rom->n_y();
  int n_tau = rom->n_tau();
  MatrixXd Q = MatrixXd::Identity(n_y, n_y);
  MatrixXd R = MatrixXd::Identity(n_tau, n_tau);

  // Prespecify the time steps
  int n_step = FLAGS_n_step;
  int knots_per_mode = FLAGS_knots_per_mode;
  std::vector<int> num_time_samples;
  std::vector<double> min_dt;
  std::vector<double> max_dt;
  for (int i = 0; i < n_step; i++) {
    num_time_samples.push_back(knots_per_mode);
    min_dt.push_back(.01);
    max_dt.push_back(.3);
  }
  int fisrt_mode_phase_index = int(knots_per_mode * FLAGS_init_phase);
  int knots_first_mode = knots_per_mode - fisrt_mode_phase_index;
  num_time_samples[0] = knots_first_mode;
  cout << "init_phase = " << FLAGS_init_phase << endl;
  cout << "knots_first_mode = " << knots_first_mode << endl;
  cout << "fisrt_mode_phase_index = " << fisrt_mode_phase_index << endl;

  // Store data
  writeCSV(dir_data + string("n_step.csv"), n_step * VectorXd::Ones(1));
  writeCSV(dir_data + string("nodes_per_step.csv"),
           knots_per_mode * VectorXd::Ones(1));

  // Read in initial robot state
  string model_dir_n_pref = dir_model + to_string(FLAGS_iter) + string("_") +
                            to_string(FLAGS_sample) + string("_");
  cout << "model_dir_n_pref = " << model_dir_n_pref << endl;
  int n_sample_raw =
      readCSV(model_dir_n_pref + string("time_at_knots.csv")).size();
  VectorXd x_init;  // we assume that solution from files are in left stance
  x_init = readCSV(model_dir_n_pref + string("state_at_knots.csv"))
               .col(int(n_sample_raw * FLAGS_init_phase));
  // Mirror x_init if it's right stance
  if (!FLAGS_start_with_left_stance) {
    x_init.head(plant.num_positions()) =
        state_mirror.MirrorPos(x_init.head(plant.num_positions()));
    x_init.tail(plant.num_velocities()) =
        state_mirror.MirrorVel(x_init.tail(plant.num_velocities()));
  }

  if (FLAGS_disturbance != 0) {
    //    x_init(9) += FLAGS_disturbance / 1;
  }

  // Testing
  std::vector<string> name_list = {"base_qw",
                                   "base_qx",
                                   "base_qy",
                                   "base_qz",
                                   "base_x",
                                   "base_y",
                                   "base_z",
                                   "hip_roll_left",
                                   "hip_roll_right",
                                   "hip_yaw_left",
                                   "hip_yaw_right",
                                   "hip_pitch_left",
                                   "hip_pitch_right",
                                   "knee_left",
                                   "knee_right",
                                   "ankle_joint_left",
                                   "ankle_joint_right",
                                   "toe_left",
                                   "toe_right"};
  std::map<string, int> positions_map =
      multibody::makeNameToPositionsMap(plant);
  /*for (auto name : name_list) {
    cout << name << ", " << init_state(positions_map.at(name)) << endl;
  }*/
  // TODO: find out why the initial left knee position is not within the joint
  //  limits.
  //  Could be that the constraint tolerance is too high in rom optimization

  bool with_init_guess = true;
  // Provide initial guess
  VectorXd h_guess(knots_per_mode);
  MatrixXd r_guess(n_y, knots_per_mode);
  MatrixXd dr_guess(n_y, knots_per_mode);
  MatrixXd tau_guess(n_tau, knots_per_mode);
  VectorXd x_guess_left_in_front;
  VectorXd x_guess_right_in_front;
  if (with_init_guess) {
    VectorXd h_guess_raw =
        readCSV(model_dir_n_pref + string("time_at_knots.csv")).col(0);
    MatrixXd r_guess_raw =
        readCSV(model_dir_n_pref + string("t_and_y.csv")).bottomRows(n_y);
    MatrixXd dr_guess_raw =
        readCSV(model_dir_n_pref + string("t_and_ydot.csv")).bottomRows(n_y);
    MatrixXd tau_guess_raw =
        readCSV(model_dir_n_pref + string("t_and_tau.csv")).bottomRows(n_tau);
    VectorXd x_guess_left_in_front_raw =
        readCSV(model_dir_n_pref + string("state_at_knots.csv")).col(0);
    VectorXd x_guess_right_in_front_raw =
        readCSV(model_dir_n_pref + string("state_at_knots.csv")).rightCols(1);
    cout << "\nWARNING: last column of state_at_knots.csv should be pre-impact "
            "state.\n";
    // TODO: store both pre and post impact in rom optimization

    // TODO: reconstruct cubic spline and resample
    double duration = h_guess_raw.tail(1)(0);
    for (int i = 0; i < knots_per_mode; i++) {
      h_guess(i) = duration / (knots_per_mode - 1) * i;
    }
    for (int i = 0; i < knots_per_mode; i++) {
      int n_mat_col_r = r_guess_raw.cols();
      int idx_r =
          (int)round(double(i * (n_mat_col_r - 1)) / (knots_per_mode - 1));
      r_guess.col(i) = r_guess_raw.col(idx_r);
      int n_mat_col_dr = dr_guess_raw.cols();
      int idx_dr =
          (int)round(double(i * (n_mat_col_dr - 1)) / (knots_per_mode - 1));
      dr_guess.col(i) = dr_guess_raw.col(idx_dr);
      int n_mat_col_tau = tau_guess_raw.cols();
      int idx_tau =
          (int)round(double(i * (n_mat_col_tau - 1)) / (knots_per_mode - 1));
      tau_guess.col(i) = tau_guess_raw.col(idx_tau);
    }
    x_guess_left_in_front = x_guess_left_in_front_raw;
    x_guess_right_in_front = x_guess_right_in_front_raw;

    cout << "initial guess duration = " << duration << endl;
    // cout << "h_guess = " << h_guess << endl;
    // cout << "r_guess = " << r_guess << endl;
    // cout << "dr_guess = " << dr_guess << endl;
    // cout << "tau_guess = " << tau_guess << endl;
    // cout << "x_guess_left_in_front = " << x_guess_left_in_front << endl;
    // cout << "x_guess_right_in_front = " << x_guess_right_in_front << endl;
  }

  // Get foot contacts
  bool one_contact_per_foot = true;
  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);
  auto right_toe = RightToeFront(plant);
  auto right_heel = RightToeRear(plant);
  Vector3d mid_contact_point = (left_toe.first + left_heel.first) / 2;
  auto left_toe_mid = std::pair<const Vector3d, const Frame<double>&>(
      mid_contact_point, plant.GetFrameByName("toe_left"));
  auto right_toe_mid = std::pair<const Vector3d, const Frame<double>&>(
      mid_contact_point, plant.GetFrameByName("toe_right"));
  std::vector<std::pair<const Vector3d, const drake::multibody::Frame<double>&>>
      left_contacts;
  std::vector<std::pair<const Vector3d, const drake::multibody::Frame<double>&>>
      right_contacts;
  if (one_contact_per_foot) {
    left_contacts.push_back(left_toe_mid);
    right_contacts.push_back(right_toe_mid);
  } else {
    left_contacts.push_back(left_toe);
    left_contacts.push_back(left_heel);
    right_contacts.push_back(right_toe);
    right_contacts.push_back(right_heel);
  }

  // Get joint limits of the robot
  std::vector<string> l_r_pair = {"_left", "_right"};
  std::vector<std::string> joint_names = {
      "hip_roll", "hip_yaw", "hip_pitch", "knee", "ankle_joint", "toe"};
  std::vector<std::tuple<string, double, double>> joint_name_lb_ub;
  for (const auto& left_right : l_r_pair) {
    for (const auto& name : joint_names) {
      joint_name_lb_ub.emplace_back(
          name + left_right,
          plant.GetJointByName(name + left_right).position_lower_limits()(0),
          plant.GetJointByName(name + left_right).position_upper_limits()(0));
      /*cout << "name = " << name + left_right << endl;
      cout << "lb = "
           << plant.GetJointByName(name + left_right).position_lower_limits()(0)
           << endl;
      cout << "ub = "
           << plant.GetJointByName(name + left_right).position_upper_limits()(0)
           << endl;*/
    }
  }

  // Goal position
  VectorXd final_position(2);
  final_position << FLAGS_final_position, 0;

  // Construct
  cout << "\nConstructing optimization problem...\n";
  auto start = std::chrono::high_resolution_clock::now();
  RomTrajOptCassie trajopt(num_time_samples, Q, R, *rom, plant, state_mirror,
                           left_contacts, right_contacts, joint_name_lb_ub,
                           x_init, FLAGS_start_with_left_stance,
                           FLAGS_zero_touchdown_impact);

  cout << "Other constraints/costs and initial guess=============\n";
  // Time step cosntraints
  int n_time_samples =
      std::accumulate(num_time_samples.begin(), num_time_samples.end(), 0) -
      num_time_samples.size() + 1;
  trajopt.AddTimeStepConstraint(min_dt, max_dt, FLAGS_equalize_timestep_size,
                                FLAGS_fix_duration,
                                h_guess(1) * (n_time_samples - 1));

  // Constraints for fourbar linkage
  // Note that if the initial pose in the constraint doesn't obey the fourbar
  // linkage relationship. You shouldn't add constraint to the initial pose here
  /*double fourbar_angle = 13.0 / 180.0 * M_PI;
  auto q0_var = trajopt.x0_vars_by_mode(0).head(plant.num_positions());
  trajopt.AddLinearEqualityConstraint(
      q0_var(positions_map.at("knee_left")) +
      q0_var(positions_map.at("ankle_joint_left")), fourbar_angle);
  trajopt.AddLinearEqualityConstraint(
      q0_var(positions_map.at("knee_right")) +
      q0_var(positions_map.at("ankle_joint_right")), fourbar_angle);*/
  /*for (int i = 0; i < num_time_samples.size(); i++) {
    auto qf_var = trajopt.xf_vars_by_mode(i);
    trajopt.AddLinearEqualityConstraint(
        qf_var(positions_map.at("knee_left")) +
        qf_var(positions_map.at("ankle_joint_left")), fourbar_angle);
    trajopt.AddLinearEqualityConstraint(
        qf_var(positions_map.at("knee_right")) +
        qf_var(positions_map.at("ankle_joint_right")), fourbar_angle);
  }*/

  // Final goal position constraint
  cout << "Adding final position constraint for full-order model...\n";
  trajopt.AddBoundingBoxConstraint(
      final_position, final_position,
      trajopt.xf_vars_by_mode(num_time_samples.size() - 1).segment(4, 2));

  // Add_robot state in cost
  bool add_x_pose_in_cost = true;
  if (add_x_pose_in_cost) {
    trajopt.AddRegularizationCost(final_position, x_guess_left_in_front,
                                  x_guess_right_in_front,
                                  false /*straight_leg_cost*/);

    // Add more regularization cost
    /*auto qf_var = trajopt.xf_vars_by_mode(num_time_samples.size()-1);
    VectorXd quat_unity(4);
    quat_unity << 1, 0, 0, 0;
    trajopt.AddQuadraticErrorCost(100*MatrixXd::Identity(4, 4), quat_unity,
                                  qf_var.head(4));*/
  } else {
    // Since there are multiple q that could be mapped to the same r, I
    // penalize on q so it get close to a certain configuration
    MatrixXd Id = MatrixXd::Identity(3, 3);
    VectorXd zero_vec = VectorXd::Zero(3);
    for (int i = 0; i < num_time_samples.size(); i++) {
      trajopt.AddQuadraticErrorCost(Id, zero_vec,
                                    trajopt.xf_vars_by_mode(i).segment(1, 3));
    }
  }

  // Default initial guess to avoid singularity (which messes with gradient)
  for (int i = 0; i < num_time_samples.size(); i++) {
    for (int j = 0; j < num_time_samples[i]; j++) {
      if ((FLAGS_rom_option == 0) || (FLAGS_rom_option == 1)) {
        trajopt.SetInitialGuess((trajopt.state_vars_by_mode(i, j))(1), 1);
      } else if (FLAGS_rom_option == 4) {
        trajopt.SetInitialGuess((trajopt.state_vars_by_mode(i, j))(2), 1);
      } else {
        DRAKE_UNREACHABLE();
      }
    }
  }

  // Initial guess for all variables
  if (!init_file.empty()) {
    VectorXd z0 = readCSV(dir_data + init_file).col(0);
    int n_dec = trajopt.decision_variables().size();
    if (n_dec > z0.rows()) {
      cout << "dim(initial guess) < dim(decision var). "
              "Fill the rest with zero's.\n";
      VectorXd old_z0 = z0;
      z0.resize(n_dec);
      z0 = VectorXd::Zero(n_dec);
      z0.head(old_z0.rows()) = old_z0;
    }
    trajopt.SetInitialGuessForAllVariables(z0);
  } else {
    trajopt.SetAllInitialGuess(h_guess, r_guess, dr_guess, tau_guess,
                               x_guess_left_in_front, x_guess_right_in_front,
                               final_position, fisrt_mode_phase_index);
  }

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  cout << "Construction time:" << elapsed.count() << "\n";

  // Testing
  cout << "\nChoose the best solver: "
       << drake::solvers::ChooseBestSolver(trajopt).name() << endl;

  // Print out the scaling factor
  for (int i = 0; i < trajopt.decision_variables().size(); i++) {
    cout << trajopt.decision_variable(i) << ", ";
    cout << trajopt.decision_variable(i).get_id() << ", ";
    cout << trajopt.FindDecisionVariableIndex(trajopt.decision_variable(i))
         << ", ";
    auto scale_map = trajopt.GetVariableScaling();
    auto it = scale_map.find(i);
    if (it != scale_map.end()) {
      cout << it->second;
    } else {
      cout << "none";
    }
    cout << ", ";
    cout << trajopt.GetInitialGuess(trajopt.decision_variable(i));
    cout << endl;
  }

  // Snopt setting
  int max_iter = 10000;

  if (FLAGS_use_ipopt) {
    // Ipopt settings adapted from CaSaDi and FROST
    auto id = drake::solvers::IpoptSolver::id();
    trajopt.SetSolverOption(id, "tol", FLAGS_feas_tol);
    trajopt.SetSolverOption(id, "dual_inf_tol", FLAGS_feas_tol);
    trajopt.SetSolverOption(id, "constr_viol_tol", FLAGS_feas_tol);
    trajopt.SetSolverOption(id, "compl_inf_tol", FLAGS_feas_tol);
    trajopt.SetSolverOption(id, "max_iter", max_iter);
    trajopt.SetSolverOption(id, "nlp_lower_bound_inf", -1e6);
    trajopt.SetSolverOption(id, "nlp_upper_bound_inf", 1e6);
    if (FLAGS_log_solver_info) {
      trajopt.SetSolverOption(id, "print_timing_statistics", "yes");
      trajopt.SetSolverOption(id, "print_level", 5);
    } else {
      trajopt.SetSolverOption(id, "print_timing_statistics", "no");
      trajopt.SetSolverOption(id, "print_level", 0);
    }

    // Set to ignore overall tolerance/dual infeasibility, but terminate when
    // primal feasible and objective fails to increase over 5 iterations.
    trajopt.SetSolverOption(id, "acceptable_compl_inf_tol", FLAGS_feas_tol);
    trajopt.SetSolverOption(id, "acceptable_constr_viol_tol", FLAGS_feas_tol);
    trajopt.SetSolverOption(id, "acceptable_obj_change_tol", 1e-3);
    trajopt.SetSolverOption(id, "acceptable_tol", 1e2);
    trajopt.SetSolverOption(id, "acceptable_iter", 5);
  } else {
    if (FLAGS_log_solver_info) {
      trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
                              "../snopt_planning.out");
    }
    trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                            "Major iterations limit", max_iter);
    trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                            0);
    trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                            "Major optimality tolerance", FLAGS_opt_tol);
    trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                            "Major feasibility tolerance", FLAGS_feas_tol);
  }

  // Pick solver
  drake::solvers::SolverId solver_id("");
  if (FLAGS_use_ipopt) {
    solver_id = drake::solvers::IpoptSolver().id();
  } else {
    solver_id = drake::solvers::ChooseBestSolver(trajopt);
  }
  auto solver = drake::solvers::MakeSolver(solver_id);

  // Solve
  cout << "\nSolving optimization problem...\n";
  start = std::chrono::high_resolution_clock::now();
  drake::solvers::MathematicalProgramResult result;
  solver->Solve(trajopt, trajopt.initial_guess(), trajopt.solver_options(),
                &result);
  finish = std::chrono::high_resolution_clock::now();
  elapsed = finish - start;
  cout << "    Solve time:" << elapsed.count() << " | ";
  SolutionResult solution_result = result.get_solution_result();
  cout << solution_result << " | ";
  cout << "Cost:" << result.get_optimal_cost() << "\n";

  // Check which solver we are using
  cout << "Solver: " << result.get_solver_id().name() << endl;

  // Extract solution
  VectorXd z_sol = result.GetSolution(trajopt.decision_variables());
  writeCSV(dir_data + string("z.csv"), z_sol);
  // cout << trajopt.decision_variables() << endl;

  VectorXd time_at_knots = trajopt.GetSampleTimes(result);
  MatrixXd state_at_knots = trajopt.GetStateSamples(result);
  MatrixXd input_at_knots = trajopt.GetInputSamples(result);
  writeCSV(dir_data + string("time_at_knots.csv"), time_at_knots);
  writeCSV(dir_data + string("state_at_knots.csv"), state_at_knots);
  writeCSV(dir_data + string("input_at_knots.csv"), input_at_knots);

  MatrixXd x0_each_mode(plant.num_positions() + plant.num_velocities(),
                        num_time_samples.size());
  MatrixXd xf_each_mode(plant.num_positions() + plant.num_velocities(),
                        num_time_samples.size());
  for (uint i = 0; i < num_time_samples.size(); i++) {
    x0_each_mode.col(i) = result.GetSolution(trajopt.x0_vars_by_mode(i));
    xf_each_mode.col(i) = result.GetSolution(trajopt.xf_vars_by_mode(i));
  }
  writeCSV(dir_data + string("x0_each_mode.csv"), x0_each_mode);
  writeCSV(dir_data + string("xf_each_mode.csv"), xf_each_mode);

  return 0;
}  // namespace goldilocks_models
}  // namespace goldilocks_models
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::goldilocks_models::planningWithRomAndFom(argc, argv);
}
