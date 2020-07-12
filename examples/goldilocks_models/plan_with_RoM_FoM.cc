#include <chrono>
#include <gflags/gflags.h>

#include "common/file_utils.h"
#include "common/find_resource.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/planning/rom_traj_opt.h"
#include "examples/goldilocks_models/reduced_order_models.h"

#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/choose_best_solver.h"
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

DEFINE_int32(robot_option, 0, "0: plannar robot. 1: cassie_fixed_spring");
DEFINE_int32(rom_option, 1, "0: LIPM. 1: LIPM with point-mass swing foot");
DEFINE_int32(iter, 20, "The iteration # of the theta that you use");
DEFINE_int32(sample, 4, "The sample # of the initial condition that you use");
DEFINE_string(init_file, "", "Initial Guess for Planning Optimization");
DEFINE_int32(n_step, 3, "Number of foot steps");
DEFINE_bool(print_snopt_file, false, "Print snopt output file");
DEFINE_bool(zero_touchdown_impact, false, "Zero impact at foot touchdown");
DEFINE_double(final_position, 2, "The final position for the robot");
DEFINE_double(disturbance, 0, "Disturbance to FoM initial state");
DEFINE_bool(fix_duration, false, "Fix the total time");
DEFINE_bool(fix_all_timestep, true, "Make all timesteps the same size");
// DEFINE_bool(add_x_pose_in_cost, false, "Add x0 and xf in the cost function");

// Planning with optimal reduced order model and full order model
// (discrete map is from full order model)
int planningWithRomAndFom(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  DRAKE_DEMAND(FLAGS_robot_option == 0);
  if (FLAGS_robot_option == 0) {
    DRAKE_DEMAND(FLAGS_rom_option != 4);
  }

  // Create MBP
  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  std::string full_name = FindResourceOrThrow(
      "examples/goldilocks_models/PlanarWalkerWithTorso.urdf");
  parser.AddModelFromFile(full_name);
  plant.mutable_gravity_field().set_gravity_vector(-9.81 *
                                                   Eigen::Vector3d::UnitZ());
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"),
                   drake::math::RigidTransform<double>());
  plant.Finalize();

  // Create autoDiff version of the plant
  MultibodyPlant<AutoDiffXd> plant_autoDiff(plant);

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
  // Check that we are using the correct model
  DRAKE_DEMAND(rom->n_y() == readCSV(dir_model + string("rom_n_y.csv"))(0, 0));
  DRAKE_DEMAND(rom->n_tau() ==
               readCSV(dir_model + string("rom_n_tau.csv"))(0, 0));
  DRAKE_DEMAND(rom->n_feature_y() ==
               readCSV(dir_model + string("rom_n_feature_y.csv"))(0, 0));
  DRAKE_DEMAND(rom->n_feature_yddot() ==
               readCSV(dir_model + string("rom_n_feature_yddot.csv"))(0, 0));
  if (rom->n_tau() != 0) {
    DRAKE_DEMAND((rom->B() - readCSV(dir_model + string("rom_B.csv"))).norm() ==
                 0);
  }

  // Update the ROM parameters from file
  VectorXd theta_y =
      readCSV(dir_model + to_string(FLAGS_iter) + string("_theta_y.csv"))
          .col(0);
  VectorXd theta_yddot =
      readCSV(dir_model + to_string(FLAGS_iter) + string("_theta_yddot.csv"))
          .col(0);
  rom->SetThetaY(theta_y);
  rom->SetThetaYddot(theta_yddot);

  // Optimization parameters
  int n_y = rom->n_y();
  int n_tau = rom->n_tau();
  MatrixXd Q = MatrixXd::Identity(n_y, n_y);
  MatrixXd R = MatrixXd::Identity(n_tau, n_tau);

  // Prespecify the time steps
  int n_step = FLAGS_n_step;
  int knots_per_mode = 20;
  std::vector<int> num_time_samples;
  std::vector<double> min_dt;
  std::vector<double> max_dt;
  for (int i = 0; i < n_step; i++) {
    num_time_samples.push_back(knots_per_mode);
    min_dt.push_back(.01);
    max_dt.push_back(.3);
  }
  int N = 0;
  for (uint i = 0; i < num_time_samples.size(); i++) N += num_time_samples[i];
  N -= num_time_samples.size() - 1;
  // cout << "N = " << N << endl;

  // Read in initial robot state
  string dir_and_pf = dir_model + to_string(FLAGS_iter) + string("_") +
                      to_string(FLAGS_sample) + string("_");
  cout << "dir_and_pf = " << dir_and_pf << endl;
  VectorXd init_state =
      readCSV(dir_and_pf + string("state_at_knots.csv")).col(0);
  if (FLAGS_disturbance != 0) {
    init_state(9) += FLAGS_disturbance / 1;  // add to floating base angle
  }

  bool with_init_guess = true;
  // Provide initial guess
  VectorXd h_guess;
  MatrixXd r_guess;
  MatrixXd dr_guess;
  MatrixXd tau_guess;
  VectorXd x_guess_left_in_front;
  VectorXd x_guess_right_in_front;
  if (with_init_guess) {
    h_guess = readCSV(dir_and_pf + string("time_at_knots.csv")).col(0);
    r_guess = readCSV(dir_and_pf + string("t_and_y.csv"))
                  .block(1, 0, n_y, knots_per_mode);
    dr_guess = readCSV(dir_and_pf + string("t_and_ydot.csv"))
                   .block(1, 0, n_y, knots_per_mode);
    tau_guess = readCSV(dir_and_pf + string("t_and_tau.csv"))
                    .block(1, 0, n_tau, knots_per_mode);
    x_guess_left_in_front =
        readCSV(dir_and_pf + string("state_at_knots.csv")).col(0);
    x_guess_right_in_front = readCSV(dir_and_pf + string("state_at_knots.csv"))
                                 .col(knots_per_mode - 1);
    cout << "\nWARNING: last column of state_at_knots.csv should be pre-impact "
            "state.\n";
    // cout << "h_guess = " << h_guess << endl;
    // cout << "r_guess = " << r_guess << endl;
    // cout << "dr_guess = " << dr_guess << endl;
    // cout << "tau_guess = " << tau_guess << endl;
    // cout << "x_guess_left_in_front = " << x_guess_left_in_front << endl;
    // cout << "x_guess_right_in_front = " << x_guess_right_in_front << endl;
  }

  // Construct
  cout << "\nConstructing optimization problem...\n";
  auto start = std::chrono::high_resolution_clock::now();
  RomTrajOptWithFomImpactMap trajopt(
      num_time_samples, min_dt, max_dt, Q, R, *rom, plant,
      FLAGS_zero_touchdown_impact, FLAGS_final_position, init_state, h_guess,
      r_guess, dr_guess, tau_guess, x_guess_left_in_front,
      x_guess_right_in_front, with_init_guess, FLAGS_fix_duration,
      FLAGS_fix_all_timestep, true, false, FLAGS_rom_option,
      FLAGS_robot_option);
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  cout << "Construction time:" << elapsed.count() << "\n";

  if (FLAGS_print_snopt_file) {
    trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
                            "../snopt.out");
  }
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                          "Major iterations limit", 10000);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level", 0);

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
  }

  // Testing
  cout << "\nChoose the best solver: "
       << drake::solvers::ChooseBestSolver(trajopt).name() << endl;

  // Solve
  cout << "\nSolving optimization problem...\n";
  start = std::chrono::high_resolution_clock::now();
  const MathematicalProgramResult result =
      Solve(trajopt, trajopt.initial_guess());
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

  MatrixXd x0_each_mode(2 * plant.num_positions(), num_time_samples.size());
  MatrixXd xf_each_mode(2 * plant.num_positions(), num_time_samples.size());
  for (uint i = 0; i < num_time_samples.size(); i++) {
    x0_each_mode.col(i) = result.GetSolution(trajopt.x0_vars_by_mode(i));
    xf_each_mode.col(i) = result.GetSolution(trajopt.xf_vars_by_mode(i));
  }
  writeCSV(dir_data + string("x0_each_mode.csv"), x0_each_mode);
  writeCSV(dir_data + string("xf_each_mode.csv"), xf_each_mode);

  return 0;
}  // int planningWithRomAndFom
}  // namespace goldilocks_models
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::goldilocks_models::planningWithRomAndFom(argc, argv);
}
