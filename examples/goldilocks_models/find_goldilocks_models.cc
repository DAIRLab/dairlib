#include <gflags/gflags.h>
#include <stdio.h>  // For removing files
#include <thread>  // multi-threading
#include <chrono>
#include <ctime>
#include <queue>  // First in first out
#include <deque>  // queue with feature of finding elements
#include <utility>  // std::pair, std::make_pair
#include <bits/stdc++.h>  // system call
#include <cmath>

#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

#include "common/find_resource.h"
#include "examples/goldilocks_models/dynamics_expression.h"
#include "examples/goldilocks_models/find_models/traj_opt_given_weigths.h"
#include "examples/goldilocks_models/kinematics_expression.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/initial_guess.h"
#include "systems/goldilocks_models/file_utils.h"

using std::cin;
using std::cout;
using std::endl;
using std::vector;
using std::pair;
using std::string;
using std::to_string;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXcd;
using Eigen::MatrixXd;
using Eigen::MatrixXi;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Body;
using drake::multibody::Parser;
using drake::AutoDiffXd;
using dairlib::FindResourceOrThrow;

namespace dairlib::goldilocks_models {

// Robot models
DEFINE_int32(robot_option, 0, "0: plannar robot. 1: cassie_fixed_spring");
// Reduced order models
DEFINE_int32(rom_option, -1, "");

// tasks
DEFINE_int32(N_sample_sl, 1, "Sampling # for stride length");
DEFINE_int32(N_sample_gi, 1, "Sampling # for ground incline");
DEFINE_bool(is_zero_touchdown_impact, false,
            "No impact force at fist touchdown");
DEFINE_bool(is_add_tau_in_cost, true, "Add RoM input in the cost function");
DEFINE_bool(is_uniform_grid, true, "Uniform grid of task space");

// inner loop
DEFINE_string(init_file, "", "Initial Guess for Trajectory Optimization");
DEFINE_bool(is_use_interpolated_initial_guess,false,"Use interpolated initial guess"
                                            " for Trajectory Optimization");
DEFINE_double(major_feasibility_tol, 1e-4,
              "nonlinear constraint violation tol");
DEFINE_int32(
    max_inner_iter, 150,
    "Max iteration # for traj opt. Sometimes, snopt takes very small steps "
    "(TODO: find out why), so maybe it's better to stop at some iterations and "
    "resolve again.");
DEFINE_int32(n_node, -1, "# of nodes for traj opt");
DEFINE_double(eps_regularization, 1e-8, "Weight of regularization term"); //1e-4

// outer loop
DEFINE_int32(iter_start, 0, "The starting iteration #. 0 is nominal traj.");
DEFINE_bool(is_stochastic, true, "Random tasks or fixed tasks");
DEFINE_bool(is_newton, false, "Newton method or gradient descent");
DEFINE_double(h_step, -1, "The step size for outer loop");
DEFINE_int32(max_outer_iter, 10000, "Max iteration # for theta update");
DEFINE_double(
    beta_momentum, 0.8,
    "The weight on the previous step direction."
    "beta_momentum = 0 means we only use gradient at current iter."
    "Momentum can give you faster convergence (seen in experiment), and avoid "
    "some local minima caused by step size."
    "See: https://distill.pub/2017/momentum/"
    "WARNING: beta_momentum is not used in newton's method");

// Solving for the cost gradient
DEFINE_int32(method_to_solve_system_of_equations, 3,
             "Method 0: use optimization program to solve it "
             "Method 1: use schur complement "
             "Method 2: use inverse() directly "
             "Method 3: use Moore-Penrose pseudo inverse ");

// How to update the model iterations
DEFINE_bool(start_current_iter_as_rerun, false,
            "Is `iter_start` a rerun? If it is, then you start with the stored "
            "tasks and use previous solution as initial guess");
DEFINE_int32(N_rerun, -1, "snopt might settle down at a bad sub-optimal"
                          " solution, so we rerun.");
DEFINE_double(fail_threshold, 0.2,
              "Maximum acceptable failure rate of samples");
DEFINE_bool(get_good_sol_from_adjacent_sample, true,
            "Get a good solution from adjacent samples to improve the solution "
            "quality of the current sample");
DEFINE_bool(use_theta_gamma_from_files,false,
        "To run the program with theta and gamma from saving files");

// Other features for how to start the program
DEFINE_bool(
    read_previous_step_size, true,
    "We need the previous step size, if it fails to evaluate at `iter_start`");
DEFINE_bool(start_iterations_with_shrinking_stepsize, false,
            "Start the iterations with shrinking step size. Skip the smaple "
            "evaluation steps.");
DEFINE_double(initial_extra_shrink_factor, -1,
              "shrinking factor for the step size");
DEFINE_bool(is_debug, false, "Debugging or not");
DEFINE_bool(is_manual_initial_theta, false,
            "Assign initial theta of our choice");

// Extend model from passive to actuated
DEFINE_bool(extend_model, false, "Extend the model in iteration # iter_start "
                                 "which is not equal to 0.");
DEFINE_int32(extend_model_iter, -1, "The starting iteration #");

// Multithread
DEFINE_bool(is_multithread, true, "Use multi-thread or not");
DEFINE_int32(n_thread_to_use, -1, "# of threads you want to use");

// Others
DEFINE_string(
    program_name, "",
    "The name of the program (to keep a record for future references)");

// Not tested yet. So backup before you try this.
bool is_to_improve_solution = false;

// Not sure why using the below function caused a problem.
// Valgrind said: Conditional jump or move depends on uninitialised value(s)
/*bool userAnsweredYes() {
  char answer[1];
  cin >> answer;
  if ((answer[0] == 'Y') || (answer[0] == 'y')) {
    return true;
  } else {
    return false;
  }
}*/

void createMBP(MultibodyPlant<double>* plant, int robot_option) {
  if (robot_option == 0) {
    Parser parser(plant);
    std::string full_name = FindResourceOrThrow(
                              "examples/goldilocks_models/PlanarWalkerWithTorso.urdf");
    parser.AddModelFromFile(full_name);
    plant->mutable_gravity_field().set_gravity_vector(
      -9.81 * Eigen::Vector3d::UnitZ());
    plant->WeldFrames(
      plant->world_frame(), plant->GetFrameByName("base"),
      drake::math::RigidTransform<double>());
    plant->Finalize();

  } else if (robot_option == 1) {
    Parser parser(plant);
    string full_name =
      FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
    parser.AddModelFromFile(full_name);
    plant->mutable_gravity_field().set_gravity_vector(
      -9.81 * Eigen::Vector3d::UnitZ());
    plant->Finalize();
  } else {
    throw std::runtime_error("Should not reach here");
  }
}
void setCostWeight(double* Q, double* R, double* all_cost_scale,
                   int robot_option) {
  if (robot_option == 0) {
    *Q = 1;
    *R = 0.1;
    //*all_cost_scale = 1;  // not implemented yet
  } else if (robot_option == 1) {
    *Q = 5 * 0.1;
    *R = 0.1 * 0.01;
    *all_cost_scale = 0.2 * 0.12;
  }
}
void setRomDim(int* n_s, int* n_tau, int rom_option) {
  if (rom_option == 0) {
    // 2D -- lipm
    *n_s = 2;
    *n_tau = 0;
  } else if (rom_option == 1) {
    // 4D -- lipm + swing foot
    *n_s = 4;
    *n_tau = 2;
  } else if (rom_option == 2) {
    // 1D -- fix com vertical acceleration
    *n_s = 1;
    *n_tau = 0;
  } else if (rom_option == 3) {
    // 3D -- fix com vertical acceleration + swing foot
    *n_s = 3;
    *n_tau = 2;
  } else {
    throw std::runtime_error("Should not reach here");
  }
}
void setRomBMatrix(MatrixXd* B_tau, int rom_option) {
  if ((rom_option == 0) || (rom_option == 2)) {
    // passive rom, so we don't need B_tau
  }
  else if (rom_option == 1) {
    DRAKE_DEMAND(B_tau->rows() == 4);
    (*B_tau)(2, 0) = 1;
    (*B_tau)(3, 1) = 1;
  }
  else if (rom_option == 3) {
    DRAKE_DEMAND(B_tau->rows() == 3);
    (*B_tau)(1, 0) = 1;
    (*B_tau)(2, 1) = 1;
  } else {
    throw std::runtime_error("Should not reach here");
  }
}
void setInitialTheta(VectorXd& theta_s, VectorXd& theta_sDDot,
                     int n_feature_s, int rom_option) {
  // // Testing intial theta
  // theta_s = 0.25*VectorXd::Ones(n_theta_s);
  // theta_sDDot = 0.5*VectorXd::Ones(n_theta_sDDot);
  // theta_s = VectorXd::Random(n_theta_s);
  // theta_sDDot = VectorXd::Random(n_theta_sDDot);

  if (rom_option == 0) {
    // 2D -- lipm
    theta_s(0) = 1;
    theta_s(1 + n_feature_s) = 1;
    theta_sDDot(0) = 1;
  } else if (rom_option == 1) {
    // 4D -- lipm + swing foot
    theta_s(0) = 1;
    theta_s(1 + n_feature_s) = 1;
    theta_s(2 + 2 * n_feature_s) = 1;
    theta_s(3 + 3 * n_feature_s) = 1;
    theta_sDDot(0) = 1;
  } else if (rom_option == 2) {
    // 1D -- fix com vertical acceleration
    theta_s(1) = 1;
  } else if (rom_option == 3) {
    // 3D -- fix com vertical acceleration + swing foot
    theta_s(1) = 1;
    theta_s(2 + 1 * n_feature_s) = 1;
    theta_s(3 + 2 * n_feature_s) = 1;
  } else {
    throw std::runtime_error("Should not reach here");
  }
}

void getInitFileName(const string dir,int total_sample_num, string * init_file, const string & nominal_traj_init_file,
                     int iter, int sample,double min_sl, double max_sl, double min_gi, double max_gi,
                     bool is_get_nominal,bool is_use_interpolated_initial_guess,
                     bool rerun_current_iteration, bool has_been_all_success,
                     bool step_size_shrinked_last_loop, int n_rerun,
                     int sample_idx_to_help, bool is_debug) {
  if (is_get_nominal && !rerun_current_iteration) {
    *init_file = nominal_traj_init_file;
  } else if (step_size_shrinked_last_loop && n_rerun == 0) {
    // the step size was shrink in previous iter and it's not a local rerun
    // (n_rerun == 0)
    if (is_use_interpolated_initial_guess){
        //      modified by Jianshu to test new initial guess
        *init_file = set_initial_guess(dir, iter, sample, total_sample_num, min_sl, max_sl, min_gi, max_gi);
    }
    else{
        *init_file = to_string(iter-1) + "_" + to_string(sample) + string("_w.csv");
    }
  } else if (sample_idx_to_help >= 0) {
    *init_file = to_string(iter) + "_" + to_string(sample_idx_to_help) +
                 string("_w.csv");
  } else if (rerun_current_iteration) {
    *init_file = to_string(iter) + "_" + to_string(sample) + string("_w.csv");
  }else if(is_use_interpolated_initial_guess){
//      modified by Jianshu to test new initial guess
      *init_file = set_initial_guess(dir, iter, sample, total_sample_num, min_sl, max_sl, min_gi, max_gi);
  } else{
      *init_file = to_string(iter - 1) +  "_" +
                   to_string(sample) + string("_w.csv");
  }

  // Testing:
  if (is_to_improve_solution) {
    cout << "testing with manual init file: ";
    *init_file = to_string(iter) +  "_" +
                 to_string(sample) + string("_w.csv");
    cout << *init_file << endl;
  }

  //Testing
  if (is_debug) {
    // Hacks for improving solution quality

    *init_file = to_string(iter) + "_" + to_string(sample) + string("_w.csv");
  }
}

void remove_old_multithreading_files(const string& dir, int iter, int N_sample) {
  cout << "\nRemoving old thread_finished.csv files... ";
  for (int i = 0; i < N_sample; i++) {
    string prefix = to_string(iter) + "_" + to_string(i) + "_";
    if (file_exist(dir + prefix + "thread_finished.csv")) {
      bool rm = (remove((dir + prefix + string("thread_finished.csv")).c_str()) == 0);
      if ( !rm ) cout << "Error deleting files\n";
      cout << prefix + "thread_finished.csv removed\n";
    }
  }
  cout << "Done.\n";
}

int selectThreadIdxToWait(const vector<pair<int, int>> & assigned_thread_idx,
                          string dir, int iter) {
  bool no_files_exsit = true;
  int counter = 0;
  while (no_files_exsit) {
    // cout << "Check if any file exists...\n";
    for (unsigned int i = 0; i < assigned_thread_idx.size(); i++) {
      string prefix = to_string(iter) +  "_" +
                      to_string(assigned_thread_idx[i].second) + "_";
      if (file_exist(dir + prefix + "thread_finished.csv")) {
        bool rm = (remove((dir + prefix + string("thread_finished.csv")).c_str()) == 0);
        if ( !rm ) cout << "Error deleting files\n";
        // cout << prefix + "thread_finished.csv exists\n";
        return i;
      }
    }
    if (no_files_exsit) {
      if ((counter % 60 == 0)) {
        // cout << "No files exists yet. Sleep for 1 seconds.\n";
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    counter++;
  }
  // should never reach here
  cout << "Error: The code should never reach here.\n";
  return -1;
}

void waitForAllThreadsToJoin(vector<std::thread*> * threads,
                             vector<pair<int, int>> * assigned_thread_idx,
                             const string& dir, int iter) {
  //TODO: can I kill the thread instead of waiting for it to finish?

  while (!assigned_thread_idx->empty()) {
    // Select index to wait and delete csv files
    int selected_idx = selectThreadIdxToWait(*assigned_thread_idx, dir, iter);
    int thread_to_wait_idx = (*assigned_thread_idx)[selected_idx].first;
    //string string_to_be_print = "Waiting for thread #" +
    //                            to_string(thread_to_wait_idx) + " to join...\n";
    // cout << string_to_be_print;
    (*threads)[thread_to_wait_idx]->join();
    delete (*threads)[thread_to_wait_idx];
    //string_to_be_print = "Thread #" + to_string(thread_to_wait_idx) +
    //                     " has joined.\n";
    // cout << string_to_be_print;
    /*cout << "Before erase: ";
    for (int i = 0; i < assigned_thread_idx->size(); i++) {
      cout << (*assigned_thread_idx)[i].second << ", ";
    }
    cout << endl;*/
    assigned_thread_idx->erase(assigned_thread_idx->begin() + selected_idx);
    /*cout << "After erase: ";
    for (int i = 0; i < assigned_thread_idx->size(); i++) {
      cout << (*assigned_thread_idx)[i].second << ", ";
    }
    cout << endl;*/
  }
}

void extendModel(string dir, int iter, int n_feature_s,
                 int & n_s, int & n_sDDot, int & n_tau,
                 int & n_feature_sDDot,
                 int & n_theta_s, int & n_theta_sDDot, int & n_theta,
                 MatrixXd & B_tau, VectorXd & theta_s, VectorXd & theta_sDDot,
                 VectorXd & theta, VectorXd & prev_theta,
                 VectorXd & step_direction,
                 VectorXd & prev_step_direction, double & ave_min_cost_so_far,
                 int & rom_option, int robot_option) {

  VectorXd theta_s_append = readCSV(dir +
                                    string("theta_s_append.csv")).col(0);
  int n_extend = theta_s_append.rows() / n_feature_s;

  // update rom_option
  if(rom_option == 0) {
    rom_option = 1;
  } else if (rom_option == 2) {
    rom_option = 3;
  } else {
    throw std::runtime_error("Should not reach here");
  }

  // update n_s, n_sDDot and n_tau
  int old_n_s = n_s;
  n_s += n_extend;
  n_sDDot += n_extend;
  n_tau += n_extend;
  // update n_feature_sDDot
  int old_n_feature_sDDot = n_feature_sDDot;
  DynamicsExpression dyn_expression(n_sDDot, 0, rom_option, robot_option);
  VectorXd dummy_s = VectorXd::Zero(n_s);
  n_feature_sDDot = dyn_expression.getFeature(dummy_s, dummy_s).size();
  // update n_theta_s and n_theta_sDDot
  n_theta_s = n_s * n_feature_s;
  n_theta_sDDot = n_sDDot * n_feature_sDDot;
  n_theta = n_theta_s + n_theta_sDDot;
  cout << "Updated n_s = " << n_s << endl;
  cout << "Updated n_sDDot = " << n_sDDot << endl;
  cout << "Updated n_tau = " << n_tau << endl;
  cout << "Updated n_feature_sDDot = " << n_feature_sDDot << endl;
  cout << "Updated n_theta_s = " << n_theta_s << endl;
  cout << "Updated n_theta_sDDot = " << n_theta_sDDot << endl;
  cout << "Updated n_theta = " << n_theta << endl;

  // update B_tau
  MatrixXd B_tau_old = B_tau;
  B_tau.resize(n_sDDot, n_tau);
  B_tau = MatrixXd::Zero(n_sDDot, n_tau);
  B_tau.block(0, 0, B_tau_old.rows(), B_tau_old.cols()) = B_tau_old;
  B_tau.block(B_tau_old.rows(), B_tau_old.cols(), n_extend, n_extend) =
    MatrixXd::Identity(n_extend, n_extend);
  cout << "Updated B_tau = \n" << B_tau << endl;
  writeCSV(dir + string("B_tau (before extension).csv"), B_tau_old);
  writeCSV(dir + string("B_tau.csv"), B_tau);
  // update theta_s
  string prefix = to_string(iter) +  "_";
  writeCSV(dir + prefix + string("theta_s (before extension).csv"),
           theta_s);
  MatrixXd theta_s_old = theta_s;
  theta_s.resize(n_theta_s);
  theta_s << theta_s_old, theta_s_append;
  // update theta_sDDot
  writeCSV(dir + prefix + string("theta_sDDot (before extension).csv"),
           theta_sDDot);
  MatrixXd theta_sDDot_old = theta_sDDot;
  theta_sDDot.resize(n_theta_sDDot);
  theta_sDDot = VectorXd::Zero(n_theta_sDDot);
  VectorXd new_idx = readCSV(dir +
                             string("theta_sDDot_new_index.csv")).col(0);
  for (int i = 0; i < old_n_feature_sDDot; i++)
    for (int j = 0; j < old_n_s; j++)
      theta_sDDot(new_idx(i) + j * n_feature_sDDot) = theta_sDDot_old(
            i + j * old_n_feature_sDDot);
  // update theta
  theta.resize(n_theta);
  theta << theta_s, theta_sDDot;

  // Some setup
  prev_theta.resize(n_theta);
  prev_theta = theta;
  step_direction.resize(n_theta);
  prev_step_direction.resize(n_theta);
  prev_step_direction =
      VectorXd::Zero(n_theta);  // must initialize it because of momentum term
  ave_min_cost_so_far = std::numeric_limits<double>::infinity();
}

void readApproxQpFiles(vector<VectorXd> * w_sol_vec, vector<MatrixXd> * A_vec,
                       /*vector<MatrixXd> * H_vec,*/
                       vector<VectorXd> * y_vec,
                       vector<VectorXd> * lb_vec, vector<VectorXd> * ub_vec,
                       vector<VectorXd> * b_vec, vector<VectorXd> * c_vec,
                       vector<MatrixXd> * B_vec,
                       int N_sample, int iter, string dir) {
  // The order of samples in each vector must start from 0 to N_sample (because
  // of the code where you compare the current cost and previous cost)
  for (int sample = 0; sample < N_sample; sample++) {
    string prefix = to_string(iter) +  "_" + to_string(sample) + "_";
    VectorXd success =
      readCSV(dir + prefix + string("is_success.csv")).col(0);
    if (success(0)) {
      w_sol_vec->push_back(readCSV(dir + prefix + string("w.csv")));
      A_vec->push_back(readCSV(dir + prefix + string("A.csv")));
//      H_vec->push_back(readCSV(dir + prefix + string("H.csv")));
      y_vec->push_back(readCSV(dir + prefix + string("y.csv")));
      lb_vec->push_back(readCSV(dir + prefix + string("lb.csv")));
      ub_vec->push_back(readCSV(dir + prefix + string("ub.csv")));
      b_vec->push_back(readCSV(dir + prefix + string("b.csv")));
      c_vec->push_back(readCSV(dir + prefix + string("c.csv")));
      B_vec->push_back(readCSV(dir + prefix + string("B.csv")));

      bool rm = true;
      rm = (remove((dir + prefix + string("A.csv")).c_str()) == 0) & rm;
//      rm = (remove((dir + prefix + string("H.csv")).c_str()) == 0) & rm;
      rm = (remove((dir + prefix + string("y.csv")).c_str()) == 0) & rm;
      rm = (remove((dir + prefix + string("lb.csv")).c_str()) == 0) & rm;
      rm = (remove((dir + prefix + string("ub.csv")).c_str()) == 0) & rm;
      rm = (remove((dir + prefix + string("b.csv")).c_str()) == 0) & rm;
      rm = (remove((dir + prefix + string("B.csv")).c_str()) == 0) & rm;
      if ( !rm )
        cout << "Error deleting files\n";
    }
  }
}

void extractActiveAndIndependentRows(
    int sample, double active_tol, double indpt_row_tol, string dir,
    const vector<MatrixXd>& B_vec, const vector<MatrixXd>& A_vec,
    const vector<std::shared_ptr<MatrixXd>>& H_vec,
    const vector<VectorXd>& b_vec, const vector<VectorXd>& lb_vec,
    const vector<VectorXd>& ub_vec, const vector<VectorXd>& y_vec,
    const vector<VectorXd>& w_sol_vec,
    int method_to_solve_system_of_equations) {
  string prefix = to_string(sample) + "_";

  DRAKE_ASSERT(b_vec[sample].cols() == 1);
  DRAKE_ASSERT(lb_vec[sample].cols() == 1);
  DRAKE_ASSERT(ub_vec[sample].cols() == 1);
  DRAKE_ASSERT(y_vec[sample].cols() == 1);
  DRAKE_ASSERT(w_sol_vec[sample].cols() == 1);

  int nt_i = B_vec[sample].cols();
  int nw_i = A_vec[sample].cols();

  int nl_i = 0;
  for (int i = 0; i < y_vec[sample].rows(); i++) {
    if (y_vec[sample](i) >= ub_vec[sample](i) - active_tol ||
        y_vec[sample](i) <= lb_vec[sample](i) + active_tol)
      nl_i++;
  }

  MatrixXd A_active(nl_i, nw_i);
  MatrixXd B_active(nl_i, nt_i);

  nl_i = 0;
  for (int i = 0; i < y_vec[sample].rows(); i++) {
    if (y_vec[sample](i) >= ub_vec[sample](i) - active_tol ||
        y_vec[sample](i) <= lb_vec[sample](i) + active_tol) {
      A_active.row(nl_i) = A_vec[sample].row(i);
      B_active.row(nl_i) = B_vec[sample].row(i);
      nl_i++;
    }
  }

  bool is_testing = true;
  if (is_testing) {
    // Run a quadprog to check if the solution to the following problem is 0
    // Theoratically, it should be 0. Otherwise, something is wrong
    // min 0.5*w^T Q w + c^T w
    // st  A w = 0
    if (sample == 0) {
      cout << "\n (After extracting active constraints) Run traj opt to "
           "check if your quadratic approximation is correct\n";
      cout << "sample# | Solve Status | Cost | w_sol norm | (this should be 0 "
              "if w=0 is optimal)\n";
    }
    nl_i = A_active.rows();
    MathematicalProgram quadprog;
    auto w2 = quadprog.NewContinuousVariables(nw_i, "w2");
    quadprog.AddLinearConstraint( A_active,
                                  VectorXd::Zero(nl_i),
                                  VectorXd::Zero(nl_i),
                                  w2);
    quadprog.AddQuadraticCost(*(H_vec[sample]), b_vec[sample], w2);

    // Testing
//    drake::solvers::SnoptSolver snopt_solver;
//    const auto result = snopt_solver.Solve(quadprog);
    const auto result = Solve(quadprog);

//    const auto result = Solve(quadprog);
    auto solution_result = result.get_solution_result();
    if (result.is_success()) {
      VectorXd w_sol_check = result.GetSolution(
          quadprog.decision_variables());
      cout << sample << " | " << solution_result << " | "
           << result.get_optimal_cost() << " | " << w_sol_check.norm() << " | "
           << w_sol_check.transpose() * b_vec[sample] << endl;
    } else {
      cout << sample << " | " << solution_result << " | "
           << result.get_optimal_cost() << endl;
    }
  }

  // Only add the rows that are linearly independent if the method requires A to
  // be positive definite
  if (method_to_solve_system_of_equations != 3) {
    // extract_method = 0: Do SVD each time when adding a row. (This has been
    //  working, but we later realized that the way we extract B matrix might be
    //  incorrect in theory)
    // extract_method = 1: Do SVD only once (see the notes on 20200220).
    int extract_method = 1;

    if (extract_method == 0) {
      /*cout << "n_w = " << nw_i << endl;
      cout << "Start extracting independent rows of A (# of rows = " << nl_i << ")\n";*/
      vector<int> full_row_rank_idx;
      full_row_rank_idx.push_back(0);
      for (int i = 1; i < nl_i; i++) {
        // cout << "total i = " << nl_i;
        // cout << ", i = " << i << endl;
        // Construct test matrix
        int n_current_rows = full_row_rank_idx.size();
        MatrixXd A_test(n_current_rows + 1, nw_i);
        for (unsigned int j = 0 ; j < full_row_rank_idx.size(); j++) {
          A_test.block(j, 0, 1, nw_i) =
              A_active.row(full_row_rank_idx[j]);
        }
        A_test.block(n_current_rows, 0, 1, nw_i) = A_active.row(i);

        // Perform svd to check rank
        Eigen::BDCSVD<MatrixXd> svd(A_test);
        // double sigular_value = svd.singularValues()(n_current_rows);
        if (svd.singularValues()(n_current_rows) > indpt_row_tol) {
          full_row_rank_idx.push_back(i);
        }

        if ((int)full_row_rank_idx.size() == nw_i) {
          cout << "# of A's row is the same as the # of col. So stop adding rows.\n";
          break;
        }
      }
      nl_i = full_row_rank_idx.size();
      /*cout << "Finished extracting independent rows of A (# of rows = " <<
         nl_i << ")\n\n";*/

      // Assign the rows
      MatrixXd A_processed(nl_i, nw_i);
      MatrixXd B_processed(nl_i, nt_i);
      for (int i = 0; i < nl_i; i++) {
        A_processed.row(i) = A_active.row(full_row_rank_idx[i]);
        B_processed.row(i) = B_active.row(full_row_rank_idx[i]);
      }

      // Store the results in csv files
      VectorXd nw_i_VectorXd(1); nw_i_VectorXd << nw_i;
      VectorXd nl_i_VectorXd(1); nl_i_VectorXd << nl_i;
      writeCSV(dir + prefix + string("nw_i.csv"), nw_i_VectorXd);
      writeCSV(dir + prefix + string("nl_i.csv"), nl_i_VectorXd);
      writeCSV(dir + prefix + string("A_processed.csv"), A_processed);
      writeCSV(dir + prefix + string("B_processed.csv"), B_processed);
    } else if (extract_method == 1) {
      // SVD
      Eigen::BDCSVD<MatrixXd> svd(A_active,
                                  Eigen::ComputeFullU | Eigen::ComputeFullV);

      // find the rank of the matrix (use absolute tolerance)
      const auto& singular_values = svd.singularValues();
      int rank = 0;
      for (rank = 0; rank < singular_values.size(); ++rank) {
        if (singular_values(rank) < indpt_row_tol) {
          break;
        }
      }

      // Assign the rows
      // (I believe either adjoint() or transpose() works here because U is a
      // real matrix when A is a real matrix)
      MatrixXd A_processed =
          svd.matrixU().block(0, 0, nl_i, rank).transpose() * A_active;
      MatrixXd B_processed =
          svd.matrixU().block(0, 0, nl_i, rank).transpose() * B_active;

      // Store the results in csv files
      VectorXd nw_i_VectorXd(1); nw_i_VectorXd << nw_i;
      VectorXd nl_i_VectorXd(1); nl_i_VectorXd << rank;
      writeCSV(dir + prefix + string("nw_i.csv"), nw_i_VectorXd);
      writeCSV(dir + prefix + string("nl_i.csv"), nl_i_VectorXd);
      writeCSV(dir + prefix + string("A_processed.csv"), A_processed);
      writeCSV(dir + prefix + string("B_processed.csv"), B_processed);
    } else {
      throw std::runtime_error("Should not reach here");
    }

    // cout << "sample #" << sample;
    // cout << "    A active and independent rows = " << nl_i << endl;
  } else {
    // No need to extract independent rows, so store the result right away

    // Store the results in csv files
    VectorXd nw_i_VectorXd(1); nw_i_VectorXd << nw_i;
    VectorXd nl_i_VectorXd(1); nl_i_VectorXd << nl_i;
    writeCSV(dir + prefix + string("nw_i.csv"), nw_i_VectorXd);
    writeCSV(dir + prefix + string("nl_i.csv"), nl_i_VectorXd);
    writeCSV(dir + prefix + string("A_processed.csv"), A_active);
    writeCSV(dir + prefix + string("B_processed.csv"), B_active);
  }
}

void readNonredundentMatrixFile(vector<int> * nw_vec,
                                vector<int> * nl_vec,
                                vector<MatrixXd> * A_active_vec,
                                vector<MatrixXd> * B_active_vec,
                                int n_succ_sample, string dir) {
  for (int sample = 0; sample < n_succ_sample; sample++) {
    string prefix = to_string(sample) + "_";

    nw_vec->push_back(int(readCSV(dir + prefix + string("nw_i.csv"))(0)));
    nl_vec->push_back(int(readCSV(dir + prefix + string("nl_i.csv"))(0)));
    A_active_vec->push_back(readCSV(dir + prefix + string("A_processed.csv")));
    B_active_vec->push_back(readCSV(dir + prefix + string("B_processed.csv")));

    bool rm = true;
    rm = (remove((dir + prefix + string("nw_i.csv")).c_str()) == 0) & rm;
    rm = (remove((dir + prefix + string("nl_i.csv")).c_str()) == 0) & rm;
    rm = (remove((dir + prefix + string("A_processed.csv")).c_str()) == 0) & rm;
    rm = (remove((dir + prefix + string("B_processed.csv")).c_str()) == 0) & rm;
    if ( !rm )
      cout << "Error deleting files\n";
  }
}

MatrixXd solveInvATimesB(const MatrixXd & A, const MatrixXd & B) {
  MatrixXd X = (A.transpose() * A).ldlt().solve(A.transpose() * B);

  // TODO: Test if the following line works better
  // MatrixXd X = A.completeOrthogonalDecomposition().solve(B);

  MatrixXd abs_resid = (A * X - B).cwiseAbs();
  VectorXd left_one = VectorXd::Ones(abs_resid.rows());
  VectorXd right_one = VectorXd::Ones(abs_resid.cols());
  cout << "sum-abs-residual: " << left_one.transpose()*abs_resid*right_one <<
       endl;
  return X;
}
// MatrixXd solveInvATimesB(const MatrixXd & A, const VectorXd & b) {
//   MatrixXd X = (A.transpose() * A).ldlt().solve(A.transpose() * b);
//   cout << "residual-norm: "<< (A*X-b).norm() << endl;
//   return X;
// }

MatrixXd MoorePenrosePseudoInverse(const MatrixXd& mat,
                                   double singular_tolerance = 1e-8) {
  Eigen::BDCSVD<MatrixXd> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto& singular_values = svd.singularValues();
  Eigen::MatrixXd inv_singular_values(mat.cols(), mat.rows());
  inv_singular_values.setZero();
  for (unsigned int i = 0; i < singular_values.size(); ++i) {
    if (singular_values(i) > singular_tolerance) {
      inv_singular_values(i, i) = 1.0 / singular_values(i);
    } else {
      inv_singular_values(i, i) = 0;
    }
  }
  return svd.matrixV() * inv_singular_values * svd.matrixU().adjoint();
}

void calcWInTermsOfTheta(int sample, const string& dir,
                         const vector<int> & nl_vec,
                         const vector<int> & nw_vec,
                         const vector<MatrixXd> & A_active_vec,
                         const vector<MatrixXd> & B_active_vec,
                         const vector<std::shared_ptr<MatrixXd>> & H_vec,
                         const vector<VectorXd> & b_vec,
                         int method_to_solve_system_of_equations) {
  string prefix = to_string(sample) + "_";

  if (sample == 0) {
    cout << "sample # | max element of abs-Pi | qi norm (this number should be "
            "close to 0)\n";
  }

  MatrixXd Pi(nw_vec[sample], B_active_vec[sample].cols());
  VectorXd qi(nw_vec[sample]);
  if (method_to_solve_system_of_equations == 0) {
    // Method 0: use optimization program to solve it??? ///////////////////////
    throw std::runtime_error(
        "method_to_solve_system_of_equations = 0 is not implemented yet.");
  } else if (method_to_solve_system_of_equations == 1) {
    // Method 1: use schur complement (see notes) //////////////////////////////
    // This one requires the Hessian H to be pd.
    // Also, although this method might be more computationally efficient, the
    // accuracy is not as high as when we use inverse() directly. The reason is
    // that the condition number of A and invH is high, so AinvHA' makes it very
    // ill-conditioned.
    MatrixXd AinvHA = A_active_vec[sample] * solveInvATimesB(
        *(H_vec[sample]), A_active_vec[sample].transpose());
    VectorXd invQc = solveInvATimesB(*(H_vec[sample]), b_vec[sample]);
    MatrixXd E = solveInvATimesB(AinvHA, B_active_vec[sample]);
    VectorXd F = -solveInvATimesB(AinvHA, A_active_vec[sample] * invQc);
    // Testing
    Eigen::BDCSVD<MatrixXd> svd(AinvHA);
    cout << "AinvHA':\n";
    cout << "  biggest singular value is " << svd.singularValues()(0) << endl;
    cout << "  smallest singular value is "
         << svd.singularValues().tail(1) << endl;
    cout << "The condition number of A and invH are large. That's why AinvHA'"
            "is ill-conditioned.\n";
    // cout << "singular values are \n" << svd.singularValues() << endl;

    Pi = -solveInvATimesB(*(H_vec[sample]),
                          A_active_vec[sample].transpose() * E);
    qi = -solveInvATimesB(*(H_vec[sample]),
                          b_vec[sample] + A_active_vec[sample].transpose() * F);
    cout << "qi norm (this number should be close to 0) = "
         << qi.norm() << endl;
  } else if (method_to_solve_system_of_equations == 2) {
    // Method 2: use inverse() directly ////////////////////////////////////////
    // (This one requires the Hessian H to be pd.)
    // This method has been working pretty well, but it requires H to be pd. And
    // in order to get pd H, we need to extract independent row of matrix A,
    // which takes too much time in the current method.

    // H_ext = [H A'; A 0]
    int nl_i = nl_vec[sample];
    int nw_i = nw_vec[sample];
    MatrixXd H_ext(nw_i + nl_i, nw_i + nl_i);
    H_ext.block(0, 0, nw_i, nw_i) = *(H_vec[sample]);
    H_ext.block(0, nw_i, nw_i, nl_i) = A_active_vec[sample].transpose();
    H_ext.block(nw_i, 0, nl_i, nw_i) = A_active_vec[sample];
    H_ext.block(nw_i, nw_i, nl_i, nl_i) = MatrixXd::Zero(nl_i, nl_i);

    // Testing
    // Eigen::BDCSVD<MatrixXd> svd(*(H_vec[sample]));
    // cout << "H:\n";
    // cout << "  biggest singular value is " << svd.singularValues()(0) <<
    // endl; cout << "  smallest singular value is "
    //         << svd.singularValues().tail(1) << endl;
    // // cout << "singular values are \n" << svd.singularValues() << endl;
    // // Testing
    /*if (sample == 0) {
      Eigen::BDCSVD<MatrixXd> svd_3(H_ext);
      cout << "H_ext:\n";
      cout << "  biggest singular value is " << svd_3.singularValues()(0)
           << endl;
      cout << "  smallest singular value is " << svd_3.singularValues().tail(1)
           << endl;
    }*/

    // cout << "\nStart inverting the matrix.\n";
    MatrixXd inv_H_ext = H_ext.inverse();
    // cout << "Finsihed inverting the matrix.\n";
    // // Testing
    // Eigen::BDCSVD<MatrixXd> svd_5(inv_H_ext);
    // cout << "inv_H_ext:\n";
    // cout << "  biggest singular value is " << svd_5.singularValues()(0) <<
    // endl; cout << "  smallest singular value is "
    //      << svd_5.singularValues().tail(1) << endl;

    MatrixXd inv_H_ext11 = inv_H_ext.block(0, 0, nw_i, nw_i);
    MatrixXd inv_H_ext12 = inv_H_ext.block(0, nw_i, nw_i, nl_i);

    Pi = -inv_H_ext12 * B_active_vec[sample];
    qi = -inv_H_ext11 * b_vec[sample];
  } else if (method_to_solve_system_of_equations == 3) {
    // Method 3: use Moore–Penrose pseudo inverse //////////////////////////////

    // H_ext = [H A'; A 0]
    int nl_i = nl_vec[sample];
    int nw_i = nw_vec[sample];
    MatrixXd H_ext(nw_i + nl_i, nw_i + nl_i);
    H_ext.block(0, 0, nw_i, nw_i) = *(H_vec[sample]);
    H_ext.block(0, nw_i, nw_i, nl_i) = A_active_vec[sample].transpose();
    H_ext.block(nw_i, 0, nl_i, nw_i) = A_active_vec[sample];
    H_ext.block(nw_i, nw_i, nl_i, nl_i) = MatrixXd::Zero(nl_i, nl_i);

    MatrixXd inv_H_ext = MoorePenrosePseudoInverse(H_ext, 1e-8);

    MatrixXd inv_H_ext11 = inv_H_ext.block(0, 0, nw_i, nw_i);
    MatrixXd inv_H_ext12 = inv_H_ext.block(0, nw_i, nw_i, nl_i);

    Pi = -inv_H_ext12 * B_active_vec[sample];
    qi = -inv_H_ext11 * b_vec[sample];
  }

  writeCSV(dir + prefix + string("Pi.csv"), Pi);
  writeCSV(dir + prefix + string("qi.csv"), qi);

  // Testing
  MatrixXd abs_Pi = Pi.cwiseAbs();
  VectorXd left_one = VectorXd::Ones(abs_Pi.rows());
  VectorXd right_one = VectorXd::Ones(abs_Pi.cols());
  // cout << "sum-abs-Pi: " <<
  //      left_one.transpose()*abs_Pi*right_one << endl;
  // cout << "sum-abs-Pi divide by m*n: " <<
  //      left_one.transpose()*abs_Pi*right_one / (abs_Pi.rows()*abs_Pi.cols())
  //      << endl;
  double max_Pi_element = abs_Pi(0, 0);
  for (int i = 0; i < abs_Pi.rows(); i++)
    for (int j = 0; j < abs_Pi.cols(); j++) {
      if (abs_Pi(i, j) > max_Pi_element) max_Pi_element = abs_Pi(i, j);
    }
  /*string to_be_print = "sample #" + to_string(sample) + ":  " +
                       "max element of abs-Pi = " + to_string(max_Pi_element) +
                       "\n           qi norm (this number should be close to 0) = " +
                       to_string(qi.norm()) + "\n";*/
  string to_be_print = to_string(sample) + " | " + to_string(max_Pi_element) +
                       " | " + to_string(qi.norm()) + "\n";
  cout << to_be_print;
}

void readPiQiFile(vector<MatrixXd> * P_vec, vector<VectorXd> * q_vec,
                  int n_succ_sample, const string& dir) {
  for (int sample = 0; sample < n_succ_sample; sample++) {
    string prefix = to_string(sample) + "_";

    P_vec->push_back(readCSV(dir + prefix + string("Pi.csv")));
    q_vec->push_back(readCSV(dir + prefix + string("qi.csv")));

    bool rm = true;
    rm = (remove((dir + prefix + string("Pi.csv")).c_str()) == 0) & rm;
    rm = (remove((dir + prefix + string("qi.csv")).c_str()) == 0) & rm;
    if ( !rm )
      cout << "Error deleting files\n";
  }
}

MatrixXi GetAdjSampleIndices(int N_sample_sl, int N_sample_gi, int N_sample) {
  MatrixXi adjacent_sample_indices = -1 * MatrixXi::Ones(N_sample, 4);
  MatrixXi delta_idx(2, 2);
  delta_idx << 1, 0, 0, 1;
  for (int j = 0; j < N_sample_gi; j++) {  // ground incline axis
    for (int i = 0; i < N_sample_sl; i++) {  // stride length axis
      int current_sample_idx = i + j * N_sample_sl;
      for (int k = 0; k < delta_idx.rows(); k++) {
        int new_i = i + delta_idx(k, 0);
        int new_j = j + delta_idx(k, 1);
        int adjacent_sample_idx = new_i + new_j * N_sample_sl;
        if ((new_i >= 0) && (new_i < N_sample_sl) && (new_j >= 0) &&
            (new_j < N_sample_gi)) {
          // Add to adjacent_sample_idx (both directions)
          for (int l = 0; l < 4; l++) {
            if (adjacent_sample_indices(current_sample_idx, l) < 0) {
              adjacent_sample_indices(current_sample_idx, l) =
                  adjacent_sample_idx;
              break;
            }
          }
          for (int l = 0; l < 4; l++) {
            if (adjacent_sample_indices(adjacent_sample_idx, l) < 0) {
              adjacent_sample_indices(adjacent_sample_idx, l) =
                  current_sample_idx;
              break;
            }
          }
        }
      }
    }
  }
  return adjacent_sample_indices;
}
bool IsSampleBeingEvaluated(const vector<pair<int, int>>& assigned_thread_idx,
                            int sample_idx) {
  for (auto& member : assigned_thread_idx) {
    if (member.second == sample_idx) {
      return true;
    }
  }
  return false;
}

// `GetAdjacentHelper` and `RecordSolutionQualityAndQueueList` are methods for
// getting good solution from adjacent samples
void GetAdjacentHelper(int sample_idx, MatrixXi& sample_idx_waiting_to_help,
                       MatrixXi& sample_idx_that_helped,
                       int& sample_idx_to_help) {
  for (int i = 0; i < 4; i++) {
    if (sample_idx_waiting_to_help(sample_idx, i) >= 0) {
      sample_idx_to_help = sample_idx_waiting_to_help(sample_idx, i);
      // remove sample_idx_to_help from sample_idx_waiting_to_help
      sample_idx_waiting_to_help(sample_idx, i) = -1;

      // add sample_idx_to_help to sample_idx_that_helped
      for (int j = 0; j < 4; j++) {
        if (sample_idx_that_helped(sample_idx, j) < 0) {
          sample_idx_that_helped(sample_idx, j) = sample_idx_to_help;
          break;
        }
      }
      break;
    }
  }
  cout << "before adding idx # " << sample_idx << endl;
  cout << "sample_idx_waiting_to_help = \n"
       << sample_idx_waiting_to_help.transpose() << endl;
  cout << "sample_idx_that_helped = \n"
       << sample_idx_that_helped.transpose() << endl;
  DRAKE_DEMAND(sample_idx_to_help != -1);  // must exist a helper
}
void RecordSolutionQualityAndQueueList(
    const string& dir, const string& prefix, int sample_idx,
    const vector<pair<int, int>>& assigned_thread_idx,
    const MatrixXi& adjacent_sample_indices,
    double max_cost_increase_rate_before_ask_for_help,
    double max_adj_cost_diff_rate_before_ask_for_help,
    bool is_limit_difference_of_two_adjacent_costs, int sample_success,
    bool current_sample_is_queued, const vector<int>& n_rerun, int N_rerun,
    vector<double>& each_min_cost_so_far, vector<int>& is_good_solution,
    MatrixXi& sample_idx_waiting_to_help, MatrixXi& sample_idx_that_helped,
    std::deque<int>& awaiting_sample_idx) {
  double sample_cost = (readCSV(dir + prefix + string("c.csv")))(0, 0);

  // When the current sample cost is lower than before, update
  // 1. `each_min_cost_so_far` and
  bool current_sample_improved =
      (sample_cost < each_min_cost_so_far[sample_idx]);
  if (current_sample_improved) {
    each_min_cost_so_far[sample_idx] = sample_cost;
  }

  // Calculate low_below_adjacent_cost and high_above_adjacent_cost
  vector<int> low_adjacent_cost_idx;
  vector<int> high_adjacent_cost_idx;
  if (is_limit_difference_of_two_adjacent_costs) {
    for (int i = 0; i < adjacent_sample_indices.cols(); i++) {
      int adj_idx = adjacent_sample_indices(sample_idx, i);
      if (adj_idx == -1) continue;

      double cost_diff =
          sample_cost - each_min_cost_so_far[adj_idx];
      // if the current sample cost is much higher than the adjacent cost
      if (cost_diff > max_adj_cost_diff_rate_before_ask_for_help *
                          each_min_cost_so_far[sample_idx]) {
        // If the adjacent sample has a good solution and has finished basic reruns
        if ((is_good_solution[adj_idx] == 1) &&
            (n_rerun[adj_idx] >= N_rerun)) {
          low_adjacent_cost_idx.push_back(adj_idx);
        }
      }
      // if the current sample cost is much lower than the adjacent cost
      else if (cost_diff < -max_adj_cost_diff_rate_before_ask_for_help *
                                 each_min_cost_so_far[sample_idx]) {
        // If the adjacent sample has a good solution and has finished basic reruns
        if ((is_good_solution[adj_idx] == 1) &&
            (n_rerun[adj_idx] >= N_rerun)) {
          high_adjacent_cost_idx.push_back(adj_idx);
        }
      }
    }
  }
  bool too_high_above_adjacent_cost = !low_adjacent_cost_idx.empty();
  //bool too_low_below_adjacent_cost = !high_adjacent_cost_idx.empty();

  // Printing
  /*cout << "low_adjacent_cost_idx = ";
  for(auto mem : low_adjacent_cost_idx) {
    cout << mem << ", ";
  } cout << endl;
  cout << "high_adjacent_cost_idx = ";
  for(auto mem : high_adjacent_cost_idx) {
    cout << mem << ", ";
  } cout << endl;*/


  // Record whether or not the current sample got a good solution. A good
  // solution (of the current sample) means:
  // 1. optimal solution found
  // 2. the cost didn't increase too much compared to that of the
  // previous iteration
  // 3. (optional) the cost is not too high above the adjacent costs
  if ((sample_success == 1) &&
      (sample_cost <= (1 + max_cost_increase_rate_before_ask_for_help) *
                          each_min_cost_so_far[sample_idx]) &&
      !too_high_above_adjacent_cost) {
    // Set the current sample to be having good solution
    is_good_solution[sample_idx] = 1;

    // Remove the helpers which wait to help the current sample since it's
    // successful. However, the removal not necessary in the algorithm.

    // Look for any adjacent sample that needs help
    for (int j = 0; j < adjacent_sample_indices.cols(); j++) {
      int adj_idx = adjacent_sample_indices(sample_idx, j);
      if (adj_idx == -1) continue;

      // if the current sample improved after it helped others, than wipe it off
      // the helped_list, and also add the idx that needs help
      if (current_sample_improved) {
        // Remove current sample from sample_idx_that_helped.
        bool already_exist = false;
        for (int i = 0; i < 4; i++) {
          if (sample_idx_that_helped(adj_idx, i) == sample_idx) {
            sample_idx_that_helped(adj_idx, i) = -1;
            already_exist = true;
            break;
          }
        }
        // Add current sample to sample_idx_waiting_to_help.
        if (already_exist) {
          for (int i = 0; i < 4; i++) {
            if (sample_idx_waiting_to_help(adj_idx, i) == -1) {
              sample_idx_waiting_to_help(adj_idx, i) = sample_idx;
              break;
            }
          }
        }
      }

      bool this_adjacent_sample_needs_help = false;
      bool current_sample_has_helped = false;

      bool adj_has_bad_sol = is_good_solution[adj_idx] == 0;
      bool adj_has_too_high_cost =
          (find(high_adjacent_cost_idx.begin(), high_adjacent_cost_idx.end(),
                adj_idx) != high_adjacent_cost_idx.end());

      /*cout << "(sample_idx, adj_idx, adj_has_bad_sol, adj_has_too_high_cost) = "
           << sample_idx << ", " << adj_idx << ", " << adj_has_bad_sol << ", "
           << adj_has_too_high_cost << endl;*/
      bool revert_good_adj_sol_to_bad_sol = false; // for printing
      if (adj_has_bad_sol) {
        this_adjacent_sample_needs_help = true;

        // Add current sample to the helper list of adjacent sample
        // (add if it doesn't exist in both sample_idx_waiting_to_help
        //  and sample_idx_that_helped)
        bool already_exist = false;
        for (int i = 0; i < 4; i++) {
          already_exist = already_exist || (sample_idx_waiting_to_help(
                                                adj_idx, i) == sample_idx);
        }
        for (int i = 0; i < 4; i++) {
          already_exist = already_exist ||
                          (sample_idx_that_helped(adj_idx, i) == sample_idx);
          current_sample_has_helped =
              current_sample_has_helped ||
              (sample_idx_that_helped(adj_idx, i) == sample_idx);
        }
        if (!already_exist) {
          for (int i = 0; i < 4; i++) {
            if (sample_idx_waiting_to_help(adj_idx, i) == -1) {
              sample_idx_waiting_to_help(adj_idx, i) = sample_idx;
              break;
            }
          }
        }
      } else if (adj_has_too_high_cost) {
        this_adjacent_sample_needs_help = true;

        is_good_solution[adj_idx] = 0;
        revert_good_adj_sol_to_bad_sol = true;

        // Add current sample to sample_idx_waiting_to_help.
        bool already_exist = false;
        for (int i = 0; i < 4; i++) {
          already_exist = already_exist || (sample_idx_waiting_to_help(
                                                adj_idx, i) == sample_idx);
        }
        if (!already_exist) {
          for (int i = 0; i < 4; i++) {
            if (sample_idx_waiting_to_help(adj_idx, i) == -1) {
              sample_idx_waiting_to_help(adj_idx, i) = sample_idx;
              break;
            }
          }
        }
        // Remove current sample from sample_idx_that_helped.
        for (int i = 0; i < 4; i++) {
          if (sample_idx_that_helped(adj_idx, i) == sample_idx) {
            sample_idx_that_helped(adj_idx, i) = -1;
            break;
          }
        }
      }
      // end if (adjacent sample has bad sol) and else if

      // Queue adjacent samples if
      // 1. it's not in the awaiting_sample_idx and not being evaluated
      // 2. the adjacent sample needs help
      // 3. the current sample hasn't helped the adjacent sample
      if (this_adjacent_sample_needs_help && !current_sample_has_helped) {
        if (revert_good_adj_sol_to_bad_sol) {
          cout << "idx #" << sample_idx
               << " cost is too low below that of adjacent idx #" << adj_idx
               << ", so revert the flag to bad sol. ";
        } else {
          cout << "idx #" << sample_idx << " got good sol, and idx #" << adj_idx
               << " needs help. ";
        }
        if ((find(awaiting_sample_idx.begin(), awaiting_sample_idx.end(),
                  adj_idx) == awaiting_sample_idx.end()) &&
            !(IsSampleBeingEvaluated(assigned_thread_idx, adj_idx))) {
          awaiting_sample_idx.push_back(adj_idx);
          cout << "Add #" << adj_idx << " to queue";
        }
        cout << "\n";
      }
    }  // end for (Look for any adjacent sample that needs help)

  } else {
    // Set the current sample to be having bad solution
    is_good_solution[sample_idx] = 0;

    // Look for any adjacent sample that can help
    for (int j = 0; j < adjacent_sample_indices.cols(); j++) {
      bool this_adjacent_sample_can_help = false;
      bool this_adjacent_sample_is_waiting_to_help = false;

      // if the adjacent sample has a good solution, then add it to the
      // helper list
      int adj_idx = adjacent_sample_indices(sample_idx, j);
      if (adj_idx == -1) continue;

      bool adj_has_good_sol = is_good_solution[adj_idx] == 1;
      bool adj_has_too_low_cost =
          (find(low_adjacent_cost_idx.begin(), low_adjacent_cost_idx.end(),
                adj_idx) != low_adjacent_cost_idx.end());

      bool low_adj_cost_idx_has_helped = false;
      for (int i = 0; i < 4; i++) {
        low_adj_cost_idx_has_helped = low_adj_cost_idx_has_helped ||
            (sample_idx_that_helped(sample_idx, i) == adj_idx);
      }

      bool add_adj_as_helper_because_low_cost = false; // for printing message
      if (adj_has_too_low_cost && !low_adj_cost_idx_has_helped) {
        this_adjacent_sample_can_help = true;
        this_adjacent_sample_is_waiting_to_help = true;

        add_adj_as_helper_because_low_cost = true;

        // Add adj_idx to the top of the helper list because it has a good
        // solution (very low cost)
        // TODO: You can improve this algorithm, since currently if there are
        // two adj with low cost, then you might push one of the adj in the back
        // of the list.
        int already_exist_matrix_idx = -1;
        for (int i = 0; i < 4; i++) {
          if (sample_idx_waiting_to_help(sample_idx, i) == adj_idx) {
            already_exist_matrix_idx = i;
            break;
          }
        }
        int first_helper_idx = sample_idx_waiting_to_help(sample_idx, 0);
        sample_idx_waiting_to_help(sample_idx, 0) = adj_idx;
        if (already_exist_matrix_idx < 0) {
          for (int i = 1; i < 4; i++) {
            if (sample_idx_waiting_to_help(sample_idx, i) == -1) {
              sample_idx_waiting_to_help(sample_idx, i) = first_helper_idx;
              break;
            }
          }
        } else if (already_exist_matrix_idx > 0) {
          for (int i = 1; i < 4; i++) {
            if (sample_idx_waiting_to_help(sample_idx, i) == adj_idx) {
              sample_idx_waiting_to_help(sample_idx, i) = first_helper_idx;
              break;
            }
          }
        }
      } else if (adj_has_good_sol) {
        this_adjacent_sample_can_help = true;

        // The helper list
        // (add if it doesn't exist in both sample_idx_waiting_to_help
        //  and sample_idx_that_helped)
        bool already_exist = false;
        for (int i = 0; i < 4; i++) {
          already_exist = already_exist || (sample_idx_waiting_to_help(
                                                sample_idx, i) == adj_idx);
          this_adjacent_sample_is_waiting_to_help = already_exist;
        }
        for (int i = 0; i < 4; i++) {
          already_exist = already_exist ||
                          (sample_idx_that_helped(sample_idx, i) == adj_idx);
        }
        if (!already_exist) {
          for (int i = 0; i < 4; i++) {
            if (sample_idx_waiting_to_help(sample_idx, i) == -1) {
              sample_idx_waiting_to_help(sample_idx, i) = adj_idx;
              break;
            }
          }
          this_adjacent_sample_is_waiting_to_help = true;
        }
      }

      // Queue the current sample back if
      // 1. the current sample is not queued yet
      // 2. the adjacent sample can help
      // 3. the adjacent sample is waiting to help current sample
      if (!current_sample_is_queued && this_adjacent_sample_can_help &&
          this_adjacent_sample_is_waiting_to_help) {
        awaiting_sample_idx.push_back(sample_idx);
        current_sample_is_queued = true;
        if (add_adj_as_helper_because_low_cost) {
          cout << "idx #" << sample_idx
               << " cost is too high above adjacent idx #" << adj_idx
               << ", so add #" << sample_idx << " to queue\n";
        } else {
          cout << "idx #" << sample_idx << " got bad sol ";
          if (sample_success == 0) {
            cout << "(snopt didn't find an optimal sol)";
          } else {
            cout << "(cost increased too much)";
          }
          cout << ", and idx #" << adj_idx
               << " can help, so add #" << sample_idx << " to queue\n";
        }
      }
    }  // end for (Look for any adjacent sample that can help)
  }    // end if current sample has good solution
}

// Calculate the cost gradient and its norm
void CalcCostGradientAndNorm(int n_succ_sample, const vector<MatrixXd>& P_vec,
                             const vector<VectorXd>& q_vec,
                             const vector<VectorXd>& b_vec, const string& dir,
                             const string& prefix, VectorXd* gradient_cost,
                             double* norm_grad_cost) {
  cout << "Calculating gradient\n";
  gradient_cost->setZero();
  for (int sample = 0; sample < n_succ_sample; sample++) {
    (*gradient_cost) += P_vec[sample].transpose() * b_vec[sample];
  }
  (*gradient_cost) /= n_succ_sample;

  // Calculate gradient norm
  (*norm_grad_cost) = gradient_cost->norm();
  writeCSV(dir + prefix + string("norm_grad_cost.csv"),
           (*norm_grad_cost) * VectorXd::Ones(1));
  cout << "gradient_cost norm: " << (*norm_grad_cost) << endl << endl;
}

// Newton's method (not exactly the same, cause Q_theta is not pd but psd)
// See your IOE611 lecture notes on page 7-17 to page 7-20
void CalcNewtonStepAndNewtonDecrement(int n_theta, int n_succ_sample,
                                      const vector<MatrixXd>& P_vec,
                                      const vector<std::shared_ptr<MatrixXd>>& H_vec,
                                      const VectorXd& gradient_cost,
                                      const string& dir, const string& prefix,
                                      VectorXd* newton_step,
                                      double* lambda_square) {
  /*// Check if Q_theta is pd
  cout << "Checking if Q_theta is psd...\n";
  MatrixXd Q_theta = MatrixXd::Zero(n_theta, n_theta);
  for (int sample = 0; sample < n_succ_sample; sample++)
    Q_theta += P_vec[sample].transpose()*(*(H_vec[sample]))*P_vec[sample];
  VectorXd eivals_real = Q_theta.eigenvalues().real();
  for (int i = 0; i < eivals_real.size(); i++) {
    if (eivals_real(i) <= 0)
      cout << "Q_theta is not positive definite (with e-value = "
           << eivals_real(i) << ")\n";
  }
  cout << endl;*/

  // cout << "Getting Newton step\n";
  MatrixXd Q_theta = MatrixXd::Zero(n_theta, n_theta);
  for (int sample = 0; sample < n_succ_sample; sample++) {
    Q_theta += P_vec[sample].transpose() * (*(H_vec[sample])) * P_vec[sample] /
               n_succ_sample;
  }
  double mu = 1e-4;  // 1e-6 caused unstable and might diverge
  MatrixXd inv_Q_theta =
      (Q_theta + mu * MatrixXd::Identity(n_theta, n_theta)).inverse();
  (*newton_step) = -inv_Q_theta * gradient_cost;

  // Testing
  /*Eigen::BDCSVD<MatrixXd> svd(inv_Q_theta);
  cout << "inv_Q_theta's smallest and biggest singular value " <<
       svd.singularValues().tail(1) << ", " <<
       svd.singularValues()(0) << endl;*/

  // Newton decrement (can be a criterion to terminate your newton steps)
  (*lambda_square) = -gradient_cost.transpose() * (*newton_step);
  cout << "lambda_square = " << (*lambda_square) << endl;

  // Store Newton decrement in a file
  writeCSV(dir + prefix + string("lambda_square.csv"),
           (*lambda_square) * VectorXd::Ones(1));
}

// Calculate the step direction and its norm
void GetStepDirectionAndNorm(bool is_newton, const VectorXd& newton_step,
                             const VectorXd& gradient_cost,
                             double beta_momentum, const string& dir,
                             const string& prefix,
                             VectorXd* prev_step_direction,
                             VectorXd* step_direction,
                             double* step_direction_norm) {
  if (is_newton) {
    (*step_direction) = newton_step;
  } else {
    // gradient descent with momentum term
    (*step_direction) = -gradient_cost + beta_momentum * (*prev_step_direction);
  }
  writeCSV(dir + prefix + string("step_direction.csv"), (*step_direction));
  (*prev_step_direction) = (*step_direction);

  // Calculate ans store the step direction norm
  (*step_direction_norm) = step_direction->norm();
  cout << "step_direction norm: " << (*step_direction_norm) << endl << endl;
  writeCSV(dir + prefix + string("step_direction_norm.csv"),
           (*step_direction_norm) * VectorXd::Ones(1));
}

// Get the step size (heuristically tuned)
void GetHeuristicStepSize(double h_step, double step_direction_norm,
                          const string& dir, const string& prefix,
                          double* current_iter_step_size) {
  // (*current_iter_step_size) = h_step;
  if (step_direction_norm > 1) {
//    (*current_iter_step_size) = h_step / sqrt(step_direction_norm);  // Heuristic
    (*current_iter_step_size) = h_step / step_direction_norm;  // Heuristic
  } else {
    (*current_iter_step_size) = h_step;
  }

  // Store the step size in a file
  writeCSV(dir + prefix + string("step_size.csv"),
           (*current_iter_step_size) * VectorXd::Ones(1));
  cout << "step size = " << (*current_iter_step_size) << "\n\n";
}

// Check if the model has achieved an optimum
bool HasAchievedOptimum(bool is_newton, double stopping_threshold,
                        double lambda_square, double norm_grad_cost) {
  if (is_newton) {
    if (lambda_square < stopping_threshold) {
      cout << "Found optimal theta.\n\n";
      return true;
    }
  } else {
    if (norm_grad_cost < stopping_threshold) {
      cout << "Found optimal theta.\n\n";
      return true;
    }
  }
  return false;
}

int findGoldilocksModels(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  cout << "Trail name: " << FLAGS_program_name << endl;

  if (FLAGS_is_multithread) {
    cout << "Make sure that you turned off snopt log and constraint jacobian "
         "writing.\nProceed? (Y/N)\n";
    char answer[1];
    cin >> answer;
    if (!((answer[0] == 'Y') || (answer[0] == 'y'))) {
      cout << "Ending the program.\n";
      return 0;
    } else {
      cout << "Continue constructing the problem...\n";
    }
  }

  // Create MBP
  MultibodyPlant<double> plant(0.0);
  createMBP(&plant, FLAGS_robot_option);

  // Create autoDiff version of the plant
  MultibodyPlant<AutoDiffXd> plant_autoDiff(plant);

  // Random number generator
  std::random_device randgen;
  std::default_random_engine e1(randgen());
  std::random_device randgen2;
  std::default_random_engine e2(randgen2());

  // Files parameters
  /*const string dir = "examples/goldilocks_models/find_models/data/robot_" +
      to_string(FLAGS_robot_option) + "/";*/
  const string dir = "../dairlib_data/goldilocks_models/find_models/robot_" +
                     to_string(FLAGS_robot_option) + "/";
  string init_file = FLAGS_init_file;
  string prefix = "";
  if (!CreateFolderIfNotExist(dir)) return 0;

  // Parameters for tasks (stride length and ground incline)
  cout << "\nTasks settings:\n";
  bool uniform_grid = FLAGS_is_uniform_grid;

  int N_sample_sl = FLAGS_N_sample_sl;
  int N_sample_gi = FLAGS_N_sample_gi;
  int N_sample = N_sample_sl * N_sample_gi;  // 1;
  double delta_stride_length;
  double stride_length_0;
  if (FLAGS_robot_option == 0) {
    delta_stride_length = 0.015;
    stride_length_0 = 0.3;
  } else if (FLAGS_robot_option == 1) {
    if (FLAGS_is_stochastic) {
      delta_stride_length = 0.015;//0.066; // 0.066 might be too big;
    } else {
      delta_stride_length = 0.1;
    }
    stride_length_0 = 0.2;  //0.15
  } else {
    throw std::runtime_error("Should not reach here");
    delta_stride_length = 0;
    stride_length_0 = 0;
  }
  double delta_ground_incline;
  double ground_incline_0 = 0;
  if (FLAGS_robot_option == 0) {
    delta_ground_incline = 0.05;
  } else if (FLAGS_robot_option == 1) {
    if (FLAGS_is_stochastic) {
      delta_ground_incline = 0.05;//0.066; // 0.066 might be too big
    } else {
      delta_ground_incline = 0.08;
    }
  } else {
    throw std::runtime_error("Should not reach here");
    delta_ground_incline = 0;
  }
  double duration = 0.4;
  if (FLAGS_robot_option == 0) {
    duration = 0.746;  // Fix the duration now since we add cost ourselves
  } else if (FLAGS_robot_option == 1) {
    duration = 0.4; // 0.4;
  }
  cout << "duration = " << duration << endl;
  DRAKE_DEMAND(N_sample_sl % 2 == 1);
  DRAKE_DEMAND(N_sample_gi % 2 == 1);
  cout << "use_theta_gamma_from_files = " << FLAGS_use_theta_gamma_from_files << endl;
  uniform_grid ? cout << "Uniform grid\n" : cout << "Non-Uniform grid\n";
  cout << "N_sample_sl = " << N_sample_sl << endl;
  cout << "N_sample_gi = " << N_sample_gi << endl;
  cout << "delta_stride_length = " << delta_stride_length << endl;
  cout << "stride_length_0 = " << stride_length_0 << endl;
  cout << "delta_ground_incline = " << delta_ground_incline << endl;
  cout << "ground_incline_0 = " << ground_incline_0 << endl;
  double min_stride_length =
      (FLAGS_is_stochastic)
          ? stride_length_0 -
                delta_stride_length * ((N_sample_sl - 1) / 2 + 0.5)
          : stride_length_0 - delta_stride_length * ((N_sample_sl - 1) / 2);
  double max_stride_length =
      (FLAGS_is_stochastic)
          ? stride_length_0 +
                delta_stride_length * ((N_sample_sl - 1) / 2 + 0.5)
          : stride_length_0 + delta_stride_length * ((N_sample_sl - 1) / 2);
  double min_ground_incline =
      (FLAGS_is_stochastic)
          ? ground_incline_0 -
                delta_ground_incline * ((N_sample_gi - 1) / 2 + 0.5)
          : ground_incline_0 - delta_ground_incline * ((N_sample_gi - 1) / 2);
  double max_ground_incline =
      (FLAGS_is_stochastic)
          ? ground_incline_0 +
                delta_ground_incline * ((N_sample_gi - 1) / 2 + 0.5)
          : ground_incline_0 + delta_ground_incline * ((N_sample_gi - 1) / 2);
  DRAKE_DEMAND(min_stride_length >= 0);
  cout << "stride length ranges from " << min_stride_length << " to "
       << max_stride_length << endl;
  cout << "ground incline ranges from " << min_ground_incline << " to "
       << max_ground_incline << endl;
  VectorXd previous_ground_incline = VectorXd::Zero(N_sample);
  VectorXd previous_stride_length = VectorXd::Zero(N_sample);
  if (FLAGS_start_current_iter_as_rerun ||
      FLAGS_start_iterations_with_shrinking_stepsize) {
    for (int i = 0; i < N_sample; i++) {
      previous_ground_incline(i) =
          readCSV(dir + to_string(FLAGS_iter_start) + "_" + to_string(i) +
                  string("_ground_incline.csv"))(0);
      previous_stride_length(i) =
          readCSV(dir + to_string(FLAGS_iter_start) + "_" + to_string(i) +
                  string("_stride_length.csv"))(0);
    }
    // print
    /*for (int i = 0; i < N_sample; i++) {
      cout << previous_ground_incline(i) << ", ";
    }
    cout << endl;
    for (int i = 0; i < N_sample; i++) {
      cout << previous_stride_length(i) << ", ";
    }
    cout << endl;*/
  }

  // Parameters for the outer loop optimization
  cout << "\nOptimization setting (outer loop):\n";
  int iter_start = FLAGS_iter_start;
  int max_outer_iter = FLAGS_max_outer_iter;
  double stopping_threshold = 1e-4;

  // beta_momentum = 0 means we only use gradient at current iter.
  // Momentum can give you faster convergence. And get out of a local minimum
  // caused by step size. See: https://distill.pub/2017/momentum/ WARNING:
  // beta_momentum is not used in newton's method
  double beta_momentum = FLAGS_beta_momentum;
  double h_step;
  if (FLAGS_h_step > 0) {
    h_step = FLAGS_h_step;
  } else {
    h_step = 1e-3;
    if (FLAGS_robot_option == 0) {
      // After adding tau
      // 1e-4 doesn't diverge   // This is with  h_step / sqrt(norm_grad_cost(0));
      // 1e-3 diverges
      // Before adding tau
      // 1e-3 is small enough to avoid gittering at the end
      // 1e-2 is a good compromise on both speed and gittering
      // 1e-1 caused divergence when close to optimal sol
      h_step = 1e-4;
      /*if (beta_momentum != 0) {
        // haven't tried or tuned this yet.
        h_step = 1e-5;
      }*/
    } else if (FLAGS_robot_option == 1) {
      // Without tau: (This is with  h_step / sqrt(norm_grad_cost(0));)
      //  1e-4: doesn't always decrease with a fixed task
      //  1e-5: barely increase with a fixed task

      // Both with and without tau (I believe), fixed task.
      // h_step = 1e-3;  // This is with h_step / norm_grad_cost_double. (and with
                      // old traj opt)

      // (20200216) After using new traj opt
      h_step = 1e-4;  // maybe h_step shouldn't be too high, because rom
                      // constraint is the constraint that is hard to satisfy?
      if (!FLAGS_is_stochastic) {
        h_step = 1e-3;  // we can always shrink steps if the cost goes up with
                        // fixed tasks (it should go down theoretically)
      }
      if (beta_momentum != 0) {
        // haven't tried or tuned this yet.
        h_step = 1e-3;
      }
    }
  }
  double eps_regularization = FLAGS_eps_regularization;
  double indpt_row_tol = 1e-6;//1e-6
  bool is_newton = FLAGS_is_newton;
  bool is_stochastic = FLAGS_is_stochastic;
  int N_rerun;
  if (FLAGS_N_rerun > -1) {
    N_rerun = FLAGS_N_rerun;
  } else {
    if (FLAGS_robot_option == 0) {
      N_rerun = 1;
    } else if (FLAGS_robot_option == 1) {
      N_rerun = 1;//2;
    } else {
      N_rerun = 0;
    }
  }
  const int method_to_solve_system_of_equations =
      FLAGS_method_to_solve_system_of_equations;
  // With bigger momentum, you might need a larger tolerance
  double max_sample_cost_increase_rate = 0;
  if (FLAGS_robot_option == 0) {
    max_sample_cost_increase_rate = FLAGS_is_stochastic? 2.0: 0.01;
  } else if (FLAGS_robot_option== 1) {
    max_sample_cost_increase_rate = FLAGS_is_stochastic? 0.5: 0.01; //0.3
  } else {
    throw std::runtime_error("Should not reach here");
  }
  double max_average_cost_increase_rate = 0;
  if (FLAGS_robot_option == 0) {
    max_average_cost_increase_rate = FLAGS_is_stochastic? 0.5: 0.01;
  } else if (FLAGS_robot_option== 1) {
    max_average_cost_increase_rate = FLAGS_is_stochastic? 0.2: 0.01;//0.15
  } else {
    throw std::runtime_error("Should not reach here");
  }
  // Since sometimes the above increase rates are too restrictive and cause the
  // optimization to get stuck in some iteration, we relax the increase rate
  // every `n_shrink_before_relaxing_tolerance` times of step size shrinking
  int n_shrink_before_relaxing_tolerance = 6;
  is_newton ? cout << "Newton method\n" : cout << "Gradient descent method\n";
  is_stochastic ? cout << "Stochastic\n" : cout << "Non-stochastic\n";
  FLAGS_is_use_interpolated_initial_guess ? cout << "New_initial_guess\n" : cout << "Original_initial_guess\n";
  cout << "Step size = " << h_step << endl;
  cout << "beta_momentum = " << beta_momentum << endl;
  cout << "eps_regularization = " << eps_regularization << endl;
  cout << "is_add_tau_in_cost = " << FLAGS_is_add_tau_in_cost << endl;
  FLAGS_is_zero_touchdown_impact ? cout << "Zero touchdown impact\n" :
                                        cout << "Non-zero touchdown impact\n";
  cout << "# of re-run in each iteration = " << N_rerun << endl;
  cout << "Failure rate threshold before seeing a all-success iteration = "
       << FLAGS_fail_threshold << endl;
  cout << "method_to_solve_system_of_equations = "
       << method_to_solve_system_of_equations << endl;
  cout << "The maximum rate the cost of each sample cost can increase before "
          "shrinking step size = "
       << max_sample_cost_increase_rate << endl;
  cout << "The maximum rate the averaged cost can increase before shrinking "
          "step size = "
       << max_average_cost_increase_rate << endl;
  cout << "n_shrink_before_relaxing_tolerance = "
       << n_shrink_before_relaxing_tolerance << endl;
  // Outer loop setting - help from adjacent samples
  bool get_good_sol_from_adjacent_sample =
      FLAGS_get_good_sol_from_adjacent_sample;
  if (FLAGS_robot_option == 0) {
    // five-link robot doesn't seem to need help
    get_good_sol_from_adjacent_sample = false;
  }
  double max_cost_increase_rate_before_ask_for_help = 0.1;
  if (FLAGS_robot_option == 0) {
    max_cost_increase_rate_before_ask_for_help = 0.5;
  }
  double max_adj_cost_diff_rate_before_ask_for_help = 0.1;
  if (FLAGS_robot_option == 0) {
    max_adj_cost_diff_rate_before_ask_for_help = 0.5;
  }
  bool is_limit_difference_of_two_adjacent_costs =
      max_adj_cost_diff_rate_before_ask_for_help > 0;
  cout << "Get good sols from adjacent samples to improve solution quality? "
       << get_good_sol_from_adjacent_sample << endl;
  if (get_good_sol_from_adjacent_sample) {
    cout << "The maximum rate the cost can increase before asking adjacent "
            "samples for help = "
         << max_cost_increase_rate_before_ask_for_help << endl;
    cout << "The maximum cost difference rate between two adjacent samples = "
         << max_adj_cost_diff_rate_before_ask_for_help << endl;
  }
  /// Notes: currently, there are a few conditions under any of which the
  /// program would rerun trajectory optimization:
  ///  1. if N_rerun is not 0, then after SNOPT found a solution this program
  ///  feeds the solution back to SNOPT as a initial condition to resolve the
  ///  program for N_rerun number of times.
  ///  2. after N_rerun number of reruns, if there are two adjacent samples
  ///  whose cost ratio (difference) are greater than a certain value decided by
  ///  `max_adj_cost_diff_rate_before_ask_for_help` , then this program
  ///  re-evaluates the sample with a high cost and feed it the solution with
  ///  low cost as an initial condition.
  ///  3. after N_rerun number of reruns, if any sample cost increases over a
  ///  certain rate determined by `max_cost_increase_rate_before_ask_for_help`,
  ///  then the program re-evaluates the sample (if any adjacent sample has
  ///  lower cost).
  ///  4. if all the samples are evaluated successfully and if the total cost
  ///  increases over a certain rate determined by
  ///  `max_cost_increase_rate_before_ask_for_help`, then the program rerun the
  ///  iteration

  // Parameters for the inner loop optimization
  int max_inner_iter = FLAGS_max_inner_iter;
  if (FLAGS_robot_option == 0) {
    max_inner_iter = 300;
  }
  double Q = 0; // Cost on velocity
  double R = 0;  // Cost on input effort
  double all_cost_scale = 1;
  setCostWeight(&Q, &R, &all_cost_scale, FLAGS_robot_option);
  int n_node = 20;
  if (FLAGS_robot_option == 0) {
    n_node = 20;
  } else if (FLAGS_robot_option == 1) {
    n_node = 20;
  }
  if (FLAGS_n_node > 0) n_node = FLAGS_n_node;
  cout << "\nOptimization setting (inner loop):\n";
  cout << "max_inner_iter = " << max_inner_iter << endl;
  cout << "major_optimality_tolerance = " << FLAGS_major_feasibility_tol << endl;
  cout << "major_feasibility_tolerance = " << FLAGS_major_feasibility_tol << endl;
  cout << "n_node = " << n_node << endl;
  if (FLAGS_robot_option == 1) {
    // If the node density is too low, it's harder for SNOPT to converge well.
    // The ratio of distance per nodes = 0.2/16 is fine for snopt, but
    // 0.3 / 16 is too high.
    // However, currently it takes too much time to compute with many nodes, so
    // we try 0.3/24.
    double max_distance_per_node = 0.3 / 16;
    DRAKE_DEMAND((max_stride_length / n_node) <= max_distance_per_node);
  }

  // Reduced order model parameters
  cout << "\nReduced-order model setting:\n";
  cout << "Warning: Need to make sure that the implementation in "
       "DynamicsExpression agrees with n_s and n_tau.\n";
  int rom_option = (FLAGS_rom_option >= 0) ? FLAGS_rom_option : 0;
  int n_s = 0;
  int n_tau = 0;
  setRomDim(&n_s, &n_tau, rom_option);
  int n_sDDot = n_s; // Assume that are the same (no quaternion)
  MatrixXd B_tau = MatrixXd::Zero(n_sDDot, n_tau);
  setRomBMatrix(&B_tau, rom_option);
  cout << "n_s = " << n_s << ", n_tau = " << n_tau << endl;
  cout << "B_tau = \n" << B_tau << endl;
  writeCSV(dir + string("B_tau.csv"), B_tau);
  cout << "rom_option = " << rom_option << " ";
  switch (rom_option) {
    case 0: cout << "(2D -- lipm)\n";
      break;
    case 1: cout << "(4D -- lipm + swing foot)\n";
      break;
    case 2: cout << "(1D -- fix com vertical acceleration)\n";
      break;
    case 3: cout << "(3D -- fix com vertical acceleration + swing foot)\n";
      break;
  }
  cout << "Make sure that n_s and B_tau are correct.\nProceed? (Y/N)\n";
  char answer[1];
  cin >> answer;
  if (!((answer[0] == 'Y') || (answer[0] == 'y'))) {
    cout << "Ending the program.\n";
    return 0;
  } else {
    cout << "Continue constructing the problem...\n";
  }

  // Reduced order model setup
  KinematicsExpression<double> kin_expression(n_s, 0, &plant, FLAGS_robot_option);
  DynamicsExpression dyn_expression(n_sDDot, 0, rom_option, FLAGS_robot_option);
  VectorXd dummy_q = VectorXd::Ones(plant.num_positions());
  VectorXd dummy_s = VectorXd::Ones(n_s);
  int n_feature_s = kin_expression.getFeature(dummy_q).size();
  int n_feature_sDDot =
    dyn_expression.getFeature(dummy_s, dummy_s).size();
  cout << "n_feature_s = " << n_feature_s << endl;
  cout << "n_feature_sDDot = " << n_feature_sDDot << endl;
  int n_theta_s = n_s * n_feature_s;
  int n_theta_sDDot = n_sDDot * n_feature_sDDot;
  VectorXd theta_s(n_theta_s);
  VectorXd theta_sDDot(n_theta_sDDot);

  // Initial guess of theta
  theta_s = VectorXd::Zero(n_theta_s);
  theta_sDDot = VectorXd::Zero(n_theta_sDDot);
  if (iter_start == 0) {
    setInitialTheta(theta_s, theta_sDDot, n_feature_s, rom_option);
    cout << "Make sure that you use the right initial theta.\nProceed? (Y/N)\n";
    char answer[1];
    cin >> answer;
    if (!((answer[0] == 'Y') || (answer[0] == 'y'))) {
      cout << "Ending the program.\n";
      return 0;
    } else {
      cout << "Continue constructing the problem...\n";
    }
  }
  else {
    if (!FLAGS_is_manual_initial_theta) {
      theta_s = readCSV(dir + to_string(iter_start) +
                        string("_theta_s.csv")).col(0);
      theta_sDDot = readCSV(dir + to_string(iter_start) +
                            string("_theta_sDDot.csv")).col(0);
    }
    else {
      MatrixXd theta_s_0_mat =
        readCSV(dir + string("theta_s_0.csv"));
      MatrixXd theta_sDDot_0_mat =
        readCSV(dir + string("theta_sDDot_0.csv"));
      theta_s.head(theta_s_0_mat.rows()) = theta_s_0_mat.col(0);
      theta_sDDot.head(theta_sDDot_0_mat.rows()) = theta_sDDot_0_mat.col(0);
    }
  }

  // Vectors/Matrices for the outer loop
  vector<VectorXd> w_sol_vec;
//  vector<MatrixXd> H_vec;
  vector<std::shared_ptr<MatrixXd>> H_vec(N_sample);
  for (int i = 0; i < N_sample; i++) {
    auto matrix_i = std::make_shared<MatrixXd>();
    H_vec[i] = matrix_i;
  }
  vector<VectorXd> b_vec;
  vector<VectorXd> c_vec;
  vector<MatrixXd> A_vec;
  vector<MatrixXd> A_active_vec;
  vector<VectorXd> lb_vec;
  vector<VectorXd> ub_vec;
  vector<VectorXd> y_vec;
  vector<MatrixXd> B_vec;
  vector<MatrixXd> B_active_vec;

  // Multithreading setup
  cout << "\nMultithreading settings:\n";
  int CORES = static_cast<int>(std::thread::hardware_concurrency());
  if (FLAGS_n_thread_to_use > 0) CORES = FLAGS_n_thread_to_use;
  cout << "is multithread? " << FLAGS_is_multithread << endl;
  cout << "# of threads to be used " << CORES << endl;
  if (FLAGS_is_multithread) {
    remove_old_multithreading_files(dir, iter_start, N_sample);
  }

  // Some setup
  cout << "\nOther settings:\n";
  cout << "is_debug? " << FLAGS_is_debug << endl;
  cout << "is_manual_initial_theta = " << FLAGS_is_manual_initial_theta << endl;
  double ave_min_cost_so_far = std::numeric_limits<double>::infinity();
  std::vector<double> each_min_cost_so_far(
      N_sample, std::numeric_limits<double>::infinity());
  if (iter_start > 1  && !FLAGS_is_debug) {
    for (int iter = iter_start - 1; iter > 0; iter--) {
      // Check if the cost for all samples exist
      bool all_exsit = true;
      for (int i = 0; i < N_sample; i++) {
        all_exsit = all_exsit && file_exist(dir + to_string(iter) + "_" +
                                            to_string(i) + string("_c.csv"));
      }
      if (!all_exsit) {
        break;
      }

      // Get total cost and individual cost
      double old_total_cost = 0;
      for (int i = 0; i < N_sample; i++) {
        double c = readCSV(dir + to_string(iter) +  "_" +
            to_string(i) + string("_c.csv"))(0,0);
        old_total_cost += c;

        // Assign individual cost
        if (each_min_cost_so_far[i] > c) {
          each_min_cost_so_far[i] = c;
        }
      }

      // Assign ave_min_cost_so_far
      if (ave_min_cost_so_far > old_total_cost / N_sample) {
        ave_min_cost_so_far = old_total_cost / N_sample;
      }
    }
    cout << "ave_min_cost_so_far = " << ave_min_cost_so_far << endl;
  }
  // Tasks setup
  std::uniform_real_distribution<> dist_sl(
          -delta_stride_length / 2, delta_stride_length / 2);
  vector<double> delta_stride_length_vec;
  for (int i = 0 - N_sample_sl / 2; i < N_sample_sl - N_sample_sl / 2; i++)
      delta_stride_length_vec.push_back(i * delta_stride_length);
  std::uniform_real_distribution<> dist_gi(
          -delta_ground_incline / 2, delta_ground_incline / 2);
  vector<double> delta_ground_incline_vec;
  for (int i = 0 - N_sample_gi / 2; i < N_sample_gi - N_sample_gi / 2; i++)
      delta_ground_incline_vec.push_back(i * delta_ground_incline);
  if(!uniform_grid && is_stochastic){
      std::uniform_real_distribution<> dist_sl(
              min_stride_length, max_stride_length);
      std::uniform_real_distribution<> dist_gi(
              min_ground_incline, max_ground_incline);
  }

  // Some setup
  int n_theta = n_theta_s + n_theta_sDDot;
  VectorXd theta(n_theta);
  theta << theta_s, theta_sDDot;
  bool rerun_current_iteration = FLAGS_start_current_iter_as_rerun;
  bool has_been_all_success = iter_start > 1;
  if ((FLAGS_start_current_iter_as_rerun && (iter_start >= 1)) ||
      (!FLAGS_start_current_iter_as_rerun && (iter_start >= 2))) {
    int iter_check_all_success =
        FLAGS_start_current_iter_as_rerun ? iter_start : iter_start - 1;

    bool samples_are_success = true;
    for (int i = 0; i < N_sample; i++) {
      samples_are_success =
          samples_are_success &&
          readCSV(dir + to_string(iter_check_all_success) + "_" + to_string(i) +
                  string("_is_success.csv"))(0);
    }
    has_been_all_success = samples_are_success;
  }
  cout << "has_been_all_success? " << has_been_all_success << endl;
  cout << "iteration #" << iter_start << " is a rerun? "
       << rerun_current_iteration << endl;

  VectorXd step_direction;
  VectorXd prev_step_direction =
      VectorXd::Zero(n_theta);  // must initialize this because of momentum term
  if (iter_start > 1) {
    cout << "Reading previous step direction... (will get memory issue if the "
            "file doesn't exist)\n";
    step_direction =
        readCSV(dir + to_string(iter_start - 1) + string("_step_direction.csv"))
            .col(0);
    prev_step_direction =
        readCSV(dir + to_string(iter_start - 1) + string("_step_direction.csv"))
            .col(0);
  }
  double current_iter_step_size = h_step;
  if ((iter_start > 1) && FLAGS_read_previous_step_size) {
    cout << "Reading previous step size... (will get memory issue if the file "
            "doesn't exist)\n";
    current_iter_step_size = readCSV(dir + to_string(iter_start - 1) +
                                     string("_step_size.csv"))(0, 0);
  }

  VectorXd prev_theta = theta;
  if (iter_start > 1) {
    MatrixXd prev_theta_s_mat =
      readCSV(dir + to_string(iter_start - 1) + string("_theta_s.csv"));
    MatrixXd prev_theta_sDDot_mat =
      readCSV(dir + to_string(iter_start - 1) + string("_theta_sDDot.csv"));
    prev_theta << prev_theta_s_mat.col(0), prev_theta_sDDot_mat.col(0);
  }

  bool start_iterations_with_shrinking_stepsize =
      FLAGS_start_iterations_with_shrinking_stepsize;
  if (FLAGS_start_iterations_with_shrinking_stepsize) {
    DRAKE_DEMAND(FLAGS_read_previous_step_size);
  }

  if (FLAGS_initial_extra_shrink_factor > 0) {
    DRAKE_DEMAND(FLAGS_start_iterations_with_shrinking_stepsize);
    current_iter_step_size /= FLAGS_initial_extra_shrink_factor;
  }

  bool step_size_shrinked_last_loop = false;

  bool extend_model = FLAGS_extend_model;
  int extend_model_iter = (FLAGS_extend_model_iter == -1) ?
                          iter_start : FLAGS_extend_model_iter;
  extend_model_iter = (extend_model_iter == 0) ? 1 : extend_model_iter;
  bool has_visit_this_iter_for_model_extension = false;
  if (extend_model) {
    cout << "\nWill extend the model at iteration # " << extend_model_iter <<
         " by ";
    VectorXd theta_s_append = readCSV(dir +
                                      string("theta_s_append.csv")).col(0);
    DRAKE_DEMAND(theta_s_append.rows() % n_feature_s == 0);
    int n_extend = theta_s_append.rows() / n_feature_s;
    cout << n_extend << " dimension.\n";

    cout << "Make sure that you include both old and new version of dynamics "
         "feature.\nProceed? (Y/N)\n";
    char answer[1];
    cin >> answer;
    if (!((answer[0] == 'Y') || (answer[0] == 'y'))) {
      cout << "Ending the program.\n";
      return 0;
    } else {
      cout << "Continue constructing the problem...\n";
    }
  }

  // Setup for getting good solution from adjacent samples
  // (In 2D tasks space, the adjacent sample# for each sample is 4)
  const MatrixXi adjacent_sample_indices =
      GetAdjSampleIndices(N_sample_sl, N_sample_gi, N_sample);
  cout << "adjacent_sample_indices = \n"
       << adjacent_sample_indices.transpose() << endl;

  cout << "\nStart iterating...\n";
  // Start the gradient descent
  int iter;
  int n_shrink_step = 0;
  for (iter = iter_start; iter <= max_outer_iter; iter++)  {
    bool is_get_nominal = iter == 0;

    if (is_to_improve_solution) {
      theta_s = readCSV(dir + to_string(iter) +
                        string("_theta_s.csv")).col(0);
      theta_sDDot = readCSV(dir + to_string(iter) +
                            string("_theta_sDDot.csv")).col(0);
    }

    if (start_iterations_with_shrinking_stepsize) {
      n_shrink_step++;
    }

    // Print info about iteration # and current time
    if (!start_iterations_with_shrinking_stepsize) {
      auto end = std::chrono::system_clock::now();
      std::time_t end_time = std::chrono::system_clock::to_time_t(end);
      cout << "Current time: " << std::ctime(&end_time);
      cout << "************ Iteration " << iter << " ("
           << n_shrink_step << "-time step size shrinking) *************" << endl;
      if (iter != 0) {
        cout << "theta_sDDot.head(6) = " << theta_sDDot.head(6).transpose() << endl;
      }
    }

    // store initial parameter values
    prefix = to_string(iter) +  "_";
    if (!is_get_nominal || !FLAGS_is_debug) {
      writeCSV(dir + prefix + string("theta_s.csv"), theta_s);
      writeCSV(dir + prefix + string("theta_sDDot.csv"), theta_sDDot);
    }

    // setup for each iteration
    int max_inner_iter_pass_in = is_get_nominal ? 200 : max_inner_iter;
    bool extend_model_this_iter = extend_model && (iter == extend_model_iter) &&
                                  !has_visit_this_iter_for_model_extension;
    if (iter == extend_model_iter)
      has_visit_this_iter_for_model_extension = true;

    // Clear the vectors/matrices before trajectory optimization
    A_vec.clear();
    B_vec.clear();
//    H_vec.clear();
    A_active_vec.clear();
    B_active_vec.clear();
    lb_vec.clear();
    ub_vec.clear();
    y_vec.clear();
    b_vec.clear();
    c_vec.clear();
    w_sol_vec.clear();

    // Run trajectory optimization for different tasks first
    bool all_samples_are_success = true;
    bool a_sample_is_success = false;
    bool success_rate_is_high_enough = true;
    if (start_iterations_with_shrinking_stepsize) {
      // skip the sample evaluation
    } else {
      // Print message
      cout << "sample# (rerun #) | stride | incline | init_file | Status | "
              "Solve time | Cost (tau cost)\n";

      // Create vector of threads for multithreading
      vector<std::thread*> threads(std::min(CORES, N_sample));

      // Create a index list indicating which thread is available for use
      std::queue<int> available_thread_idx;
      for (int i = 0; i < std::min(CORES, N_sample); i++)
        available_thread_idx.push(i);
      vector<pair<int, int>> assigned_thread_idx;

      // Setup for rerun
      // rerun is a "hack" for snopt solver. Since it doesn't always find a good
      // solution in one solve, we rerun it a few times.
      std::vector<int> n_rerun(N_sample, -1);
      std::deque<int> awaiting_sample_idx;
      for (int i = 0; i < N_sample; i++)
        awaiting_sample_idx.push_back(i);

      // Set up for feeding good sample solution to adjacent bad samples
      std::vector<int> is_good_solution(N_sample, -1);  // -1 means unset,
                                                        // 0 is bad, 1 is good
      // In the following int matrices, each row is a list that contains the
      // sample idx that can help (or helped)
      // -1 means empty.
      // Since in 2D tasks space, the adjacent sample# for each sample is 4, we
      // initialize the column number with 4.
      // TODO: you can re-implement this with
      //  std::vector<std::shared_ptr<std::vector<int>>>
      //  so the code is cleaner.
      MatrixXi sample_idx_waiting_to_help = -1 * MatrixXi::Ones(N_sample, 4);
      MatrixXi sample_idx_that_helped = -1 * MatrixXi::Ones(N_sample, 4);
      std::vector<double> local_each_min_cost_so_far = each_min_cost_so_far;

      // Set up for deciding if we should update the solution
      std::vector<double> cost_threshold_for_update(
          N_sample, std::numeric_limits<double>::infinity());

      // Evaluate samples
      int n_failed_sample = 0;
      while (!awaiting_sample_idx.empty() || !assigned_thread_idx.empty()) {
        /*cout << "awaiting_sample_idx = ";
        for (auto mem : awaiting_sample_idx) {
          cout << mem << ", ";
        } cout << endl;
        cout << "assigned_sample_idx = ";
        for (auto mem : assigned_thread_idx) {
          cout << mem.second << ", ";
        } cout << endl;*/

        //std::system("lscpu | grep CPU\\ MHz"); // print the current cpu clock speed
        //std::system("top -bn2 | grep \"Cpu(s)\" | sed \"s/.*, *\\([0-9.]*\\)%* id.*/\1/\" | awk '{print 100 - $1\"%\"}'"); // print the CPU usage
        //std::system("free -m"); // print memory usage

        // Evaluate a sample when there is an available thread. Otherwise, wait.
        if (!awaiting_sample_idx.empty() && !available_thread_idx.empty()) {
          // Pick a sample to evaluate
          int sample_idx = awaiting_sample_idx.front();
          awaiting_sample_idx.pop_front();

          // Record # of reruns
          n_rerun[sample_idx] += 1;
          bool current_sample_is_a_rerun =
              rerun_current_iteration || (n_rerun[sample_idx] > 0);

          // setup for each sample
          double stride_length =
              stride_length_0 +
              delta_stride_length_vec[sample_idx % N_sample_sl];
          double ground_incline =
              ground_incline_0 +
              delta_ground_incline_vec[sample_idx / N_sample_sl];
          if (!is_get_nominal && is_stochastic) {
              if (uniform_grid){
                  stride_length += dist_sl(e1);
                  ground_incline += dist_gi(e2);
              }
              else{
                  stride_length = dist_sl(e1);
                  ground_incline = dist_gi(e2);
              }
          }

          // if use the gamma from file, overwrite the gamma
          bool use_gamma_from_files = FLAGS_use_theta_gamma_from_files;
          if(use_gamma_from_files)
          {
              stride_length = (readCSV(dir + to_string(iter) + string("_")
                      + to_string(sample_idx) + string("_stride_length.csv")))(0,0);
              ground_incline = (readCSV(dir + to_string(iter) + string("_")
                      + to_string(sample_idx) + string("_ground_incline.csv")))(0,0);
          }

          // Store the tasks or overwrite it with previous tasks
          // (You need step_size_shrinked_last_loop because you might start the
          // program with shrinking step size)
          if (current_sample_is_a_rerun || step_size_shrinked_last_loop) {
            stride_length = previous_stride_length(sample_idx);
            ground_incline = previous_ground_incline(sample_idx);
          } else {
            previous_stride_length(sample_idx) = stride_length;
            previous_ground_incline(sample_idx) = ground_incline;
          }
          // Store tasks in files so we can use it in visualization
          prefix = to_string(iter) +  "_" + to_string(sample_idx) + "_";
          writeCSV(dir + prefix + string("stride_length.csv"),
                   stride_length * MatrixXd::Ones(1, 1));
          writeCSV(dir + prefix + string("ground_incline.csv"),
                   ground_incline * MatrixXd::Ones(1, 1));

          // (Feature -- get initial guess from adjacent successful samples)
          // If the current sample already finished N_rerun, then it means that
          // there exists a adjacent sample that can help the current sample.
          int sample_idx_to_help = -1;
          if (get_good_sol_from_adjacent_sample) {
            if (n_rerun[sample_idx] > N_rerun) {
              cout << "is_good_solution = ";
              for (auto & mem : is_good_solution) {
                cout << mem << ", ";
              } cout << endl;
              GetAdjacentHelper(sample_idx, sample_idx_waiting_to_help,
                                sample_idx_that_helped, sample_idx_to_help);
            }
          }

          // Get file name of initial seed
          string init_file_pass_in;
          int total_sample_num = N_sample_sl*N_sample_gi;
          bool is_use_interpolated_initial_guess = FLAGS_is_use_interpolated_initial_guess;
          getInitFileName(dir, total_sample_num, &init_file_pass_in, init_file, iter, sample_idx,
                          min_stride_length, max_stride_length, min_ground_incline, max_ground_incline,
                          is_get_nominal, is_use_interpolated_initial_guess,
                          current_sample_is_a_rerun, has_been_all_success,
                          step_size_shrinked_last_loop, n_rerun[sample_idx],
                          sample_idx_to_help,
                          FLAGS_is_debug);


          // Set up feasibility and optimality tolerance
          // TODO: tighten tolerance at the last rerun for getting better
          //  solution?

          // Trajectory optimization with fixed model parameters
          //string string_to_be_print = "Adding sample #" + to_string(sample_idx) +
          //  " to thread #" + to_string(available_thread_idx.front()) + "...\n";
          // cout << string_to_be_print;
          threads[available_thread_idx.front()] = new std::thread(trajOptGivenWeights,
              std::ref(plant), std::ref(plant_autoDiff),
              n_s, n_sDDot, n_tau,
              n_feature_s, n_feature_sDDot, std::ref(B_tau),
              std::ref(theta_s), std::ref(theta_sDDot),
              stride_length, ground_incline,
              duration, n_node, max_inner_iter_pass_in,
              FLAGS_major_feasibility_tol, FLAGS_major_feasibility_tol,
              std::ref(dir), init_file_pass_in, prefix,
              std::ref(H_vec),
              Q, R, all_cost_scale,
              eps_regularization,
              is_get_nominal,
              FLAGS_is_zero_touchdown_impact,
              extend_model_this_iter,
              FLAGS_is_add_tau_in_cost,
              sample_idx, n_rerun[sample_idx],
              cost_threshold_for_update[sample_idx], N_rerun,
              rom_option,
              FLAGS_robot_option);
          //string_to_be_print = "Finished adding sample #" + to_string(sample_idx) +
          //  " to thread # " + to_string(available_thread_idx.front()) + ".\n";
          // cout << string_to_be_print;

          assigned_thread_idx.push_back(
            std::make_pair(available_thread_idx.front(), sample_idx));
          available_thread_idx.pop();
        } else {
          // Select the thread to join
          int selected_idx = selectThreadIdxToWait(assigned_thread_idx, dir, iter);
          // cout << "selected_idx = " << selected_idx << endl;

          // Wait for the selected thread to join, then delete thread
          int thread_to_wait_idx = assigned_thread_idx[selected_idx].first;
          int sample_idx = assigned_thread_idx[selected_idx].second;
          //string string_to_be_print = "Waiting for thread #" +
          //                            to_string(thread_to_wait_idx) +
          //                            " to join...\n";
          // cout << string_to_be_print;
          threads[thread_to_wait_idx]->join();
          delete threads[thread_to_wait_idx];
          //string_to_be_print = "Thread #" +
          //                     to_string(thread_to_wait_idx) +
          //                     " has joined.\n";
          // cout << string_to_be_print;
          available_thread_idx.push(thread_to_wait_idx);
          assigned_thread_idx.erase(assigned_thread_idx.begin() + selected_idx);
          // BTW, erasing middle members is computationally inefficient:
          //   http://www.cplusplus.com/reference/vector/vector/erase/

          // Queue the current sample back if
          // 1. it's not the last evaluation for this sample
          bool current_sample_is_queued = false;
          if (n_rerun[sample_idx] < N_rerun) {
            awaiting_sample_idx.push_back(sample_idx);
            current_sample_is_queued = true;
          }

          // Record success history
          prefix = to_string(iter) +  "_" + to_string(sample_idx) + "_";
          int sample_success =
              (readCSV(dir + prefix + string("is_success.csv")))(0, 0);

          // Update cost_threshold_for_update
          if ((sample_success == 1) && (n_rerun[sample_idx] >= N_rerun)) {
            auto sample_cost = (readCSV(dir + prefix + string("c.csv")))(0, 0);
            if (sample_cost < cost_threshold_for_update[sample_idx]) {
              cost_threshold_for_update[sample_idx] = sample_cost;
            }
          }

          // Get good initial guess from adjacent samples's solution
          if (get_good_sol_from_adjacent_sample) {
            if (n_rerun[sample_idx] >= N_rerun) {
              RecordSolutionQualityAndQueueList(
                  dir, prefix, sample_idx, assigned_thread_idx,
                  adjacent_sample_indices,
                  max_cost_increase_rate_before_ask_for_help,
                  max_adj_cost_diff_rate_before_ask_for_help,
                  is_limit_difference_of_two_adjacent_costs, sample_success,
                  current_sample_is_queued, n_rerun, N_rerun,
                  local_each_min_cost_so_far, is_good_solution,
                  sample_idx_waiting_to_help, sample_idx_that_helped,
                  awaiting_sample_idx);
            }
          }

          // If the current sample is queued again because it could be helped by
          // adjacent samples, then don't conclude that it's a failure yet
          auto it = find(awaiting_sample_idx.begin(), awaiting_sample_idx.end(),
                         sample_idx);
          if (it != awaiting_sample_idx.end()) {
            sample_success = 1;
          }

          // Accumulate failed samples
          if (sample_success != 1) {
            n_failed_sample++;
          }

          // Logic of fail or success
          all_samples_are_success =
              (all_samples_are_success & (sample_success == 1));
          a_sample_is_success = (a_sample_is_success | (sample_success == 1));
          double fail_rate = double(n_failed_sample) / double(N_sample);
          if (fail_rate > FLAGS_fail_threshold) {
            success_rate_is_high_enough = false;
          } else if ((fail_rate > 0) && is_get_nominal) {
            success_rate_is_high_enough = false;
          }

          // Stop evaluating if
          // 1. any sample failed after a all-success iteration
          // 2. fail rate higher than threshold before seeing all-success
          // iteration
          if ((has_been_all_success && (!all_samples_are_success)) ||
              (!has_been_all_success && (!success_rate_is_high_enough))) {
            // Wait for the assigned threads to join, and then break;
            cout << n_failed_sample << " # of samples failed to find solution."
                 " Latest failed sample is sample#" << sample_idx <<
                 ". Wait for all threads to join and stop current iteration.\n";
            waitForAllThreadsToJoin(&threads, &assigned_thread_idx, dir, iter);
            break;
          }

          // If in debug mode, stop evaluating.
          if (FLAGS_is_debug) {
            // Wait for the assigned threads to join, and then break;
            /*cout << "In debug mode. Wait for all threads to join and stop "
                    "current iteration.\n";
            waitForAllThreadsToJoin(&threads, &assigned_thread_idx, dir, iter);
            break;*/
          }
        }
      }  // while(sample < N_sample)
    }  // end if-else (start_iterations_with_shrinking_stepsize)
    if (FLAGS_is_debug) break;

    // cout << "Only run for 1 iteration. for testing.\n";
    // for (int i = 0; i < 100; i++) {cout << '\a';}  // making noise to notify
    // break;

    // For testing
    if (is_to_improve_solution) {
      cout << "Not updating the parameters. for testing.\n";
      continue;
    }

    // Logic for how to iterate
    if (start_iterations_with_shrinking_stepsize) {
      rerun_current_iteration = true;
    } else {
      if (all_samples_are_success && !is_get_nominal) {
        has_been_all_success = true;
      }
      // If all samples have been evaluated successfully in previous iteration,
      // we don't allow any failure in the following iterations
      bool current_iter_is_success = has_been_all_success
                                     ? all_samples_are_success
                                     : success_rate_is_high_enough;

      // Rerun the current iteration when the iteration was not successful
      rerun_current_iteration = !current_iter_is_success;
    }

    // Some checks to prevent wrong logic
    if (start_iterations_with_shrinking_stepsize) {
      DRAKE_DEMAND(
          !extend_model_this_iter);  // shouldn't extend model while starting
                                     // the program with adjusting step size
      DRAKE_DEMAND(iter > 1);  // shouldn't be iter 0 or 1
    }

    // Update parameters, adjusting step size or extend model
    step_size_shrinked_last_loop = false;
    if (is_get_nominal) {
      if (rerun_current_iteration) {
        iter -= 1;
      }
    } else if (extend_model_this_iter) {  // Extend the model
      cout << "Start extending model...\n";
      extendModel(dir, iter, n_feature_s,
                  n_s, n_sDDot, n_tau,
                  n_feature_sDDot,
                  n_theta_s, n_theta_sDDot, n_theta,
                  B_tau, theta_s, theta_sDDot, theta,
                  prev_theta, step_direction, prev_step_direction, ave_min_cost_so_far,
                  rom_option, FLAGS_robot_option);

      // So that we can re-run the current iter
      cout << "Reset \"has_been_all_success\" to false, in case the next iter "
           "is infeasible.\n";
      iter -= 1;
      has_been_all_success = false;
      rerun_current_iteration = true;

      // Never extend model again (we just extend it once)
      extend_model = false;
      continue;
    }  // end if extend_model_this_iter
    else if (rerun_current_iteration) {  // rerun the current iteration
      iter -= 1;

      current_iter_step_size = current_iter_step_size / 2;
      // if(current_iter_step_size<1e-5){
      //   cout<<"switch to the other method.";
      //   is_newton = !is_newton;
      // }
      cout << "Step size shrinks to " << current_iter_step_size <<
           ". Redo this iteration.\n\n";

      // Descent
      theta = prev_theta + current_iter_step_size * step_direction;

      // Assign theta_s and theta_sDDot
      theta_s = theta.head(n_theta_s);
      theta_sDDot = theta.tail(n_theta_sDDot);

      // for start_iterations_with_shrinking_stepsize
      start_iterations_with_shrinking_stepsize = false;

      step_size_shrinked_last_loop = true;
    }  // end if rerun_current_iteration
    else {
      // The code only reach here when the current iteration is successful.

      // Read in the following files of the successful samples:
      // w_sol_vec, A_vec, H_vec, y_vec, lb_vec, ub_vec, b_vec, c_vec, B_vec;
      auto start_time_read_file = std::chrono::high_resolution_clock::now();

      readApproxQpFiles(&w_sol_vec, &A_vec, /*&H_vec,*/ &y_vec, &lb_vec, &ub_vec,
                        &b_vec, &c_vec, &B_vec,
                        N_sample, iter, dir);

      // Print out elapsed time
      auto finish_time_read_file = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed_read_file =
          finish_time_read_file - start_time_read_file;
      cout << "\nTime spent on reading files of sample evaluation: "
           << to_string(int(elapsed_read_file.count())) << " seconds\n";
      cout << endl;

      // number of successful sample
      int n_succ_sample = c_vec.size();

      // Calculate the total cost of the successful samples
      double total_cost = 0;
      for (int sample = 0; sample < n_succ_sample; sample++) {
        total_cost += c_vec[sample](0) / n_succ_sample;
      }
      if (total_cost <= ave_min_cost_so_far) ave_min_cost_so_far = total_cost;

      // Print the total cost of this iteration
      cout << "total_cost = " << total_cost << " (min so far: " <<
           ave_min_cost_so_far << ")\n\n";

      // Update each cost when all samples are successful
      if (all_samples_are_success) {
        for (int sample_i = 0; sample_i < N_sample; sample_i++) {
          if (c_vec[sample_i](0) < each_min_cost_so_far[sample_i]) {
            each_min_cost_so_far[sample_i] = c_vec[sample_i](0);
          }
        }
      }

      // We further decide if we should still shrink the step size because the
      // cost is not small enough (we do this because sometimes snopt cannot
      // find a good solution when the step size is too big).
      // - We still do this when we are using stochastic tasks although the
      // tasks difficulty varies from iterations to iterations. We can
      // heuristically pick a range that contains the task difficulty variation,
      // so that we don't eliminate the sample with hard task.
      // - (changed) We don't do this on iteration 2 because sometimes the cost
      // goes up from iteration 1 to 2 (somehow).
      // - We require that ALL the samples were evaluated successfully when
      // shrinking the step size based on cost.
      if ((iter > 1) && all_samples_are_success) {
        // 1. average cost
        if (total_cost > ave_min_cost_so_far) {
          cout << "Average cost went up by "
               << (total_cost - ave_min_cost_so_far) / ave_min_cost_so_far * 100
               << "%.\n";
        }
        double tol_total_cost =
            max_average_cost_increase_rate *
            (1 + std::floor(n_shrink_step /
                            (double)n_shrink_before_relaxing_tolerance));
        if (total_cost > (1 + tol_total_cost) * ave_min_cost_so_far) {
          cout << "The cost went up too much (over " << tol_total_cost * 100
               << "%). Shrink the step size.\n\n";
          start_iterations_with_shrinking_stepsize = true;
          iter--;
          continue;
        }

        // 2. each sample cost
        double tol_sample_cost =
            max_sample_cost_increase_rate *
            (1 + std::floor(n_shrink_step /
                            (double)n_shrink_before_relaxing_tolerance));
        DRAKE_DEMAND(c_vec.size() == each_min_cost_so_far.size());
        bool exit_current_iter_to_shrink_step_size = false;
        for (int sample_i = 0; sample_i < N_sample; sample_i++) {
          // print
          if (c_vec[sample_i](0) > each_min_cost_so_far[sample_i]) {
            cout << "Cost #" << sample_i << " went up by "
                 << (c_vec[sample_i](0) - each_min_cost_so_far[sample_i]) /
                        each_min_cost_so_far[sample_i] * 100
                 << "%.\n";
          }
          // If cost goes up, we restart the iteration and shrink the step size.
          if (c_vec[sample_i](0) >
              (1 + tol_sample_cost) * each_min_cost_so_far[sample_i]) {
            cout << "The cost went up too much (over " << tol_sample_cost * 100
                 << "%). Shrink the step size.\n\n";
            start_iterations_with_shrinking_stepsize = true;
            iter--;
            exit_current_iter_to_shrink_step_size = true;
            break;
          }
        }
        if (exit_current_iter_to_shrink_step_size) continue;
      }

      // Update parameters below

      // Extract active and independent constraints (multithreading)
      auto start_time_extract = std::chrono::high_resolution_clock::now();
      vector<std::thread *> threads(std::min(CORES, n_succ_sample));
      cout << "\nExtracting active (and independent rows) of A...\n";
      int sample = 0;
      while (sample < n_succ_sample) {
        int sample_end = (sample + CORES >= n_succ_sample) ?
                         n_succ_sample : sample + CORES;
        int thread_idx = 0;
        for (int sample_i = sample; sample_i < sample_end; sample_i++) {
          threads[thread_idx] = new std::thread(
              extractActiveAndIndependentRows,
              sample_i, FLAGS_major_feasibility_tol, indpt_row_tol, dir,
              std::ref(B_vec), std::ref(A_vec), std::ref(H_vec),
              std::ref(b_vec), std::ref(lb_vec), std::ref(ub_vec),
              std::ref(y_vec), std::ref(w_sol_vec),
              method_to_solve_system_of_equations);
          thread_idx++;
        }
        thread_idx = 0;
        for (int sample_i = sample; sample_i < sample_end; sample_i++) {
          threads[thread_idx]->join();
          delete threads[thread_idx];
          thread_idx++;
        }
        sample = sample_end;
      }
      // Read the matrices after extractions
      vector<int> nw_vec;  // size of decision var of traj opt for all tasks
      vector<int> nl_vec;  // # of rows of active constraints for all tasks
      readNonredundentMatrixFile(&nw_vec, &nl_vec,
                                 &A_active_vec, &B_active_vec,
                                 n_succ_sample, dir);
      // Print out elapsed time
      auto finish_time_extract = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed_extract =
          finish_time_extract - start_time_extract;
      cout << "Time spent on extracting active (and independent rows) of A: "
           << to_string(int(elapsed_extract.count())) << " seconds\n";
      cout << endl;


      // Reference for solving a sparse linear system
      // https://eigen.tuxfamily.org/dox/group__TopicSparseSystems.html
      // https://eigen.tuxfamily.org/dox/group__LeastSquares.html

      // Our calculation below is based on the fact that the H matrices are pd and
      // symmetric, so we check them here.
      // However, H turned out not to be psd, since we have timestep h as decision
      // variable. (It came from running cost. ~h*u'*R*u, etc)
      // Fixed it by adding running cost by hand (but the timestep is fixed now).
      // Now H is always pd because we also added a regularization term.
      /*cout << "Checking if H is pd and symmetric\n";
      for (int sample = 0; sample < n_succ_sample; sample++) {
        // Check if H is symmetric
        VectorXd One_w = VectorXd::Ones(nw_vec[sample]);
        double sum =
          One_w.transpose() * (H_vec[sample] - H_vec[sample].transpose()) * One_w;
        if (sum != 0) cout << "H is not symmetric\n";

        // Check if H is pd
        VectorXd eivals_real = H_vec[sample].eigenvalues().real();
        for (int i = 0; i < eivals_real.size(); i++) {
          if (eivals_real(i) <= 0)
            cout << "H is not positive definite (with e-value = "
                 << eivals_real(i) << ")\n";
        }
      }
      cout << "Finished checking\n\n";*/


      // Get w in terms of theta (Get P_i and q_i where w = P_i * theta + q_i)
      // cout << "Getting P matrix and q vecotr\n";
      // vector<std::thread *> threads(std::min(CORES, n_succ_sample));
      auto start_time_calc_w = std::chrono::high_resolution_clock::now();
      sample = 0;
      while (sample < n_succ_sample) {
        int sample_end = (sample + CORES >= n_succ_sample) ?
                         n_succ_sample : sample + CORES;
        int thread_idx = 0;
        for (int sample_i = sample; sample_i < sample_end; sample_i++) {
          threads[thread_idx] = new std::thread(
              calcWInTermsOfTheta,
              sample_i, dir,
              std::ref(nl_vec), std::ref(nw_vec),
              std::ref(A_active_vec), std::ref(B_active_vec),
              std::ref(H_vec), std::ref(b_vec),
              method_to_solve_system_of_equations);
          thread_idx++;
        }
        thread_idx = 0;
        for (int sample_i = sample; sample_i < sample_end; sample_i++) {
          threads[thread_idx]->join();
          delete threads[thread_idx];
          thread_idx++;
        }
        sample = sample_end;
      }
      // Read P_i and q_i
      vector<MatrixXd> P_vec;
      vector<VectorXd> q_vec;
      readPiQiFile(&P_vec, &q_vec,
                   n_succ_sample, dir);
      // Print out elapsed time
      auto finish_time_calc_w = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed_calc_w =
          finish_time_calc_w - start_time_calc_w;
      cout << "Time spent on getting w in terms of theta: "
           << to_string(int(elapsed_calc_w.count())) << " seconds\n";
      cout << endl;

      prefix = to_string(iter) + "_";

      // Get gradient of the cost wrt theta and the norm of the gradient
      // Assumption: H_vec[sample] are symmetric
      VectorXd gradient_cost(n_theta);
      double norm_grad_cost;
      CalcCostGradientAndNorm(n_succ_sample, P_vec, q_vec, b_vec, dir, prefix,
                              &gradient_cost, &norm_grad_cost);

      // Calculate Newton step and the decrement
      VectorXd newton_step(n_theta);
      double lambda_square;
      CalcNewtonStepAndNewtonDecrement(n_theta, n_succ_sample, P_vec, H_vec,
                                       gradient_cost, dir, prefix, &newton_step,
                                       &lambda_square);

      // Check optimality
      if (HasAchievedOptimum(is_newton, stopping_threshold, lambda_square,
                      norm_grad_cost)) {
        break;
      }

      // Calculate step_direction
      double step_direction_norm;
      GetStepDirectionAndNorm(
          is_newton, newton_step, gradient_cost, beta_momentum, dir, prefix,
          &prev_step_direction, &step_direction, &step_direction_norm);

      // Calculate step size
      GetHeuristicStepSize(h_step, step_direction_norm, dir, prefix,
                           &current_iter_step_size);

      // Gradient descent
      prev_theta = theta;
      theta = theta + current_iter_step_size * step_direction;

      // Assign theta_s and theta_sDDot
      theta_s = theta.head(n_theta_s);
      theta_sDDot = theta.tail(n_theta_sDDot);

      // For message printed to the terminal
      n_shrink_step = 0;

      //cout << '\a';  // making noise to notify the user the end of an iteration
    }  // end if(!is_get_nominal)
  }  // end for


  // store parameter values

  // if we want to use theta from files for next iteration,we can not save new theta
  // which will then overwrite the original theta in files.
  bool use_theta_from_files = FLAGS_use_theta_gamma_from_files;
  prefix = to_string(iter + 1) +  "_";
  if (!FLAGS_is_debug) {
      if (!use_theta_from_files){
          writeCSV(dir + prefix + string("theta_s.csv"), theta_s);
          writeCSV(dir + prefix + string("theta_sDDot.csv"), theta_sDDot);
      }
  }

  return 0;
}  // int findGoldilocksModels

}  // namespace dairlib::goldilocks_models

int main(int argc, char* argv[]) {
    return dairlib::goldilocks_models::findGoldilocksModels(argc, argv);
}
