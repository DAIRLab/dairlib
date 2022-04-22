#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>   // queue with feature of finding elements
#include <queue>   // First in first out
#include <thread>  // multi-threading
#include <tuple>
#include <utility>  // std::pair, std::make_pair

#include <Eigen/QR>       // CompleteOrthogonalDecomposition
#include <bits/stdc++.h>  // system call
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/file_utils.h"
#include "common/find_resource.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/goldilocks_models/find_models/initial_guess.h"
#include "examples/goldilocks_models/find_models/traj_opt_given_weigths.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "examples/goldilocks_models/task.h"

#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::Vector3d;
using Eigen::VectorXcd;
using Eigen::VectorXd;
using std::cin;
using std::cout;
using std::endl;
using std::pair;
using std::string;
using std::to_string;
using std::vector;

using dairlib::FindResourceOrThrow;
using drake::AutoDiffXd;
using drake::geometry::SceneGraph;
using drake::multibody::Body;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;

namespace dairlib::goldilocks_models {

// clang-format off

// Robot models
DEFINE_int32(robot_option, 0, "0: plannar robot. 1: cassie_fixed_spring");
// Reduced order models
DEFINE_int32(rom_option, 0, "");

// tasks
// Note that it's more convenient to use duration than velocity, because if we
// fix velocity, the duration become 0 when stride length is 0 which combined
// with swing foot height constraint would make the problem infeasible.
DEFINE_int32(N_sample_sl, 1, "Sampling # for stride length");
DEFINE_int32(N_sample_gi, 1, "Sampling # for ground incline");
DEFINE_int32(N_sample_du, 1, "Sampling # for stride duration");
DEFINE_int32(N_sample_tr, 1, "Sampling # for turning rate");
DEFINE_int32(N_sample_ph, 1, "Sampling # for pelvis height");
DEFINE_int32(N_sample_sm, 1, "Sampling # for stance width");
DEFINE_double(sl_min, 0.25, "min stride length");
DEFINE_double(sl_max, 0.25, "max stride length");
DEFINE_double(gi_min, 0, "min ground incline");
DEFINE_double(gi_max, 0, "max ground incline");
DEFINE_double(du_min, 0.35, "min stride duration");
DEFINE_double(du_max, 0.35, "max stride duration");
DEFINE_double(tr_min, 0, "min turning rate");
DEFINE_double(tr_max, 0, "max turning rate");
DEFINE_double(ph_min, 0.8, "min pelvis height");
DEFINE_double(ph_max, 0.8, "max pelvis height");
DEFINE_double(sm_min, 0.0, "min stance width");
DEFINE_double(sm_max, 0.15, "max stance width");
DEFINE_double(stride_length_center, 0.0, "stride length center for grid method");
DEFINE_double(ground_incline_center, 0.0, "ground incline center for grid method");
DEFINE_double(turning_rate_center, 0.0, "turning rate center for grid method");
DEFINE_double(pelvis_height_center, 0.95, "pelvis height center for grid method");
DEFINE_double(swing_margin_center, 0.03, "swing margin center for grid method");
DEFINE_double(stride_length_delta, 0.03, "stride length delta for grid method");
DEFINE_double(ground_incline_delta, 0.05, "ground incline delta for grid method");
DEFINE_double(turning_rate_delta, 0.125, "turning rate delta for grid method");
DEFINE_double(pelvis_height_delta, 0.05, "pelvis height delta for grid method");
DEFINE_double(swing_margin_delta, 0.02, "swing margin delta for grid method");
DEFINE_bool(is_zero_touchdown_impact, false,
            "No impact force at fist touchdown");
DEFINE_bool(is_add_tau_in_cost, true, "Add RoM input in the cost function");
DEFINE_bool(is_grid_task, true,
    "Uniform grid of task space. If non-uniform grid, use the interpolated "
    "initial guess and restrict the number of samples");

// cost weights (use flags for convenience)
DEFINE_double(Q, 0, "cost weight Q");
DEFINE_double(R, 0, "cost weight R");
DEFINE_double(w_joint_accel, 0, "cost weight w_joint_accel");

// inner loop
DEFINE_string(init_file, "",
              "Initial Guess for Trajectory Optimization. "
              "E.g. w0.csv");
DEFINE_double(major_optimality_tol, 2e-4,
              "tolerance for optimality condition (complementarity gap)");
DEFINE_double(major_feasibility_tol, 1e-4,
              "nonlinear constraint violation tol");
DEFINE_int32(max_inner_iter, 150,
    "Max iteration # for traj opt. Sometimes, snopt takes very small steps "
    "(TODO: find out why), so maybe it's better to stop at some iterations and "
    "resolve again."
    "Note that this number cannot be too small. Otherwise, it might never solve "
    "successfuly");
DEFINE_bool(fix_node_number, true, "Fix the number of nodes in traj opt");
DEFINE_double(node_density, 40, "# of nodes per second in traj opt");
// Two things you need to be careful about node density (keep an eye on these
// in the future):
// 1. If the number of nodes are too high, it will take much time to solve.
//    (30 is probably considered to be high)
// 2. If the # of nodes per distance is too low, it's harder for SNOPT to
// converge well. E.g. (Cassie) the ratio of distance per nodes = 0.2/16 is fine
// for SNOPT, but 0.3 / 16 is too high.
DEFINE_double(eps_regularization, 1e-9,
              "Weight of regularization term");  // 1e-4
DEFINE_bool(use_database, false,
    "use solutions from database to create initial guesses for traj opt");
DEFINE_bool(snopt_scaling, false, "SNOPT built-in scaling feature");
DEFINE_bool(ipopt, false, "Use IPOPT as solver instead of SNOPT");

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
// - I didn't benchmark if method 4 to 6 get very slow when model parameter size
//  is big (i.e. B matrix has many columns).
// - The solving speed increases from method 4 to method 6 (method4 is about 10x
//  faster than method6). As for the accuracy, method 5 and 6 are better and
//  more robust than method 4. (method 5 appears to be more accurate as of
//  2021/06/12)
// - Method 1 and 2 require the matrix (A in Ax=b) being positive definite.
DEFINE_int32(method_to_solve_system_of_equations, 5,
             "Method 0: use optimization program to solve it "
             "Method 1: inverse the matrix by schur complement "
             "Method 2: inverse the matrix by inverse() "
             "Method 3: use Moore-Penrose pseudo inverse "
             "Method 4: linear solve by householderQr "
             "Method 5: linear solve by ColPivHouseholderQR "
             "Method 6: linear solve by bdcSvd ");

// How to update the model iterations
DEFINE_bool(start_current_iter_as_rerun, false,
            "Is `iter_start` a rerun? If it is, then you start with the stored "
            "tasks and use previous solution as initial guess");
DEFINE_int32(N_rerun, -1,
    "snopt might settle down at a bad sub-optimal solution, so we rerun.");
DEFINE_double(fail_threshold, 0.2,
              "Maximum acceptable failure rate of samples");
DEFINE_bool(get_good_sol_from_adjacent_sample, true,
            "Get a good solution from adjacent samples to improve the solution "
            "quality of the current sample");

// Other features for how to start the program
DEFINE_bool(read_previous_step_size, true,
    "We need the previous step size, if it fails to evaluate at `iter_start`");
DEFINE_bool(start_iterations_with_shrinking_stepsize, false,
            "Start the iterations with shrinking step size. Skip the smaple "
            "evaluation steps.");
DEFINE_double(initial_extra_shrink_factor, -1,
              "shrinking factor for the step size");
DEFINE_bool(is_debug, false, "Debugging or not");
DEFINE_bool(no_model_update, false,
    "This is used after the model has been optimized. We turn this flag on to "
    "evaluate the given model's performance on different task");
DEFINE_int32(delta_iter, 1, "Sometimes I want to skip iterations when "
                            "re-evaluating");

// Extend model from passive to actuated
DEFINE_bool(extend_model, false,
            "Extend the model in iteration # iter_start "
            "which is not equal to 0.");
DEFINE_int32(extend_model_iter, -1, "The starting iteration #");

// Multithread
DEFINE_bool(is_multithread, true, "Use multi-thread or not");
DEFINE_int32(n_thread_to_use, -1, "# of threads you want to use");

// Others
DEFINE_string(program_name, "",
              "The name of the program (to keep a record for future references)");
DEFINE_string(data_folder_name, "",
              "if non-empty, then we create one extra layer of folder");
DEFINE_bool(turn_off_cin, false, "disable std::cin to the program");

// clang-format on

// Testing
DEFINE_bool(com_accel_constraint, false, "");
DEFINE_bool(cubic_spline_in_rom_constraint, false, "");
DEFINE_bool(swing_foot_cublic_spline, false, "");
DEFINE_bool(zero_ending_pelvis_angular_vel, false, "");

DEFINE_bool(only_update_wrt_main_cost, false, "");

void setCostWeight(double* Q, double* R, double* w_joint_accel,
                   double* all_cost_scale, int robot_option) {
  if (robot_option == 0) {
    *Q = 1;
    *R = 0.1;
    //*w_joint_accel = 0.0001;  // not implemented yet
    //*all_cost_scale = 1;  // not implemented yet
  } else if (robot_option == 1) {
    *Q = 0.1;  // big weight: 0.1; small weight 0.005
    *R = 0.0002;
    // Final weight for joint accel is w_joint_accel * W_Q
    *w_joint_accel = 0.0001;  // big: 0.002; small: 0.0001
    *all_cost_scale = 1 /* * 0.6*/;
  }
}

void getInitFileName(string* init_file, const string& nominal_traj_init_file,
                     int iter, int sample, bool is_get_nominal,
                     bool rerun_current_iteration, bool has_been_all_success,
                     bool step_size_shrinked_last_loop, int n_rerun,
                     int sample_idx_to_help, const string& dir,
                     const TasksGenerator* task_gen, const Task& task,
                     const ReducedOrderModel& rom, bool non_grid_task,
                     bool use_database, int robot_option, bool no_model_update,
                     int delta_iter) {
  if (is_get_nominal && !rerun_current_iteration) {
    if (use_database) {
      *init_file = SetInitialGuessByInterpolation(
          dir, iter, sample, task_gen, task, rom, use_database, robot_option);
    } else {
      *init_file = nominal_traj_init_file;
    }
  } else if (step_size_shrinked_last_loop && n_rerun == 0) {
    // the step size was shrink in previous iter and it's not a local rerun
    // (n_rerun == 0)
    if (non_grid_task) {
      *init_file = SetInitialGuessByInterpolation(
          dir, iter, sample, task_gen, task, rom, use_database, robot_option);
    } else {
      *init_file =
          to_string(iter - 1) + "_" + to_string(sample) + string("_w.csv");
    }
  } else if (sample_idx_to_help >= 0) {
    *init_file = to_string(iter) + "_" + to_string(sample_idx_to_help) +
                 string("_w.csv");
  } else if (rerun_current_iteration) {
    *init_file = to_string(iter) + "_" + to_string(sample) + string("_w.csv");
  } else if (no_model_update) {
    *init_file = to_string(iter - delta_iter) + "_" + to_string(sample) +
                 string("_w.csv");
  } else {
    if (non_grid_task) {
      *init_file = SetInitialGuessByInterpolation(
          dir, iter, sample, task_gen, task, rom, use_database, robot_option);
    } else {
      *init_file =
          to_string(iter - 1) + "_" + to_string(sample) + string("_w.csv");
    }
  }
}

int selectThreadIdxToWait(const vector<pair<int, int>>& assigned_thread_idx,
                          vector<std::shared_ptr<int>> thread_finished_vec) {
  int counter = 0;
  while (true) {
    // cout << "Check if any thread has finished...\n";
    for (unsigned int i = 0; i < assigned_thread_idx.size(); i++) {
      if (*(thread_finished_vec[assigned_thread_idx[i].second]) == 1) {
        *(thread_finished_vec[assigned_thread_idx[i].second]) = 0;
        // cout << "sample"<<assigned_thread_idx[i].second<<" just finished\n";
        return i;
      }
    }
    if ((counter % 60 == 0)) {
      // cout << "No files exists yet. Sleep for 1 seconds.\n";
    }
    counter++;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

void waitForAllThreadsToJoin(vector<std::thread*>* threads,
                             vector<pair<int, int>>* assigned_thread_idx,
                             vector<std::shared_ptr<int>> thread_finished_vec) {
  // TODO: can I kill the thread instead of waiting for it to finish?

  while (!assigned_thread_idx->empty()) {
    // Select index to wait and delete csv files
    int selected_idx =
        selectThreadIdxToWait(*assigned_thread_idx, thread_finished_vec);
    int thread_to_wait_idx = (*assigned_thread_idx)[selected_idx].first;
    /*string string_to_be_print = "Waiting for thread #" +
                                to_string(thread_to_wait_idx) + " to join...\n";
    cout << string_to_be_print;*/
    (*threads)[thread_to_wait_idx]->join();
    delete (*threads)[thread_to_wait_idx];
    // string_to_be_print = "Thread #" + to_string(thread_to_wait_idx) +
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

void extendModel(const string& dir, int iter, ReducedOrderModel& rom,
                 VectorXd& prev_theta, VectorXd& step_direction,
                 VectorXd& prev_step_direction, double& ave_min_cost_so_far,
                 int& rom_option, int robot_option) {
  throw std::runtime_error(
      "Model extension implementation hasn't not been updated");
  /*
    int n_feature_y = rom.n_feature_y();

    VectorXd theta_s_append = readCSV(dir +
                                      string("theta_s_append.csv")).col(0);
    int n_extend = theta_s_append.rows() / n_feature_y;

    // update rom_option
    if(rom_option == 0) {
      rom_option = 1;
    } else if (rom_option == 2) {
      rom_option = 3;
    } else {
      throw std::runtime_error("Should not reach here");
    }

    // update n_y, n_yddot and n_tau
    int old_n_y = rom.n_y();
    rom.set_n_y(rom.n_y() + n_extend);
    rom.set_n_yddot(rom.n_yddot() + n_extend);
    rom.set_n_tau(rom.n_tau() + n_extend);
    int n_y = rom.n_y();
    int n_yddot = rom.n_yddot();
    int n_tau = rom.n_tau();

    // update n_feature_yddot
    int old_n_feature_sDDot = rom.n_feature_yddot();
    DynamicsExpression dyn_expression(n_yddot, 0, rom_option, robot_option);
    VectorXd dummy_s = VectorXd::Zero(n_y);
    rom.set_n_feature_yddot(dyn_expression.getFeature(dummy_s, dummy_s).size());
    int n_feature_yddot = rom.n_feature_yddot();
    cout << "Updated n_y = " << n_y << endl;
    cout << "Updated n_yddot = " << n_yddot << endl;
    cout << "Updated n_tau = " << n_tau << endl;
    cout << "Updated n_feature_yddot = " << n_feature_yddot << endl;

    // update B_tau
    MatrixXd B_tau_old = rom.B();
    MatrixXd B_tau(n_yddot, n_tau);
    B_tau = MatrixXd::Zero(n_yddot, n_tau);
    B_tau.block(0, 0, B_tau_old.rows(), B_tau_old.cols()) = B_tau_old;
    B_tau.block(B_tau_old.rows(), B_tau_old.cols(), n_extend, n_extend) =
      MatrixXd::Identity(n_extend, n_extend);
    rom.SetB(B_tau);
    cout << "Updated B_tau = \n" << rom.B() << endl;
    writeCSV(dir + string("B_tau (before extension).csv"), B_tau_old);
    writeCSV(dir + string("B_tau.csv"), rom.B());
    // update theta_y
    string prefix = to_string(iter) +  "_";
    writeCSV(dir + prefix + string("theta_y (before extension).csv"),
             rom.theta_y());
    VectorXd theta_y(n_y * n_feature_y);
    theta_y << rom.theta_y(), theta_s_append;
    rom.SetThetaY(theta_y);
    // update theta_yddot
    writeCSV(dir + prefix + string("theta_yddot (before extension).csv"),
             rom.theta_yddot());
    VectorXd theta_sDDot_old = rom.theta_yddot();
    VectorXd theta_yddot = VectorXd::Zero(n_yddot * n_feature_yddot);
    VectorXd new_idx = readCSV(dir +
                               string("theta_sDDot_new_index.csv")).col(0);
    for (int i = 0; i < old_n_feature_sDDot; i++)
      for (int j = 0; j < old_n_y; j++)
        theta_yddot(new_idx(i) + j * n_feature_yddot) = theta_sDDot_old(
              i + j * old_n_feature_sDDot);
    rom.SetThetaYddot(theta_yddot);

    // Some setup
    prev_theta.resize(rom.n_theta());
    prev_theta = rom.theta();
    step_direction.resize(rom.n_theta());
    prev_step_direction.resize(rom.n_theta());
    prev_step_direction = VectorXd::Zero(
        rom.n_theta());  // must initialize it because of momentum term
    ave_min_cost_so_far = std::numeric_limits<double>::infinity();

    rom.CheckModelConsistency();
    */
}

void extractActiveAndIndependentRows(int sample, double active_tol,
                                     double indpt_row_tol, string dir,
                                     const SubQpData& QPs,
                                     int method_to_solve_system_of_equations) {
  string prefix = to_string(sample) + "_";

  DRAKE_ASSERT(QPs.b_vec[sample]->cols() == 1);
  DRAKE_ASSERT(QPs.lb_vec[sample]->cols() == 1);
  DRAKE_ASSERT(QPs.ub_vec[sample]->cols() == 1);
  DRAKE_ASSERT(QPs.y_vec[sample]->cols() == 1);
  DRAKE_ASSERT(QPs.w_sol_vec[sample]->cols() == 1);

  int nt_i = QPs.B_vec[sample]->cols();
  int nw_i = QPs.A_vec[sample]->cols();

  int nl_i = 0;
  for (int i = 0; i < QPs.y_vec[sample]->rows(); i++) {
    if ((*(QPs.y_vec[sample]))(i) >= (*(QPs.ub_vec[sample]))(i)-active_tol ||
        (*(QPs.y_vec[sample]))(i) <= (*(QPs.lb_vec[sample]))(i) + active_tol)
      nl_i++;
  }

  MatrixXd A_active(nl_i, nw_i);
  MatrixXd B_active(nl_i, nt_i);

  nl_i = 0;
  for (int i = 0; i < QPs.y_vec[sample]->rows(); i++) {
    if ((*(QPs.y_vec[sample]))(i) >= (*(QPs.ub_vec[sample]))(i)-active_tol ||
        (*(QPs.y_vec[sample]))(i) <= (*(QPs.lb_vec[sample]))(i) + active_tol) {
      A_active.row(nl_i) = QPs.A_vec[sample]->row(i);
      B_active.row(nl_i) = QPs.B_vec[sample]->row(i);
      nl_i++;
    }
  }

  bool is_testing = false;
  if (is_testing) {
    // Run a quadprog to check if the solution to the following problem is 0
    // Theoratically, it should be 0. Otherwise, something is wrong
    // min 0.5*w^T Q w + c^T w
    // st  A w = 0
    if (sample == 0) {
      cout << "\n (After extracting active constraints) Run traj opt to "
              "check if your quadratic approximation is correct\n";
      cout << "sample# | Solve Status | Solve time | Cost | w_sol norm | (this "
              "should be 0 if w=0 is optimal)\n";
    }
    nl_i = A_active.rows();
    MathematicalProgram quadprog;
    auto w2 = quadprog.NewContinuousVariables(nw_i, "w2");
    quadprog.AddLinearConstraint(A_active, VectorXd::Zero(nl_i),
                                 VectorXd::Zero(nl_i), w2);
    quadprog.AddQuadraticCost(*(QPs.H_vec[sample]), *(QPs.b_vec[sample]), w2);

    // (Testing) use snopt to solve the QP
    bool use_snopt = true;
    drake::solvers::SnoptSolver snopt_solver;

    auto start = std::chrono::high_resolution_clock::now();
    const auto result =
        use_snopt ? snopt_solver.Solve(quadprog) : Solve(quadprog);
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;

    auto solution_result = result.get_solution_result();
    if (result.is_success()) {
      VectorXd w_sol_check = result.GetSolution(quadprog.decision_variables());
      cout << sample << " | " << solution_result << " | " << elapsed.count()
           << "| " << result.get_optimal_cost() << " | " << w_sol_check.norm()
           << " | " << w_sol_check.transpose() * (*(QPs.b_vec[sample])) << endl;
    } else {
      cout << sample << " | " << solution_result << " | " << elapsed.count()
           << " | " << result.get_optimal_cost() << endl;
    }
  }

  // Only add the rows that are linearly independent if the method requires A to
  // be positive definite
  if (method_to_solve_system_of_equations == 1 ||
      method_to_solve_system_of_equations == 2) {
    // extract_method = 0: Do SVD each time when adding a row. (This has been
    //  working, but we later realized that the way we extract B matrix might be
    //  incorrect in theory)
    // extract_method = 1: Do SVD only once (see the notes on 20200220).
    int extract_method = 1;

    if (extract_method == 0) {
      /*cout << "n_w = " << nw_i << endl;
      cout << "Start extracting independent rows of A (# of rows = " << nl_i
           << ")\n";*/
      vector<int> full_row_rank_idx;
      full_row_rank_idx.push_back(0);
      for (int i = 1; i < nl_i; i++) {
        // cout << "total i = " << nl_i;
        // cout << ", i = " << i << endl;
        // Construct test matrix
        int n_current_rows = full_row_rank_idx.size();
        MatrixXd A_test(n_current_rows + 1, nw_i);
        for (unsigned int j = 0; j < full_row_rank_idx.size(); j++) {
          A_test.block(j, 0, 1, nw_i) = A_active.row(full_row_rank_idx[j]);
        }
        A_test.block(n_current_rows, 0, 1, nw_i) = A_active.row(i);

        // Perform svd to check rank
        Eigen::BDCSVD<MatrixXd> svd(A_test);
        // double sigular_value = svd.singularValues()(n_current_rows);
        if (svd.singularValues()(n_current_rows) > indpt_row_tol) {
          full_row_rank_idx.push_back(i);
        }

        if ((int)full_row_rank_idx.size() == nw_i) {
          cout << "A.row() == A.cols(), so stop adding rows.\n";
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
      *(QPs.nw_vec[sample]) = nw_i;
      *(QPs.nl_vec[sample]) = nl_i;
      QPs.A_active_vec[sample]->resizeLike(A_processed);
      QPs.B_active_vec[sample]->resizeLike(B_processed);
      *(QPs.A_active_vec[sample]) = A_processed;
      *(QPs.B_active_vec[sample]) = B_processed;
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
      *(QPs.nw_vec[sample]) = nw_i;
      *(QPs.nl_vec[sample]) = rank;
      QPs.A_active_vec[sample]->resizeLike(A_processed);
      QPs.B_active_vec[sample]->resizeLike(B_processed);
      *(QPs.A_active_vec[sample]) = A_processed;
      *(QPs.B_active_vec[sample]) = B_processed;
    } else {
      throw std::runtime_error("Should not reach here");
    }

    // cout << "sample #" << sample;
    // cout << "    A active and independent rows = " << nl_i << endl;
  } else {
    // No need to extract independent rows, so store the result right away

    // Store the results in csv files
    *(QPs.nw_vec[sample]) = nw_i;
    *(QPs.nl_vec[sample]) = nl_i;
    QPs.A_active_vec[sample]->resizeLike(A_active);
    QPs.B_active_vec[sample]->resizeLike(B_active);
    *(QPs.A_active_vec[sample]) = A_active;
    *(QPs.B_active_vec[sample]) = B_active;
  }
}

MatrixXd solveInvATimesB(const MatrixXd& A, const MatrixXd& B) {
  // Least squares solution to AX = B
  MatrixXd X = (A.transpose() * A).ldlt().solve(A.transpose() * B);

  // TODO: Test if the following line works better
  // MatrixXd X = A.completeOrthogonalDecomposition().solve(B);

  MatrixXd abs_resid = (A * X - B).cwiseAbs();
  VectorXd left_one = VectorXd::Ones(abs_resid.rows());
  VectorXd right_one = VectorXd::Ones(abs_resid.cols());
  cout << "sum-abs-residual: " << left_one.transpose() * abs_resid * right_one
       << endl;
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

void calcWInTermsOfTheta(int sample, const string& dir, const SubQpData& QPs,
                         int method_to_solve_system_of_equations) {
  string prefix = to_string(sample) + "_";

  if (sample == 0) {
    cout << "sample # | max element of abs-Pi | qi norm (this number should be "
            "close to 0)\n";
  }

  MatrixXd Pi(*(QPs.nw_vec[sample]), QPs.B_active_vec[sample]->cols());
  VectorXd qi(*(QPs.nw_vec[sample]));
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
    MatrixXd AinvHA = (*(QPs.A_active_vec[sample])) *
                      solveInvATimesB(*(QPs.H_vec[sample]),
                                      QPs.A_active_vec[sample]->transpose());
    VectorXd invQ = solveInvATimesB(*(QPs.H_vec[sample]), *(QPs.b_vec[sample]));
    MatrixXd E = solveInvATimesB(AinvHA, *(QPs.B_active_vec[sample]));
    VectorXd F = -solveInvATimesB(AinvHA, (*(QPs.A_active_vec[sample])) * invQ);
    // Testing
    Eigen::BDCSVD<MatrixXd> svd(AinvHA);
    cout << "AinvHA':\n";
    cout << "  biggest singular value is " << svd.singularValues()(0) << endl;
    cout << "  smallest singular value is " << svd.singularValues().tail(1)
         << endl;
    cout << "The condition number of A and invH are large. That's why AinvHA'"
            "is ill-conditioned.\n";
    // cout << "singular values are \n" << svd.singularValues() << endl;

    Pi = -solveInvATimesB(*(QPs.H_vec[sample]),
                          QPs.A_active_vec[sample]->transpose() * E);
    qi = -solveInvATimesB(
        *(QPs.H_vec[sample]),
        (*(QPs.b_vec[sample])) + QPs.A_active_vec[sample]->transpose() * F);
    cout << "qi norm (this number should be close to 0) = " << qi.norm()
         << endl;
  } else if ((method_to_solve_system_of_equations >= 2) &&
             (method_to_solve_system_of_equations <= 6)) {
    double residual_tol = 1e-4;
    bool successful = false;
    while (!successful) {
      // H_ext = [H A'; A 0]
      int nl_i = (*(QPs.nl_vec[sample]));
      int nw_i = (*(QPs.nw_vec[sample]));
      MatrixXd H_ext(nw_i + nl_i, nw_i + nl_i);
      H_ext.block(0, 0, nw_i, nw_i) = (*(QPs.H_vec[sample]));
      H_ext.block(0, nw_i, nw_i, nl_i) = QPs.A_active_vec[sample]->transpose();
      H_ext.block(nw_i, 0, nl_i, nw_i) = (*(QPs.A_active_vec[sample]));
      H_ext.block(nw_i, nw_i, nl_i, nl_i) = MatrixXd::Zero(nl_i, nl_i);

      if (method_to_solve_system_of_equations == 2) {
        // Method 2: use inverse() directly ////////////////////////////////////
        // (This one requires the Hessian H to be pd.)
        // This method has been working pretty well, but it requires H to be pd.
        // And in order to get pd H, we need to extract independent row of
        // matrix A, which takes too much time in the current method.
        MatrixXd inv_H_ext = H_ext.inverse();

        MatrixXd inv_H_ext11 = inv_H_ext.block(0, 0, nw_i, nw_i);
        MatrixXd inv_H_ext12 = inv_H_ext.block(0, nw_i, nw_i, nl_i);

        Pi = -inv_H_ext12 * (*(QPs.B_active_vec[sample]));
        qi = -inv_H_ext11 * (*(QPs.b_vec[sample]));

        successful = true;  // Don't use other methods after this
      } else if (method_to_solve_system_of_equations == 3) {
        // Method 3: use Moore–Penrose pseudo inverse //////////////////////////
        MatrixXd inv_H_ext = MoorePenrosePseudoInverse(H_ext, 1e-8);

        MatrixXd inv_H_ext11 = inv_H_ext.block(0, 0, nw_i, nw_i);
        MatrixXd inv_H_ext12 = inv_H_ext.block(0, nw_i, nw_i, nl_i);

        Pi = -inv_H_ext12 * (*(QPs.B_active_vec[sample]));
        qi = -inv_H_ext11 * (*(QPs.b_vec[sample]));

        successful = true;  // Don't use other methods after this
      } else if (method_to_solve_system_of_equations == 4) {
        // Method 4: linear solve with householderQr ///////////////////////////
        MatrixXd B_aug =
            MatrixXd::Zero(H_ext.rows(), QPs.B_active_vec[sample]->cols());
        B_aug.bottomRows(nl_i) = -(*(QPs.B_active_vec[sample]));

        MatrixXd sol = H_ext.householderQr().solve(B_aug);
        Pi = sol.topRows(nw_i);
        qi = VectorXd::Zero(nw_i);

        double max_error = (H_ext * sol - B_aug).cwiseAbs().maxCoeff();
        if (max_error > residual_tol || std::isnan(max_error) ||
            std::isinf(max_error)) {
          cout << "  sample " << sample << "'s max_error = " << max_error
               << "; switch to method 5\n";
          method_to_solve_system_of_equations = 5;
        } else {
          successful = true;
        }

        /*// Testing
        writeCSV(dir + to_string(sample) + "_H_ext_nan", H_ext, true);
        writeCSV(dir + to_string(sample) + "_B_aug_nan", B_aug, true);
        writeCSV(dir + to_string(sample) + "_Pi_nan", Pi, true);
        writeCSV(dir + to_string(sample) + "_nw_i_nan",
                 nw_i * VectorXd::Ones(1), true);*/

      } else if (method_to_solve_system_of_equations == 5) {
        // Method 5: linear solve with ColPivHouseholderQR /////////////////////
        MatrixXd B_aug =
            MatrixXd::Zero(H_ext.rows(), QPs.B_active_vec[sample]->cols());
        B_aug.bottomRows(nl_i) = -(*(QPs.B_active_vec[sample]));

        MatrixXd sol = Eigen::ColPivHouseholderQR<MatrixXd>(H_ext).solve(B_aug);
        Pi = sol.topRows(nw_i);
        qi = VectorXd::Zero(nw_i);

        double max_error = (H_ext * sol - B_aug).cwiseAbs().maxCoeff();
        if (max_error > residual_tol || std::isnan(max_error) ||
            std::isinf(max_error)) {
          cout << "  sample " << sample << "'s max_error = " << max_error
               << "; move on.\n";
        }
        break;

      } else if (method_to_solve_system_of_equations == 6) {
        // Method 6: linear solve with bdcSvd //////////////////////////////////
        // https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
        MatrixXd B_aug =
            MatrixXd::Zero(H_ext.rows(), QPs.B_active_vec[sample]->cols());
        B_aug.bottomRows(nl_i) = -(*(QPs.B_active_vec[sample]));

        /*Pi = H_ext.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                 .solve(B_aug)
                 .topRows(nw_i);
        qi = VectorXd::Zero(nw_i);*/

        MatrixXd sol = H_ext.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                           .solve(B_aug);
        Pi = sol.topRows(nw_i);
        qi = VectorXd::Zero(nw_i);

        double max_error = (H_ext * sol - B_aug).cwiseAbs().maxCoeff();
        if (max_error > residual_tol || std::isnan(max_error) ||
            std::isinf(max_error)) {
          cout << "  sample " << sample << "'s max_error = " << max_error
               << "; move on.\n";
        }
        break;
      }
    }
  } else {
    throw std::runtime_error("Should not reach here");
  }

  QPs.P_vec[sample]->resizeLike(Pi);
  QPs.q_vec[sample]->resizeLike(qi);
  *(QPs.P_vec[sample]) = Pi;
  *(QPs.q_vec[sample]) = qi;

  // Testing
  MatrixXd abs_Pi = Pi.cwiseAbs();
  //  VectorXd left_one = VectorXd::Ones(abs_Pi.rows());
  //  VectorXd right_one = VectorXd::Ones(abs_Pi.cols());
  // cout << "sum-abs-Pi: " <<
  //      left_one.transpose()*abs_Pi*right_one << endl;
  // cout << "sum-abs-Pi divide by m*n: " <<
  //      left_one.transpose()*abs_Pi*right_one / (abs_Pi.rows()*abs_Pi.cols())
  //      << endl;
  double max_Pi_element = Pi.cwiseAbs().maxCoeff();
  /*string to_be_print =
      "sample #" + to_string(sample) + ":  " +
      "max element of abs-Pi = " + to_string(max_Pi_element) +
      "\n           qi norm (this number should be close to 0) = " +
      to_string(qi.norm()) + "\n";*/
  string to_be_print = to_string(sample) + " | " + to_string(max_Pi_element) +
                       " | " + to_string(qi.norm()) + "\n";
  cout << to_be_print;
}

MatrixXi GetAdjSampleIndices(GridTasksGenerator task_gen_grid,
                             vector<int> n_node_vec) {
  // Setup
  int task_dim = task_gen_grid.dim_nondeg();
  int N_sample = task_gen_grid.total_sample_number();

  // cout << "Constructing adjacent index list...\n";
  MatrixXi adjacent_sample_indices =
      -1 * MatrixXi::Ones(N_sample, 2 * task_dim);
  MatrixXi delta_idx = MatrixXi::Identity(3, 3);
  for (int sample_idx = 0; sample_idx < N_sample; sample_idx++) {
    for (int i = 0; i < task_dim; i++) {
      vector<int> new_index_tuple =
          task_gen_grid.get_forward_map().at(sample_idx);
      new_index_tuple[i] += 1;

      // The new index tuple has to be valid
      if (new_index_tuple[i] < task_gen_grid.sample_numbers()[i]) {
        int adjacent_sample_idx =
            task_gen_grid.get_inverse_map().at(new_index_tuple);
        // Number of nodes should be the same so that we can set initial guess
        if (n_node_vec[sample_idx] == n_node_vec[adjacent_sample_idx]) {
          // Add to adjacent_sample_idx (both directions)
          for (int l = 0; l < 2 * task_dim; l++) {
            if (adjacent_sample_indices(sample_idx, l) < 0) {
              adjacent_sample_indices(sample_idx, l) = adjacent_sample_idx;
              break;
            }
          }
          for (int l = 0; l < 2 * task_dim; l++) {
            if (adjacent_sample_indices(adjacent_sample_idx, l) < 0) {
              adjacent_sample_indices(adjacent_sample_idx, l) = sample_idx;
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
                       int& sample_idx_to_help, int task_dim) {
  for (int i = 0; i < 2 * task_dim; i++) {
    if (sample_idx_waiting_to_help(sample_idx, i) >= 0) {
      sample_idx_to_help = sample_idx_waiting_to_help(sample_idx, i);
      // remove sample_idx_to_help from sample_idx_waiting_to_help
      sample_idx_waiting_to_help(sample_idx, i) = -1;

      // add sample_idx_to_help to sample_idx_that_helped
      for (int j = 0; j < 2 * task_dim; j++) {
        if (sample_idx_that_helped(sample_idx, j) < 0) {
          sample_idx_that_helped(sample_idx, j) = sample_idx_to_help;
          break;
        }
      }
      break;
    }
  }
  //  cout << "before adding idx # " << sample_idx << endl;
  //  cout << "sample_idx_waiting_to_help = \n"
  //       << sample_idx_waiting_to_help.transpose() << endl;
  //  cout << "sample_idx_that_helped = \n"
  //       << sample_idx_that_helped.transpose() << endl;
  DRAKE_DEMAND(sample_idx_to_help != -1);  // must exist a helper
}
void RecordSolutionQualityAndQueueList(
    const string& dir, const string& prefix, int sample_idx,
    const vector<pair<int, int>>& assigned_thread_idx,
    const MatrixXi& adjacent_sample_indices,
    double max_cost_increase_rate_before_ask_for_help,
    double max_adj_cost_diff_rate_before_ask_for_help,
    bool is_limit_difference_of_two_adjacent_costs, bool sample_success,
    bool current_sample_is_queued, int task_dim, const vector<int>& n_rerun,
    int N_rerun, vector<double>& each_min_cost_so_far,
    vector<double>& sample_status, MatrixXi& sample_idx_waiting_to_help,
    MatrixXi& sample_idx_that_helped, std::deque<int>& awaiting_sample_idx) {
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

      double cost_diff = sample_cost - each_min_cost_so_far[adj_idx];
      // if the current sample cost is much higher than the adjacent cost
      if (cost_diff > max_adj_cost_diff_rate_before_ask_for_help *
                          each_min_cost_so_far[sample_idx]) {
        // If the adjacent sample has a good solution and has finished basic
        // reruns
        if ((sample_status[adj_idx] == 1) && (n_rerun[adj_idx] >= N_rerun)) {
          low_adjacent_cost_idx.push_back(adj_idx);
        }
      }
      // if the current sample cost is much lower than the adjacent cost
      else if (cost_diff < -max_adj_cost_diff_rate_before_ask_for_help *
                               each_min_cost_so_far[sample_idx]) {
        // If the adjacent sample has a good solution and has finished basic
        // reruns
        if ((sample_status[adj_idx] == 1) && (n_rerun[adj_idx] >= N_rerun)) {
          high_adjacent_cost_idx.push_back(adj_idx);
        }
      }
    }
  }
  bool too_high_above_adjacent_cost = !low_adjacent_cost_idx.empty();
  // bool too_low_below_adjacent_cost = !high_adjacent_cost_idx.empty();

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
  if (sample_success &&
      (sample_cost <= (1 + max_cost_increase_rate_before_ask_for_help) *
                          each_min_cost_so_far[sample_idx]) &&
      !too_high_above_adjacent_cost) {
    // Set the current sample to be having good solution
    sample_status[sample_idx] = 1;

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
        for (int i = 0; i < 2 * task_dim; i++) {
          if (sample_idx_that_helped(adj_idx, i) == sample_idx) {
            sample_idx_that_helped(adj_idx, i) = -1;
            already_exist = true;
            break;
          }
        }
        // Add current sample to sample_idx_waiting_to_help.
        if (already_exist) {
          for (int i = 0; i < 2 * task_dim; i++) {
            if (sample_idx_waiting_to_help(adj_idx, i) == -1) {
              sample_idx_waiting_to_help(adj_idx, i) = sample_idx;
              break;
            }
          }
        }
      }

      bool this_adjacent_sample_needs_help = false;
      bool current_sample_has_helped = false;

      bool adj_has_bad_sol = sample_status[adj_idx] == 0;
      bool adj_has_too_high_cost =
          (find(high_adjacent_cost_idx.begin(), high_adjacent_cost_idx.end(),
                adj_idx) != high_adjacent_cost_idx.end());
      /*
      cout << "(sample_idx, adj_idx, adj_has_bad_sol, adj_has_too_high_cost) = "
           << sample_idx << ", " << adj_idx << ", " << adj_has_bad_sol << ", "
           << adj_has_too_high_cost << endl;*/
      bool revert_good_adj_sol_to_bad_sol = false;  // for printing
      if (adj_has_bad_sol) {
        this_adjacent_sample_needs_help = true;

        // Add current sample to the helper list of adjacent sample
        // (add if it doesn't exist in both sample_idx_waiting_to_help
        //  and sample_idx_that_helped)
        bool already_exist = false;
        for (int i = 0; i < 2 * task_dim; i++) {
          already_exist = already_exist || (sample_idx_waiting_to_help(
                                                adj_idx, i) == sample_idx);
        }
        for (int i = 0; i < 2 * task_dim; i++) {
          already_exist = already_exist ||
                          (sample_idx_that_helped(adj_idx, i) == sample_idx);
          current_sample_has_helped =
              current_sample_has_helped ||
              (sample_idx_that_helped(adj_idx, i) == sample_idx);
        }
        if (!already_exist) {
          for (int i = 0; i < 2 * task_dim; i++) {
            if (sample_idx_waiting_to_help(adj_idx, i) == -1) {
              sample_idx_waiting_to_help(adj_idx, i) = sample_idx;
              break;
            }
          }
        }
      } else if (adj_has_too_high_cost) {
        this_adjacent_sample_needs_help = true;

        sample_status[adj_idx] = 0;
        revert_good_adj_sol_to_bad_sol = true;

        // Add current sample to sample_idx_waiting_to_help.
        bool already_exist = false;
        for (int i = 0; i < 2 * task_dim; i++) {
          already_exist = already_exist || (sample_idx_waiting_to_help(
                                                adj_idx, i) == sample_idx);
        }
        if (!already_exist) {
          for (int i = 0; i < 2 * task_dim; i++) {
            if (sample_idx_waiting_to_help(adj_idx, i) == -1) {
              sample_idx_waiting_to_help(adj_idx, i) = sample_idx;
              break;
            }
          }
        }
        // Remove current sample from sample_idx_that_helped.
        for (int i = 0; i < 2 * task_dim; i++) {
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
    sample_status[sample_idx] = 0;

    // Look for any adjacent sample that can help
    for (int j = 0; j < adjacent_sample_indices.cols(); j++) {
      bool this_adjacent_sample_can_help = false;
      bool this_adjacent_sample_is_waiting_to_help = false;

      // if the adjacent sample has a good solution, then add it to the
      // helper list
      int adj_idx = adjacent_sample_indices(sample_idx, j);
      if (adj_idx == -1) continue;

      bool adj_has_good_sol = sample_status[adj_idx] == 1;
      bool adj_has_too_low_cost =
          (find(low_adjacent_cost_idx.begin(), low_adjacent_cost_idx.end(),
                adj_idx) != low_adjacent_cost_idx.end());

      bool low_adj_cost_idx_has_helped = false;
      for (int i = 0; i < 2 * task_dim; i++) {
        low_adj_cost_idx_has_helped =
            low_adj_cost_idx_has_helped ||
            (sample_idx_that_helped(sample_idx, i) == adj_idx);
      }

      bool add_adj_as_helper_because_low_cost = false;  // for printing message
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
        for (int i = 0; i < 2 * task_dim; i++) {
          if (sample_idx_waiting_to_help(sample_idx, i) == adj_idx) {
            already_exist_matrix_idx = i;
            break;
          }
        }
        int first_helper_idx = sample_idx_waiting_to_help(sample_idx, 0);
        sample_idx_waiting_to_help(sample_idx, 0) = adj_idx;
        if (already_exist_matrix_idx < 0) {
          for (int i = 1; i < 2 * task_dim; i++) {
            if (sample_idx_waiting_to_help(sample_idx, i) == -1) {
              sample_idx_waiting_to_help(sample_idx, i) = first_helper_idx;
              break;
            }
          }
        } else if (already_exist_matrix_idx > 0) {
          for (int i = 1; i < 2 * task_dim; i++) {
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
        for (int i = 0; i < 2 * task_dim; i++) {
          already_exist = already_exist || (sample_idx_waiting_to_help(
                                                sample_idx, i) == adj_idx);
          this_adjacent_sample_is_waiting_to_help = already_exist;
        }
        for (int i = 0; i < 2 * task_dim; i++) {
          already_exist = already_exist ||
                          (sample_idx_that_helped(sample_idx, i) == adj_idx);
        }
        if (!already_exist) {
          for (int i = 0; i < 2 * task_dim; i++) {
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
          if (!sample_success) {
            cout << "(snopt didn't find an optimal sol)";
          } else {
            cout << "(cost increased too much)";
          }
          cout << ", and idx #" << adj_idx << " can help, so add #"
               << sample_idx << " to queue\n";
        }
      }
    }  // end for (Look for any adjacent sample that can help)
  }    // end if current sample has good solution
}

// Calculate the cost gradient and its norm
void CalcCostGradientAndNorm(vector<int> successful_idx_list,
                             const vector<std::shared_ptr<MatrixXd>>& P_vec,
                             const vector<std::shared_ptr<VectorXd>>& q_vec,
                             const vector<std::shared_ptr<VectorXd>>& b_vec,
                             const string& dir, const string& prefix,
                             VectorXd* gradient_cost, double* norm_grad_cost) {
  cout << "Calculating gradient\n";
  gradient_cost->setZero();
  for (auto idx : successful_idx_list) {
    (*gradient_cost) += P_vec[idx]->transpose() * (*(b_vec[idx]));
  }
  (*gradient_cost) /= successful_idx_list.size();

  // Calculate gradient norm
  (*norm_grad_cost) = gradient_cost->norm();
  writeCSV(dir + prefix + string("norm_grad_cost.csv"),
           (*norm_grad_cost) * VectorXd::Ones(1));
  cout << "gradient_cost norm: " << (*norm_grad_cost) << endl << endl;
}

// Newton's method (not exactly the same, cause Q_theta is not pd but psd)
// See your IOE611 lecture notes on page 7-17 to page 7-20
void CalcNewtonStepAndNewtonDecrement(
    int n_theta, vector<int> successful_idx_list,
    const vector<std::shared_ptr<MatrixXd>>& P_vec,
    const vector<std::shared_ptr<MatrixXd>>& H_vec,
    const VectorXd& gradient_cost, const string& dir, const string& prefix,
    VectorXd* newton_step, double* lambda_square) {
  /*// Check if Q_theta is pd
  cout << "Checking if Q_theta is psd...\n";
  MatrixXd Q_theta = MatrixXd::Zero(n_theta, n_theta);
  for (auto idx : successful_idx_list)
    Q_theta += P_vec[idx]->transpose()*(*(H_vec[idx]))*(*(P_vec[idx]));
  VectorXd eivals_real = Q_theta.eigenvalues().real();
  for (int i = 0; i < eivals_real.size(); i++) {
    if (eivals_real(i) <= 0)
      cout << "Q_theta is not positive definite (with e-value = "
           << eivals_real(i) << ")\n";
  }
  cout << endl;*/

  // cout << "Getting Newton step\n";
  MatrixXd Q_theta = MatrixXd::Zero(n_theta, n_theta);
  for (auto idx : successful_idx_list) {
    Q_theta += P_vec[idx]->transpose() * (*(H_vec[idx])) * (*(P_vec[idx])) /
               successful_idx_list.size();
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
    // Heuristic
    // (*current_iter_step_size) = h_step / sqrt(step_direction_norm);
    // Heuristic
    (*current_iter_step_size) = h_step / step_direction_norm;
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

/*void remove_old_multithreading_files(const string& dir, int iter,
                                     int N_sample) {
  cout << "\nRemoving old thread_finished.csv files... ";
  for (int i = 0; i < N_sample; i++) {
    string prefix = to_string(iter) + "_" + to_string(i) + "_";
    if (file_exist(dir + prefix + "thread_finished.csv")) {
      bool rm =
          (remove((dir + prefix + string("thread_finished.csv")).c_str()) == 0);
      if (!rm) cout << "Error deleting files\n";
      cout << prefix + "thread_finished.csv removed\n";
    }
  }
  cout << "Done.\n";
}*/

/*void readApproxQpFiles(vector<VectorXd> * w_sol_vec, vector<MatrixXd> * A_vec,
                       vector<MatrixXd> * H_vec,
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
      H_vec->push_back(readCSV(dir + prefix + string("H.csv")));
      y_vec->push_back(readCSV(dir + prefix + string("y.csv")));
      lb_vec->push_back(readCSV(dir + prefix + string("lb.csv")));
      ub_vec->push_back(readCSV(dir + prefix + string("ub.csv")));
      b_vec->push_back(readCSV(dir + prefix + string("b.csv")));
      c_vec->push_back(readCSV(dir + prefix + string("c.csv")));
      B_vec->push_back(readCSV(dir + prefix + string("B.csv")));

      bool rm = true;
      rm = (remove((dir + prefix + string("A.csv")).c_str()) == 0) & rm;
      rm = (remove((dir + prefix + string("H.csv")).c_str()) == 0) & rm;
      rm = (remove((dir + prefix + string("y.csv")).c_str()) == 0) & rm;
      rm = (remove((dir + prefix + string("lb.csv")).c_str()) == 0) & rm;
      rm = (remove((dir + prefix + string("ub.csv")).c_str()) == 0) & rm;
      rm = (remove((dir + prefix + string("b.csv")).c_str()) == 0) & rm;
      rm = (remove((dir + prefix + string("B.csv")).c_str()) == 0) & rm;
      if ( !rm )
        cout << "Error deleting files\n";
    }
  }
}*/

/*void readNonredundentMatrixFile(vector<int> * nw_vec,
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
}*/

/*void readPiQiFile(vector<MatrixXd> * P_vec, vector<VectorXd> * q_vec,
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
}*/

class SampleSuccessMonitor {
 public:
  explicit SampleSuccessMonitor(int n_sample) {
    n_sample_ = n_sample;
    is_success_vec_ = std::vector<double>(n_sample, -1);
  }

  double Read(int sample_idx) const { return is_success_vec_.at(sample_idx); };
  void Write(int sample_idx, double data) {
    is_success_vec_[sample_idx] = data;
  };
  void Print() const {
    cout << "is_success_vec_ = ";
    for (auto& mem : is_success_vec_) {
      cout << mem << ", ";
    }
    cout << endl;
  }

  bool IsAllSuccess() const {
    int n_successful_sample = 0;
    for (const auto& is_success : is_success_vec_) {
      if (is_success == 1) n_successful_sample++;
    }
    return (n_successful_sample == n_sample_);
  }
  bool IsNoFail() const {
    bool no_fail = true;
    for (const auto& is_success : is_success_vec_) {
      if (is_success == 0) {
        no_fail = false;
        break;
      }
    }
    return no_fail;
  }
  bool IsSuccessRateHighEnough(double fail_rate_threshold,
                               bool is_get_nominal) {
    bool success_rate_is_high_enough = true;
    double fail_rate = double(GetNumberOfFailedSamples()) / double(n_sample_);
    if (fail_rate > fail_rate_threshold) {
      success_rate_is_high_enough = false;
    } else if ((fail_rate > 0) && is_get_nominal) {
      success_rate_is_high_enough = false;
    }
    return success_rate_is_high_enough;
  }

  int GetNumberOfFailedSamples() const {
    int n_failed_sample = 0;
    for (const auto& is_success : is_success_vec_) {
      if (is_success == 0) n_failed_sample++;
    }
    return n_failed_sample;
  }

 private:
  int n_sample_;
  std::vector<double> is_success_vec_;  // -1 means unset
};

int findGoldilocksModels(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  DRAKE_DEMAND((FLAGS_robot_option == 0) || FLAGS_robot_option == 1);
  if (FLAGS_robot_option == 0) {
    DRAKE_DEMAND(FLAGS_rom_option != 4);
    DRAKE_DEMAND(FLAGS_rom_option != 5);
    DRAKE_DEMAND(FLAGS_rom_option != 6);
  }

  cout << "\nTrial name: " << FLAGS_program_name << endl;
  std::time_t current_time =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  cout << "Current time: " << std::ctime(&current_time);
  //  if (!FLAGS_is_debug) {
  cout << "Git commit hash: " << endl;
  int sys_ret = std::system("git rev-parse HEAD");
  DRAKE_DEMAND(sys_ret != -1);
  cout << "\n\n==============================\n";
  cout << "Result of \"git status\":" << endl;
  sys_ret = std::system("git status");
  DRAKE_DEMAND(sys_ret != -1);
  cout << "\n\n==============================\n";
  cout << "Result of \"git diff\":" << endl;
  sys_ret = std::system("git --no-pager diff");
  //  sys_ret = std::system("git diff-index HEAD");
  DRAKE_DEMAND(sys_ret != -1);
  cout << "\n==============================\n\n";
  //  }

  // Create MBP
  drake::logging::set_log_level("err");  // ignore warnings about joint limits
  MultibodyPlant<double> plant(0.0);
  CreateMBP(&plant, FLAGS_robot_option);

  // Create autoDiff version of the plant
  MultibodyPlant<AutoDiffXd> plant_autoDiff(plant);
  cout << endl;

  // Files parameters
  string folder_name =
      FLAGS_data_folder_name.empty() ? "" : FLAGS_data_folder_name + "/";
  const string dir = "../dairlib_data/goldilocks_models/find_models/" +
                     folder_name + "robot_" + to_string(FLAGS_robot_option) +
                     "/";
  cout << "dir = " << dir << endl;
  string init_file = FLAGS_init_file;
  string prefix = "";
  if (!CreateFolderIfNotExist(dir)) return 0;

  // Parameters for tasks
  cout << "\nTasks settings:\n";
  bool is_grid_task = FLAGS_is_grid_task;
  bool is_stochastic = FLAGS_is_stochastic;
  // if (FLAGS_no_model_update)
  //   is_stochastic = false;  // I commented this out because it seems like
  //                           // plots look better (with interpolation func)
  TasksGenerator* task_gen;
  GridTasksGenerator task_gen_grid;
  UniformTasksGenerator task_gen_uniform;
  if (is_grid_task) {
    if (FLAGS_robot_option == 0) {
      task_gen_grid = GridTasksGenerator(
          3, {"stride_length", "ground_incline", "duration"},
          {FLAGS_N_sample_sl, FLAGS_N_sample_gi, FLAGS_N_sample_du},
          {0.25, 0, 0.4}, {0.015, 0.05, 0.05},
          std::vector<bool>(3, is_stochastic));
    } else if (FLAGS_robot_option == 1) {
      task_gen_grid = GridTasksGenerator(
          6,
          {"stride_length", "ground_incline", "duration", "turning_rate",
           "pelvis_height", "swing_margin"},
          {FLAGS_N_sample_sl, FLAGS_N_sample_gi, FLAGS_N_sample_du,
           FLAGS_N_sample_tr, FLAGS_N_sample_ph, FLAGS_N_sample_sm},
          {FLAGS_stride_length_center, FLAGS_ground_incline_center, 0.35,
           FLAGS_turning_rate_center, FLAGS_pelvis_height_center,
           FLAGS_swing_margin_center},
          {FLAGS_stride_length_delta, FLAGS_ground_incline_delta, 0.05,
           FLAGS_turning_rate_delta, FLAGS_pelvis_height_delta,
           FLAGS_swing_margin_delta},
          {(FLAGS_N_sample_sl > 1) && is_stochastic,
           (FLAGS_N_sample_gi > 1) && is_stochastic,
           (FLAGS_N_sample_du > 1) && is_stochastic,
           (FLAGS_N_sample_tr > 1) && is_stochastic,
           (FLAGS_N_sample_ph > 1) && is_stochastic,
           (FLAGS_N_sample_sm > 1) && is_stochastic});
    } else {
      throw std::runtime_error("Should not reach here");
      task_gen_grid = GridTasksGenerator();
    }
    task_gen = &task_gen_grid;
  } else {
    if (FLAGS_robot_option == 0) {
      task_gen_uniform = UniformTasksGenerator(
          3, {"stride_length", "ground_incline", "duration"},
          {FLAGS_N_sample_sl, FLAGS_N_sample_gi, FLAGS_N_sample_du},
          {FLAGS_sl_min, FLAGS_gi_min, FLAGS_du_min},
          {FLAGS_sl_max, FLAGS_gi_max, FLAGS_du_max});
    } else if (FLAGS_robot_option == 1) {
      task_gen_uniform = UniformTasksGenerator(
          6,
          {"stride_length", "ground_incline", "duration", "turning_rate",
           "pelvis_height", "swing_margin"},
          {FLAGS_N_sample_sl, FLAGS_N_sample_gi, FLAGS_N_sample_du,
           FLAGS_N_sample_tr, FLAGS_N_sample_ph, FLAGS_N_sample_sm},
          {FLAGS_sl_min, FLAGS_gi_min, FLAGS_du_min, FLAGS_tr_min, FLAGS_ph_min,
           FLAGS_sm_min},
          {FLAGS_sl_max, FLAGS_gi_max, FLAGS_du_max, FLAGS_tr_max, FLAGS_ph_max,
           FLAGS_sm_max});
    } else {
      throw std::runtime_error("Should not reach here");
      task_gen_uniform = UniformTasksGenerator();
    }
    task_gen = &task_gen_uniform;
  }

  // Tasks setup
  task_gen->PrintInfo();
  //  DRAKE_DEMAND(task_gen->task_min("stride length") >= 0);
  DRAKE_DEMAND(task_gen->task_min("duration") >= 0);
  int N_sample = task_gen->total_sample_number();
  Task task(task_gen->names());
  vector<VectorXd> previous_task(N_sample, VectorXd::Zero(task_gen->dim()));
  if (FLAGS_start_current_iter_as_rerun ||
      FLAGS_start_iterations_with_shrinking_stepsize) {
    for (int i = 0; i < N_sample; i++) {
      VectorXd pre_task = readCSV(dir + to_string(FLAGS_iter_start) + "_" +
                                  to_string(i) + string("_task.csv"))
                              .col(0);
      DRAKE_DEMAND(pre_task.rows() == task_gen->dim());
      previous_task[i] = pre_task;
    }
  }
  VectorXd n_samples(task_gen->sample_numbers().size());
  for (int i = 0; i < n_samples.size(); i++) {
    n_samples(i) = task_gen->sample_numbers().at(i);
  }
  writeCSV(dir + string("n_samples.csv"), n_samples);
  writeCSV(dir + string("n_sample.csv"), N_sample * VectorXd::Ones(1));
  SaveStringVecToCsv(task_gen->names(), dir + string("task_names.csv"));

  // Parameters for the outer loop optimization
  cout << "\nOptimization setting (outer loop):\n";
  int iter_start = FLAGS_iter_start;
  int max_outer_iter = FLAGS_max_outer_iter;
  int delta_iter = FLAGS_delta_iter;
  double stopping_threshold = 1e-4;

  double beta_momentum = FLAGS_beta_momentum;
  double h_step;
  if (FLAGS_h_step > 0) {
    h_step = FLAGS_h_step;
  } else {
    h_step = 1e-3;
    if (FLAGS_robot_option == 0) {
      // After adding tau
      // 1e-4 doesn't diverge // This is with  h_step / sqrt(norm_grad_cost(0));
      // 1e-3 diverges
      // Before adding tau
      // 1e-3 is small enough to avoid gittering at the end
      // 1e-2 is a good compromise on both speed and gittering
      // 1e-1 caused divergence when close to optimal sol
      h_step = 1e-3;
      /*if (beta_momentum != 0) {
        // haven't tried or tuned this yet.
        h_step = 1e-5;
      }*/
    } else if (FLAGS_robot_option == 1) {
      // Without tau: (This is with  h_step / sqrt(norm_grad_cost(0));)
      //  1e-4: doesn't always decrease with a fixed task
      //  1e-5: barely increase with a fixed task

      // Both with and without tau (I believe), fixed task.
      // h_step = 1e-3;  // This is with h_step / norm_grad_cost_double.
      // (and with old traj opt)

      // (20200216) After using new traj opt
      h_step = 1e-4;  // maybe h_step shouldn't be too high, because rom
                      // constraint is the constraint that is hard to satisfy?
      if (!is_stochastic) {
        h_step = 1e-3;  // we can always shrink steps if the cost goes up with
                        // fixed tasks (it should go down theoretically)
      }
      if (beta_momentum != 0) {
        // haven't tried or tuned this yet.
        h_step = 1e-3;
      }

      // Set the step size for particular models
      if (FLAGS_rom_option >= 10 && FLAGS_rom_option <= 14) {
        h_step = 5e-3;
      }
    }
  }
  double indpt_row_tol = 1e-6;  // 1e-6
  bool is_newton = FLAGS_is_newton;
  int N_rerun;
  if (FLAGS_N_rerun > -1) {
    N_rerun = FLAGS_N_rerun;
  } else {
    if (FLAGS_robot_option == 0) {
      N_rerun = 1;
    } else if (FLAGS_robot_option == 1) {
      N_rerun = 2;  // 2;
    } else {
      N_rerun = 0;
    }
  }
  const int method_to_solve_system_of_equations =
      FLAGS_method_to_solve_system_of_equations;
  // With bigger momentum, you might need a larger tolerance.
  // If not uniform grid, considering sample cost increase makes no sense.
  // So, use a very large tolerance on increase rate.
  double max_sample_cost_increase_rate = 0;
  if (FLAGS_robot_option == 0) {
    if (is_grid_task) {
      max_sample_cost_increase_rate = is_stochastic ? 2.0 : 0.01;
    } else {
      max_sample_cost_increase_rate = std::numeric_limits<double>::infinity();
    }
  } else if (FLAGS_robot_option == 1) {
    if (is_grid_task) {
      max_sample_cost_increase_rate = is_stochastic ? 2.0 : 0.01;
      // 0.5 // 0.3
    } else {
      max_sample_cost_increase_rate = std::numeric_limits<double>::infinity();
    }
  } else {
    throw std::runtime_error("Should not reach here");
  }
  // Increase the tolerance for restricted number
  double max_average_cost_increase_rate = 0;
  if (FLAGS_robot_option == 0) {
    if (is_grid_task) {
      max_average_cost_increase_rate = is_stochastic ? 0.5 : 0.01;
    } else {
      max_average_cost_increase_rate = 2;
    }
  } else if (FLAGS_robot_option == 1) {
    if (is_grid_task) {
      max_average_cost_increase_rate = is_stochastic ? 0.2 : 0.01;  // 0.15
    } else {
      max_average_cost_increase_rate = 1;
    }
  } else {
    throw std::runtime_error("Should not reach here");
  }
  // Since sometimes the above increase rates are too restrictive and cause the
  // optimization to get stuck in some iteration, we relax the increase rate
  // every `n_shrink_before_relaxing_tolerance` times of step size shrinking
  int n_shrink_before_relaxing_tolerance = 6;
  is_newton ? cout << "Newton method\n" : cout << "Gradient descent method\n";
  is_stochastic ? cout << "Stochastic\n" : cout << "Non-stochastic\n";
  cout << "Step size = " << h_step << endl;
  cout << "beta_momentum = " << beta_momentum << endl;
  cout << "indpt_row_tol = " << indpt_row_tol << endl;
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
  cout << "only_update_wrt_main_cost = " << FLAGS_only_update_wrt_main_cost
       << endl;
  // Outer loop setting - help from adjacent samples
  bool get_good_sol_from_adjacent_sample =
      FLAGS_get_good_sol_from_adjacent_sample;
  if (!is_grid_task) {
    get_good_sol_from_adjacent_sample = false;
    cout << "WARNING: setting `get_good_sol_from_adjacent_sample` to false "
            "for non-grid task\n";
  }

  double max_cost_increase_rate_before_ask_for_help = 0.1;
  if (FLAGS_robot_option == 0) {
    max_cost_increase_rate_before_ask_for_help = 0.5;
  } else if (FLAGS_robot_option == 1) {
    max_cost_increase_rate_before_ask_for_help = 0.15;  // 0.1
  }
  double max_adj_cost_diff_rate_before_ask_for_help = 0.1;
  if (FLAGS_robot_option == 0) {
    max_adj_cost_diff_rate_before_ask_for_help = 0.5;
  } else if (FLAGS_robot_option == 1) {
    max_adj_cost_diff_rate_before_ask_for_help = 0.5;  // 0.1
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

  // Parameters for the main costs (i.e. non-regularization cost)
  double Q = 0;              // Cost on velocity
  double R = 0;              // Cost on input effort
  double w_joint_accel = 0;  // Cost on joint acceleration
  double all_cost_scale = 1;
  setCostWeight(&Q, &R, &w_joint_accel, &all_cost_scale, FLAGS_robot_option);
  if (FLAGS_Q > 0) Q = FLAGS_Q;
  if (FLAGS_R > 0) R = FLAGS_R;
  if (FLAGS_w_joint_accel > 0) w_joint_accel = FLAGS_w_joint_accel;
  cout << "Q = " << Q << endl;
  cout << "R = " << R << endl;
  cout << "w_joint_accel = " << w_joint_accel << endl;

  // Parameters for the inner loop optimization
  int max_inner_iter = FLAGS_max_inner_iter;
  if (FLAGS_robot_option == 0) {
    max_inner_iter = 300;
  }
  cout << "\nOptimization setting (inner loop):\n";
  cout << "max_inner_iter = " << max_inner_iter << endl;
  cout << "major_optimality_tol = " << FLAGS_major_optimality_tol << endl;
  cout << "major_feasibility_tol = " << FLAGS_major_feasibility_tol << endl;
  cout << "use SNOPT built-in scaling? " << FLAGS_snopt_scaling << endl;
  cout << "use Ipopt instead of Snopt?" << FLAGS_ipopt << endl;
  cout << "Fix number of nodes in traj opt? " << FLAGS_fix_node_number << endl;
  if (!FLAGS_fix_node_number)
    cout << "node_density = " << FLAGS_node_density << endl;
  // Inner loop setup
  if (!is_grid_task) {
    DRAKE_DEMAND(FLAGS_fix_node_number);
  }
  cout << "n_node for each sample = \n";
  vector<int> n_node_vec(N_sample, 20);
  if (!FLAGS_fix_node_number) {
    for (int sample_idx = 0; sample_idx < N_sample; sample_idx++) {
      task.set(task_gen_grid.NewNominalTask(sample_idx));
      double duration = task.get("duration");
      n_node_vec[sample_idx] = int(FLAGS_node_density * duration);
      cout << n_node_vec[sample_idx] << ", ";
    }
    cout << endl;
    cout << "WARNING: we will *only* add adjacent samples to the list if it has"
            " the same number of nodes\n";
  } else {
    cout << n_node_vec[0] << "\n";
  }
  cout << "eps_regularization = " << FLAGS_eps_regularization << endl;
  cout << "is_add_tau_in_cost = " << FLAGS_is_add_tau_in_cost << endl;
  FLAGS_is_zero_touchdown_impact ? cout << "Zero touchdown impact\n"
                                 : cout << "Non-zero touchdown impact\n";
  InnerLoopSetting inner_loop_setting = InnerLoopSetting();
  inner_loop_setting.Q_double = Q;
  inner_loop_setting.R_double = R;
  inner_loop_setting.w_joint_accel = w_joint_accel;
  inner_loop_setting.eps_reg = FLAGS_eps_regularization;
  inner_loop_setting.all_cost_scale = all_cost_scale;
  inner_loop_setting.is_add_tau_in_cost = FLAGS_is_add_tau_in_cost;
  inner_loop_setting.is_zero_touchdown_impact = FLAGS_is_zero_touchdown_impact;
  inner_loop_setting.mu = 0.8;  // Note that this affect force. Not just impulse
  inner_loop_setting.max_iter = max_inner_iter;
  inner_loop_setting.major_optimality_tol = FLAGS_major_optimality_tol;
  inner_loop_setting.major_feasibility_tol = FLAGS_major_feasibility_tol;
  inner_loop_setting.snopt_log = false;
  inner_loop_setting.snopt_scaling = FLAGS_snopt_scaling;
  inner_loop_setting.use_ipopt = FLAGS_ipopt;
  inner_loop_setting.directory = dir;
  inner_loop_setting.com_accel_constraint = FLAGS_com_accel_constraint;
  inner_loop_setting.cubic_spline_in_rom_constraint =
      FLAGS_cubic_spline_in_rom_constraint;  // for testing
  inner_loop_setting.swing_foot_cublic_spline_constraint =
      FLAGS_swing_foot_cublic_spline;  // for testing
  inner_loop_setting.zero_ending_pelvis_angular_vel =
      FLAGS_zero_ending_pelvis_angular_vel;  // for testing
  cout << "mu = " << inner_loop_setting.mu << endl;
  cout << "directory = " << dir << endl;
  cout << "com_accel_constraint = " << inner_loop_setting.com_accel_constraint
       << endl;
  cout << "cubic_spline_in_rom_constraint = "
       << inner_loop_setting.cubic_spline_in_rom_constraint << endl;
  cout << "swing_foot_cublic_spline_constraint (zero impact) = "
       << inner_loop_setting.swing_foot_cublic_spline_constraint << endl;
  cout << "zero_ending_pelvis_angular_vel = "
       << inner_loop_setting.zero_ending_pelvis_angular_vel << endl;
  if (inner_loop_setting.snopt_log) {
    cout << "WARNING: you are printing snopt log for Cassie (could slow down "
            "the optimization)!\n";
  }

  // Construct reduced order model
  cout << "\nReduced-order model setting:\n";
  cout << "rom_option = " << FLAGS_rom_option << endl;
  std::unique_ptr<ReducedOrderModel> rom =
      CreateRom(FLAGS_rom_option, FLAGS_robot_option, plant);
  writeCSV(dir + string("rom_B.csv"), rom->B());
  writeCSV(dir + string("rom_n_y.csv"), rom->n_y() * VectorXd::Ones(1));
  writeCSV(dir + string("rom_n_tau.csv"), rom->n_tau() * VectorXd::Ones(1));
  writeCSV(dir + string("rom_n_feature_y.csv"),
           rom->n_feature_y() * VectorXd::Ones(1));
  writeCSV(dir + string("rom_n_feature_yddot.csv"),
           rom->n_feature_yddot() * VectorXd::Ones(1));

  // Reduced order model setup
  if (iter_start != 0) {
    VectorXd theta_y =
        readCSV(dir + to_string(iter_start) + string("_theta_y.csv")).col(0);
    VectorXd theta_yddot =
        readCSV(dir + to_string(iter_start) + string("_theta_yddot.csv"))
            .col(0);
    if (rom->n_theta_y() != theta_y.size()) {
      cout << "rom->n_theta_y() = " << rom->n_theta_y() << endl;
      cout << "theta_y.size() = " << theta_y.size() << endl;
      cout << dir + to_string(iter_start) + string("_theta_y.csv") << endl;
    }
    if (rom->n_theta_yddot() != theta_yddot.size()) {
      cout << "rom->n_theta_yddot() = " << rom->n_theta_yddot() << endl;
      cout << "theta_yddot.size() = " << theta_yddot.size() << endl;
      cout << dir + to_string(iter_start) + string("_theta_yddot.csv") << endl;
    }
    rom->SetThetaY(theta_y);
    rom->SetThetaYddot(theta_yddot);
  }

  // Vectors/Matrices for the outer loop
  SubQpData QPs(N_sample);

  // Multithreading setup
  cout << "\nMultithreading settings:\n";
  int CORES = static_cast<int>(std::thread::hardware_concurrency());
  cout << "# of threads availible on this computer: " << CORES << endl;
  if (FLAGS_n_thread_to_use > 0) CORES = FLAGS_n_thread_to_use;
  cout << "is multithread? " << FLAGS_is_multithread << endl;
  cout << "# of threads to be used " << CORES << endl;
  vector<std::shared_ptr<int>> thread_finished_vec(N_sample);
  for (int i = 0; i < N_sample; i++) {
    thread_finished_vec[i] = std::make_shared<int>(0);
  }

  // Some setup
  cout << "\nOther settings:\n";
  cout << "is_debug? " << FLAGS_is_debug << endl;
  cout << "no_model_update? " << FLAGS_no_model_update << endl;
  double ave_min_cost_so_far = std::numeric_limits<double>::infinity();
  std::vector<double> each_min_cost_so_far(
      N_sample, std::numeric_limits<double>::infinity());
  if (iter_start > 1 && !FLAGS_is_debug) {
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
        double c = readCSV(dir + to_string(iter) + "_" + to_string(i) +
                           string("_c.csv"))(0, 0);
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

  // Some setup
  bool rerun_current_iteration = FLAGS_start_current_iter_as_rerun;
  bool has_been_all_success = iter_start > 1;
  if (FLAGS_no_model_update) {  // do nothing
  } else if ((FLAGS_start_current_iter_as_rerun && (iter_start >= 1)) ||
             (!FLAGS_start_current_iter_as_rerun && (iter_start >= 2))) {
    int iter_check_all_success =
        FLAGS_start_current_iter_as_rerun ? iter_start : iter_start - 1;

    bool samples_are_success = true;
    for (int i = 0; i < N_sample; i++) {
      samples_are_success =
          samples_are_success &&
          (readCSV(dir + to_string(iter_check_all_success) + "_" +
                   to_string(i) + string("_is_success.csv"))(0, 0) == 1);
    }
    has_been_all_success = samples_are_success;

    cout << "has_been_all_success? " << has_been_all_success << " (breakdown: ";
    for (int i = 0; i < N_sample; i++) {
      cout << readCSV(dir + to_string(iter_check_all_success) + "_" +
                      to_string(i) + string("_is_success.csv"))(0, 0)
           << ", ";
    }
    cout << ")\n";
  } else {
    cout << "has_been_all_success? " << has_been_all_success << endl;
  }
  cout << "iteration #" << iter_start << " is a rerun? "
       << rerun_current_iteration << endl;

  VectorXd step_direction;
  VectorXd prev_step_direction = VectorXd::Zero(
      rom->n_theta());  // must initialize this because of momentum term
  if (iter_start > 1 && !FLAGS_is_debug && !FLAGS_no_model_update) {
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
  if ((iter_start > 1) && FLAGS_read_previous_step_size && !FLAGS_is_debug &&
      !FLAGS_no_model_update) {
    cout << "Reading previous step size... (will get memory issue if the file "
            "doesn't exist)\n";
    current_iter_step_size = readCSV(dir + to_string(iter_start - 1) +
                                     string("_step_size.csv"))(0, 0);
  }

  VectorXd prev_theta = rom->theta();
  if (iter_start > 1 && !FLAGS_is_debug) {
    MatrixXd prev_theta_y_mat =
        readCSV(dir + to_string(iter_start - 1) + string("_theta_y.csv"));
    MatrixXd prev_theta_yddot_mat =
        readCSV(dir + to_string(iter_start - 1) + string("_theta_yddot.csv"));
    prev_theta << prev_theta_y_mat.col(0), prev_theta_yddot_mat.col(0);
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
  int extend_model_iter =
      (FLAGS_extend_model_iter == -1) ? iter_start : FLAGS_extend_model_iter;
  extend_model_iter = (extend_model_iter == 0) ? 1 : extend_model_iter;
  bool has_visit_this_iter_for_model_extension = false;
  if (extend_model) {
    throw std::runtime_error(
        "Model extension implementation hasn't not been updated");
    /*cout << "\nWill extend the model at iteration # " << extend_model_iter
         << " by ";
    VectorXd theta_s_append =
        readCSV(dir + string("theta_s_append.csv")).col(0);
    DRAKE_DEMAND(theta_s_append.rows() % n_feature_y == 0);
    int n_extend = theta_s_append.rows() / n_feature_y;
    cout << n_extend << " dimension.\n";

    cout << "Make sure that you include both old and new version of dynamics"
            "feature.\n";
    if (!FLAGS_turn_off_cin) {
      cout << "Proceed? (Y/N)\n";
      char answer[1];
      cin >> answer;
      if (!((answer[0] == 'Y') || (answer[0] == 'y'))) {
        cout << "Ending the program.\n";
        return 0;
      } else {
        cout << "Continue constructing the problem...\n";
      }
    }*/
  }

  // Some checks
  if (FLAGS_no_model_update) {
    // We don't want this folder to be empty, because no_model_update is
    // supposed to be used after the model optimizaiton is finished. I just
    // check if 1_theta_y.csv exists
    cout << "Check file: " << dir + "1_theta_y.csv" << endl;
    DRAKE_DEMAND(file_exist(dir + "1_theta_y.csv"));
  }

  // Setup for getting good solution from adjacent samples
  MatrixXi adjacent_sample_indices;
  if (is_grid_task) {
    adjacent_sample_indices = GetAdjSampleIndices(task_gen_grid, n_node_vec);
    cout << "adjacent_sample_indices = \n" << adjacent_sample_indices << endl;
  }

  cout << "\nStart iterating...\n";
  // Start the gradient descent
  int iter;
  int n_shrink_step = 0;
  auto iter_start_time = std::chrono::system_clock::now();
  for (iter = iter_start; iter <= max_outer_iter; iter += delta_iter) {
    bool is_get_nominal = iter == 0;

    // Print info about iteration # and current time
    if (!start_iterations_with_shrinking_stepsize) {
      auto clock_now = std::chrono::system_clock::now();
      if (!is_get_nominal) {
        std::chrono::duration<double> iteration_elapse =
            clock_now - iter_start_time;
        iter_start_time = clock_now;
        cout << "\nLast iteration takes " << iteration_elapse.count() << "s.\n";
      }
      std::time_t current_time =
          std::chrono::system_clock::to_time_t(clock_now);
      cout << "Current time: " << std::ctime(&current_time);
      cout << "************ Iteration " << iter << " (" << n_shrink_step
           << "-time step size shrinking) *************" << endl;
      if (iter != 0) {
        cout << "theta_yddot.head(6) = "
             << rom->theta_yddot().head(6).transpose() << endl;
      }
    }

    // Store parameter values
    prefix = to_string(iter) + "_";
    if (!FLAGS_no_model_update) {
      if (!is_get_nominal || !FLAGS_is_debug) {
        writeCSV(dir + prefix + string("theta_y.csv"), rom->theta_y());
        writeCSV(dir + prefix + string("theta_yddot.csv"), rom->theta_yddot());
      }
    }

    // setup for each iteration
    int max_inner_iter_pass_in = is_get_nominal ? 200 : max_inner_iter;
    bool extend_model_this_iter = extend_model && (iter == extend_model_iter) &&
                                  !has_visit_this_iter_for_model_extension;
    if (iter == extend_model_iter)
      has_visit_this_iter_for_model_extension = true;
    if (extend_model_this_iter) {
      cout << "WILL EXTEND MODEL IN THIS ITERATION\n";
    }

    // Run trajectory optimization for different tasks first
    SampleSuccessMonitor sample_monitor(N_sample);
    if (start_iterations_with_shrinking_stepsize) {
      // skip the sample evaluation
    } else {
      // Print message
      cout << "sample# (rerun #)";
      for (auto& mem : task_gen->names()) {
        cout << " | " << mem;
      }
      cout << " | init_file | Status | Solve time | Cost (tau cost)\n";

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
      for (int i = 0; i < N_sample; i++) awaiting_sample_idx.push_back(i);

      // Set up for feeding good sample solution to adjacent bad samples
      std::vector<double> sample_status(N_sample, -1);  // -1 means unset,
                                                        // 0 is bad, 1 is good
      // In the following int matrices, each row is a list that contains the
      // sample idx that can help (or helped)
      // -1 means empty.
      // TODO: you can re-implement this with
      //  std::vector<std::shared_ptr<std::vector<int>>>
      //  so the code is cleaner.
      MatrixXi sample_idx_waiting_to_help =
          -1 * MatrixXi::Ones(N_sample, 2 * task_gen->dim());
      MatrixXi sample_idx_that_helped =
          -1 * MatrixXi::Ones(N_sample, 2 * task_gen->dim());
      std::vector<double> local_each_min_cost_so_far = each_min_cost_so_far;

      // Set up for deciding if we should update the solution
      std::vector<double> cost_threshold_for_update(
          N_sample, std::numeric_limits<double>::infinity());

      // Evaluate samples
      while (!awaiting_sample_idx.empty() || !assigned_thread_idx.empty()) {
        cout << "awaiting_sample_idx = ";
        for (auto mem : awaiting_sample_idx) {
          cout << mem << ", ";
        }
        cout << endl;
        cout << "assigned_sample_idx = ";
        for (auto mem : assigned_thread_idx) {
          cout << mem.second << ", ";
        }
        cout << endl;

        // clang-format off
        //std::system("lscpu | grep CPU\\ MHz"); // print the current cpu clock speed
        //std::system("top -bn2 | grep \"Cpu(s)\" | sed \"s/.*, *\\([0-9.]*\\)%* id.*/\1/\" | awk '{print 100 - $1\"%\"}'"); // print the CPU usage
        //std::system("free -m"); // print memory usage
        // clang-format on

        // Evaluate a sample when there is an available thread. Otherwise, wait.
        if (!awaiting_sample_idx.empty() && !available_thread_idx.empty()) {
          // Pick a sample to evaluate
          int sample_idx = awaiting_sample_idx.front();
          awaiting_sample_idx.pop_front();

          // Record # of reruns
          n_rerun[sample_idx] += 1;
          bool current_sample_is_a_rerun =
              rerun_current_iteration || (n_rerun[sample_idx] > 0);

          // Prefix for the file name
          prefix = to_string(iter) + "_" + to_string(sample_idx) + "_";

          // Generate a new task or use the same task if this is a rerun
          // (You need step_size_shrinked_last_loop because you might start the
          // program with shrinking step size)
          if (!FLAGS_is_debug) {
            if (current_sample_is_a_rerun || step_size_shrinked_last_loop) {
              task.set(CopyVectorXdToStdVector(previous_task[sample_idx]));
            } else {
              if (is_grid_task && is_get_nominal) {
                task.set(task_gen_grid.NewNominalTask(sample_idx));
              } else {
                task.set(task_gen->NewTask(sample_idx));
              }
              // Map std::vector to VectorXd and create a copy of VectorXd
              Eigen::VectorXd task_vectorxd = Eigen::Map<const VectorXd>(
                  task.get().data(), task.get().size());
              previous_task[sample_idx] = task_vectorxd;
              // Store task in files
              writeCSV(dir + prefix + string("task.csv"), task_vectorxd);
            }
          } else {
            task.set(task_gen_grid.NewNominalTask(sample_idx));
            /*task.set(CopyVectorXdToStdVector(
                readCSV(dir + prefix + string("task.csv")).col(0)));*/

            task.Print();
            writeCSV(dir + prefix + string("task.csv"),
                     Eigen::Map<const VectorXd>(task.get().data(),
                                                task.get().size()));
          }

          // (Feature -- get initial guess from adjacent successful samples)
          // If the current sample already finished N_rerun (and was queued back
          // to here), then it means that there exists a adjacent sample that
          // can help the current sample.
          int sample_idx_to_help = -1;
          if (get_good_sol_from_adjacent_sample && !FLAGS_is_debug) {
            if ((n_rerun[sample_idx] > N_rerun) &&
                (sample_monitor.Read(sample_idx) != 0.5)) {
              sample_monitor.Print();
              GetAdjacentHelper(sample_idx, sample_idx_waiting_to_help,
                                sample_idx_that_helped, sample_idx_to_help,
                                task_gen->dim_nondeg());
            }
          }

          // Get file name of initial seed
          string init_file_pass_in;
          if (!FLAGS_is_debug) {
            getInitFileName(&init_file_pass_in, init_file, iter, sample_idx,
                            is_get_nominal, current_sample_is_a_rerun,
                            has_been_all_success, step_size_shrinked_last_loop,
                            n_rerun[sample_idx], sample_idx_to_help, dir,
                            task_gen, task, *rom, !is_grid_task,
                            FLAGS_use_database, FLAGS_robot_option,
                            FLAGS_no_model_update, delta_iter);
          } else {
            init_file_pass_in = init_file.empty() ? to_string(iter) + "_" +
                                                        to_string(sample_idx) +
                                                        string("_w.csv")
                                                  : init_file;
            if (n_rerun[sample_idx] > 0) {
              init_file_pass_in = to_string(iter) + "_" +
                                  to_string(sample_idx) + string("_w.csv");
            }
            cout << "init_file_pass_in = " << init_file_pass_in << endl;
          }

          // Set up feasibility and optimality tolerance
          // TODO: tighten tolerance at the last rerun for getting better
          //  solution?

          // Some inner loop setting
          inner_loop_setting.n_node = n_node_vec[sample_idx];
          inner_loop_setting.max_iter = max_inner_iter_pass_in;
          inner_loop_setting.prefix = prefix;
          inner_loop_setting.init_file = init_file_pass_in;

          // Trajectory optimization with fixed model parameters
          // string string_to_be_print = "Adding sample #" +
          // to_string(sample_idx) +
          //  " to thread #" + to_string(available_thread_idx.front()) +
          //  "...\n";
          // cout << string_to_be_print;
          threads[available_thread_idx.front()] = new std::thread(
              trajOptGivenWeights, std::ref(plant), std::ref(plant_autoDiff),
              std::ref(*rom), inner_loop_setting, task, std::ref(QPs),
              std::ref(thread_finished_vec), is_get_nominal,
              extend_model_this_iter, sample_idx, n_rerun[sample_idx],
              cost_threshold_for_update[sample_idx], N_rerun, FLAGS_rom_option,
              FLAGS_robot_option);
          // string_to_be_print = "Finished adding sample #" +
          //  to_string(sample_idx) +
          //  " to thread # " + to_string(available_thread_idx.front()) + ".\n";
          // cout << string_to_be_print;

          assigned_thread_idx.push_back(
              std::make_pair(available_thread_idx.front(), sample_idx));
          available_thread_idx.pop();
        } else {
          // Select the thread to join
          cout << "selectThreadIdxToWait\n";
          int selected_idx =
              selectThreadIdxToWait(assigned_thread_idx, thread_finished_vec);
          cout << "selected_idx = " << selected_idx << endl;

          // Wait for the selected thread to join, then delete thread
          int thread_to_wait_idx = assigned_thread_idx[selected_idx].first;
          int sample_idx = assigned_thread_idx[selected_idx].second;
          // string string_to_be_print = "Waiting for thread #" +
          //                            to_string(thread_to_wait_idx) +
          //                            " to join...\n";
          // cout << string_to_be_print;
          threads[thread_to_wait_idx]->join();
          delete threads[thread_to_wait_idx];
          // string_to_be_print = "Thread #" +
          //                     to_string(thread_to_wait_idx) +
          //                     " has joined.\n";
          // cout << string_to_be_print;
          available_thread_idx.push(thread_to_wait_idx);
          assigned_thread_idx.erase(assigned_thread_idx.begin() + selected_idx);
          // BTW, erasing middle members is computationally inefficient:
          //   http://www.cplusplus.com/reference/vector/vector/erase/

          // Record success history
          prefix = to_string(iter) + "_" + to_string(sample_idx) + "_";
          sample_monitor.Write(
              sample_idx,
              (readCSV(dir + prefix + string("is_success.csv")))(0, 0));
          bool sample_success =
              ((readCSV(dir + prefix + string("is_success.csv")))(0, 0) ==
               SAMPLE_STATUS_CODE::SUCCESS);
          // We don't allow infinite numbers of reruns due to snopt's iteration
          // limit, so we cap it to 2*N_rerun
          bool need_to_rerun_due_to_iteration_limit =
              ((readCSV(dir + prefix + string("is_success.csv")))(0, 0) ==
               SAMPLE_STATUS_CODE::ITERATION_LIMIT) &&
              (n_rerun[sample_idx] < 2 * N_rerun);

          // Queue the current sample back if
          // 1. it's not the last evaluation for this sample, OR
          // 2. it reached solver's iteration limit (but not too many times)
          bool current_sample_is_queued = false;
          if ((n_rerun[sample_idx] < N_rerun) ||
              need_to_rerun_due_to_iteration_limit) {
            awaiting_sample_idx.push_back(sample_idx);
            current_sample_is_queued = true;
          }

          // Update cost_threshold_for_update
          if (sample_success && (n_rerun[sample_idx] >= N_rerun)) {
            auto sample_cost = (readCSV(dir + prefix + string("c.csv")))(0, 0);
            if (sample_cost < cost_threshold_for_update[sample_idx]) {
              cost_threshold_for_update[sample_idx] = sample_cost;
            }
          }

          // Get good initial guess from adjacent samples's solution
          // (we dont' get help from adjacent samples when extending model,
          // because the length of decision variable is not the same)
          if (get_good_sol_from_adjacent_sample && !extend_model_this_iter &&
              (n_rerun[sample_idx] >= N_rerun) &&
              !need_to_rerun_due_to_iteration_limit) {
            RecordSolutionQualityAndQueueList(
                dir, prefix, sample_idx, assigned_thread_idx,
                adjacent_sample_indices,
                max_cost_increase_rate_before_ask_for_help,
                max_adj_cost_diff_rate_before_ask_for_help,
                is_limit_difference_of_two_adjacent_costs, sample_success,
                current_sample_is_queued, task_gen->dim_nondeg(), n_rerun,
                N_rerun, local_each_min_cost_so_far, sample_status,
                sample_idx_waiting_to_help, sample_idx_that_helped,
                awaiting_sample_idx);

            // If the current sample is queued again because it could be
            // helped by adjacent samples, then don't conclude that it's a
            // failure yet
            auto it = find(awaiting_sample_idx.begin(),
                           awaiting_sample_idx.end(), sample_idx);
            if (it != awaiting_sample_idx.end()) {
              sample_monitor.Write(sample_idx, -1);
            }
          }

          // Logic of fail or success
          bool no_sample_failed_so_far = sample_monitor.IsNoFail();
          bool success_rate_is_high_enough =
              sample_monitor.IsSuccessRateHighEnough(FLAGS_fail_threshold,
                                                     is_get_nominal);
          cout << "Update success/fail flags after sample_idx #" << sample_idx
               << "... ";
          cout << "(no_sample_failed_so_far, "
                  "success_rate_is_high_enough) = ("
               << no_sample_failed_so_far << ", " << success_rate_is_high_enough
               << ")\n";

          // If the flag `no_model_update` is on, then we never stop the sample
          // evaluation early.
          if (FLAGS_no_model_update) {
            continue;
          }

          // Stop evaluating if
          // 1. any sample failed after a all-success iteration
          // 2. fail rate higher than threshold before seeing all-success
          // iteration
          /*if ((has_been_all_success && !no_sample_failed_so_far) ||
              (!has_been_all_success && (!success_rate_is_high_enough))) {*/
          if ((has_been_all_success && !no_sample_failed_so_far &&
               (n_shrink_step < 3)) ||
              !success_rate_is_high_enough) {
            // Wait for the assigned threads to join, and then break;
            cout << "(has_been_all_success, no_sample_failed_so_far, "
                    "success_rate_is_high_enough) = ("
                 << has_been_all_success << ", " << no_sample_failed_so_far
                 << ", " << success_rate_is_high_enough << ")\n";
            cout << sample_monitor.GetNumberOfFailedSamples()
                 << " # of samples failed to find solution."
                    " Latest failed sample is sample#"
                 << sample_idx
                 << ". Wait for all threads to join and stop current "
                    "iteration.\n";
            waitForAllThreadsToJoin(&threads, &assigned_thread_idx,
                                    thread_finished_vec);
            break;
          }

          // If in debug mode, stop evaluating.
          if (FLAGS_is_debug) {
            // Wait for the assigned threads to join, and then break;
            /*cout << "In debug mode. Wait for all threads to join and stop "
                    "current iteration.\n";
            waitForAllThreadsToJoin(&threads, &assigned_thread_idx,
                                    thread_finished_vec);
            break;*/
          }
        }
      }  // while(sample < N_sample)
    }    // end if-else (start_iterations_with_shrinking_stepsize)
    if (FLAGS_is_debug) break;
    if (FLAGS_no_model_update) {
      VectorXd theta_y =
          readCSV(dir + to_string(iter + 1) + string("_theta_y.csv")).col(0);
      VectorXd theta_yddot =
          readCSV(dir + to_string(iter + 1) + string("_theta_yddot.csv"))
              .col(0);
      if (rom->n_theta_y() != theta_y.size()) {
        cout << "rom->n_theta_y() = " << rom->n_theta_y() << endl;
        cout << "theta_y.size() = " << theta_y.size() << endl;
        cout << dir + to_string(iter + 1) + string("_theta_y.csv") << endl;
      }
      if (rom->n_theta_yddot() != theta_yddot.size()) {
        cout << "rom->n_theta_yddot() = " << rom->n_theta_yddot() << endl;
        cout << "theta_yddot.size() = " << theta_yddot.size() << endl;
        cout << dir + to_string(iter + 1) + string("_theta_yddot.csv") << endl;
      }
      rom->SetThetaY(theta_y);
      rom->SetThetaYddot(theta_yddot);
      continue;
    }

    // cout << "Only run for 1 iteration. for testing.\n";
    // for (int i = 0; i < 100; i++) {cout << '\a';}  // making noise to notify
    // break;

    // Update some flags
    bool all_samples_succeeded = sample_monitor.IsAllSuccess();
    bool success_rate_is_high_enough = sample_monitor.IsSuccessRateHighEnough(
        FLAGS_fail_threshold, is_get_nominal);

    // Logic for how to iterate
    if (start_iterations_with_shrinking_stepsize) {
      rerun_current_iteration = true;
    } else {
      if (all_samples_succeeded && !is_get_nominal) {
        has_been_all_success = true;
      }
      // If all samples have been evaluated successfully in previous iteration,
      // we don't allow any failure in the following iterations
      bool current_iter_is_success = has_been_all_success
                                         ? all_samples_succeeded
                                         : success_rate_is_high_enough;

      // Rerun the current iteration when the iteration was not successful
      rerun_current_iteration = !current_iter_is_success;

      cout << "\nDecide how to iterate...\n";
      cout << "has_been_all_success = " << has_been_all_success << endl;
      cout << "all_samples_succeeded = " << all_samples_succeeded << endl;
      cout << "success_rate_is_high_enough = " << success_rate_is_high_enough
           << endl;
      cout << "current_iter_is_success = " << current_iter_is_success << endl;
      cout << "rerun_current_iteration = " << rerun_current_iteration << "\n\n";
    }

    // Some checks to prevent wrong logic
    if (start_iterations_with_shrinking_stepsize) {
      DRAKE_DEMAND(
          !extend_model_this_iter);  // shouldn't extend model while starting
                                     // the program with adjusting step size
      DRAKE_DEMAND(iter > 1);        // shouldn't be iter 0 or 1
    }

    // Update parameters, adjusting step size or extend model
    step_size_shrinked_last_loop = false;
    if (is_get_nominal) {
      if (rerun_current_iteration) {
        iter -= 1;
      }
    } else if (extend_model_this_iter) {  // Extend the model
      cout << "Start extending model...\n";
      extendModel(dir, iter, *rom, prev_theta, step_direction,
                  prev_step_direction, ave_min_cost_so_far, FLAGS_rom_option,
                  FLAGS_robot_option);

      // So that we can re-run the current iter
      cout << "Reset \"has_been_all_success\" to false, in case the next iter "
              "is infeasible.\n";
      iter -= 1;
      has_been_all_success = false;
      rerun_current_iteration = true;

      // Never extend model again (we just extend it once)
      extend_model = false;
      continue;
    }                                    // end if extend_model_this_iter
    else if (rerun_current_iteration) {  // rerun the current iteration
      // We only shrink step if it's iteration 2 or higher
      if (iter != 1) {
        current_iter_step_size = current_iter_step_size / 2;
        // if(current_iter_step_size<1e-5){
        //   cout<<"switch to the other method.";
        //   is_newton = !is_newton;
        // }
        cout << "Step size shrinks to " << current_iter_step_size
             << ". Redo this iteration.\n\n";

        // Descent
        rom->SetTheta(prev_theta + current_iter_step_size * step_direction);

        n_shrink_step++;
      }

      // Some logic for iterating
      iter -= 1;
      start_iterations_with_shrinking_stepsize = false;
      step_size_shrinked_last_loop = true;
      // The name `step_size_shrinked_last_loop` is a bit misleading if it's
      // iter 1 but I think we still need it for getInitFileName()
    }  // end if rerun_current_iteration
    else {
      // The code only reach here when the current iteration is successful.

      /*// Read in the following files of the successful samples:
      // w_sol_vec, A_vec, H_vec, y_vec, lb_vec, ub_vec, b_vec, c_vec, B_vec;
      auto start_time_read_file = std::chrono::high_resolution_clock::now();

      readApproxQpFiles(&w_sol_vec, &A_vec, &H_vec, &y_vec, &lb_vec, &ub_vec,
                        &b_vec, &c_vec, &B_vec,
                        N_sample, iter, dir);

      // Print out elapsed time
      auto finish_time_read_file = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed_read_file =
          finish_time_read_file - start_time_read_file;
      cout << "\nTime spent on reading files of sample evaluation: "
           << to_string(int(elapsed_read_file.count())) << " seconds\n";
      cout << endl;*/

      // Construct an index list for the successful sample
      std::vector<int> successful_idx_list;
      for (uint i = 0; i < N_sample; i++) {
        if (sample_monitor.Read(i) == 1) {
          successful_idx_list.push_back(i);
        }
      }

      // number of successful sample
      int n_succ_sample = successful_idx_list.size();

      // Ensure that the QP approximations are all assigned (added checks here
      // so that we can debug in the future faster)
      for (auto idx : successful_idx_list) {
        // We only checked B matrix.
        // The number of cols (n_theta) should never be 0 if B was assigned.
        DRAKE_DEMAND(QPs.B_vec[idx]->cols() > 0);
      }

      // TODO: we only consider successful samples here. double check if the
      //  following implementation is correct

      // Calculate the total cost of the successful samples
      double total_cost = 0;
      for (auto idx : successful_idx_list) {
        total_cost += (*(QPs.c_vec[idx]))(0) / n_succ_sample;
      }
      if (total_cost <= ave_min_cost_so_far) ave_min_cost_so_far = total_cost;

      // Print the total cost of this iteration
      cout << "total_cost = " << total_cost
           << " (min so far: " << ave_min_cost_so_far << ")\n\n";

      // Update each cost when all samples are successful
      if (all_samples_succeeded) {
        for (int idx = 0; idx < N_sample; idx++) {
          if ((*(QPs.c_vec[idx]))(0) < each_min_cost_so_far[idx]) {
            each_min_cost_so_far[idx] = (*(QPs.c_vec[idx]))(0);
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
      if ((iter > 1) && all_samples_succeeded) {
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
        DRAKE_DEMAND(QPs.c_vec.size() == each_min_cost_so_far.size());
        bool exit_current_iter_to_shrink_step_size = false;
        for (auto idx : successful_idx_list) {
          // print
          if ((*(QPs.c_vec[idx]))(0) > each_min_cost_so_far[idx]) {
            cout << "Cost #" << idx << " went up by "
                 << ((*(QPs.c_vec[idx]))(0) - each_min_cost_so_far[idx]) /
                        each_min_cost_so_far[idx] * 100
                 << "%.\n";
          }
          // If cost goes up, we restart the iteration and shrink the step size.
          if ((*(QPs.c_vec[idx]))(0) >
              (1 + tol_sample_cost) * each_min_cost_so_far[idx]) {
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
      {
        cout << "\nExtracting active (and independent rows) of A...\n";
        vector<std::thread*> threads(std::min(CORES, n_succ_sample));
        int temp_start_of_idx_list = 0;
        while (temp_start_of_idx_list < n_succ_sample) {
          int temp_end_of_idx_list =
              (temp_start_of_idx_list + CORES >= n_succ_sample)
                  ? n_succ_sample
                  : temp_start_of_idx_list + CORES;
          int thread_idx = 0;
          for (int idx_of_idx_list = temp_start_of_idx_list;
               idx_of_idx_list < temp_end_of_idx_list; idx_of_idx_list++) {
            threads[thread_idx] = new std::thread(
                extractActiveAndIndependentRows,
                successful_idx_list[idx_of_idx_list],
                FLAGS_major_feasibility_tol, indpt_row_tol, dir, std::ref(QPs),
                method_to_solve_system_of_equations);
            thread_idx++;
          }
          thread_idx = 0;
          for (int idx_of_idx_list = temp_start_of_idx_list;
               idx_of_idx_list < temp_end_of_idx_list; idx_of_idx_list++) {
            threads[thread_idx]->join();
            delete threads[thread_idx];
            thread_idx++;
          }
          temp_start_of_idx_list = temp_end_of_idx_list;
        }
      }
      /*// Read the matrices after extractions
      readNonredundentMatrixFile(&nw_vec, &nl_vec,
                                 &A_active_vec, &B_active_vec,
                                 n_succ_sample, dir);*/
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
      // Our calculation below is based on the fact that the H matrices are pd
      // and symmetric, so we check them here.
      // However, H turned out not to be psd, since we have timestep h as
      // decision variable. (It came from running cost. ~h*u'*R*u, etc)
      // Fixed it by adding cost manually (but the timestep is fixed now).
      // Now H is always pd because we also added a regularization term.
      /*cout << "Checking if H is pd and symmetric\n";
      for (int sample = 0; sample < n_succ_sample; sample++) {
        // Check if H is symmetric
        VectorXd One_w = VectorXd::Ones(nw_vec[sample]);
        double sum = One_w.transpose() *
                     (H_vec[sample] - H_vec[sample].transpose()) * One_w;
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
      auto start_time_calc_w = std::chrono::high_resolution_clock::now();
      {
        // cout << "Getting P matrix and q vecotr\n";
        vector<std::thread*> threads(std::min(CORES, n_succ_sample));
        int temp_start_of_idx_list = 0;
        while (temp_start_of_idx_list < n_succ_sample) {
          int temp_end_of_idx_list =
              (temp_start_of_idx_list + CORES >= n_succ_sample)
                  ? n_succ_sample
                  : temp_start_of_idx_list + CORES;
          int thread_idx = 0;
          for (int idx_of_idx_list = temp_start_of_idx_list;
               idx_of_idx_list < temp_end_of_idx_list; idx_of_idx_list++) {
            threads[thread_idx] = new std::thread(
                calcWInTermsOfTheta, successful_idx_list[idx_of_idx_list], dir,
                QPs, method_to_solve_system_of_equations);
            thread_idx++;
          }
          thread_idx = 0;
          for (int idx_of_idx_list = temp_start_of_idx_list;
               idx_of_idx_list < temp_end_of_idx_list; idx_of_idx_list++) {
            threads[thread_idx]->join();
            delete threads[thread_idx];
            thread_idx++;
          }
          temp_start_of_idx_list = temp_end_of_idx_list;
        }
      }
      /*// Read P_i and q_i
      readPiQiFile(&P_vec, &q_vec, n_succ_sample, dir);*/
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
      VectorXd gradient_cost(rom->n_theta());
      double norm_grad_cost;
      CalcCostGradientAndNorm(
          successful_idx_list, QPs.P_vec, QPs.q_vec,
          FLAGS_only_update_wrt_main_cost ? QPs.b_main_vec : QPs.b_vec, dir,
          prefix, &gradient_cost, &norm_grad_cost);

      // Calculate Newton step and the decrement
      VectorXd newton_step(rom->n_theta());
      double lambda_square;
      CalcNewtonStepAndNewtonDecrement(rom->n_theta(), successful_idx_list,
                                       QPs.P_vec, QPs.H_vec, gradient_cost, dir,
                                       prefix, &newton_step, &lambda_square);

      // Check optimality
      if (HasAchievedOptimum(is_newton, stopping_threshold, lambda_square,
                             norm_grad_cost)) {
        break;
      }

      // Calculate step_direction
      double step_direction_norm;
      GetStepDirectionAndNorm(is_newton, newton_step, gradient_cost,
                              beta_momentum, dir, prefix, &prev_step_direction,
                              &step_direction, &step_direction_norm);

      // Calculate step size
      GetHeuristicStepSize(h_step, step_direction_norm, dir, prefix,
                           &current_iter_step_size);

      // Gradient descent
      prev_theta = rom->theta();
      rom->SetTheta(rom->theta() + current_iter_step_size * step_direction);

      // For message printed to the terminal
      n_shrink_step = 0;

      // cout << '\a';  // making noise to notify the user the iteration ends
    }  // end if(!is_get_nominal)
  }    // end for

  cout << "\nExited the outer loop.\n";
  cout << '\a';  // making noise to notify the user the end of an iteration
  current_time =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  cout << "Current time: " << std::ctime(&current_time) << "\n\n";

  // store parameter values
  prefix = to_string(iter) + "_";
  if (!FLAGS_is_debug) {
    writeCSV(dir + prefix + string("theta_y.csv"), rom->theta_y());
    writeCSV(dir + prefix + string("theta_yddot.csv"), rom->theta_yddot());
  }

  return 0;
}  // int findGoldilocksModels

}  // namespace dairlib::goldilocks_models

int main(int argc, char* argv[]) {
  return dairlib::goldilocks_models::findGoldilocksModels(argc, argv);
}
