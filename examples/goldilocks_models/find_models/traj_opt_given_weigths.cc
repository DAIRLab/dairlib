#include "examples/goldilocks_models/find_models/traj_opt_given_weigths.h"

#include <memory>
#include <chrono>
#include <string>

#include "common/find_resource.h"
#include "examples/goldilocks_models/dynamics_expression.h"
#include "examples/goldilocks_models/find_models/traj_opt_helper_func.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/goldilocks_models/file_utils.h"
#include "systems/goldilocks_models/symbolic_manifold.h"
#include "systems/primitives/subvector_pass_through.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using drake::VectorX;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::trajectories::PiecewisePolynomial;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::MatrixXDecisionVariable;
using drake::symbolic::Variable;
using drake::symbolic::Expression;
using std::shared_ptr;
using std::cout;
using std::endl;
using std::string;
using std::map;

using drake::geometry::SceneGraph;
using drake::multibody::Body;
using drake::multibody::Parser;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

using dairlib::systems::trajectory_optimization::DirconAbstractConstraint;
using dairlib::systems::trajectory_optimization::DirconDynamicConstraint;
using dairlib::systems::trajectory_optimization::DirconKinConstraintType;
using dairlib::systems::trajectory_optimization::DirconKinematicConstraint;
using dairlib::systems::trajectory_optimization::DirconOptions;
using dairlib::systems::trajectory_optimization::HybridDircon;

// using Isometry3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>

namespace dairlib::goldilocks_models {

void addRegularization(bool is_get_nominal, double eps_reg,
                       GoldilocksModelTrajOpt& gm_traj_opt) {
  // Add regularization term here so that hessian is pd (for outer loop), so
  // that we can use schur complement method
  // TODO(yminchen): should I add to all decision variable? or just state?
  if (!is_get_nominal) {
    auto w = gm_traj_opt.dircon->decision_variables();
    for (int i = 0; i < w.size(); i++)
      gm_traj_opt.dircon->AddQuadraticCost(eps_reg * w(i)*w(i));
    // cout << "w = " << w << endl;
    // cout << "Check the order of decisiion variable: \n";
    // for (int i = 0; i < w.size(); i++)
    //   cout << gm_traj_opt.dircon->FindDecisionVariableIndex(w(i)) << endl;
  }
}

void setInitialGuessFromFile(const string& directory,
                             const string& init_file,
                             GoldilocksModelTrajOpt& gm_traj_opt) {
  VectorXd w0 = readCSV(directory + init_file).col(0);
  int n_dec = gm_traj_opt.dircon->decision_variables().rows();
  if (n_dec > w0.rows()) {
    cout << "dim(initial guess) < dim(decision var). "
         "Fill the rest with zero's.\n";
    VectorXd old_w0 = w0;
    w0.resize(n_dec);
    w0 = VectorXd::Zero(n_dec);
    w0.head(old_w0.rows()) = old_w0;
  }
  gm_traj_opt.dircon->SetInitialGuessForAllVariables(w0);
}

void augmentConstraintToFixThetaScaling(MatrixXd & B, MatrixXd & A,
                                        VectorXd & y, VectorXd & lb, VectorXd & ub,
                                        int n_s, int n_feature_s,
                                        const VectorXd & theta_s, int sample_idx) {
  // sum theta over a row = const

  int n_c = B.rows();
  int n_t = B.cols();
  int n_w = A.cols();

  MatrixXd B_old = B;
  B.resize(n_c + n_s, n_t);
  B = MatrixXd::Zero(n_c + n_s, n_t);
  B.block(0, 0, n_c, n_t) = B_old;
  for (int i = 0; i < n_s; i++) {
    B.block(n_c + i, i * n_feature_s, 1, n_feature_s) =
      VectorXd::Ones(n_feature_s).transpose();
  }

  MatrixXd A_old = A;
  A.resize(n_c + n_s, n_w);
  A = MatrixXd::Zero(n_c + n_s, n_w);
  A.block(0, 0, n_c, n_w) = A_old;

  MatrixXd y_old = y;
  y.resize(n_c + n_s);
  VectorXd y_append = VectorXd::Zero(n_s);
  for (int i = 0; i < n_s; i++) {
    for (int j = 0; j < n_feature_s; j++) {
      y_append(i) += theta_s(j + i * n_feature_s);
    }
  }
  y << y_old, y_append;

  MatrixXd lb_old = lb;
  lb.resize(n_c + n_s);
  lb << lb_old, y_append;

  MatrixXd ub_old = ub;
  ub.resize(n_c + n_s);
  ub << ub_old, y_append;

  if (sample_idx == 0)
    cout << "parameters sum per position = " << y_append.transpose() << endl;
}

void extractResult(VectorXd& w_sol,
                   GoldilocksModelTrajOpt& gm_traj_opt,
                   const MathematicalProgramResult& result,
                   std::chrono::duration<double> elapsed,
                   std::vector<int> num_time_samples,
                   int& N,
                   const MultibodyPlant<double> & plant,
                   const MultibodyPlant<AutoDiffXd> & plant_autoDiff,
                   int n_s, int n_sDDot, int n_tau,
                   int n_feature_s,
                   int n_feature_sDDot,
                   const MatrixXd& B_tau,
                   const VectorXd & theta_s, const VectorXd & theta_sDDot,
                   double stride_length, double ground_incline,
                   double duration, int max_iter,
                   const string& directory,
                   const string& init_file, const string& prefix,
                   double Q_double, double R_double,
                   double eps_reg,
                   bool is_get_nominal,
                   bool is_zero_touchdown_impact,
                   bool extend_model,
                   bool is_add_tau_in_cost,
                   int sample_idx, int n_rerun,
                   double cost_threshold_for_update, int N_rerun,
                   int rom_option,
                   int robot_option,
                   vector<DirconKinematicDataSet<double>*> dataset_list,
                   bool is_print_for_debugging) {
  //
  // int n_q = plant.num_positions();
  // int n_v = plant.num_velocities();
  // int n_x = n_q + n_v;
  // int n_u = plant.num_actuators();

  // Extract solution
  SolutionResult solution_result = result.get_solution_result();
  double tau_cost = 0;
  if (is_add_tau_in_cost) {
    // Way 1
    for (auto const & binding : gm_traj_opt.tau_cost_bindings) {
      const auto& tau_i = binding.variables();
      VectorXd tau_i_double = result.GetSolution(tau_i);
      VectorXd y_val(1);
      binding.evaluator()->Eval(tau_i_double, &y_val);
      tau_cost += y_val(0);
    }
  }

  /*cout << "sample_idx# = " << sample_idx << endl;
  cout << "    stride_length = " << stride_length << " | "
       << "ground_incline = " << ground_incline << " | "
       << "init_file = " << init_file << endl;
  cout << "    Solve time:" << elapsed.count() << " | ";
  cout << solution_result <<  " | ";
  cout << "Cost:" << result.get_optimal_cost() <<
       " (tau cost = " << tau_cost << ")\n";*/
  /*string string_to_print = "sample #" + to_string(sample_idx) +
                           " (rerun #" + to_string(n_rerun) + ")"
                           "\n    stride_length = " + to_string(stride_length) +
                           " | ground_incline = " + to_string(ground_incline) +
                           " | init_file = " + init_file +
                           "\n    Solve time:" + to_string(elapsed.count()) +
                           " | " + to_string(solution_result) +
                           " | Cost:" + to_string(result.get_optimal_cost()) +
                           " (tau cost = " + to_string(tau_cost) + ")\n";*/
  string string_to_print = "  " + to_string(sample_idx) +
      " (" + to_string(n_rerun) + ")"
      " | " + to_string(stride_length) +
      " | " + to_string(ground_incline) +
      " | " + init_file +
      " | " + to_string(solution_result) +
      " | " + to_string(elapsed.count()) +
      " | " + to_string(result.get_optimal_cost()) +
      " (" + to_string(tau_cost) + ")\n";
  cout << string_to_print;

  // Check which solver we are using
  // cout << "Solver: " << result.get_solver_id().name() << endl;

  if ((result.get_optimal_cost() > cost_threshold_for_update) &&
      (n_rerun > N_rerun)) {
    cout << "the cost of idx #" << sample_idx
         << " is higher than before, skip\n";
    return;
  }

  VectorXd is_success(1);
  if (result.is_success()) is_success << 1;
  else is_success << 0;
  writeCSV(directory + prefix + string("is_success.csv"), is_success);

  // bool constraint_satisfied = solvers::CheckGenericConstraints(*trajopt, result, 1e-5);
  // cout << "constraint_satisfied = " << constraint_satisfied << endl;

  // Store time, state, and its derivatives for cubic spline reconstruction
  VectorXd t_cubic_spline;
  MatrixXd x_cubic_spline;
  MatrixXd xdot_cubic_spline;
  gm_traj_opt.ConstructStateCubicSplineInfo(
      result, plant, num_time_samples, dataset_list, &t_cubic_spline,
      &x_cubic_spline, &xdot_cubic_spline);
  writeCSV(directory + prefix + string("t_cubic_spline.csv"), t_cubic_spline);
  writeCSV(directory + prefix + string("x_cubic_spline.csv"), x_cubic_spline);
  writeCSV(directory + prefix + string("xdot_cubic_spline.csv"),
           xdot_cubic_spline);

  // Get the solution of all the decision variable
  w_sol = result.GetSolution(gm_traj_opt.dircon->decision_variables());
  writeCSV(directory + prefix + string("w.csv"), w_sol);
  // if (result.is_success())
  //   writeCSV(directory + prefix + string("w (success).csv"), w_sol);
  if (is_print_for_debugging) {
    for (int i = 0; i < w_sol.size(); i++) {
      cout << i << ": " << gm_traj_opt.dircon->decision_variables()[i] << ", "
           << w_sol[i] << endl;
    } cout << endl;
  }

  // Store the time, state, and input at knot points
  VectorXd time_at_knots = gm_traj_opt.dircon->GetSampleTimes(result);
  MatrixXd state_at_knots = gm_traj_opt.dircon->GetStateSamples(result);
  MatrixXd input_at_knots = gm_traj_opt.dircon->GetInputSamples(result);
  auto xf = gm_traj_opt.dircon->state_vars_by_mode(num_time_samples.size() - 1,
            num_time_samples[num_time_samples.size() - 1] - 1);
//  state_at_knots.col(N - 1) = result.GetSolution(xf);
  // cout << "you'll need to update state_at_knots if it's multiple modes\n";
  writeCSV(directory + prefix + string("time_at_knots.csv"), time_at_knots);
  writeCSV(directory + prefix + string("state_at_knots.csv"), state_at_knots);
  writeCSV(directory + prefix + string("input_at_knots.csv"), input_at_knots);
  if (is_print_for_debugging) {
    cout << "time_at_knots = \n" << time_at_knots << "\n";
    cout << "state_at_knots = \n" << state_at_knots << "\n";
    //  cout << "state_at_knots.size() = " << state_at_knots.size() << endl;
    cout << "input_at_knots = \n" << input_at_knots << "\n";
  }

  // Also store lambda. We might need to look at it in the future!
  // (save it so we don't need to rerun)
  std::ofstream ofile;
  ofile.open(directory + prefix + string("lambda_at_knots.txt"),
             std::ofstream::out);
  // cout << "lambda_at_knots = \n";
  for (unsigned int mode = 0; mode < num_time_samples.size(); mode++) {
    for (int index = 0; index < num_time_samples[mode]; index++) {
      auto lambdai = gm_traj_opt.dircon->force(mode, index);
      ofile << result.GetSolution(lambdai).transpose() << endl;
      if (is_print_for_debugging) {
        cout << result.GetSolution(lambdai).transpose() << endl;
      }
    }
  }
  ofile.close();

  VectorXd c(1);
  c << result.get_optimal_cost();
  VectorXd c_without_tau(1);
  c_without_tau << c(0) - tau_cost;
  writeCSV(directory + prefix + string("c.csv"), c);
  writeCSV(directory + prefix + string("c_without_tau.csv"), c_without_tau);

  // Testing
  /*bool is_check_tau = true;
  if (is_check_tau) {
    int N_accum = 0;
    for (unsigned int l = 0; l < num_time_samples.size() ; l++) {
      for (int m = 0; m < num_time_samples[l] - 1 ; m++) {
        int i = N_accum + m;
        cout << "i = " << i << endl;
        // Get tau_append
        auto x_i = gm_traj_opt.dircon->state_vars_by_mode(l, m);
        auto tau_i = gm_traj_opt.reduced_model_input(i, n_tau);
        auto x_iplus1 = gm_traj_opt.dircon->state_vars_by_mode(l, m + 1);
        auto tau_iplus1 = gm_traj_opt.reduced_model_input(i + 1, n_tau);
        auto h_btwn_knot_i_iplus1 = gm_traj_opt.dircon->timestep(i);
        VectorXd x_i_sol = result.GetSolution(x_i);
        VectorXd tau_i_sol = result.GetSolution(tau_i);
        VectorXd x_iplus1_sol = result.GetSolution(x_iplus1);
        VectorXd tau_iplus1_sol = result.GetSolution(tau_iplus1);
        VectorXd h_i_sol = result.GetSolution(h_btwn_knot_i_iplus1);

        VectorXd tau_append_head =
          gm_traj_opt.dynamics_constraint_at_head->computeTauToExtendModel(
            x_i_sol, x_iplus1_sol, h_i_sol, theta_s);
        cout << "tau_head = " << tau_append_head.transpose() << endl;
      }
      N_accum += num_time_samples[l];
      N_accum -= 1;  // due to overlaps between modes
    }
  }*/

  // Testing: print out the timesteps
  // for(int i = 0; i<N-1 ; i++){
  //   auto h_i = gm_traj_opt.dircon->timestep(i);
  //   VectorXd h_i_sol = gm_traj_opt.dircon->GetSolution(h_i);
  //   cout << "h_"<< i <<"_sol = " << h_i_sol << endl;
  // }
  /*auto h_0 = gm_traj_opt.dircon->timestep(0);
  VectorXd h_0_sol = result.GetSolution(h_0);
  cout << "timestep = " << h_0_sol << endl << endl;*/

  // Testing: print out the vertical pos
  /*for(int i = 0; i<N ; i++){
    auto x_i = gm_traj_opt.dircon->state(i);
    VectorXd x_i_sol = result.GetSolution(x_i);
    cout << "x1_"<< i <<"_sol = " << x_i_sol(1) << endl;
  }*/

  // Testing: see what are the decision varaibles
  // cout << "\nAll decision variable:\n"
  //   << gm_traj_opt.dircon->decision_variables() << endl;

  // Testing - see the solution values (to decide how big we should scale them)
  // VectorXd z = result.GetSolution(
  //                    gm_traj_opt.dircon->decision_variables());
  // for(int i = 0; i<z.size(); i++){
  //   cout << gm_traj_opt.dircon->decision_variables()[i] << ", " << z[i] << endl;
  // }
  // cout << endl;
}

void postProcessing(const VectorXd& w_sol,
                    GoldilocksModelTrajOpt& gm_traj_opt,
                    const MathematicalProgramResult& result,
                    std::chrono::duration<double> elapsed,
                    std::vector<int> num_time_samples,
                    int& N,
                    const MultibodyPlant<double> & plant,
                    const MultibodyPlant<AutoDiffXd> & plant_autoDiff,
                    int n_s, int n_sDDot, int n_tau,
                    int n_feature_s,
                    int n_feature_sDDot,
                    const MatrixXd& B_tau,
                    const VectorXd & theta_s, const VectorXd & theta_sDDot,
                    double stride_length, double ground_incline,
                    double duration, int max_iter,
                    const string& directory,
                    const string& init_file,
                    const string& prefix,
                    double Q_double, double R_double,
                    double eps_reg,
                    bool is_get_nominal,
                    bool is_zero_touchdown_impact,
                    bool extend_model,
                    bool is_add_tau_in_cost,
                    int sample_idx, int n_rerun,
                    double cost_threshold_for_update, int N_rerun,
                    int rom_option,
                    int robot_option) {
  if (is_get_nominal || !result.is_success()) {
    // Do nothing.
  } else if (extend_model) {  // Extending the model
    VectorXd theta_s_append = readCSV(directory +
                                      string("theta_s_append.csv")).col(0);
    int n_extend = theta_s_append.rows() / n_feature_s;

    // Update trajectory optimization solution
    // Assume that tau is at the end of the decision variable
    VectorXd tau_new = VectorXd::Zero((n_tau + n_extend) * N);
    int N_accum = 0;
    for (unsigned int l = 0; l < num_time_samples.size() ; l++) {
      for (int m = 0; m < num_time_samples[l] - 1 ; m++) {
        int i = N_accum + m;
        // cout << "i = " << i << endl;
        // Get tau_append
        auto x_i = gm_traj_opt.dircon->state_vars_by_mode(l, m);
        auto x_iplus1 = gm_traj_opt.dircon->state_vars_by_mode(l, m + 1);
        auto h_btwn_knot_i_iplus1 = gm_traj_opt.dircon->timestep(i);
        VectorXd x_i_sol = result.GetSolution(x_i);
        VectorXd x_iplus1_sol = result.GetSolution(x_iplus1);
        VectorXd h_i_sol = result.GetSolution(h_btwn_knot_i_iplus1);

        VectorXd tau_append_head =
          gm_traj_opt.dynamics_constraint_at_head->computeTauToExtendModel(
            x_i_sol, x_iplus1_sol, h_i_sol, theta_s_append);
        // cout << "tau_append_head = " << tau_append_head.transpose() << endl;
        // VectorXd tau_append_tail =
        //   gm_traj_opt.dynamics_constraint_at_tail->computeTauToExtendModel(
        //     x_i_sol, x_iplus1_sol, h_i_sol, theta_s_append);
        // cout << "tau_append_tail = " << tau_append_tail.transpose() << endl;

        // Update tau
        tau_new.segment(i * (n_tau + n_extend) + n_tau, n_extend) +=
          tau_append_head;
        // tau_new.segment((i + 1) * (n_tau + n_extend) + n_tau, n_extend) +=
        //   tau_append_tail;
      }
      N_accum += num_time_samples[l];
      N_accum -= 1;  // due to overlaps between modes
    }
    // store new w_sol
    VectorXd w_sol_new(w_sol.rows() + n_extend * N);
    w_sol_new << w_sol.head(w_sol.rows() - n_tau * N), tau_new;
    writeCSV(directory + prefix + string("w (no extension).csv"), w_sol);
    writeCSV(directory + prefix + string("w.csv"), w_sol_new);

    // Create a file that shows the new index of theta_sDDot
    // Assume that the new features include all the old features (in dynamics)
    VectorXd prime_numbers = createPrimeNumbers(2 * (n_s + n_extend));

    int new_rom_option = 0;
    if (rom_option == 0) {
      new_rom_option = 1;
    } else if (rom_option == 2) {
      new_rom_option = 3;
    } else {
      DRAKE_DEMAND(false);
    }
    DynamicsExpression dyn_expr_old(n_sDDot, 0, rom_option, robot_option);
    DynamicsExpression dyn_expr_new(n_sDDot + n_extend, 0, new_rom_option, robot_option);
    VectorXd dummy_s_new = prime_numbers.head(n_s + n_extend);
    VectorXd dummy_sDDot_new = prime_numbers.tail(n_s + n_extend);
    VectorXd dummy_s_old = dummy_s_new.head(n_s);
    VectorXd dummy_sDDot_old = dummy_sDDot_new.head(n_s);
    VectorXd feat_old = dyn_expr_old.getFeature(dummy_s_old, dummy_sDDot_old);
    VectorXd feat_new = dyn_expr_new.getFeature(dummy_s_new, dummy_sDDot_new);

    VectorXd new_idx(feat_old.rows());
    for (int i = 0; i < feat_old.rows(); i++) {
      int idx = -1;
      for (int j = 0; j < feat_new.rows(); j++) {
        if (feat_old(i) == feat_new(j))
          idx = j;
      }
      if (idx == -1)
        cout << "Failed to create the index list automatically.\n";

      DRAKE_DEMAND(idx > -1);
      new_idx(i) = idx;
    }
    writeCSV(directory + string("theta_sDDot_new_index.csv"), new_idx);

  } else {
    if ((result.get_optimal_cost() > cost_threshold_for_update) &&
        (n_rerun > N_rerun)) {
      return;
    }

    // Assume theta is fixed. Get the linear approximation of
    //      // the cosntraints and second order approximation of the cost.
    // cout << "\nGetting A, H, y, lb, ub, b.\n";
    MatrixXd A, H;
    VectorXd y, lb, ub, b;
    // cout << "LinearizeConstraints...\n";
    solvers::LinearizeConstraints(
      *gm_traj_opt.dircon, w_sol, &y, &A, &lb, &ub);
    // cout << "SecondOrderCost...\n";
    solvers::SecondOrderCost(*gm_traj_opt.dircon, w_sol, &H, &b);

    // Get matrix B (~get feature vectors)
    // cout << "\nGetting B.\n";
    int n_theta_s = theta_s.size();
    int n_theta_sDDot = theta_sDDot.size();
    int n_theta = n_theta_s + n_theta_sDDot;
    MatrixXd B = MatrixXd::Zero(A.rows(), n_theta);
    // Get the row index of B matrix where dynamics constraint starts
    VectorXd ind_head = solvers::GetConstraintRows(
                          *gm_traj_opt.dircon,
                          gm_traj_opt.dynamics_constraint_at_head_bindings[0]);
    // cout << "ind_head = " << ind_head(0) << endl;
    // VectorXd ind_tail = solvers::GetConstraintRows(
    //                       *gm_traj_opt.dircon.get(),
    //                       gm_traj_opt.dynamics_constraint_at_tail_bindings[0]);
    // cout << "ind_tail = " << ind_tail(0) << endl;
    int N_accum = 0;
    for (unsigned int l = 0; l < num_time_samples.size() ; l++) {
      for (int m = 0; m < num_time_samples[l] - 1 ; m++) {
        int i = N_accum + m;
        // cout << "i = " << i << endl;
        // Get the gradient value first
        auto x_i = gm_traj_opt.dircon->state_vars_by_mode(l, m);
        auto tau_i = gm_traj_opt.reduced_model_input(i, n_tau);
        auto x_iplus1 = gm_traj_opt.dircon->state_vars_by_mode(l, m + 1);
        auto tau_iplus1 = gm_traj_opt.reduced_model_input(i + 1, n_tau);
        auto h_btwn_knot_i_iplus1 = gm_traj_opt.dircon->timestep(i);
        VectorXd x_i_sol = result.GetSolution(x_i);
        VectorXd tau_i_sol = result.GetSolution(tau_i);
        VectorXd x_iplus1_sol = result.GetSolution(x_iplus1);
        VectorXd tau_iplus1_sol = result.GetSolution(tau_iplus1);
        VectorXd h_i_sol = result.GetSolution(h_btwn_knot_i_iplus1);

        MatrixXd dyn_gradient_head =
          gm_traj_opt.dynamics_constraint_at_head->getGradientWrtTheta(
            x_i_sol, tau_i_sol, x_iplus1_sol, tau_iplus1_sol, h_i_sol);
        // MatrixXd dyn_gradient_tail =
        //   gm_traj_opt.dynamics_constraint_at_tail->getGradientWrtTheta(
        //     x_i_sol, tau_i_sol, x_iplus1_sol, tau_iplus1_sol, h_i_sol);

        // Fill in B matrix
        B.block(ind_head(0) + i * n_sDDot, 0, n_sDDot, n_theta)
          = dyn_gradient_head;
        // B.block(ind_tail(0) + i * 2 * n_sDDot, 0, n_sDDot, n_theta)
        //   = dyn_gradient_tail;
        // cout << "row " << ind_head(0) + i * 2 * n_sDDot << endl;
        // cout << "row " << ind_tail(0) + i * 2 * n_sDDot << endl << endl;
      }
      N_accum += num_time_samples[l];
      N_accum -= 1;  // due to overlaps between modes
    }

    // Augment the constraint matrices and vectors (B, A, y, lb, ub)
    // so that we fix the scaling of the model parameters
    /*augmentConstraintToFixThetaScaling(B, A, y, lb, ub,
                                       n_s, n_feature_s, theta_s, sample_idx);*/

    // Push the solution to the vector
    /*w_sol_vec->push_back(w_sol);
    H_vec->push_back(H);
    b_vec->push_back(b);
    c_vec->push_back(c);
    A_vec->push_back(A);
    lb_vec->push_back(lb);
    ub_vec->push_back(ub);
    y_vec->push_back(y);
    B_vec->push_back(B);*/

    // Store the vectors and matrices
    // cout << "\nStoring vectors and matrices into csv.\n";
    writeCSV(directory + prefix + string("H.csv"), H);
    writeCSV(directory + prefix + string("b.csv"), b);
    writeCSV(directory + prefix + string("A.csv"), A);
    writeCSV(directory + prefix + string("lb.csv"), lb);
    writeCSV(directory + prefix + string("ub.csv"), ub);
    writeCSV(directory + prefix + string("y.csv"), y);
    writeCSV(directory + prefix + string("B.csv"), B);

    // Store s, ds, dds and tau into csv files
    // cout << "\nStoring s, ds and dds into csv.\n";
    std::vector<VectorXd> s_vec;
    std::vector<VectorXd> ds_vec;
    std::vector<VectorXd> dds_vec;
    std::vector<VectorXd> tau_vec;
    std::vector<VectorXd> h_vec;
    N_accum = 0;
    // for (unsigned int l = 0; l < num_time_samples.size() ; l++) {
    for (unsigned int l = 0; l < 1 ; l++) { // just look at the first mode now
      for (int m = 0; m < num_time_samples[l]; m++) {
        int i = N_accum + m;
        // Get the gradient value first
        auto x_i = gm_traj_opt.dircon->state_vars_by_mode(l, m);
        auto tau_i = gm_traj_opt.reduced_model_input(i, n_tau);
        VectorXd x_i_sol = result.GetSolution(x_i);
        VectorXd tau_i_sol = result.GetSolution(tau_i);

        VectorXd s(n_s);
        VectorXd ds(n_s);
        gm_traj_opt.dynamics_constraint_at_head->getSAndSDot(x_i_sol, s, ds);
        VectorXd dds =
          gm_traj_opt.dynamics_constraint_at_head->getSDDot(s, ds, tau_i_sol);
        s_vec.push_back(s);
        ds_vec.push_back(ds);
        dds_vec.push_back(dds);
        tau_vec.push_back(tau_i_sol);

        if (m < num_time_samples[l] - 1) {
          auto h_i = gm_traj_opt.dircon->timestep(i);
          VectorXd h_i_sol = result.GetSolution(h_i);
          h_vec.push_back(h_i_sol);
        }
      }
      N_accum += num_time_samples[l];
      N_accum -= 1;  // due to overlaps between modes
    }
    PiecewisePolynomial<double> s_spline = createCubicSplineGivenSAndSdot(
        h_vec, s_vec, ds_vec);
    storeSplineOfS(h_vec, s_spline, directory, prefix);
    storeTau(h_vec, tau_vec, directory, prefix);
    // checkSplineOfS(h_vec, dds_vec, s_spline);




    // Below are all for debugging /////////////////////////////////////////////

    // Checking B
    // BTW, the code only work in the case of s = q_1 ^2 and dds = s^3
    bool is_checking_matrix_B = false;
    if (is_checking_matrix_B) {
      N_accum = 0;
      for (unsigned int l = 0; l < 1 ; l++) { // just look at the first mode now
        for (int m = 0; m < num_time_samples[l] - 1; m++) {
          int i = N_accum + m;
          cout << "i = " << i << endl;

          // From finite differencing
          auto x_i = gm_traj_opt.dircon->state_vars_by_mode(l, m);
          auto tau_i = gm_traj_opt.reduced_model_input(i, n_tau);
          auto x_iplus1 = gm_traj_opt.dircon->state_vars_by_mode(l, m + 1);
          auto tau_iplus1 = gm_traj_opt.reduced_model_input(i + 1, n_tau);
          auto h_btwn_knot_i_iplus1 = gm_traj_opt.dircon->timestep(i);
          VectorXd x_i_sol = result.GetSolution(x_i);
          VectorXd tau_i_sol = result.GetSolution(tau_i);
          VectorXd x_iplus1_sol = result.GetSolution(x_iplus1);
          VectorXd tau_iplus1_sol = result.GetSolution(tau_iplus1);
          VectorXd h_i_sol = result.GetSolution(h_btwn_knot_i_iplus1);
          double h_i = h_i_sol(0);

          MatrixXd grad_head_byFD =
            gm_traj_opt.dynamics_constraint_at_head->getGradientWrtTheta(
              x_i_sol, tau_i_sol, x_iplus1_sol, tau_iplus1_sol, h_i_sol);
          // MatrixXd grad_tail_byFD =
          //   gm_traj_opt.dynamics_constraint_at_tail->getGradientWrtTheta(
          //     x_i_sol, tau_i_sol, x_iplus1_sol, tau_iplus1_sol, h_i_sol);

          // From hand calculation (theta_s part)
          double phis_i = x_i_sol(1) * x_i_sol(1);
          double dphis_i = 2 * x_i_sol(1) * x_i_sol(1 + 7);
          double phis_iplus1 = x_iplus1_sol(1) * x_iplus1_sol(1);
          double dphis_iplus1 = 2 * x_iplus1_sol(1) * x_iplus1_sol(1 + 7);
          double grad_head_exact =
            (-6 * (phis_i - phis_iplus1) -
             2 * h_i * (2 * dphis_i + dphis_iplus1)) / (h_i * h_i) -
            theta_sDDot(0) * (3 * pow(theta_s(0), 2) * pow(phis_i, 3));
          // double grad_tail_exact =
          //   (6 * (phis_i - phis_iplus1) +
          //    2 * h_i * (dphis_i + 2 * dphis_iplus1)) / (h_i * h_i) -
          //   theta_sDDot(0) * (3 * pow(theta_s(0), 2) * pow(phis_iplus1, 3));

          // From hand calculation (theta_sddot part)
          double dyn_feature_i = pow(theta_s(0) * phis_i, 3);
          // double dyn_feature_iplus1 = pow(theta_s(0) * phis_iplus1, 3);

          // Compare the values
          cout << grad_head_byFD << " (by finite difference)" << endl;
          cout << grad_head_exact << " " << -dyn_feature_i <<
               " (analytically (exact solution))" << endl;
          cout << "  differnce = " << grad_head_byFD(0, 0) - grad_head_exact <<
               ", " << grad_head_byFD(0, 1) + dyn_feature_i << endl;
          // cout << grad_tail_byFD << " (by finite difference)" << endl;
          // cout << grad_tail_exact << " " << -dyn_feature_iplus1 <<
          //      " (analytically (exact solution))" << endl;
          // cout << "  differnce = " << grad_tail_byFD(0, 0) - grad_tail_exact <<
          //      ", " << grad_tail_byFD(0, 1) + dyn_feature_iplus1 << endl;
        }
        N_accum += num_time_samples[l];
        N_accum -= 1;  // due to overlaps between modes
      }
    }

    // Checking the accuracy of s and sdot calculation
    // BTW, the code only work in the case of y = q_1 ^2
    bool is_checking_s_sdot = false;
    if (is_checking_s_sdot) {
      N_accum = 0;
      for (unsigned int l = 0; l < 1 ; l++) { // just look at the first mode now
        for (int m = 0; m < num_time_samples[l] - 1; m++) {
          // int i = N_accum + m;
          // cout << "i = " << i << endl;

          auto x_i = gm_traj_opt.dircon->state_vars_by_mode(l, m);
          auto x_iplus1 = gm_traj_opt.dircon->state_vars_by_mode(l, m + 1);
          VectorXd x_i_sol = result.GetSolution(x_i);
          VectorXd x_iplus1_sol = result.GetSolution(x_iplus1);

          VectorXd s_i(n_s);
          VectorXd ds_i(n_s);
          VectorXd s_iplus1(n_s);
          VectorXd ds_iplus1(n_s);
          gm_traj_opt.dynamics_constraint_at_head->getSAndSDot(
            x_i_sol, s_i, ds_i);
          // cout << "ds_i_byhand - ds_i = " <<
          // theta_s(0) * 2 * x_i_sol(1)*x_i_sol(1 + 7) - ds_i(0) << endl;
          gm_traj_opt.dynamics_constraint_at_head->getSAndSDot(
            x_iplus1_sol, s_iplus1, ds_iplus1);
          // cout << "ds_iplus1_byhand - ds_iplus1 = " <<
          // theta_s(0) * 2 * x_iplus1_sol(1)*x_iplus1_sol(1 + 7) - ds_iplus1(0) << endl;
        }
        N_accum += num_time_samples[l];
        N_accum -= 1;  // due to overlaps between modes
      }
    }






    bool is_comparing_two_constraint_linearization = false;
    if (is_comparing_two_constraint_linearization) {
      /*// Comparing the new cosntraint linearization and the old lienarization
      MatrixXd A_new;
      VectorXd y_new, lb_new, ub_new;
      systems::trajectory_optimization::newlinearizeConstraints(
        gm_traj_opt.dircon.get(), w_sol, y_new, A_new, lb_new, ub_new);
      // reorganize the rows or A
      cout << "Reorganize the rows or A\n";
      int nl_i = A.rows();
      int nw_i = A.cols();
      cout << "size of A = " << A.rows() << ", " <<  A.cols() << endl;
      cout << "size of A_new = " << A_new.rows() << ", " <<  A_new.cols() << endl;
      vector<int> new_row_idx;
      VectorXd rowi(nw_i);
      VectorXd rowj(nw_i);
      VectorXd normalized_rowi(nw_i);
      VectorXd normalized_rowj(nw_i);
      for (int i = 0; i < nl_i; i++) { // A_new
        for (int j = 0; j < nl_i; j++) { // A
          rowi = A_new.row(i).transpose();
          rowj = A.row(j).transpose();
          normalized_rowi = rowi / rowi.norm();
          normalized_rowj = rowj / rowj.norm();
          if ((normalized_rowi - normalized_rowj).norm() < 1e-15) {
            if (rowi.norm() != rowj.norm()) {
              cout << i << "-th row of A_new: scale are different by " <<
                   rowi.norm() / rowj.norm() << endl;
            }
            // check ub and lb
            // cout << "("<<i<<","<<j<<")"<<": \n";
            // cout << "  ub(j) = " << ub(j) << endl;
            // cout << "  ub_new(i) = " << ub_new(i) << endl;
            // cout << "  lb(j) = " << lb(j) << endl;
            // cout << "  lb_new(i) = " << lb_new(i) << endl;
            if (ub(j) == ub_new(i) && lb(j) == lb_new(i) ) {
              // Maybe there are duplicated rows, so check if already assigned the row
              bool is_existing = false;
              for (int idx : new_row_idx) {
                if (idx == j) {
                  is_existing = true;
                  break;
                }
              }
              if (!is_existing) {
                new_row_idx.push_back(j);
                break;
              }
            }
          }
          if (j == nl_i - 1) cout << i << "-th row of A_new has no corresponding A\n";
        }
      }
      MatrixXd A_new_reorg(nl_i, nw_i);
      for (int i = 0; i < nl_i; i++) {
        A_new_reorg.row(new_row_idx[i]) = A_new.row(i);
      }
      cout << "size of new_row_idx = " << new_row_idx.size() << endl;
      // Check if new_row_idx covers every index
      for (int i = 0; i < nl_i; i++) {
        for (int j = 0; j < nl_i; j++) {
          if (new_row_idx[j] == i) {
            break;
          }
          if (j == nl_i - 1 ) cout << i << "-th row of A doesn't get assigned to\n";
        }
      }
      // for (int i:new_row_idx) cout << i << endl;

      // compare A with A_new_reorg
      MatrixXd diff_A = A_new_reorg - A;
      MatrixXd abs_diff_A = diff_A.cwiseAbs();
      VectorXd left_one = VectorXd::Ones(abs_diff_A.rows());
      VectorXd right_one = VectorXd::Ones(abs_diff_A.cols());
      cout << "sum-abs-diff_A: " <<
           left_one.transpose()*abs_diff_A*right_one << endl;
      cout << "sum-abs-diff_A divide by m*n: " <<
           left_one.transpose()*abs_diff_A*right_one /
           (abs_diff_A.rows()*abs_diff_A.cols())
           << endl;
      double max_diff_A_element = abs_diff_A(0, 0);
      for (int i = 0; i < abs_diff_A.rows(); i++)
        for (int j = 0; j < abs_diff_A.cols(); j++) {
          if (abs_diff_A(i, j) > max_diff_A_element) {
            max_diff_A_element = abs_diff_A(i, j);
            cout << "(" << i << "," << j << ")" << ": max_diff_A_element = " <<
                 max_diff_A_element << endl;
          }
        }
      cout << "max_diff_A_element = " << max_diff_A_element << endl;*/
    }









    // int A_row = A.rows();
    // int A_col = A.cols();
    // cout << "A_row = " << A_row << endl;
    // cout << "A_col = " << A_col << endl;
    // int max_row_col = (A_row > A_col) ? A_row : A_col;
    // Eigen::BDCSVD<MatrixXd> svd_5(A);
    // cout << "A:\n";
    // cout << "  biggest singular value is " << svd_5.singularValues()(0) << endl;
    // cout << "  smallest singular value is "
    //      << svd_5.singularValues()(max_row_col - 1) << endl;





    /*cout << "\ncheck if H is diagonal: \n";
    MatrixXd H_test = H;
    int nw = H_test.rows();
    for (int i = 0; i < nw; i++) {
      H_test(i, i) = 0;
    }
    if (VectorXd::Ones(nw).transpose()*H_test * VectorXd::Ones(nw) == 0)
      cout << "H is diagonal" << endl;

    cout << "checking b\n";
    cout << "b.norm() = " << b.norm() << endl;

    cout << "norm of y = " << y.norm() << endl;





    // Get the index of the rows of active constraints
    vector<int> active_eq_row_idx;
    vector<int> ub_active_ineq_row_idx;
    vector<int> lb_active_ineq_row_idx;
    double tol = 1e-8; //1e-4
    for (int i = 0; i < y.rows(); i++) {
      if (ub(i) == lb(i))
        active_eq_row_idx.push_back(i);
      else if (y(i) >= ub(i) - tol)
        ub_active_ineq_row_idx.push_back(i);
      else if (y(i) <= lb(i) + tol)
        lb_active_ineq_row_idx.push_back(i);
    }
    unsigned int n_ae = active_eq_row_idx.size();
    unsigned int n_aub = ub_active_ineq_row_idx.size();
    unsigned int n_alb = lb_active_ineq_row_idx.size();
    cout << "n_ae = " << n_ae << endl;
    cout << "n_aub = " << n_aub << endl;
    cout << "n_alb = " << n_alb << endl;




    cout << "\nRun traj opt to check if your quadratic approximation is correct\n";
    int nl_i = A.rows();
    int nw_i = A.cols();
    MathematicalProgram quadprog;
    auto dw = quadprog.NewContinuousVariables(nw_i, "dw");
    quadprog.AddLinearConstraint( A,
                                  lb - y,
                                  ub - y,
                                  dw);
    quadprog.AddQuadraticCost(H, b, dw);
    // quadprog.SetSolverOption(drake::solvers::GurobiSolver::id(), "BarConvTol", 1E-9);
    quadprog.SetSolverOption(drake::solvers::SnoptSolver::id(),
                             "Print file", "snopt.out");
    quadprog.SetSolverOption(drake::solvers::SnoptSolver::id(),
                             "Major iterations limit", 10000);
    // quadprog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Major feasibility tolerance", 1.0e-14); //1.0e-10
    // quadprog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Minor feasibility tolerance", 1.0e-14); //1.0e-10
    const auto result2 = Solve(quadprog);
    auto solution_result2 = result2.get_solution_result();
    cout << solution_result2 << endl;
    cout << "Cost:" << result2.get_optimal_cost() << endl;
    VectorXd dw_sol = result2.GetSolution(quadprog.decision_variables());
    cout << "dw_sol norm:" << dw_sol.norm() << endl;
    // cout << "dw_sol = \n" << dw_sol << endl;
    cout << "This should be zero\n" << VectorXd::Ones(nl_i).transpose()*A*dw_sol <<
         endl; // dw in null space
    cout << "if this is not zero, then w=0 is not optimal: " << dw_sol.transpose()*b
         << endl;
    cout << "Finished traj opt\n\n";

    // vector<double> w_sol_sort;
    // for(int i=0; i<dw_sol.size(); i++){
    //   w_sol_sort.push_back(dw_sol(i));
    // }
    // std::sort(w_sol_sort.begin(), w_sol_sort.end());
    // for(double w_sol_sort_ele : w_sol_sort)
    //   cout << w_sol_sort_ele << endl;

    // // Check if dw=0 violates any constraint
    // VectorXd QP_ub = ub-y;
    // VectorXd QP_lb = lb-y;
    // for(int i=0; i<nl_i; i++){
    //   if(QP_ub(i)<0)
    //     cout<< "row " << i << ": upper bound is smaller than 0 by " << QP_ub(i) << endl;
    //   if(QP_lb(i)>0)
    //     cout<< "row " << i << ": lower bound is larger than 0 by " << QP_lb(i) << endl;
    // }
    // cout << endl;
    // // Check if dw* to the QP violates any constraint
    // VectorXd QP_constraint_val = A*dw_sol;
    // for(int i=0; i<nl_i; i++){
    //   if(QP_ub(i) < QP_constraint_val(i))
    //     cout<< "row " << i << ": upper bound constraint violation by " << QP_constraint_val(i) - QP_ub(i) << endl;
    //   if(QP_lb(i) > QP_constraint_val(i))
    //     cout<< "row " << i << ": lower bound constraint violation by " << QP_constraint_val(i) - QP_lb(i) << endl;
    // }
    // cout << endl;




    // Plug back and check the cost and constraints of nonlinear programming
    double eps = 1e-1;
    unsigned int n_show = 10;  // number of rows of constraints you want to show
    // cost
    cout << "checking the cost of the original nonlinear programming and the approximated quadratic programming\n";
    for (int i = 0; i < 10 ; i++) {
      VectorXd w_sol_test = w_sol + i * eps * dw_sol;
      MatrixXd H2;
      VectorXd b2;
      double c_nonlinear;
      c_nonlinear = solvers::SecondOrderCost(
                      *gm_traj_opt.dircon.get(), w_sol_test, &H2, &b2) - c_double;
      cout << "i = " << i << endl;
      cout << "  c_nonlinear = " << c_nonlinear << endl;
      VectorXd dw_sol_test = i * eps * dw_sol;
      double c_aquadprog = 0.5 * dw_sol_test.transpose() * H * dw_sol_test + b.dot(
                             dw_sol_test) + c_double - c_double;
      cout << "  c_aquadprog = " << c_aquadprog << endl;
      cout << "  c_aquadprog - c_nonlinear = " << c_aquadprog - c_nonlinear << endl;
    }
    // constraint
    if (n_ae) {
      cout << "\nchecking the equality constraints of the original nonlinear programming and the approximated quadratic programming\n";
      // pick constraint violation row index
      std::vector<int> constraint_vio_row_idx;
      for (unsigned int i = 0; i < 10 ; i++) {
        VectorXd w_sol_test = w_sol + i * eps * dw_sol;
        MatrixXd A2;
        VectorXd y2, lb2, ub2;
        solvers::LinearizeConstraints(
          *gm_traj_opt.dircon.get(), w_sol_test, &y2, &A2, &lb2, &ub2);
        unsigned int k = 0;
        for (unsigned int j = 0; j < n_ae; j++) {
          double violation = y2(active_eq_row_idx[j]) - ub(active_eq_row_idx[j]);
          if (abs(violation) > 1e-8) {
            constraint_vio_row_idx.push_back(j);
            k++;
            if (k == n_show)
              break;
          }
          if (i == n_show - 1 && j == n_ae - 1 && k < n_show) {
            cout << "There are only " << k << " # of violations\n";
          }
        }
        if (constraint_vio_row_idx.size() >= n_show)
          break;
        else if (i != 10 - 1)
          constraint_vio_row_idx.clear();
      }
      cout << "  Row index of violation = ";
      for (int j : constraint_vio_row_idx) {
        cout << j << ", ";
      }
      cout << endl;
      // evaluate the chosen rows
      for (unsigned int i = 0; i < 10 ; i++) {
        cout << "i = " << i << endl;
        VectorXd w_sol_test = w_sol + i * eps * dw_sol;
        MatrixXd A2;
        VectorXd y2, lb2, ub2;
        solvers::LinearizeConstraints(
          *gm_traj_opt.dircon.get(), w_sol_test, &y2, &A2, &lb2, &ub2);
        cout << "  nonlinear_constraint_val = ";
        for (int j : constraint_vio_row_idx) {
          double violation = y2(active_eq_row_idx[j]) - ub(active_eq_row_idx[j]);
          cout << violation << ", ";
        }
        cout << endl;
      }
    }
    if (n_aub) {
      cout << "\nchecking the inequality constraints (active upper bound) of the original nonlinear programming and the approximated quadratic programming\n";
      for (unsigned int i = 0; i < 10 ; i++) {
        VectorXd w_sol_test = w_sol + i * eps * dw_sol;
        MatrixXd A2;
        VectorXd y2, lb2, ub2;
        solvers::LinearizeConstraints(
          *gm_traj_opt.dircon.get(), w_sol_test, &y2, &A2, &lb2, &ub2);
        VectorXd nonlinear_constraint_val = VectorXd::Zero(n_show);
        unsigned int k = 0;
        for (unsigned int j = 0; j < n_aub; j++) {
          double violation =
            y2(ub_active_ineq_row_idx[j]) - ub(ub_active_ineq_row_idx[j]);
          if (violation > 1e-8) {
            nonlinear_constraint_val(k) = violation;
            k++;
            if (k == n_show)
              break;
          }
          if (j == n_aub - 1 && k < n_show) {
            cout << "There are only " << k << " # of violations\n";
          }
        }
        cout << "  nonlinear_constraint_val = "
             << nonlinear_constraint_val.transpose() << endl;
      }
    }
    if (n_alb) {
      cout << "\nchecking the inequality constraints (active lower bound) of the original nonlinear programming and the approximated quadratic programming\n";
      for (unsigned int i = 0; i < 10 ; i++) {
        VectorXd w_sol_test = w_sol + i * eps * dw_sol;
        MatrixXd A2;
        VectorXd y2, lb2, ub2;
        solvers::LinearizeConstraints(
          *gm_traj_opt.dircon.get(), w_sol_test, &y2, &A2, &lb2, &ub2);
        VectorXd nonlinear_constraint_val = VectorXd::Zero(n_show);
        unsigned int k = 0;
        for (unsigned int j = 0; j < n_alb; j++) {
          double violation =
            y2(lb_active_ineq_row_idx[j]) - lb(lb_active_ineq_row_idx[j]);
          if (violation < - 1e-8) {
            nonlinear_constraint_val(k) = violation;
            k++;
            if (k == n_show)
              break;
          }
          if (j == n_alb - 1 && k < n_show) {
            cout << "There are only " << k << " # of violations\n";
          }
        }
        cout << "  nonlinear_constraint_val = "
             << nonlinear_constraint_val.transpose() << endl;
      }
    }*/
  }  // end of if(!is_get_nominal)
}

void fiveLinkRobotTrajOpt(const MultibodyPlant<double> & plant,
                          const MultibodyPlant<AutoDiffXd> & plant_autoDiff,
                          int n_s, int n_sDDot, int n_tau,
                          int n_feature_s,
                          int n_feature_sDDot,
                          const MatrixXd& B_tau,
                          const VectorXd & theta_s, const VectorXd & theta_sDDot,
                          double stride_length, double ground_incline,
                          double duration, int n_node, int max_iter,
                          const string& directory,
                          const string& init_file, const string& prefix,
                          double Q_double, double R_double,
                          double all_cost_scale,
                          double eps_reg,
                          bool is_get_nominal,
                          bool is_zero_touchdown_impact,
                          bool extend_model,
                          bool is_add_tau_in_cost,
                          int sample_idx, int n_rerun,
                          double cost_threshold_for_update, int N_rerun,
                          int rom_option,
                          int robot_option) {
  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant);
  map<string, int> input_map = multibody::makeNameToActuatorsMap(plant);
  // for (auto const& element : pos_map)
  //   cout << element.first << " = " << element.second << endl;
  // std::cout << "\n";
  // for (auto const& element : vel_map)
  //   cout << element.first << " = " << element.second << endl;
  // std::cout << "\n";
  // for (auto const& element : input_map)
  //   cout << element.first << " = " << element.second << endl;
  // std::cout << "\n";

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_x = n_q + n_v;
  int n_u = plant.num_actuators();
  // std::cout<<"n_x = "<<n_x<<"\n";
  // std::cout<<"n_u = "<<n_u<<"\n";

  const Body<double>& left_lower_leg = plant.GetBodyByName("left_lower_leg");
  const Body<double>& right_lower_leg = plant.GetBodyByName("right_lower_leg");

  Vector3d pt;
  pt << 0, 0, -.5;
  bool isXZ = true;
  Vector3d ground_normal(sin(ground_incline), 0, cos(ground_incline));
  auto leftFootConstraint = DirconPositionData<double>(plant, left_lower_leg,
                                                       pt, isXZ, ground_normal);
  auto rightFootConstraint = DirconPositionData<double>(
      plant, right_lower_leg, pt, isXZ, ground_normal);

  double mu = 1;
  leftFootConstraint.addFixedNormalFrictionConstraints(mu);
  rightFootConstraint.addFixedNormalFrictionConstraints(mu);

  std::vector<DirconKinematicData<double>*> leftConstraints;
  leftConstraints.push_back(&leftFootConstraint);
  auto ls_dataset = DirconKinematicDataSet<double>(plant, &leftConstraints);

  std::vector<DirconKinematicData<double>*> rightConstraints;
  rightConstraints.push_back(&rightFootConstraint);
  auto rs_dataset = DirconKinematicDataSet<double>(plant, &rightConstraints);

  auto leftOptions = DirconOptions(ls_dataset.countConstraints(), plant);
  leftOptions.setConstraintRelative(0, true);
  auto rightOptions = DirconOptions(rs_dataset.countConstraints(), plant);
  rightOptions.setConstraintRelative(0, true);

  std::vector<DirconOptions> options_list;
  options_list.push_back(leftOptions);
  options_list.push_back(rightOptions);

  // Constraint scaling
  for (int i = 0; i < 2; i++) {
    // before adding rom constraint
    /*// Dynamic constraints
    options_list[i].setDynConstraintScaling(1.0 / 40.0, 0, 4);
    options_list[i].setDynConstraintScaling(1.0 / 60.0, 5, 5);
    options_list[i].setDynConstraintScaling(1.0 / 40.0, 6, 6);
    options_list[i].setDynConstraintScaling(1.0 / 60.0, 7, 7);
    options_list[i].setDynConstraintScaling(1.0 / 40.0, 8, 8);
    options_list[i].setDynConstraintScaling(1.0 / 100.0, 9, 9);
    options_list[i].setDynConstraintScaling(1.0 / 1000.0, 10, 10);
    options_list[i].setDynConstraintScaling(1.0 / 300.0, 11, 11);
    options_list[i].setDynConstraintScaling(1.0 / 3000.0, 12, 12);
    options_list[i].setDynConstraintScaling(1.0 / 400.0, 13, 13);
    // Kinematic constraints
    options_list[i].setKinConstraintScaling(1.0 / 1000.0, 0, 0);
    options_list[i].setKinConstraintScaling(1.0 / 50.0, 1, 1);
    // Impact constraints
    options_list[i].setImpConstraintScaling(1.0 / 20.0, 0, 1);
    options_list[i].setImpConstraintScaling(1.0 / 5.0, 2, 2);
    options_list[i].setImpConstraintScaling(1.0 / 3.0, 3, 3);
    options_list[i].setImpConstraintScaling(1.0 / 5.0, 4, 4);
    options_list[i].setImpConstraintScaling(1.0 / 1.0, 5, 5);
    options_list[i].setImpConstraintScaling(1.0 / 3.0, 6, 6);*/

    // after adding rom constraint
    // Dynamic constraints
    options_list[i].setDynConstraintScaling(1.0 / 40.0, 0, 3);
    options_list[i].setDynConstraintScaling(1.0 / 60.0, 4, 5);
    options_list[i].setDynConstraintScaling(1.0 / 200.0, 6, 6); // end of pos
    options_list[i].setDynConstraintScaling(1.0 / 180.0, 7, 7);
    options_list[i].setDynConstraintScaling(1.0 / 120.0, 8, 8);
    options_list[i].setDynConstraintScaling(1.0 / 300.0, 9, 9);
    options_list[i].setDynConstraintScaling(1.0 / 1000.0, 10, 10);
    options_list[i].setDynConstraintScaling(1.0 / 1500.0, 11, 11);
    options_list[i].setDynConstraintScaling(1.0 / 3000.0, 12, 12);
    options_list[i].setDynConstraintScaling(1.0 / 2800.0, 13, 13);
    // Kinematic constraints
    options_list[i].setKinConstraintScaling(1.0 / 1000.0, 0, 0);
    options_list[i].setKinConstraintScaling(1.0 / 25.0, 1, 1);
    // Impact constraints
    options_list[i].setImpConstraintScaling(1.0 / 20.0, 0, 1);
    options_list[i].setImpConstraintScaling(1.0 / 5.0, 2, 2);
    options_list[i].setImpConstraintScaling(1.0 / 3.0, 3, 3);
    options_list[i].setImpConstraintScaling(1.8 / 5.0, 4, 4);
    options_list[i].setImpConstraintScaling(1.8 / 1.0, 5, 5);
    options_list[i].setImpConstraintScaling(1.8 / 3.0, 6, 6);
  }

  // Stated in the MultipleShooting class:
  // This class assumes that there are a fixed number (N) time steps/samples,
  // and that the trajectory is discretized into timesteps h (N-1 of these),
  // state x (N of these), and control input u (N of these).
  std::vector<int> num_time_samples;
  num_time_samples.push_back(n_node);
  num_time_samples.push_back(1);
  std::vector<double> min_dt;
  min_dt.push_back(.01);
  min_dt.push_back(.01);
  std::vector<double> max_dt;
  max_dt.push_back(.3);
  max_dt.push_back(.3);

  int N = 0;
  for (int num_time_sample : num_time_samples)
    N += num_time_sample;
  N -= num_time_samples.size() - 1; //Overlaps between modes
  // std::cout<<"N = "<<N<<"\n";

  std::vector<DirconKinematicDataSet<double>*> dataset_list;
  dataset_list.push_back(&ls_dataset);
  dataset_list.push_back(&rs_dataset);

  auto trajopt = std::make_unique<HybridDircon<double>>(plant,
                 num_time_samples, min_dt, max_dt, dataset_list, options_list);

  // You can comment this out to not put any constraint on the time
  // However, we need it now, since we add the running cost by hand
  trajopt->AddDurationBounds(duration, duration);

//  cout << "WARNING: you are printing snopt log.\n";
//  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
//                           "../snopt_sample#"+to_string(sample_idx)+".out");
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", max_iter);
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Iterations limit", 100000);  // QP subproblems
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                           0);
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Scale option",
                           0);  // 0 // snopt doc said try 2 if seeing snopta exit 40
  // trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
  //                          "Major optimality tolerance", 1e-5);  // target nonlinear constraint violation
  // trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
  //                          "Major feasibility tolerance", 1e-5);  // target complementarity gap

  // Periodicity constraints
  auto x0 = trajopt->initial_state();
  // auto xf = trajopt->final_state();
  auto xf = trajopt->state_vars_by_mode(
      num_time_samples.size() - 1,
      num_time_samples[num_time_samples.size() - 1] - 1);

  // trajopt->AddLinearConstraint(x0(pos_map.at("planar_z")) == xf(
  //                                pos_map.at("planar_z")));
  trajopt->AddLinearConstraint(x0(pos_map.at("planar_roty")) == xf(
                                 pos_map.at("planar_roty")));
  trajopt->AddLinearConstraint(x0(pos_map.at("left_hip_pin")) == xf(
                                 pos_map.at("right_hip_pin")));
  trajopt->AddLinearConstraint(x0(pos_map.at("left_knee_pin")) == xf(
                                 pos_map.at("right_knee_pin")));
  trajopt->AddLinearConstraint(x0(pos_map.at("right_hip_pin")) == xf(
                                 pos_map.at("left_hip_pin")));
  trajopt->AddLinearConstraint(x0(pos_map.at("right_knee_pin")) == xf(
                                 pos_map.at("left_knee_pin")));

  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("planar_xdot"))
                               == xf(n_q + vel_map.at("planar_xdot")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("planar_zdot"))
                               == xf(n_q + vel_map.at("planar_zdot")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("planar_rotydot"))
                               == xf(n_q + vel_map.at("planar_rotydot")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("left_hip_pindot"))
                               == xf(n_q + vel_map.at("right_hip_pindot")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("left_knee_pindot"))
                               == xf(n_q + vel_map.at("right_knee_pindot")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("right_hip_pindot"))
                               == xf(n_q + vel_map.at("left_hip_pindot")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("right_knee_pindot"))
                               == xf(n_q + vel_map.at("left_knee_pindot")));

  // u periodic constraint
  auto u0 = trajopt->input(0);
  auto uf = trajopt->input(N - 1);
  trajopt->AddLinearConstraint(u0(input_map.at("left_hip_torque")) == uf(
                                 input_map.at("right_hip_torque")));
  trajopt->AddLinearConstraint(u0(input_map.at("right_hip_torque")) == uf(
                                 input_map.at("left_hip_torque")));
  trajopt->AddLinearConstraint(u0(input_map.at("left_knee_torque")) == uf(
                                 input_map.at("right_knee_torque")));
  trajopt->AddLinearConstraint(u0(input_map.at("right_knee_torque")) == uf(
                                 input_map.at("left_knee_torque")));

  // Knee joint limits
  auto x = trajopt->state();
  trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("left_knee_pin")) >=
                                        5.0 / 180.0 * M_PI);
  trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("right_knee_pin")) >=
                                        5.0 / 180.0 * M_PI);
  trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("left_knee_pin")) <=
                                        M_PI / 2.0);
  trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("right_knee_pin")) <=
                                        M_PI / 2.0);

  // hip joint limits
  trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("left_hip_pin")) >=
                                        -M_PI / 2.0);
  trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("right_hip_pin")) >=
                                        -M_PI / 2.0);
  trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("left_hip_pin")) <=
                                        M_PI / 2.0);
  trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("right_hip_pin")) <=
                                        M_PI / 2.0);

  // x-distance constraint constraints
  trajopt->AddLinearConstraint(x0(pos_map.at("planar_x")) == 0);
  trajopt->AddLinearConstraint(xf(pos_map.at("planar_x")) ==
                               stride_length);

  // make sure it's left stance
  trajopt->AddLinearConstraint(x0(pos_map.at("left_hip_pin")) <=
                               x0(pos_map.at("right_hip_pin")));

  // testing -- swing foot height constraint
  Vector3d z_hat(0, 0, 1);
  Eigen::Quaterniond q;
  q.setFromTwoVectors(z_hat, ground_normal);
  Eigen::Matrix3d T_ground_incline = q.matrix().transpose();
  auto right_foot_constraint_z = std::make_shared<OneDimBodyPosConstraint>(
      &plant, "right_lower_leg", pt, T_ground_incline, 2,
      0 , std::numeric_limits<double>::infinity());
  for (int index = 1; index < num_time_samples[0] - 1; index++) {
    auto x_i = trajopt->state(index);
    trajopt->AddConstraint(right_foot_constraint_z, x_i.head(n_q));
  }

  // Scale decision variable
  // time
  trajopt->ScaleTimeVariables(0.04);
  // state
//  trajopt->ScaleStateVariables(10, n_q, n_q + n_v - 1);
  // input
//  trajopt->ScaleInputVariables(10, 0, 3);
  // force
  trajopt->ScaleForceVariables(
      50, 0, 0, ls_dataset.countConstraintsWithoutSkipping() - 1);
  trajopt->ScaleForceVariables(
      50, 1, 0, rs_dataset.countConstraintsWithoutSkipping() - 1);
  // impulse
//  trajopt->ScaleImpulseVariables(
//      10, 0, 0, rs_dataset.countConstraintsWithoutSkipping() - 1);  // 0.1
  // Constraint slack
//  trajopt->ScaleKinConstraintSlackVariables(50, 0, 0, 5);
//  trajopt->ScaleKinConstraintSlackVariables(500, 0, 6, 7);

  // Add cost
  MatrixXd R = R_double * MatrixXd::Identity(n_u, n_u);
  MatrixXd Q = Q_double * MatrixXd::Identity(n_v, n_v);
  // Don't use AddRunningCost, cause it makes cost Hessian to be indefinite
  // I'll fix the timestep and add cost myself
  /*auto u = trajopt->input();
  trajopt->AddRunningCost(u.transpose()*R * u);
  trajopt->AddRunningCost(x.transpose()*Q * x);*/
  // Add running cost by hand (Trapezoidal integration):
  double timestep = duration / (N - 1);
  trajopt->AddQuadraticCost(Q * timestep / 2, VectorXd::Zero(n_v), x0.tail(n_v));
  trajopt->AddQuadraticCost(R * timestep / 2, VectorXd::Zero(n_u), u0);
  for (int i = 1; i <= N - 2; i++) {
    auto ui = trajopt->input(i);
    auto xi = trajopt->state(i);
    trajopt->AddQuadraticCost(Q * timestep, VectorXd::Zero(n_v), xi.tail(n_v));
    trajopt->AddQuadraticCost(R * timestep, VectorXd::Zero(n_u), ui);
  }
  trajopt->AddQuadraticCost(Q * timestep / 2, VectorXd::Zero(n_v), xf.tail(n_v));
  trajopt->AddQuadraticCost(R * timestep / 2, VectorXd::Zero(n_u), uf);
  // Add regularization term here so that hessian is pd (for outer loop), so
  // that we can use schur complement method
  // Actually, Hessian still has zero eigen value because of h_var and z_var
  /*MatrixXd I_x = MatrixXd::Identity(n_x, n_x);
  trajopt->AddQuadraticCost(eps_reg*I_x*timestep/2,VectorXd::Zero(n_x),x0);
  for(int i=1; i<=N-2; i++){
    auto xi = trajopt->state(i);
    trajopt->AddQuadraticCost(eps_reg*I_x*timestep,VectorXd::Zero(n_x),xi);
  }
  trajopt->AddQuadraticCost(eps_reg*I_x*timestep/2,VectorXd::Zero(n_x),xf);*/

  // Zero impact at touchdown
  if (is_zero_touchdown_impact)
    trajopt->AddLinearConstraint(MatrixXd::Ones(1, 1),
                                 VectorXd::Zero(1),
                                 VectorXd::Zero(1),
                                 trajopt->impulse_vars(0).tail(1));

  // Move the trajectory optmization problem into GoldilocksModelTrajOpt
  // where we add the constraints for reduced order model
  GoldilocksModelTrajOpt gm_traj_opt(
    n_s, n_sDDot, n_tau, n_feature_s, n_feature_sDDot,
    B_tau, theta_s, theta_sDDot,
    std::move(trajopt), &plant_autoDiff, &plant, num_time_samples,
    is_get_nominal, is_add_tau_in_cost, rom_option, robot_option);

  addRegularization(is_get_nominal, eps_reg, gm_traj_opt);

  // initial guess if the file exists
  if (!init_file.empty()) {
    setInitialGuessFromFile(directory, init_file, gm_traj_opt);
  } else {
    // Add random initial guess first (the seed for RNG is fixed)
    gm_traj_opt.dircon->SetInitialGuessForAllVariables(
        VectorXd::Random(gm_traj_opt.dircon->decision_variables().size()));
  }

  // Testing
  // cout << "Choose the best solver: " <<
  //      drake::solvers::ChooseBestSolver(*(gm_traj_opt.dircon)).name() << endl;

  // cout << "Solving DIRCON (based on MultipleShooting)\n";
  auto start = std::chrono::high_resolution_clock::now();
  const MathematicalProgramResult result = Solve(
        *gm_traj_opt.dircon, gm_traj_opt.dircon->initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;

  bool is_print_for_debugging = false;
  VectorXd w_sol;
  extractResult(w_sol, gm_traj_opt, result, elapsed,
                num_time_samples, N,
                plant, plant_autoDiff,
                n_s, n_sDDot, n_tau, n_feature_s, n_feature_sDDot, B_tau,
                theta_s, theta_sDDot,
                stride_length, ground_incline,
                duration, max_iter,
                directory, init_file, prefix,
                Q_double, R_double, eps_reg,
                is_get_nominal, is_zero_touchdown_impact,
                extend_model, is_add_tau_in_cost,
                sample_idx, n_rerun, cost_threshold_for_update, N_rerun,
                rom_option,
                robot_option,
                dataset_list, is_print_for_debugging);
  postProcessing(w_sol, gm_traj_opt, result, elapsed,
                 num_time_samples, N,
                 plant, plant_autoDiff,
                 n_s, n_sDDot, n_tau, n_feature_s, n_feature_sDDot, B_tau,
                 theta_s, theta_sDDot,
                 stride_length, ground_incline,
                 duration, max_iter,
                 directory, init_file, prefix,
                 Q_double, R_double, eps_reg,
                 is_get_nominal, is_zero_touchdown_impact,
                 extend_model, is_add_tau_in_cost,
                 sample_idx, n_rerun, cost_threshold_for_update, N_rerun,
                 rom_option,
                 robot_option);
}


void cassieTrajOpt(const MultibodyPlant<double> & plant,
                   const MultibodyPlant<AutoDiffXd> & plant_autoDiff,
                   int n_s, int n_sDDot, int n_tau,
                   int n_feature_s,
                   int n_feature_sDDot,
                   const MatrixXd& B_tau,
                   const VectorXd & theta_s, const VectorXd & theta_sDDot,
                   double stride_length, double ground_incline,
                   double duration, int n_node, int max_iter,
                   double major_optimality_tol,
                   double major_feasibility_tol,
                   const string& directory,
                   const string& init_file, const string& prefix,
                   double Q_double, double R_double, double all_cost_scale,
                   double eps_reg,
                   bool is_get_nominal,
                   bool is_zero_touchdown_impact,
                   bool extend_model,
                   bool is_add_tau_in_cost,
                   int sample_idx, int n_rerun,
                   double cost_threshold_for_update, int N_rerun,
                   int rom_option, int robot_option) {
  // Dircon parameter
  double minimum_timestep = 0.01;
  DRAKE_DEMAND(duration / (n_node - 1) >= minimum_timestep);

  // Walking modes
  int walking_mode = 0; // 0: instant change of support
  // 1: single double single
  // 2: heel to toe
  DRAKE_DEMAND(walking_mode == 0);
  // If you want to use walking_mode 1 or 2, then you should either
  // 1. finish multi-phase ROM, or
  // 2. write the code here so it takes care of multi-mode. (e.g. only have rom
  //    in the first phase, etc)

  // Cost on velocity and input
  double w_Q = Q_double * all_cost_scale;
  double w_Q_vy = w_Q * 10000 * all_cost_scale;  // prevent the pelvis from rocking in y
  double w_Q_vz = w_Q * 10000 * all_cost_scale;  // prevent the pelvis from rocking in z
  double w_Q_swing_toe = w_Q * 1 * all_cost_scale;  // prevent the swing toe from shaking
  double w_R = R_double * all_cost_scale;
  double w_R_swing_toe = w_R * 1 * all_cost_scale;  // prevent the swing toe from shaking
  // Cost on force (the final weight is w_lambda^2)
  double w_lambda = 1.0e-4 * all_cost_scale * all_cost_scale;
  // Cost on difference over time
  double w_lambda_diff = 0.000001 * 0.1 * all_cost_scale;
  double w_v_diff = 0.01 * 5 * 0.1 * all_cost_scale;
  double w_u_diff = 0.00001 * 0.1 * all_cost_scale;
  // Cost on position
  double w_q_hip_roll = 1 * 5 * 10 * all_cost_scale;
  double w_q_hip_yaw = 1 * 5 * all_cost_scale;
  double w_q_quat_xyz = 1 * 5 * 10 * all_cost_scale;

  // Optional constraints
  // This seems to be important at higher walking speeds
  bool constrain_stance_leg_fourbar_force = false;

  // Temporary solution for get rid of redundant four bar constraint in the
  // second mode (since we have periodic constraints on state and input, it's
  // fine to just ignore the four bar constraint (position, velocity and
  // acceleration levels))
  bool four_bar_in_right_support = false;

  // Create ground normal for the problem
  Vector3d ground_normal(sin(ground_incline), 0, cos(ground_incline));

  // Create maps for joints
  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant);

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();
  // int n_x = n_q + n_v;
  // cout << "n_q = " << n_q << "\n";
  // cout << "n_v = " << n_v << "\n";
  // cout << "n_x = " << n_x << "\n";
  // cout << "n_u = " << n_u << "\n";
  // cout << "floating_positions_start = " <<
  //      plant.GetBodyByName("pelvis").floating_positions_start() << endl;
  // cout << "floating_velocities_start = " <<
  //      plant.GetBodyByName("pelvis").floating_velocities_start() << endl;

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
    for (const auto & asy_joint_name : asy_joint_names) {
      joint_names.push_back(asy_joint_name + l_r_pair.first);
      motor_names.push_back(asy_joint_name + l_r_pair.first + "_motor");
    }
    for (unsigned int i = 0; i < sym_joint_names.size(); i++) {
      joint_names.push_back(sym_joint_names[i] + l_r_pair.first);
      if (sym_joint_names[i].compare("ankle_joint") != 0) {
        motor_names.push_back(sym_joint_names[i] + l_r_pair.first + "_motor");
      }
    }
  }


  // Set up contact/distance constraints
  const Body<double>& toe_left = plant.GetBodyByName("toe_left");
  const Body<double>& toe_right = plant.GetBodyByName("toe_right");
  Vector3d pt_front_contact(-0.0457, 0.112, 0);
  Vector3d pt_rear_contact(0.088, 0, 0);
  bool isXZ = false;
  auto left_toe_front_constraint = DirconPositionData<double>(
      plant, toe_left, pt_front_contact, isXZ, ground_normal);
  auto left_toe_rear_constraint = DirconPositionData<double>(
      plant, toe_left, pt_rear_contact, isXZ, ground_normal);
  auto right_toe_front_constraint = DirconPositionData<double>(
      plant, toe_right, pt_front_contact, isXZ, ground_normal);
  auto right_toe_rear_constraint = DirconPositionData<double>(
      plant, toe_right, pt_rear_contact, isXZ, ground_normal);
  double mu = 1;
  left_toe_front_constraint.addFixedNormalFrictionConstraints(mu);
  left_toe_rear_constraint.addFixedNormalFrictionConstraints(mu);
  right_toe_front_constraint.addFixedNormalFrictionConstraints(mu);
  right_toe_rear_constraint.addFixedNormalFrictionConstraints(mu);

  const auto& thigh_left = plant.GetBodyByName("thigh_left");
  const auto& heel_spring_left = plant.GetBodyByName("heel_spring_left");
  const auto& thigh_right = plant.GetBodyByName("thigh_right");
  const auto& heel_spring_right = plant.GetBodyByName("heel_spring_right");
  double rod_length = 0.5012;  // from cassie_utils
  Vector3d pt_on_heel_spring = Vector3d(.11877, -.01, 0.0);
  Vector3d pt_on_thigh_left = Vector3d(0.0, 0.0, 0.045);
  Vector3d pt_on_thigh_right = Vector3d(0.0, 0.0, -0.045);
  auto distance_constraint_left = DirconDistanceData<double>(
      plant, thigh_left, pt_on_thigh_left, heel_spring_left, pt_on_heel_spring,
      rod_length);
  auto distance_constraint_right = DirconDistanceData<double>(
      plant, thigh_right, pt_on_thigh_right, heel_spring_right,
      pt_on_heel_spring, rod_length);

  // get rid of redundant constraint
  vector<int> skip_constraint_inds;
  skip_constraint_inds.push_back(3);

  // Left support
  vector<DirconKinematicData<double>*> ls_constraint;
  ls_constraint.push_back(&left_toe_front_constraint);
  ls_constraint.push_back(&left_toe_rear_constraint);
  ls_constraint.push_back(&distance_constraint_left);
  ls_constraint.push_back(&distance_constraint_right);
  auto ls_dataset = DirconKinematicDataSet<double>(plant, &ls_constraint,
                                                   skip_constraint_inds);
  // Right support
  vector<DirconKinematicData<double>*> rs_constraint;
  rs_constraint.push_back(&right_toe_front_constraint);
  rs_constraint.push_back(&right_toe_rear_constraint);
  if (four_bar_in_right_support) {
    rs_constraint.push_back(&distance_constraint_left);
    rs_constraint.push_back(&distance_constraint_right);
  }
  auto rs_dataset = DirconKinematicDataSet<double>(plant, &rs_constraint,
                                                   skip_constraint_inds);

  // Set up options
  std::vector<DirconOptions> options_list;
  options_list.push_back(DirconOptions(ls_dataset.countConstraints(), plant));
  options_list.push_back(DirconOptions(rs_dataset.countConstraints(), plant));

  // set force cost weight
  for (int i = 0; i < 2; i++) {
    options_list[i].setForceCost(w_lambda);
  }

  // Be careful in setting relative constraint, because we skip constraints
  ///                 || lf/rf | lr/rr | fourbar
  /// Before skipping || 0 1 2 | 3 4 5 | 6 7
  /// After skipping  || 0 1 2 |   3 4 | 5 6
  for (int i = 0; i < 2; i++) {
    options_list[i].setConstraintRelative(0, true);
    options_list[i].setConstraintRelative(1, true);
    options_list[i].setConstraintRelative(3, true);
  }
  // Constraint scaling
  for (int i = 0; i < 2; i++) {
    double s = 1;  // scale everything together

    if (is_get_nominal) {
      // old constraint scaling (from traj opt of cassie, without rom constraint)
      // Dynamic constraints
      options_list[i].setDynConstraintScaling(s * 1.0 / 30.0, 0, 3);
      options_list[i].setDynConstraintScaling(s * 1.0 / 60.0, 4, 16);
      options_list[i].setDynConstraintScaling(s * 1.0 / 300.0, 17, 18);
      options_list[i].setDynConstraintScaling(s * 1.0 / 600.0, 19, 28);
      options_list[i].setDynConstraintScaling(s * 1.0 / 3000.0, 29, 34);
      options_list[i].setDynConstraintScaling(s * 1.0 / 60000.0, 35, 36);
      // Kinematic constraints
      int n_l = options_list[i].getNumConstraints();
      options_list[i].setKinConstraintScaling(s * 1.0 / 6000.0, 0, 4);
      options_list[i].setKinConstraintScaling(s * 1.0 / 10.0, n_l + 0, n_l + 4);
      options_list[i].setKinConstraintScaling(s * 1.0, 2 * n_l + 0, 2 * n_l + 4);
      if (i == 0 || four_bar_in_right_support) {
        options_list[i].setKinConstraintScaling(s * 1.0 / 600.0 * 2, 5, 6);
        options_list[i].setKinConstraintScaling(s * 1.0, n_l + 5, n_l + 6);
        options_list[i].setKinConstraintScaling(s * 1.0 * 20, 2 * n_l + 5,
                                                2 * n_l + 6);
      }
      // Impact constraints
      options_list[i].setImpConstraintScaling(s * 1.0 / 50.0, 0, 2);
      options_list[i].setImpConstraintScaling(s * 1.0 / 300.0, 3, 5);
      options_list[i].setImpConstraintScaling(s * 1.0 / 24.0, 6, 7);
      options_list[i].setImpConstraintScaling(s * 1.0 / 6.0, 8, 9);
      options_list[i].setImpConstraintScaling(s * 1.0 / 12.0, 10, 13);
      options_list[i].setImpConstraintScaling(s * 1.0 / 2.0, 14, 15);
      options_list[i].setImpConstraintScaling(s * 1.0, 16, n_v - 1);
    } else {
      // new constraint scaling (20200227; after adding rom constraint)
      // Dynamic constraints
      options_list[i].setDynConstraintScaling(s * 1.0 / 30.0, 0, 3);
      options_list[i].setDynConstraintScaling(s * 1.0 / 60.0, 4, 16);
      options_list[i].setDynConstraintScaling(s * 1.0 / 300.0, 17, 18);
      options_list[i].setDynConstraintScaling(s * 1.0 / 600.0, 19, 26);
      options_list[i].setDynConstraintScaling(s * 1.0 / 3000.0, 27, 28);
      options_list[i].setDynConstraintScaling(s * 1.0 / 3000.0, 29, 34);
      options_list[i].setDynConstraintScaling(s * 1.0 / 60000.0, 35, 36);
      // Kinematic constraints
      int n_l = options_list[i].getNumConstraints();
      options_list[i].setKinConstraintScaling(s * 1.0 / 600.0, 0, 4);
      options_list[i].setKinConstraintScaling(s * 1.0 / 1.0, n_l + 0, n_l + 4);
      options_list[i].setKinConstraintScaling(s * 1.0, 2 * n_l + 0, 2 * n_l + 4);
      if (i == 0 || four_bar_in_right_support) {
        options_list[i].setKinConstraintScaling(s * 1.0 / 60.0 * 2, 5, 6);
        options_list[i].setKinConstraintScaling(s * 10.0, n_l + 5, n_l + 6);
        options_list[i].setKinConstraintScaling(s * 1.0 * 20, 2 * n_l + 5,
                                                2 * n_l + 6);
      }
      // Impact constraints
      options_list[i].setImpConstraintScaling(s * 1.0 / 5.0, 0, 2);
      options_list[i].setImpConstraintScaling(s * 1.0 / 30.0, 3, 5);
      options_list[i].setImpConstraintScaling(s * 10.0 / 24.0, 6, 7);
      options_list[i].setImpConstraintScaling(s * 10.0 / 6.0, 8, 9);
      options_list[i].setImpConstraintScaling(s * 10.0 / 12.0, 10, 10);
      options_list[i].setImpConstraintScaling(s * 1.0 / 12.0, 11, 11);
      options_list[i].setImpConstraintScaling(s * 10.0 / 12.0, 12, 12);
      options_list[i].setImpConstraintScaling(s * 1.0 / 12.0, 13, 13);
      options_list[i].setImpConstraintScaling(s * 10.0 / 2.0, 14, 14);
      options_list[i].setImpConstraintScaling(s * 1.0 / 2.0, 15, 15);
      options_list[i].setImpConstraintScaling(s * 50.0, 16, 16);
      options_list[i].setImpConstraintScaling(s * 1.0, n_v - 1, n_v - 1);
    }
  }

  // timesteps and modes setting
  vector<double> min_dt;
  vector<double> max_dt;
  min_dt.push_back(minimum_timestep);
  min_dt.push_back(minimum_timestep);
  max_dt.push_back(.3);
  max_dt.push_back(.3);
  vector<int> num_time_samples;
  num_time_samples.push_back(n_node);
  num_time_samples.push_back(1);
  vector<DirconKinematicDataSet<double>*> dataset_list;
  dataset_list.push_back(&ls_dataset);
  dataset_list.push_back(&rs_dataset);

  auto trajopt = std::make_unique<HybridDircon<double>>(
      plant, num_time_samples, min_dt, max_dt, dataset_list, options_list);

  // Snopt settings
//  cout << "WARNING: you are printing snopt log.\n";
//  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
//                           "../snopt_sample#"+to_string(sample_idx)+".out");
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", max_iter);
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Iterations limit", 100000);  // QP subproblems
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                           0);  // 0
  trajopt->SetSolverOption(
      drake::solvers::SnoptSolver::id(), "Scale option",
      0);  // snopt doc said try 2 if seeing snopta exit 40
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major optimality tolerance",
                           major_optimality_tol);  // target nonlinear constraint violation
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major feasibility tolerance",
                           major_feasibility_tol);  // target complementarity gap

  int N = 0;
  for (uint i = 0; i < num_time_samples.size(); i++) N += num_time_samples[i];
  N -= num_time_samples.size() - 1;  // because of overlaps between modes

  // Get the decision variables that will be used
  auto u = trajopt->input();
  auto x = trajopt->state();
  auto u0 = trajopt->input(0);
  auto uf = trajopt->input(N - 1);
  auto x0 = trajopt->initial_state();
  auto xf = trajopt->state_vars_by_mode(
      num_time_samples.size() - 1,
      num_time_samples[num_time_samples.size() - 1] - 1);
  auto x_mid = trajopt->state(int(num_time_samples[0] / 2));

  // Fix time duration
  trajopt->AddDurationBounds(duration, duration);

  // x position constraint
  trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_x")));
  trajopt->AddBoundingBoxConstraint(stride_length, stride_length,
                                    xf(pos_map.at("base_x")));

  // testing(initial floating base)
  // trajopt->AddLinearConstraint(x0(pos_map.at("base_z")) == 1);
  // trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_vz")) == 0);

  // Floating base periodicity
  trajopt->AddLinearConstraint(x0(pos_map.at("base_qw")) ==
      xf(pos_map.at("base_qw")));
  trajopt->AddLinearConstraint(x0(pos_map.at("base_qx")) ==
      -xf(pos_map.at("base_qx")));
  trajopt->AddLinearConstraint(x0(pos_map.at("base_qy")) ==
      xf(pos_map.at("base_qy")));
  trajopt->AddLinearConstraint(x0(pos_map.at("base_qz")) ==
      -xf(pos_map.at("base_qz")));
  trajopt->AddLinearConstraint(x0(pos_map.at("base_y")) ==
      -xf(pos_map.at("base_y")));
  if (ground_incline == 0) {
    trajopt->AddLinearConstraint(x0(pos_map.at("base_z")) ==
        xf(pos_map.at("base_z")));
  }
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_wx")) ==
      xf(n_q + vel_map.at("base_wx")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_wy")) ==
      -xf(n_q + vel_map.at("base_wy")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_wz")) ==
      xf(n_q + vel_map.at("base_wz")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_vx")) ==
      xf(n_q + vel_map.at("base_vx")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_vy")) ==
      -xf(n_q + vel_map.at("base_vy")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_vz")) ==
      xf(n_q + vel_map.at("base_vz")));

  // The legs joint positions/velocities/torque should be mirrored between legs
  // (notice that hip yaw and roll should be asymmetric instead of symmetric.)
  for (const auto& l_r_pair : l_r_pairs) {
    for (const auto & asy_joint_name : asy_joint_names) {
      // positions
      trajopt->AddLinearConstraint(
          x0(pos_map.at(asy_joint_name + l_r_pair.first)) ==
              -xf(pos_map.at(asy_joint_name + l_r_pair.second)));
      // velocities
      trajopt->AddLinearConstraint(
          x0(n_q + vel_map.at(asy_joint_name + l_r_pair.first + "dot")) ==
              -xf(n_q + vel_map.at(asy_joint_name + l_r_pair.second + "dot")));
      // inputs
      trajopt->AddLinearConstraint(
          u0(act_map.at(asy_joint_name + l_r_pair.first + "_motor")) ==
              -uf(act_map.at(asy_joint_name + l_r_pair.second + "_motor")));
    }
    for (unsigned int i = 0; i < sym_joint_names.size(); i++) {
      // positions
      trajopt->AddLinearConstraint(
          x0(pos_map.at(sym_joint_names[i] + l_r_pair.first)) ==
              xf(pos_map.at(sym_joint_names[i] + l_r_pair.second)));
      // velocities
      trajopt->AddLinearConstraint(
          x0(n_q + vel_map.at(sym_joint_names[i] + l_r_pair.first + "dot")) ==
              xf(n_q + vel_map.at(sym_joint_names[i] + l_r_pair.second + "dot")));
      // inputs (ankle joint is not actuated)
      if (sym_joint_names[i].compare("ankle_joint") != 0) {
        trajopt->AddLinearConstraint(
            u0(act_map.at(sym_joint_names[i] + l_r_pair.first + "_motor")) ==
                uf(act_map.at(sym_joint_names[i] + l_r_pair.second + "_motor")));
      }
    }
  }  // end for (l_r_pairs)

  // joint limits
  for (const auto& member : joint_names) {
    trajopt->AddConstraintToAllKnotPoints(
        x(pos_map.at(member)) <=
            plant.GetJointByName(member).position_upper_limits()(0));
    trajopt->AddConstraintToAllKnotPoints(
        x(pos_map.at(member)) >=
            plant.GetJointByName(member).position_lower_limits()(0));
  }

  // u limit
  for (int i = 0; i < N; i++) {
    auto ui = trajopt->input(i);
    trajopt->AddBoundingBoxConstraint(VectorXd::Constant(n_u, -300),
                                      VectorXd::Constant(n_u, +300), ui);
  }

  if (is_get_nominal) {
//    cout << "Adding zero COM height acceleration constraint\n";
//    auto com_vel_constraint = std::make_shared<ComHeightVelConstraint>(&plant);
//    std::unordered_map<int, double> com_vel_constraint_scale;
//    com_vel_constraint_scale.insert(std::pair<int, double>(0, 0.1));
//    com_vel_constraint->SetConstraintScaling(com_vel_constraint_scale);
//    for (int index = 0; index < num_time_samples[0] - 1; index++) {
//      auto x0 = trajopt->state(index);
//      auto x1 = trajopt->state(index + 1);
//      trajopt->AddConstraint(com_vel_constraint, {x0, x1});
//    }
  }

  // toe position constraint in y direction (avoid leg crossing)
  auto left_foot_constraint = std::make_shared<OneDimBodyPosConstraint>(
      &plant, "toe_left", Vector3d::Zero(), MatrixXd::Identity(3, 3), 1, 0.05,
      std::numeric_limits<double>::infinity());
  auto right_foot_constraint = std::make_shared<OneDimBodyPosConstraint>(
      &plant, "toe_right", Vector3d::Zero(),MatrixXd::Identity(3, 3), 1,
      -std::numeric_limits<double>::infinity(), -0.05);
  // scaling
  /*std::unordered_map<int, double> odbp_constraint_scale;
  odbp_constraint_scale.insert(std::pair<int, double>(0, 0.5));
  left_foot_constraint->SetConstraintScaling(odbp_constraint_scale);
  right_foot_constraint->SetConstraintScaling(odbp_constraint_scale);*/
  for (int index = 0; index < num_time_samples[0]; index++) {
    auto x = trajopt->state(index);
    trajopt->AddConstraint(left_foot_constraint, x.head(n_q));
    trajopt->AddConstraint(right_foot_constraint, x.head(n_q));
  }
  // toe height constraint at mid stance (avoid foot scuffing)
  Vector3d z_hat(0, 0, 1);
  Eigen::Quaterniond q;
  q.setFromTwoVectors(z_hat, ground_normal);
  Eigen::Matrix3d T_ground_incline = q.matrix().transpose();
/*  auto right_foot_constraint_z0 = std::make_shared<OneDimBodyPosConstraint>(
      &plant, "toe_right", Vector3d::Zero(), T_ground_incline, 2, 0.1,
      std::numeric_limits<double>::infinity());
  auto x_mid = trajopt->state(num_time_samples[0] / 2);
  trajopt->AddConstraint(right_foot_constraint_z0, x_mid.head(n_q));*/

  // testing -- swing foot contact point height constraint
  for (int index = 1; index < num_time_samples[0] - 1; index++) {
    double h_min = 0;
    if (index == int(num_time_samples[0]/2)) {
      h_min = 0.1;  // 10 centimeter high in the mid point
    }

    auto right_foot_constraint_z1 = std::make_shared<OneDimBodyPosConstraint>(
        &plant, "toe_right", pt_front_contact, T_ground_incline, 2,
        h_min , std::numeric_limits<double>::infinity());
    auto right_foot_constraint_z2 = std::make_shared<OneDimBodyPosConstraint>(
        &plant, "toe_right", pt_rear_contact, T_ground_incline, 2,
        h_min , std::numeric_limits<double>::infinity());

    auto x_i = trajopt->state(index);
    trajopt->AddConstraint(right_foot_constraint_z1, x_i.head(n_q));
    trajopt->AddConstraint(right_foot_constraint_z2, x_i.head(n_q));
  }
//  // testing -- vertical touchdown velocity
//  trajopt->AddLinearConstraint(trajopt->impulse_vars(0)(0) == 0);
//  trajopt->AddLinearConstraint(trajopt->impulse_vars(0)(3) == 0);
//  // testing -- prevent backward foot velocity
//  auto right_foot_vel_constraint = std::make_shared<OneDimBodyVelConstraint>(
//      &plant, "toe_right", Vector3d::Zero(), T_ground_incline, 0,
//      0 , std::numeric_limits<double>::infinity());
//  for (int index = 1; index < num_time_samples[0] - 1; index++) {
//    auto x_i = trajopt->state(index);
//    trajopt->AddConstraint(right_foot_vel_constraint, x_i);
//  }

  // testing -- swing foot pos at mid stance is the average of the start and the
  // end of the stance
  auto swing_foot_mid_stance_xy_constraint =
      std::make_shared<SwingFootXYPosAtMidStanceConstraint>(&plant, "toe_right",
                                                            Vector3d::Zero());
  trajopt->AddConstraint(swing_foot_mid_stance_xy_constraint,
                         {x0.head(n_q), x_mid.head(n_q), xf.head(n_q)});

  // testing -- lock the swing toe joint position (otherwise it shakes too much)
  // Somehow the swing toe doesn't shake that much anymore after adding
  // constraint such that the two contact points has to be above the ground?
  /*auto d = trajopt->NewContinuousVariables(1, "d_toe");
  for (int index = 0; index < num_time_samples[0] - 1; index++) {
    auto x0 = trajopt->state(index);
    auto x1 = trajopt->state(index + 1);
    trajopt->AddLinearConstraint(x0(n_q - 1) + d(0) == x1(n_q - 1));
  }*/

  // Testing -- constraint on initial floating base
  if (true /*&& ground_incline == 0*/) {
    trajopt->AddConstraint(x0(0) == 1);
  }
  // Testing -- constraint on normal force
  for (unsigned int mode = 0; mode < num_time_samples.size(); mode++) {
    for (int index = 0; index < num_time_samples[mode]; index++) {
      auto lambda = trajopt->force(mode, index);
      trajopt->AddLinearConstraint(lambda(2) >= 10);
      trajopt->AddLinearConstraint(lambda(5) >= 10);
    }
  }
  // Testing -- constraint left four-bar force (seems to help in high speed)
  if (constrain_stance_leg_fourbar_force) {
    for (unsigned int mode = 0; mode < num_time_samples.size(); mode++) {
      for (int index = 0; index < num_time_samples[mode]; index++) {
        if (four_bar_in_right_support) {
          auto lambda = trajopt->force(mode, index);
          trajopt->AddLinearConstraint(lambda(6) <= 0);  // left leg four bar
        }
      }
    }
  }

  // Scale decision variable
  // time
  trajopt->ScaleTimeVariables(0.008);
  // state
  trajopt->ScaleStateVariables(0.5, 0, 3);
  trajopt->ScaleStateVariables(10, n_q, n_q + n_v - 3);
  trajopt->ScaleStateVariables(10, n_q + n_v - 2, n_q + n_v - 2);
  trajopt->ScaleStateVariables(10, n_q + n_v - 1, n_q + n_v - 1);
  // input
  trajopt->ScaleInputVariables(100, 0, 9);
  // force
  trajopt->ScaleForceVariables(
      1000, 0, 0, ls_dataset.countConstraintsWithoutSkipping() - 1);
  trajopt->ScaleForceVariables(
      1000, 1, 0, rs_dataset.countConstraintsWithoutSkipping() - 1);
  // impulse
  trajopt->ScaleImpulseVariables(
      10, 0, 0, rs_dataset.countConstraintsWithoutSkipping() - 1);  // 0.1
  // quaternion slack
  trajopt->ScaleQuaternionSlackVariables(30);
  // Constraint slack
  trajopt->ScaleKinConstraintSlackVariables(50, 0, 0, 5);
  trajopt->ScaleKinConstraintSlackVariables(500, 0, 6, 7);

  // add cost
  MatrixXd W_Q = w_Q * MatrixXd::Identity(n_v, n_v);
  W_Q(4, 4) = w_Q_vy;
  W_Q(5, 5) = w_Q_vz;
  W_Q(n_v - 1, n_v - 1) = w_Q_swing_toe;
  MatrixXd W_R = w_R * MatrixXd::Identity(n_u, n_u);
  W_R(n_u - 1, n_u - 1) = w_R_swing_toe;
  /*trajopt->AddRunningCost(x.tail(n_v).transpose() * W_Q * x.tail(n_v));
  trajopt->AddRunningCost(u.transpose() * W_R * u);*/
  // Add cost without time
  auto fixed_dt = duration / (N - 1);
  for (int i = 0; i < N - 1; i++) {
    auto v0 = trajopt->state(i).tail(n_v);
    auto v1 = trajopt->state(i + 1).tail(n_v);
    trajopt->AddCost(((v0.transpose() * W_Q * v0) * fixed_dt / 2)(0));
    trajopt->AddCost(((v1.transpose() * W_Q * v1) * fixed_dt / 2)(0));
  }
  for (int i = 0; i < N - 1; i++) {
    auto u0 = trajopt->input(i);
    auto u1 = trajopt->input(i + 1);
    trajopt->AddCost(((u0.transpose() * W_R * u0) * fixed_dt / 2)(0));
    trajopt->AddCost(((u1.transpose() * W_R * u1) * fixed_dt / 2)(0));
  }

  // add cost on force difference wrt time
  bool diff_with_force_at_collocation = false;
  if (w_lambda_diff) {
    for (int i = 0; i < N - 1; i++) {
      auto lambda0 = trajopt->force(0, i);
      auto lambda1 = trajopt->force(0, i + 1);
      auto lambdac = trajopt->collocation_force(0, i);
      if (diff_with_force_at_collocation) {
        trajopt->AddCost(w_lambda_diff *
            (lambda0 - lambdac).dot(lambda0 - lambdac));
        trajopt->AddCost(w_lambda_diff *
            (lambdac - lambda1).dot(lambdac - lambda1));
      } else {
        trajopt->AddCost(w_lambda_diff *
            (lambda0 - lambda1).dot(lambda0 - lambda1));
      }
    }
  }
  // add cost on vel difference wrt time
  MatrixXd Q_v_diff = w_v_diff * MatrixXd::Identity(n_v, n_v);
  if (w_v_diff) {
    for (int i = 0; i < N - 1; i++) {
      auto v0 = trajopt->state(i).tail(n_v);
      auto v1 = trajopt->state(i + 1).tail(n_v);
      trajopt->AddCost((v0 - v1).dot(Q_v_diff * (v0 - v1)));
    }
  }
  // add cost on input difference wrt time
  if (w_u_diff) {
    for (int i = 0; i < N - 1; i++) {
      auto u0 = trajopt->input(i);
      auto u1 = trajopt->input(i + 1);
      trajopt->AddCost(w_u_diff * (u0 - u1).dot(u0 - u1));
    }
  }
  // add cost on joint position
  if (w_q_hip_roll) {
    for (int i = 0; i < N; i++) {
      auto q = trajopt->state(i).segment(7, 2);
      trajopt->AddCost(w_q_hip_roll * q.transpose() * q);
    }
  }
  if (w_q_hip_yaw) {
    for (int i = 0; i < N; i++) {
      auto q = trajopt->state(i).segment(9, 2);
      trajopt->AddCost(w_q_hip_yaw * q.transpose() * q);
    }
  }
  if (w_q_quat_xyz) {
    for (int i = 0; i < N; i++) {
      auto q = trajopt->state(i).segment(1, 3);
      trajopt->AddCost(w_q_quat_xyz * q.transpose() * q);
    }
  }

  // Move the trajectory optimization problem into GoldilocksModelTrajOpt
  // where we add the constraints for reduced order model
  GoldilocksModelTrajOpt gm_traj_opt(
    n_s, n_sDDot, n_tau, n_feature_s, n_feature_sDDot,
    B_tau, theta_s, theta_sDDot,
    std::move(trajopt), &plant_autoDiff, &plant, num_time_samples,
    is_get_nominal, is_add_tau_in_cost, rom_option, robot_option);

  addRegularization(is_get_nominal, eps_reg, gm_traj_opt);

  // initial guess if the file exists
  if (!init_file.empty()) {
    setInitialGuessFromFile(directory, init_file, gm_traj_opt);
  } else {
    // Do inverse kinematics to get q initial guess
    vector<VectorXd> q_seed =
        GetCassieInitGuessForQ(N, stride_length, ground_incline, plant);
    // Do finite differencing to get v initial guess
    vector<VectorXd> v_seed = GetCassieInitGuessForV(
                                q_seed, duration / (N - 1), plant);
    for (int i = 0; i < N; i++) {
      auto xi = gm_traj_opt.dircon->state(i);
      VectorXd xi_seed(n_q + n_v);
      xi_seed << q_seed.at(i), v_seed.at(i);
      gm_traj_opt.dircon->SetInitialGuess(xi, xi_seed);
    }
  }
  // Careful: MUST set the initial guess for quaternion, since 0-norm quaternion
  // produces NAN value in some calculation.
  for (int i = 0; i < N; i++) {
    auto xi = gm_traj_opt.dircon->state(i);
    if ((gm_traj_opt.dircon->GetInitialGuess(xi.head(4)).norm() == 0) ||
        std::isnan(gm_traj_opt.dircon->GetInitialGuess(xi.head(4)).norm())) {
      gm_traj_opt.dircon->SetInitialGuess(xi(0), 1);
      gm_traj_opt.dircon->SetInitialGuess(xi(1), 0);
      gm_traj_opt.dircon->SetInitialGuess(xi(2), 0);
      gm_traj_opt.dircon->SetInitialGuess(xi(3), 0);
    }
  }

  // Print variable scaling
  /*for (int i = 0; i < gm_traj_opt.dircon->decision_variables().size(); i++) {
    cout << gm_traj_opt.dircon->decision_variable(i) << ", ";
    cout << gm_traj_opt.dircon->decision_variable(i).get_id() << ", ";
    cout << gm_traj_opt.dircon->FindDecisionVariableIndex(gm_traj_opt.dircon->decision_variable(i))
         << ", ";
    auto scale_map = gm_traj_opt.dircon->GetVariableScaling();
    auto it = scale_map.find(i);
    if (it != scale_map.end()) {
      cout << it->second;
    } else {
      cout << "none";
    }
    cout << ", ";
    cout << gm_traj_opt.dircon->GetInitialGuess(gm_traj_opt.dircon->decision_variable(i));
    cout << endl;
  }*/

  // Testing
  // cout << "Choose the best solver: " <<
  //      drake::solvers::ChooseBestSolver(*(gm_traj_opt.dircon)).name() << endl;

  // cout << "Solving DIRCON (based on MultipleShooting)\n";
  auto start = std::chrono::high_resolution_clock::now();
  const MathematicalProgramResult result = Solve(
        *gm_traj_opt.dircon, gm_traj_opt.dircon->initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;

  bool is_print_for_debugging = false;
  VectorXd w_sol;
  extractResult(w_sol, gm_traj_opt, result, elapsed,
                num_time_samples, N,
                plant, plant_autoDiff,
                n_s, n_sDDot, n_tau, n_feature_s, n_feature_sDDot, B_tau,
                theta_s, theta_sDDot,
                stride_length, ground_incline,
                duration, max_iter,
                directory, init_file, prefix,
                Q_double, R_double, eps_reg,
                is_get_nominal, is_zero_touchdown_impact,
                extend_model, is_add_tau_in_cost,
                sample_idx, n_rerun, cost_threshold_for_update, N_rerun,
                rom_option,
                robot_option,
                dataset_list, is_print_for_debugging);
  postProcessing(w_sol, gm_traj_opt, result, elapsed,
                 num_time_samples, N,
                 plant, plant_autoDiff,
                 n_s, n_sDDot, n_tau, n_feature_s, n_feature_sDDot, B_tau,
                 theta_s, theta_sDDot,
                 stride_length, ground_incline,
                 duration, max_iter,
                 directory, init_file, prefix,
                 Q_double, R_double, eps_reg,
                 is_get_nominal, is_zero_touchdown_impact,
                 extend_model, is_add_tau_in_cost,
                 sample_idx, n_rerun, cost_threshold_for_update, N_rerun,
                 rom_option,
                 robot_option);

  if (sample_idx == 0) {
    is_print_for_debugging = true;
  }

  if (is_print_for_debugging) {
    // Extract result for printing
    VectorXd time_at_knots = gm_traj_opt.dircon->GetSampleTimes(result);
    MatrixXd state_at_knots = gm_traj_opt.dircon->GetStateSamples(result);
    MatrixXd input_at_knots = gm_traj_opt.dircon->GetInputSamples(result);

    // Print weight
    cout << "\nw_Q = " << w_Q << endl;
    cout << "w_Q_swing_toe = " << w_Q_swing_toe << endl;
    cout << "w_R = " << w_R << endl;
    cout << "w_R_swing_toe = " << w_R_swing_toe << endl;
    cout << "w_lambda = " << w_lambda << endl;
    cout << "w_lambda_diff = " << w_lambda_diff << endl;
    cout << "w_v_diff = " << w_v_diff << endl;
    cout << "w_u_diff = " << w_u_diff << endl;
    cout << "w_q_hip_roll = " << w_q_hip_roll << endl;
    cout << "w_q_hip_yaw = " << w_q_hip_yaw << endl;
    cout << "w_q_quat_xyz = " << w_q_quat_xyz << endl;

    // Calculate each term of the cost
    double total_cost = 0;
    double cost_x = 0;
    for (int i = 0; i < N - 1; i++) {
      auto v0 = state_at_knots.col(i).tail(n_v);
      auto v1 = state_at_knots.col(i + 1).tail(n_v);
      auto h = time_at_knots(i + 1) - time_at_knots(i);
      cost_x += ((v0.transpose() * W_Q * v0) * h / 2)(0);
      cost_x += ((v1.transpose() * W_Q * v1) * h / 2)(0);
    }
    total_cost += cost_x;
    cout << "cost_x = " << cost_x << endl;
    double cost_u = 0;
    for (int i = 0; i < N - 1; i++) {
      auto u0 = input_at_knots.col(i);
      auto u1 = input_at_knots.col(i + 1);
      auto h = time_at_knots(i + 1) - time_at_knots(i);
      cost_u += ((u0.transpose() * W_R * u0) * h / 2)(0);
      cost_u += ((u1.transpose() * W_R * u1) * h / 2)(0);
    }
    total_cost += cost_u;
    cout << "cost_u = " << cost_u << endl;
    double cost_lambda = 0;
    for (int i = 0; i < num_time_samples.size(); i++) {
      for (int j = 0; j < num_time_samples[i]; j++) {
        auto lambda = result.GetSolution(gm_traj_opt.dircon->force(i, j));
        cost_lambda += (options_list[i].getForceCost() * lambda).squaredNorm();
      }
    }
    total_cost += cost_lambda;
    cout << "cost_lambda = " << cost_lambda << endl;
    // cost on force difference wrt time
    double cost_lambda_diff = 0;
    for (int i = 0; i < N - 1; i++) {
      auto lambda0 = result.GetSolution(gm_traj_opt.dircon->force(0, i));
      auto lambda1 = result.GetSolution(gm_traj_opt.dircon->force(0, i + 1));
      auto lambdac = result.GetSolution(gm_traj_opt.dircon->collocation_force(0, i));
      if (diff_with_force_at_collocation) {
        cost_lambda_diff +=
            w_lambda_diff * (lambda0 - lambdac).dot(lambda0 - lambdac);
        cost_lambda_diff +=
            w_lambda_diff * (lambdac - lambda1).dot(lambdac - lambda1);
      } else {
        cost_lambda_diff +=
            w_lambda_diff * (lambda0 - lambda1).dot(lambda0 - lambda1);
      }
    }
    total_cost += cost_lambda_diff;
    cout << "cost_lambda_diff = " << cost_lambda_diff << endl;
    // cost on vel difference wrt time
    double cost_vel_diff = 0;
    for (int i = 0; i < N - 1; i++) {
      auto v0 = result.GetSolution(gm_traj_opt.dircon->state(i).tail(n_v));
      auto v1 = result.GetSolution(gm_traj_opt.dircon->state(i + 1).tail(n_v));
      cost_vel_diff += (v0 - v1).dot(Q_v_diff * (v0 - v1));
    }
    total_cost += cost_vel_diff;
    cout << "cost_vel_diff = " << cost_vel_diff << endl;
    // cost on input difference wrt time
    double cost_u_diff = 0;
    for (int i = 0; i < N - 1; i++) {
      auto u0 = result.GetSolution(gm_traj_opt.dircon->input(i));
      auto u1 = result.GetSolution(gm_traj_opt.dircon->input(i + 1));
      cost_u_diff += w_u_diff * (u0 - u1).dot(u0 - u1);
    }
    total_cost += cost_u_diff;
    cout << "cost_u_diff = " << cost_u_diff << endl;
    // add cost on joint position
    double cost_q_hip_roll = 0;
    for (int i = 0; i < N; i++) {
      auto q = result.GetSolution(gm_traj_opt.dircon->state(i).segment(7, 2));
      cost_q_hip_roll += w_q_hip_roll * q.transpose() * q;
    }
    total_cost += cost_q_hip_roll;
    cout << "cost_q_hip_roll = " << cost_q_hip_roll << endl;
    double cost_q_hip_yaw = 0;
    for (int i = 0; i < N; i++) {
      auto q = result.GetSolution(gm_traj_opt.dircon->state(i).segment(9, 2));
      cost_q_hip_yaw += w_q_hip_yaw * q.transpose() * q;
    }
    total_cost += cost_q_hip_yaw;
    cout << "cost_q_hip_yaw = " << cost_q_hip_yaw << endl;
    // add cost on quaternion
    double cost_q_quat_xyz = 0;
    for (int i = 0; i < N; i++) {
      auto q = result.GetSolution(gm_traj_opt.dircon->state(i).segment(1, 3));
      cost_q_quat_xyz += w_q_quat_xyz * q.transpose() * q;
    }
    total_cost += cost_q_quat_xyz;
    cout << "cost_q_quat_xyz = " << cost_q_quat_xyz << endl;

    cout << "total_cost (only the nominal traj cost terms) = " << total_cost << endl;
  }  // end if is_print_for_debugging
}

void trajOptGivenWeights(const MultibodyPlant<double> & plant,
                         const MultibodyPlant<AutoDiffXd> & plant_autoDiff,
                         int n_s, int n_sDDot, int n_tau,
                         int n_feature_s,
                         int n_feature_sDDot,
                         const MatrixXd& B_tau,
                         const VectorXd & theta_s, const VectorXd & theta_sDDot,
                         double stride_length, double ground_incline,
                         double duration, int n_node, int max_iter,
                         double major_optimality_tol,
                         double major_feasibility_tol,
                         const string& directory,
                         string init_file, string prefix,
                         /*vector<VectorXd> * w_sol_vec,
                         vector<MatrixXd> * A_vec, vector<MatrixXd> * H_vec,
                         vector<VectorXd> * y_vec,
                         vector<VectorXd> * lb_vec, vector<VectorXd> * ub_vec,
                         vector<VectorXd> * b_vec,
                         vector<VectorXd> * c_vec,
                         vector<MatrixXd> * B_vec,*/
                         double Q_double, double R_double,
                         double all_cost_scale,
                         double eps_reg,
                         bool is_get_nominal,
                         bool is_zero_touchdown_impact,
                         bool extend_model,
                         bool is_add_tau_in_cost,
                         int sample_idx, int n_rerun,
                         double cost_threshold_for_update, int N_rerun,
                         int rom_option, int robot_option) {
  if (robot_option == 0) {
    fiveLinkRobotTrajOpt(plant, plant_autoDiff,
                         n_s, n_sDDot, n_tau, n_feature_s, n_feature_sDDot, B_tau,
                         theta_s, theta_sDDot,
                         stride_length, ground_incline,
                         duration, n_node, max_iter,
                         directory, init_file, prefix,
                         Q_double, R_double, all_cost_scale, eps_reg,
                         is_get_nominal, is_zero_touchdown_impact,
                         extend_model, is_add_tau_in_cost,
                         sample_idx, n_rerun, cost_threshold_for_update, N_rerun,
                         rom_option, robot_option);
  } else if (robot_option == 1) {
    cassieTrajOpt(plant, plant_autoDiff,
                  n_s, n_sDDot, n_tau, n_feature_s, n_feature_sDDot, B_tau,
                  theta_s, theta_sDDot,
                  stride_length, ground_incline,
                  duration, n_node, max_iter,
                  major_optimality_tol, major_feasibility_tol,
                  directory, init_file, prefix,
                  Q_double, R_double, all_cost_scale, eps_reg,
                  is_get_nominal, is_zero_touchdown_impact,
                  extend_model, is_add_tau_in_cost,
                  sample_idx, n_rerun, cost_threshold_for_update, N_rerun,
                  rom_option, robot_option);
  }

  // For multithreading purpose. Indicate this function has ended.
  VectorXd thread_finished(1);
  thread_finished << 1;
  writeCSV(directory + prefix + string("thread_finished.csv"), thread_finished);
}

}  // namespace dairlib::goldilocks_models

