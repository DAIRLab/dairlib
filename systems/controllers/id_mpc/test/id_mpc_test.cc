#include <iostream>
#include "systems/controllers/id_mpc/id_mpc.h"
#include "systems/controllers/id_mpc/costs/quadratic_error_cost.h"
#include "common/eigen_utils.h"

#include "drake/solvers/solve.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/gurobi_solver.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::VectorXd;
using Eigen::Vector3d;

using drake::trajectories::PiecewisePolynomial;

void TestInverseDynamics(
    const ConstrainedDynamicsInfo& info, const VectorXd q, const VectorXd& u,
    const VectorXd& lambda, std::vector<std::string> contacts) {
  VectorXd all_vars = VectorXd::Zero(info.variable_count());

  all_vars.head(q.rows()) = q;
  all_vars.segment(q.rows() + q.rows() - 1, lambda.rows()) = lambda;
  all_vars.tail(u.rows()) = u;

  VectorXd x = all_vars.head(info.nx());
  auto context = info.MakeContext<double>();
  info.SetPlantStateIfNew<double>(x, context.get());

  auto kinematics = info.EvaluateKinematics<double>(*context, contacts);
  auto tau = info.EvaluateInverseDynamics<double>(
      *context,
      kinematics,
      VectorXd::Zero(info.nv()),
      lambda
  );

  Eigen::MatrixXd B = info.get_plant().MakeActuationMatrix();
  double defect =  (B*u - tau).norm();
  DRAKE_DEMAND(defect < 0.01);
}

void TestCollocation(const ConstrainedDynamicsInfo& info,
                     const VectorXd& fixed_point_vars,
                     const std::vector<std::string>& contacts) {
  Timeline test_timeline;

  std::vector<std::unique_ptr<KnotPoint>> ks;
  std::vector<KnotPointState> states;
  for (int i = 0; i < 3; ++i) {
    auto cfg = knot_config {
      i, i == 2, i == 0, {}, {}
    };
    ks.push_back(std::make_unique<KnotPoint>(info, cfg));
    states.push_back(KnotPointState(info));
    states.back().UpdateTimestamp(i * 0.05);
    states.back().UpdateActiveContacts(contacts);
  }

  auto test_constraint_torque =
      std::make_shared<CollocationConstraint<double>>(
          *ks[0], *ks[1], &states[0], &states[1]);

  auto test_constraint_no_torque =
      std::make_shared<CollocationConstraint<double>>(
          *ks[1], *ks[2], &states[1], &states[2]);

  VectorXd constraint_result;
  VectorXd constraint_result_nc;

  test_constraint_torque->EvaluateConstraint(
      stack<double>({fixed_point_vars, fixed_point_vars.head(info.nx())}),
      &constraint_result);
  DRAKE_DEMAND(constraint_result.norm() < 1e-2);

  test_constraint_no_torque->EvaluateConstraint(
      stack<double>({
        fixed_point_vars.head(info.nx() + info.nh() + info.nc()),
        fixed_point_vars.head(info.nx())}),
      &constraint_result_nc);
  DRAKE_DEMAND(constraint_result_nc.norm() < 1e-2);
}

void CostTest() {
  
  Eigen::MatrixXd Q = Eigen::MatrixXd::Random(3, 3);
  Q = 0.5 * (Q * Q.transpose()) + 4 * Eigen::MatrixXd::Identity(3,3);
  Eigen::VectorXd x = Eigen::VectorXd::Random(3);

  auto quad_cost = QuadraticErrorCost<double>(Q, x);
  auto drake_quad = drake::solvers::MakeQuadraticErrorCost(Q, x);

  Eigen::VectorXd x_star = Eigen::VectorXd::Random(3);
  Eigen::VectorXd dx = Eigen::VectorXd::Random(3);

  Eigen::VectorXd result = Eigen::VectorXd::Zero(1);

  double drake_val = 0;
  double derived_val = 0;

  drake_quad->Eval(x_star, &result);
  drake_val = result(0);
  result.setZero();

  quad_cost.Eval(x_star, &result);
  derived_val = result(0);
  result.setZero();

  DRAKE_DEMAND(abs(derived_val - drake_val) < 1e-8);

  auto gn = quad_cost.CalcGaussNewtonApproximation(x_star);

  double gn_approx = (0.5 * dx.transpose() * gn.H * dx + gn.g.transpose() * dx)
      (0) + gn.c;
  drake_quad->Eval(x_star + dx, &result);
  DRAKE_DEMAND(abs(gn_approx - result(0)) < 1e-8);
}

int DoMain() {
  srand((unsigned int) time(0));

  CostTest();

  std::string urdf = "examples/Cassie/urdf/cassie_fixed_spring_conservative"
                     ".urdf";
  auto dynamics_info = std::make_unique<ConstrainedDynamicsInfo>(urdf);

  VectorXd rotor_inertias(10);
  rotor_inertias << 61, 61, 61, 61, 365, 365, 365, 365, 4.9, 4.9;
  rotor_inertias *= 1e-6;
  VectorXd gear_ratios(10);
  gear_ratios << 25, 25, 25, 25, 16, 16, 16, 16, 50, 50;
  std::vector<std::string> motor_joint_names = {
      "hip_roll_left_motor", "hip_roll_right_motor", "hip_yaw_left_motor",
      "hip_yaw_right_motor", "hip_pitch_left_motor", "hip_pitch_right_motor",
      "knee_left_motor",     "knee_right_motor",     "toe_left_motor",
      "toe_right_motor"};

  for (int i = 0; i < rotor_inertias.size(); ++i) {
    auto& joint_actuator = dynamics_info->get_mutable_plant().get_mutable_joint_actuator(
        drake::multibody::JointActuatorIndex(i));
    joint_actuator.set_default_rotor_inertia(rotor_inertias(i));
    joint_actuator.set_default_gear_ratio(gear_ratios(i));
    DRAKE_DEMAND(motor_joint_names[i] == joint_actuator.name());
  }

  dynamics_info->Finalize();
  dynamics_info->AddContactPoint(
      "toe_left_front",
      "toe_left",
      Vector3d(-0.0457, 0.112, 0),
      {0, 1, 2},
      0.8
  );
  dynamics_info->AddContactPoint(
      "toe_left_rear",
      "toe_left",
      Vector3d(0.088, 0, 0),
      {1, 2},
      0.8
  );
  dynamics_info->AddContactPoint(
      "toe_right_front",
      "toe_right",
      Vector3d(-0.0457, 0.112, 0),
      {0, 1, 2},
      0.8
  );
  dynamics_info->AddContactPoint(
      "toe_right_rear",
      "toe_right",
      Vector3d(0.088, 0, 0),
      {1, 2},
      0.8
  );

  dynamics_info->AddDistanceConstraint(
      "thigh_left",
      Vector3d(0.0, 0.0, 0.045),
      "heel_spring_left",
      Vector3d(.11877, -.01, 0.0),
      0.5012
  );

  dynamics_info->AddDistanceConstraint(
      "thigh_right",
      Vector3d(0.0, 0.0, -0.045),
      "heel_spring_right",
      Vector3d(.11877, -.01, 0.0),
      0.5012
  );

  VectorXd vars = VectorXd::Zero(dynamics_info->variable_count());
  VectorXd q = VectorXd::Zero(dynamics_info->nq());
  VectorXd v = VectorXd::Zero(dynamics_info->nv());
  VectorXd u = VectorXd::Zero(dynamics_info->nu());
  VectorXd lambda = VectorXd::Zero(dynamics_info->nh() + dynamics_info->nc());

  q << 1, 0, 0, 0, 0, 0, 0.95,
     0.0730404, 0, 0.571375, -1.38058, 1.60491, -1.6692,
    -0.0730404, 0, 0.571375, -1.38058, 1.60491, -1.6692;

  u <<  -2.03951, 2.04169, 0.906345, -0.861539, -5.96077, -6.16527, 45.7984,
        45.6304, -3.48936, -3.52897;

  lambda << -395.296, -395.589,
    39.7438, -9.54163, 81.7462,
   -39.8489, 4.21296, 80.2822,
    39.8174, 9.30058, 81.8166,
   -39.7123, -3.97191, 79.936;

  std::vector<std::string> contacts = {"toe_left_front", "toe_left_rear", "toe_right_front",
                                       "toe_right_rear"};

  TestInverseDynamics(*dynamics_info, q, u, lambda, contacts);

  vars.head(dynamics_info->nq()) = q;
  vars.segment(dynamics_info->nx(), dynamics_info->nh() + dynamics_info->nc()) = lambda;
  vars.tail(dynamics_info->nu()) = u;

  TestCollocation(*dynamics_info, vars, contacts);

  IDMPCParams params;
  params.dt = 0.05;
  params.N = static_cast<int>(0.5 / params.dt);
  params.num_full_torque_knots = 2;

  params.Wq = 1000 * Eigen::MatrixXd::Identity(q.rows(), q.rows());
  params.Wv = 0.001 * Eigen::MatrixXd::Identity(v.rows(), v.rows());
  params.Wu = 0.001 * Eigen::MatrixXd::Identity(u.rows(), u.rows());
  params.Wlambda = 0.0001 * Eigen::MatrixXd::Identity(
      lambda.rows(), lambda.rows());

  IDMPC mpc(params, std::move(dynamics_info));

  auto& prog = mpc.get_prog();

  Eigen::VectorXd qd = VectorXd::Zero(q.rows());
  qd << 1, -1.0603e-16, 0, 0, 0, 0, 0.7,
   0.0989926, 0, 0.897318, -2.0368, 2.26086, -1.99487,
  -0.0989926, 0, 0.897318, -2.0368, 2.26086, -1.99487;

  MPCReference reference;

  reference.q_traj_ = PiecewisePolynomial<double>(qd);
  reference.v_traj_ = PiecewisePolynomial<double>(v);
  reference.lambda_traj_ = PiecewisePolynomial<double>(lambda);
  reference.u_traj_ = PiecewisePolynomial<double>(u);

  for (int i = 0; i <= params.N; ++i) {
    vars.segment(q.size(), q.size() - 1) =
        0.01 * Eigen::VectorXd::Random(q.size() - 1);
    prog.SetInitialGuess(mpc.knot_vars(i), vars.head(mpc.knot_vars(i).rows()));

    reference.active_contacts_.push_back(contacts);
    reference.knot_times_.push_back(params.dt * i);
  }

  mpc.UpdateProblemData(reference, vars.head(2 * q.size() - 1));

  auto solver_options = drake::solvers::SolverOptions();
  solver_options.SetOption(drake::solvers::SnoptSolver::id(), "Print file",
                           "./snopt.out");
  solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                           "Major Iterations Limit", 1e6);
  solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                           "Iterations Limit", 1e6);
  solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                          "Major optimality tolerance", 1e-4);
  solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                          "Major feasibility tolerance", 1e-2);
  solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                           "Scale option", 2);

  prog.SetSolverOptions(solver_options);

  auto solver = drake::solvers::SnoptSolver();
  auto result = solver.Solve(prog);
  std::cout << result.get_solution_result() << std::endl;
  std::cout << "solve time: " << result.get_solver_details<drake::solvers::SnoptSolver>().solve_time << std::endl;
  auto sol = mpc.GetSolutionAsLcmTrajectory(result);

  std::cout << "q: \n";
  std::cout << sol.GetTrajectory("q").datapoints << std::endl;


  solvers::QPData qp;
  auto qp_start = std::chrono::high_resolution_clock::now();
  mpc.ConstructSQPProgram(result.GetSolution(), qp);
  auto qp_end = std::chrono::high_resolution_clock::now();

  std::cout << "Building the SQP qp took " <<
      std::chrono::duration<double>(qp_end - qp_start).count() << " sec\n";

  drake::solvers::MathematicalProgram sqp_prog;
  qp.ToMathematicalProgram(sqp_prog);
  drake::solvers::OsqpSolver osqp;
  drake::solvers::GurobiSolver gurobi;
  auto osqp_result = osqp.Solve(sqp_prog);
  auto gurobi_result = gurobi.Solve(sqp_prog);
  std::cout << "OSQP Solution status: " << osqp_result.get_solution_result() << std::endl;
  std::cout << "OSQP solve took: " << osqp_result.get_solver_details<drake::solvers::OsqpSolver>().solve_time << std::endl;
  std::cout << "Gurobi Solution status: " << gurobi_result.get_solution_result() << std::endl;
  std::cout << "Gurobi solve took: " << gurobi_result.get_solver_details<drake::solvers::GurobiSolver>().optimizer_time << std::endl;

  return 0;
}

}

int main(int argc, char**argv) {
  return dairlib::systems::controllers::id_mpc::DoMain();
}