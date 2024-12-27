#include <iostream>
#include "systems/controllers/id_mpc/id_mpc.h"
#include "common/eigen_utils.h"

#include "drake/solvers/solve.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/multibody/inverse_kinematics/unit_quaternion_constraint.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::VectorXd;
using Eigen::Vector3d;

void TestInverseDynamics(
    const ConstrainedDynamicsInfo& info, const VectorXd q, const VectorXd& u,
    const VectorXd& lambda, std::vector<std::string> contacts) {
  VectorXd all_vars = VectorXd::Zero(info.variable_count());

  all_vars.head(q.rows()) = q;
  all_vars.segment(q.rows() + q.rows() - 1, u.rows()) = u;
  all_vars.tail(lambda.rows()) = lambda;

  DRAKE_DEMAND(info.get_q(all_vars) == q);
  DRAKE_DEMAND(info.get_v(all_vars) == VectorXd::Zero(q.rows() - 1));
  DRAKE_DEMAND(info.get_u(all_vars) == u);
  DRAKE_DEMAND(info.get_lh(all_vars) == lambda.head(2));
  DRAKE_DEMAND(info.get_lc(all_vars) == lambda.tail(12));

  VectorXd x = all_vars.head(info.nx());
  auto context = info.MakeContext<double>();
  info.SetPlantStateIfNew<double>(x, context.get());

  auto result = info.EvaluateDynamics<double>(
      *context,
      VectorXd::Zero(info.nv()),
      info.get_lh(all_vars),
      info.get_lc(all_vars),
      contacts
  );

  Eigen::MatrixXd B = info.get_plant().MakeActuationMatrix();
  std::cout << "inverse dynamics defect norm: " << (B*u - result.tau_).norm()
  << std::endl;
}

int DoMain() {
  std::string urdf = "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  auto dynamics_info = std::make_unique<ConstrainedDynamicsInfo>(urdf);

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
  vars.segment(dynamics_info->nx(), dynamics_info->nu()) = u;
  vars.tail(dynamics_info->nh() + dynamics_info->nc()) = lambda;

  Timeline test_timeline;
  for (int i = 0; i < 2; ++i) {
    test_timeline.knots.push_back(KnotPointState(*dynamics_info));
    test_timeline.knots.back().UpdateActiveContacts(contacts);
    test_timeline.breaks.push_back(0.05 * i);
  }

  auto test_constraint = std::make_shared<CollocationConstraint<double>>(
      &test_timeline);

  VectorXd constraint_result;
  test_constraint->EvaluateConstraint(stack<double>({vars, vars}), &constraint_result);

  std::cout << "collocation defect norm: " << constraint_result.norm()
            << std::endl;

  IDMPCParams params;
  params.dt = 0.05;
  params.N = static_cast<int>(0.8 / params.dt);

  IDMPC mpc(params, std::move(dynamics_info));

  auto& prog = mpc.get_prog();

  VectorXd vd = vars;
  vd(6) = 0.7;

  Eigen::MatrixXd Q_all = 0.0000001 *
      Eigen::MatrixXd::Identity(vars.rows(), vars.rows());
  Q_all(6,6) = 1000.0;

  for (int i = 0; i <= params.N; ++i) {
    prog.SetInitialGuess(mpc.knot_vars(i), vars);
    prog.AddQuadraticErrorCost(Q_all, vd, mpc.knot_vars(i));
    auto unit_quat = std::make_shared<drake::multibody
        ::UnitQuaternionConstraint>();

    prog.AddConstraint(unit_quat, mpc.position_vars(i).head(4));
    mpc.SetActiveContacts(i, contacts);
  }

  mpc.SetInitialState(vars.head(2 * q.size() - 1));

  auto solver_options = drake::solvers::SolverOptions();
  solver_options.SetOption(drake::solvers::SnoptSolver::id(), "Print file",
                           "./snopt.out");
  solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                           "Major Iterations Limit", 1e6);
  solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                           "Iterations Limit", 1e6);
  solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                          "Major optimality tolerance", 1e-3);
  solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                          "Major feasibility tolerance", 1e-3);

  prog.SetSolverOptions(solver_options);

  auto solver = drake::solvers::SnoptSolver();
  auto result = solver.Solve(prog);
  std::cout << result.get_solution_result() << std::endl;


  return 0;
}

}

int main(int argc, char**argv) {
  return dairlib::systems::controllers::id_mpc::DoMain();
}