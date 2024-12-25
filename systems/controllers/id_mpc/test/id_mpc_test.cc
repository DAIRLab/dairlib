#include <iostream>
#include "systems/controllers/id_mpc/constrained_dynamics_info.h"
#include "systems/controllers/id_mpc/id_mpc.h"

#include "drake/solvers/solve.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/multibody/inverse_kinematics/unit_quaternion_constraint.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::VectorXd;
using Eigen::Vector3d;

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
    -0.0730404, 0, 0.571375, -1.38058, 1.60491, -1.6692,

  u <<  -2.03951, 2.04169, 0.906345, -0.861539, -5.96077, -6.16527, 45.7984,
        45.6304, -3.48936, -3.52897;

  lambda << -395.296, -395.589,
    39.7438, -9.54163, 81.7462,
   -39.8489, 4.21296, 80.2822,
    39.8174, 9.30058, 81.8166,
   -39.7123, -3.97191, 79.936;

  vars.head(dynamics_info->nq()) = q;
  vars.segment(dynamics_info->nx(), dynamics_info->nu()) = u;
  vars.tail(dynamics_info->nh() + dynamics_info->nc()) = lambda;

  auto test_context = dynamics_info->MakeContext<double>();
  auto test_result = dynamics_info->EvaluateDynamics<double>(
    test_context.get(),
    vars,
    {"toe_left_front", "toe_left_rear", "toe_right_front",
     "toe_right_rear"});

  std::cout << test_result;



  IDMPCParams params;
  params.N = 5;
  params.dt = 0.05;

  IDMPC mpc(params, std::move(dynamics_info));

  auto& prog = mpc.get_prog();
  for (int i = 0; i <= params.N; ++i) {
    prog.SetInitialGuess(mpc.knot_vars(i), vars);
    prog.AddQuadraticErrorCost(
        Eigen::MatrixXd::Identity(q.size(), q.size()),
        q,
        mpc.position_vars(i));
    auto unit_quat = std::make_shared<drake::multibody
        ::UnitQuaternionConstraint>();

    prog.AddConstraint(unit_quat, mpc.position_vars(i).head(4));

    mpc.SetActiveContacts(
        i, {"toe_left_front", "toe_left_rear", "toe_right_front",
            "toe_right_rear"});
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
                          "Major feasibility tolerance", 1e-4);
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