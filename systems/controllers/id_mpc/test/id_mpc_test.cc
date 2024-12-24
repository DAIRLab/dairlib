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
      "toe_right_front",
      "toe_right",
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
  q << 1, VectorXd::Zero(6),
  -0.084017,  -0.00120735, 0.366012, -0.6305, 0.838878, 0.205351,
   0.084017,   0.00120735, 0.366012, -0.6305, 0.838878, 0.205351;
  for (int i = 0; i < 4; ++i) {
    vars.tail(12).segment<3>(3*i) = Vector3d(0, 0, 9.81 * 33.0 / 4.0);
  }

  vars.head(dynamics_info->nq()) = q;

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