#include "systems/controllers/footstep_planning/cf_mpfc.h"

namespace dairlib::systems::controllers {

using Eigen::Vector2d;
using Eigen::Vector3d;
using drake::Vector6d;

void TestDynamics() {

  double mass = 30;
  Vector6d x;

  x.setZero();
  x(1) = -0.1;
  x(2) = 1;
  x(3) = 1;

  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d f = Eigen::Vector3d::Zero();
  f(1) = -0.1 * 9.81 * mass;
  f(2) = 9.81 * mass;

  auto start = std::chrono::high_resolution_clock::now();

  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::VectorXd c;

  nonlinear_pendulum::LinearizeTrapezoidalCollocationConstraint(
      0.1, x, x, Vector2d::Zero(), Vector2d::Zero(), mass, A, B, c);

  Eigen::MatrixXd Ax;
  Eigen::MatrixXd Bd;
  Eigen::Vector4d a;


  nonlinear_pendulum::LinearizeALIPReset(
      x, p, -0.2 * Eigen::Vector3d::UnitY(), mass, Ax, Bd, a);
  auto end = std::chrono::high_resolution_clock::now();

  std::cout << "AD took " <<
            std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " us.\n";

  std::cout << "Dynamics\nA:\n" << A << std::endl
            << "Bf:\n" << B << std::endl
            << "c:\n" << c << std::endl;

  std::cout << "Reset\nAx:\n" << Ax << std::endl
            << "Bd:\n" << Bd << std::endl
            << "x+:\n" << a << std::endl;
}

void TestMPFC() {
  auto test_gait = alip_utils::AlipGaitParams {
      0.85,
      32.0,
      0.3,
      0.1,
      0.2,
      Vector2d::Zero(),
      alip_utils::Stance::kLeft,
      alip_utils::ResetDiscretization::kZOH
  };

  cf_mpfc_params params;

  params.tracking_cost_type = alip_utils::kGait;
  params.gait_params = test_gait;
  params.nmodes = 3;
  params.nknots = 5;
  params.soft_constraint_cost = 1000;
  params.com_pos_bound = Eigen::Vector2d::Ones();
  params.com_vel_bound = 2.0 * Eigen::Vector2d::Ones();
  params.input_bounds =  8 * Eigen::Vector2d::Ones();
  params.Q = Eigen::Matrix4d::Identity();
  params.R = Eigen::Matrix3d::Identity();
  params.Qf = 100 * Eigen::Matrix4d::Identity();
  params.solver_options.SetOption(
      drake::solvers::GurobiSolver::id(), "Presolve", 1);
  params.solver_options.SetOption(
      drake::solvers::GurobiSolver::id(), "LogToConsole", 1);
  params.solver_options.SetOption(
      drake::solvers::CommonSolverOption::kPrintToConsole, 1);
  params.mu = 1;

  CFMPFC mpfc(params);

  Vector6d x;

  x.setZero();
  x(0) = 0.01;
  x(1) = 0.1;
  x(2) = 1;
  x(3) = 1;


  cf_mpfc_solution prev_sol{};

  auto sol = mpfc.Solve(x, Vector3d::Zero(), 0.25, Vector2d::Zero(),
             alip_utils::Stance::kRight, prev_sol);

  std::cout << "Solve took " << sol.total_time << "seconds\n";
  std::cout << "Solution Result: " << sol.solution_result << "\n";

  std::cout << "\nFootstep Solution:\n";
  for (const auto& p : sol.pp) {
    std::cout << p.transpose() << std::endl;
  }

  std::cout << "\nALIP state solution:\n";
  for (const auto& xx : sol.xx) {
    std::cout << xx.transpose() << std::endl;
  }

  std::cout << "\nPendulum Pos solution:\n";
  for (const auto& xc : sol.xc) {
    std::cout << xc.head<3>().transpose() << std::endl;
  }

  std::cout << "\nPendulum Vel solution:\n";
  for (const auto& xc : sol.xc) {
    std::cout << xc.tail<3>().transpose() << std::endl;
  }

  std::cout << "\npendulum input solution:\n";
  for (const auto& u : sol.uu) {
    std::cout << u.transpose() << std::endl;
  }

  std::cout << "\nInitial ALIP state sol:\n";
  std::cout << sol.xi.transpose() << std::endl;

}


int DoMain(int argc, char* argv[]) {
  TestMPFC();
  TestDynamics();
  return 0;
}

}

int main(int argc, char* argv[]) {
  return dairlib::systems::controllers::DoMain(argc, argv);
}