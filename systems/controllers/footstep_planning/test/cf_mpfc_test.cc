#include "systems/controllers/footstep_planning/cf_mpfc.h"

namespace dairlib::systems::controllers {

using cf_mpfc_utils::SrbDim;
using Eigen::Vector2d;
using Eigen::Vector3d;

void TestDynamics() {
  cf_mpfc_utils::CentroidalState<double> srbd_state;

  double mass = 30;

  srbd_state.setZero();
  srbd_state(0) = 1;
  srbd_state(4) = 1;
  srbd_state(8) = 1;
  srbd_state(10) = -0.1;
  srbd_state(11) = 1;
  srbd_state(15) = 0.2;

  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d f = Eigen::Vector3d::Zero();
  f(1) = -0.1 * 9.81 * mass;
  f(2) = 9.81 * mass;

  auto start = std::chrono::high_resolution_clock::now();

  Eigen::MatrixXd A;
  Eigen::MatrixXd Bf;
  Eigen::VectorXd c;

  cf_mpfc_utils::LinearizeSRBDynamics(
      srbd_state, {p}, {f}, Eigen::Matrix3d::Identity(), mass, A, Bf, c);


  Eigen::MatrixXd Ax;
  Eigen::MatrixXd Bd;
  Eigen::Vector4d a;


  cf_mpfc_utils::LinearizeReset(
      srbd_state, p, -0.2 * Eigen::Vector3d::UnitY(), Eigen::Matrix3d::Identity(), mass, Ax, Bd, a);
  auto end = std::chrono::high_resolution_clock::now();

  std::cout << "AD took " <<
            std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " us.\n";

  std::cout << "Dynamics\nA:\n" << A << std::endl
            << "Bf:\n" << Bf << std::endl
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
      0.4,
      Vector2d::Zero(),
      alip_utils::Stance::kLeft,
      alip_utils::ResetDiscretization::kZOH
  };

  cf_mpfc_params params;

  params.gait_params = test_gait;
  params.nmodes = 3;
  params.nknots = 5;
  params.contacts_in_stance_frame = {0.09 * Vector3d::UnitX(), -0.09 * Vector3d::UnitX()};
  params.soft_constraint_cost = 1000;
  params.com_pos_bound = Eigen::Vector2d::Ones();
  params.com_vel_bound = 2.0 * Eigen::Vector2d::Ones();
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

  cf_mpfc_utils::CentroidalState<double> srbd_state;
  srbd_state.setZero();
  srbd_state(0) = 1;
  srbd_state(4) = 1;
  srbd_state(8) = 1;
  srbd_state(10) = -0.1;
  srbd_state(11) = 1;
  srbd_state(15) = 0;

  cf_mpfc_solution prev_sol{};

  auto sol = mpfc.Solve(srbd_state, Vector3d::Zero(), 0.25, Vector2d::Zero(),
             alip_utils::Stance::kRight, Eigen::Matrix3d::Identity(),
             prev_sol);

  std::cout << "Solve took " << sol.total_time << "seconds\n";
  std::cout << "Solution Result: " << sol.solution_result << "\n";

  std::cout << "\nFootstep Solution:\n";
  for (const auto& p : sol.pp) {
    std::cout << p.transpose() << std::endl;
  }

  std::cout << "\nALIP state solution:\n";
  for (const auto& x : sol.xx) {
    std::cout << x.transpose() << std::endl;
  }

  std::cout << "\nCoM solution:\n";
  for (const auto& xc : sol.xc) {
    std::cout << xc.segment<3>(cf_mpfc_utils::com_idx).transpose() << std::endl;
  }

  std::cout << "\nw solution:\n";
  for (const auto& xc : sol.xc) {
    std::cout << xc.segment<3>(cf_mpfc_utils::w_idx).transpose() << std::endl;
  }

  std::cout << "\ncom_dot solution:\n";
  for (const auto& xc : sol.xc) {
    std::cout << xc.segment<3>(cf_mpfc_utils::com_dot_idx).transpose() << std::endl;
  }

  std::cout << "\nSRB input solution:\n";
  for (const auto& f : sol.ff) {
    std::cout << f.transpose() << std::endl;
  }

  std::cout << "\nInitial ALIP state sol:\n";
  std::cout << sol.xi.transpose() << std::endl;

}


int DoMain(int argc, char* argv[]) {
  TestMPFC();
  return 0;
}

}

int main(int argc, char* argv[]) {
  return dairlib::systems::controllers::DoMain(argc, argv);
}