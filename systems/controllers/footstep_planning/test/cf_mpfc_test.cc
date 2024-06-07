#include "systems/controllers/footstep_planning/cf_mpfc.h"

namespace dairlib::systems::controllers {

using cf_mpfc_utils::SrbDim;
using Eigen::Vector2d;
using Eigen::Vector3d;

void TestDynamics() {
  cf_mpfc_utils::CentroidalState<double> srbd_state;
  cf_mpfc_utils::CentroidalState<drake::AutoDiffXd> srbd_state_ad;

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
  Eigen::MatrixXd Bp;
  Eigen::MatrixXd Bf;
  Eigen::VectorXd c;

  cf_mpfc_utils::LinearizeSRBDynamics(
      srbd_state, {p}, {f}, Eigen::Matrix3d::Identity(), mass, A, Bp, Bf, c);


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
      0.3,
      Vector2d::UnitX(),
      alip_utils::Stance::kLeft,
      alip_utils::ResetDiscretization::kZOH
  };

  cf_mpfc_params params;

  params.gait_params = test_gait;
  params.gait_params = test_gait;
  params.nmodes = 3;
  params.nknots = 4;
  params.contacts_in_stance_frame = {0.09 * Vector3d::UnitX(), -0.09 * Vector3d::UnitX()};
  params.soft_constraint_cost = 1000;
  params.com_pos_bound = Eigen::Vector2d::Ones();
  params.com_vel_bound = 2.0 * Eigen::Vector2d::Ones();
  params.Q = Eigen::Matrix4d::Identity();
  params.R = Eigen::Matrix3d::Identity();
  params.Qf = Eigen::Matrix4d::Identity();
  params.solver_options.SetOption(
      drake::solvers::GurobiSolver::id(),
      "Presolve",
      0
  );

  CFMPFC mpfc(params);
}


int DoMain(int argc, char* argv[]) {
  TestDynamics();
  TestMPFC();
  return 0;
}

}

int main(int argc, char* argv[]) {
  return dairlib::systems::controllers::DoMain(argc, argv);
}