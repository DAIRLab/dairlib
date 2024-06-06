#include "systems/controllers/footstep_planning/cf_mpfc.h"

namespace dairlib::systems::controllers {

using cf_mpfc_utils::SrbDim;

int DoMain(int argc, char* argv[]) {
  cf_mpfc_utils::CentroidalState<double> srbd_state;
  cf_mpfc_utils::CentroidalState<drake::AutoDiffXd> srbd_state_ad;

  double mass = 10;

  srbd_state.setZero();
  srbd_state(0) = 1;
  srbd_state(4) = 1;
  srbd_state(8) = 1;
  srbd_state(11) = 1;

  drake::Vector6d p_f = drake::Vector6d::Zero();
  p_f(1) = 0.1;
  p_f(4) = -0.1 * 9.81 * mass;
  p_f(5) = 9.81 * mass;

  auto start = std::chrono::high_resolution_clock::now();

  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::VectorXd c;

  cf_mpfc_utils::LinearizeSRBDynamics(
      srbd_state, {p_f}, Eigen::Matrix3d::Identity(), mass, A, B, c);

  auto end = std::chrono::high_resolution_clock::now();

  std::cout << "AD took " <<
  std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " us.\n";

  std::cout << "A:\n" << A << std::endl
            << "B:\n" << B << std::endl
            << "c:\n" << c << std::endl;

  return 0;
}

}

int main(int argc, char* argv[]) {
  return dairlib::systems::controllers::DoMain(argc, argv);
}