#include "systems/controllers/footstep_planning/cf_mpfc.h"

namespace dairlib::systems::controllers {

int DoMain(int argc, char* argv[]) {
  cf_mpfc_utils::CentroidalState<double> srbd_state;
  cf_mpfc_utils::CentroidalState<drake::AutoDiffXd> srbd_state_ad;

  double mass = 10;

  srbd_state.setZero();
  srbd_state(0) = 1;
  srbd_state(3) = 1;
  srbd_state(6) = 1;
  srbd_state(11) = 1;

  drake::Vector6d p_f = drake::Vector6d::Zero();
  p_f(1) = 0.1;
  p_f(5) = 9.81 * mass;

  Eigen::VectorXd vars(24);
  vars.head<18>() = srbd_state;
  vars.tail<6>() = p_f;
  drake::VectorX<drake::AutoDiffXd> vars_ad = drake::math::InitializeAutoDiff(
      vars);
  srbd_state_ad = vars_ad.head<18>();
  drake::Vector6<drake::AutoDiffXd> p_f_ad = vars_ad.tail<6>();

  std::cout << "\n" << cf_mpfc_utils::SRBDynamics(
      srbd_state, {p_f}, Eigen::Matrix3d::Identity(), mass) << std::endl;

  std::cout << "\n" << drake::math::ExtractGradient(
      cf_mpfc_utils::SRBDynamics(
          srbd_state_ad,
          {p_f_ad},
          Eigen::Matrix3d::Identity(),
          mass
      )
  ) << std::endl;


  return 0;
}

}

int main(int argc, char* argv[]) {
  return dairlib::systems::controllers::DoMain(argc, argv);
}