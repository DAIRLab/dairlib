#include "systems/controllers/osc/osc_utils.h"

using Eigen::MatrixXd;

namespace dairlib {
namespace systems {
namespace controllers {

Eigen::MatrixXd PositionMapFromSpringToNoSpring(
    const RigidBodyTree<double>& tree_w_spr,
    const RigidBodyTree<double>& tree_wo_spr) {
  int n_q_wo_spr = tree_wo_spr.get_num_positions();
  int n_q_w_spr = tree_w_spr.get_num_positions();
  MatrixXd ret = MatrixXd::Zero(n_q_wo_spr, n_q_w_spr);
  for (int i = 0; i < n_q_wo_spr; i++) {
    bool successfully_added = false;
    for (int j = 0; j < n_q_w_spr; j++) {
      std::string name_wo_spr = tree_wo_spr.get_position_name(i);
      std::string name_w_spr = tree_w_spr.get_position_name(j);
      if (name_wo_spr.compare(0, name_wo_spr.size(), name_w_spr) == 0) {
        ret(i, j) = 1;
        successfully_added = true;
      }
    }
    DRAKE_DEMAND(successfully_added);
  }
  return ret;
}
Eigen::MatrixXd VelocityMapFromSpringToNoSpring(
    const RigidBodyTree<double>& tree_w_spr,
    const RigidBodyTree<double>& tree_wo_spr) {
  int n_v_wo_spr = tree_wo_spr.get_num_velocities();
  int n_v_w_spr = tree_w_spr.get_num_velocities();
  MatrixXd ret = MatrixXd::Zero(n_v_wo_spr, n_v_w_spr);
  for (int i = 0; i < n_v_wo_spr; i++) {
    bool successfully_added = false;
    for (int j = 0; j < n_v_w_spr; j++) {
      std::string name_wo_spr = tree_wo_spr.get_velocity_name(i);
      std::string name_w_spr = tree_w_spr.get_velocity_name(j);
      if (name_wo_spr.compare(0, name_wo_spr.size(), name_w_spr) == 0) {
        ret(i, j) = 1;
        successfully_added = true;
      }
    }
    DRAKE_DEMAND(successfully_added);
  }
  return ret;
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
