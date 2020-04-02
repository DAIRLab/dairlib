#pragma once

#include <Eigen/Dense>

#include "drake/multibody/rigid_body_tree.h"

namespace dairlib {
namespace systems {
namespace controllers {

// Initialize the mapping from spring to no spring
Eigen::MatrixXd PositionMapFromSpringToNoSpring(
    const RigidBodyTree<double>& tree_w_spr,
    const RigidBodyTree<double>& tree_wo_spr);
Eigen::MatrixXd VelocityMapFromSpringToNoSpring(
    const RigidBodyTree<double>& tree_w_spr,
    const RigidBodyTree<double>& tree_wo_spr);

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
