#pragma once

#include <string>
#include "drake/multibody/rigid_body_tree.h"

namespace dairlib {
namespace multibody {

// Parse an LCM log file containing lcmt_robot_output into a vector of times
// and states. Pointers are used to set the outputs of this method.
// @param tree The RigidBodyTree
// @param file The filename for the log
// @param channel The LCM channel for the lcmt_robot_output message
// @param t A pointer to an Eigen vector for the timestamps (method output)
// @param t A pointer to an Eigen matrix for the state (method output)
// @param t A pointer to an Eigen matrix for the efforts (method output)
void parseLcmOutputLog(const RigidBodyTree<double>& tree, std::string file,
    std::string channel, Eigen::VectorXd* t, Eigen::MatrixXd* x,
    Eigen::MatrixXd* u, double duration = 1.0e6);

}  // namespace multibody
}  // namespace dairlib
