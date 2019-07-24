#pragma once

#include <Eigen/Dense>

namespace dairlib {

// If the norm of quaternion is 0, set it to identity quaternion.
// Input: the configuration of the robot which must be in floating-base
void CheckZeroQuaternion(Eigen::VectorXd* q);

}  // namespace dairlib
