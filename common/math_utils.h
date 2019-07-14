#pragma once

#include <Eigen/Dense>

namespace dairlib {

// If the norm of quaternion is 0, set it to unit quaternion. Otherwise,
// normalize it. Additionally, we make the first element to be always positive.
Eigen::Quaterniond NormalizeQuaternion(Eigen::Quaterniond q);
Eigen::VectorXd NormalizeQuaternion(Eigen::VectorXd q);

}  // namespace dairlib
