#include <math.h>
#include "systems/framework/output_vector.h"

namespace dairlib {

// The functions here assume that Cassie is quaternion floating based.

Eigen::Vector3d rotateVecByQuaternion(
  Eigen::Quaterniond& q, const Eigen::Vector3d& v);
Eigen::Vector3d rotateVecFromGlobalToLocalByQuaternion(
  const Eigen::Quaterniond& q, const Eigen::Vector3d& v);
Eigen::Vector3d rotateVecFromLocalToGlobalByQuaternion(
  const Eigen::Quaterniond& q, const Eigen::Vector3d& v);

}  // namespace dairlib