#include <math.h>
#include "systems/framework/output_vector.h"

using Eigen::VectorXd;

namespace dairlib {

// The functions here assume that Cassie is quaternion floating based.

Vector3d rotateVecByQuaternion(
  Eigen::Quaterniond& q, const Vector3d& v);
Vector3d rotateVecFromGlobalToLocalByQuaternion(
  const Eigen::Quaterniond& q, const Vector3d& v);
Vector3d rotateVecFromLocalToGlobalByQuaternion(
  const Eigen::Quaterniond& q, const Vector3d& v);

}  // namespace dairlib