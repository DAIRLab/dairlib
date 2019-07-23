#include <math.h>
#include "systems/framework/output_vector.h"

namespace dairlib {

// Rotate a 3D vector by quaternion which represent a roatation
Eigen::Vector3d RotateVecByQuaternion(
  Eigen::Quaterniond q, const Eigen::Vector3d& v);

// Rotate a 3D vector from global frame to local frame
// by floating-base's quaternion
Eigen::Vector3d RotateVecFromGlobalToLocalByQuaternion(
  const Eigen::Quaterniond& q, const Eigen::Vector3d& v);

// Rotate a 3D vector from local frame to global frame
// by floating-base's quaternion
Eigen::Vector3d RotateVecFromLocalToGlobalByQuaternion(
  const Eigen::Quaterniond& q, const Eigen::Vector3d& v);

}  // namespace dairlib
