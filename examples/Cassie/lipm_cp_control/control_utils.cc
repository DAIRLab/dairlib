#include "examples/Cassie/lipm_cp_control/control_utils.h"

using Eigen::Vector3d;

namespace dairlib {

// Rotate a 3D vector by quaternion which represent a roatation
Vector3d rotateVecByQuaternion(
  Eigen::Quaterniond q, const Vector3d& v) {
  q.normalize();

  Eigen::Quaterniond p;
  p.w() = 0;
  p.vec() = v;
  Eigen::Quaterniond rotatedP = q * p * q.inverse();

  return rotatedP.vec();
}
// Rotate a 3D vector from global frame to local frame
// by floating-base's quaternion
Vector3d rotateVecFromGlobalToLocalByQuaternion(
  const Eigen::Quaterniond& q, const Vector3d& v) {
  return rotateVecByQuaternion(q, v);
}
// Rotate a 3D vector from local frame to global frame
// by floating-base's quaternion
Vector3d rotateVecFromLocalToGlobalByQuaternion(
  const Eigen::Quaterniond& q, const Vector3d& v) {
  return rotateVecByQuaternion(q.conjugate(), v);
}


}  // namespace dairlib
