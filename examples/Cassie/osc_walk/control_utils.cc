#include "examples/Cassie/osc_walk/control_utils.h"

using Eigen::Vector3d;

namespace dairlib {

Vector3d RotateVecByQuaternion(
  Eigen::Quaterniond q, const Vector3d& v) {
  q.normalize();

  Eigen::Quaterniond p;
  p.w() = 0;
  p.vec() = v;
  Eigen::Quaterniond rotatedP = q * p * q.inverse();

  return rotatedP.vec();
}

Vector3d RotateVecFromGlobalToLocalByQuaternion(
  const Eigen::Quaterniond& q, const Vector3d& v) {
  return RotateVecByQuaternion(q.conjugate(), v);
}

Vector3d RotateVecFromLocalToGlobalByQuaternion(
  const Eigen::Quaterniond& q, const Vector3d& v) {
  return RotateVecByQuaternion(q, v);
}


}  // namespace dairlib
