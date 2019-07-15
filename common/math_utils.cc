#include "common/math_utils.h"
#include <Eigen/Dense>

namespace dairlib {

Eigen::Quaterniond NormalizeQuaternion(Eigen::Quaterniond q) {
  double n = q.norm();
  if (n > 0.0) {
    if (q.w() < 0.0){
      n = -n;
    }
    q.w() = q.w() / n;
    q.x() = q.x() / n;
    q.y() = q.y() / n;
    q.z() = q.z() / n;
  } else {
    q.w() = 1.0;
    q.x() = 0.0;
    q.y() = 0.0;
    q.z() = 0.0;
  }
  return q;
}
Eigen::VectorXd NormalizeQuaternion(Eigen::VectorXd q) {
  double n = q.norm();
  if (n > 0.0) {
    if (q(0) < 0.0){
      n = -n;
    }
    q(0) = q(0) / n;
    q(1) = q(1) / n;
    q(2) = q(2) / n;
    q(3) = q(3) / n;
  } else {
    q(0) = 1.0;
    q(1) = 0.0;
    q(2) = 0.0;
    q(3) = 0.0;
  }
  return q;
}

}  // namespace dairlib
