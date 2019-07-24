#include "common/math_utils.h"
#include <Eigen/Dense>

namespace dairlib {

void CheckZeroQuaternion(Eigen::VectorXd* q) {
  if (q->segment(3,4).norm() == 0.0) {
    (*q)(3) = 1.0;
  }
}

}  // namespace dairlib
