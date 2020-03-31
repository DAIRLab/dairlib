#include "eigen_utils.h"

std::vector<double> ConvertVectorXdToStdVector(
    const Eigen::VectorXd& eigen_vec) {
  return std::vector<double>(eigen_vec.data(),
                             eigen_vec.data() + eigen_vec.size());
}