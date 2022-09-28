#include "eigen_utils.h"

std::vector<double> CopyVectorXdToStdVector(
    const Eigen::VectorXd& eigen_vec) {
  return std::vector<double>(eigen_vec.data(),
                             eigen_vec.data() + eigen_vec.size());
}

std::vector<float> CopyVectorXdToStdFloatVector(
    const Eigen::VectorXd& eigen_vec) {
  return std::vector<float>(eigen_vec.data(),
                            eigen_vec.data() + eigen_vec.size());
}