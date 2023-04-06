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

Eigen::VectorXd eigen_clamp(
    const Eigen::VectorXd& value, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub){
  DRAKE_DEMAND(value.size() == lb.size());
  DRAKE_DEMAND(value.size() == ub.size());
  Eigen::VectorXd clamped_value(value.size());
  for (int i = 0; i < value.size(); ++i){
    clamped_value[i] = std::clamp(value[i], lb[i], ub[i]);
  }
  return clamped_value;
}