#include "eigen_utils.h"

std::vector<double> CopyVectorXdToStdVector(
    const Eigen::VectorXd& eigen_vec) {
  return std::vector<double>(eigen_vec.data(),
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

std::vector<std::vector<double>> CopyMatrixXdToVectorOfVectors(
    const Eigen::MatrixXd& eigen_mat) {
  const int m = eigen_mat.rows();
  const int n = eigen_mat.cols();

  auto out = std::vector<std::vector<double>>(m, std::vector<double>(n));

  for (int i = 0; i < m; ++i) {
    // Temporary copy due to underlying data of Eigen::Matrix
    // being column major
    Eigen::VectorXd tempRow = eigen_mat.row(i);
    memcpy(out.at(i).data(), tempRow.data(), sizeof(double) * n);
  }

  return out;
}