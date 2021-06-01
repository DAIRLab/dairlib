#include "eigen_utils.h"

std::vector<double> CopyVectorXdToStdVector(const Eigen::VectorXd& eigen_vec) {
  return std::vector<double>(eigen_vec.data(),
                             eigen_vec.data() + eigen_vec.size());
}

void cnpy2eigen(std::string data_fname, Eigen::MatrixXd& mat_out) {
  cnpy::NpyArray npy_data = cnpy::npy_load(data_fname);
  int data_row = npy_data.shape[0];
  int data_col = npy_data.shape[1];
  double* ptr =
      static_cast<double*>(malloc(data_row * data_col * sizeof(double)));
  memcpy(ptr, npy_data.data<double>(), data_row * data_col * sizeof(double));
  new (&mat_out)
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(
          reinterpret_cast<double*>(ptr), data_col, data_row);
}