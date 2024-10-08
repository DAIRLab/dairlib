#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Eigenvalues>
#include <iostream>

namespace py = pybind11;

namespace {
std::pair<Eigen::Vector3d, double> normalAndErrorFromCovariance(
    int numPoint, const Eigen::Vector3d& mean, const Eigen::Matrix3d& sumSquared) {
  const Eigen::Matrix3d covarianceMatrix = sumSquared / numPoint - mean * mean.transpose();

  // Compute Eigenvectors.
  // Eigenvalues are ordered small to large.
  // Worst case bound for zero eigenvalue from : https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
  solver.computeDirect(covarianceMatrix, Eigen::DecompositionOptions::ComputeEigenvectors);
  if (solver.eigenvalues()(1) > 1e-8) {
    Eigen::Vector3d unitaryNormalVector = solver.eigenvectors().col(0);

    // Check direction of the normal vector and flip the sign upwards
    if (unitaryNormalVector.z() < 0.0) {
      unitaryNormalVector = -unitaryNormalVector;
    }
    // The first eigenvalue might become slightly negative due to numerics.
    double squareError = (solver.eigenvalues()(0) > 0.0) ? solver.eigenvalues()(0) : 0.0;
    return {unitaryNormalVector, squareError};
  } else {  // If second eigenvalue is zero, the normal is not defined.
    return {Eigen::Vector3d::UnitZ(), 1e30};
  }
}

std::pair<Eigen::Vector3d, double> computeNormalAndErrorForWindow(const Eigen::MatrixXd& windowData, int kernel_size, double resolution)  {
  // Gather surrounding data.
  size_t nPoints = 0;
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  Eigen::Matrix3d sumSquared = Eigen::Matrix3d::Zero();

  for (int kernel_col = 0; kernel_col < kernel_size; ++kernel_col) {
    for (int kernel_row = 0; kernel_row < kernel_size; ++kernel_row) {
      float height = windowData(kernel_row, kernel_col);
      if (!std::isfinite(height)) {
        continue;
      }
      // No need to account for map offset. Will substract the mean anyway.
      Eigen::Vector3d point{-kernel_row * resolution, -kernel_col * resolution, height};
      nPoints++;
      sum += point;
      sumSquared.noalias() += point * point.transpose();
    }
  }

  if (nPoints < 3) {
    // Not enough points to establish normal direction
    return {Eigen::Vector3d::UnitZ(), 1e30};
  } else {
    const Eigen::Vector3d mean = sum / nPoints;
    return normalAndErrorFromCovariance(nPoints, mean, sumSquared);
  }
}

}

namespace dairlib {
namespace pydairlib {

PYBIND11_MODULE(segmentation_utils, m) {
  m.doc() = "";

  m.def("CalculateNormalsAndSquaredError",
        [](const py::EigenDRef<const Eigen::MatrixXd>& map, int kernel_size,
           double resolution) -> std::pair<Eigen::MatrixXd, Eigen::MatrixXd> {

    kernel_size = kernel_size % 2 == 0 ? kernel_size - 1 : kernel_size;
    kernel_size = std::max(kernel_size, 5);

    Eigen::MatrixXd inclination = Eigen::MatrixXd::Zero(map.rows(), map.cols());
    Eigen::MatrixXd error = Eigen::MatrixXd::Zero(map.rows(), map.cols());
    Eigen::Vector3d n;
    double sigma_squared;

    int pad = kernel_size / 2;
    for (int i = pad; i < map.rows() - pad; ++i) {
      for (int j = pad; j < map.cols() - pad; ++j) {
        std::tie(n, sigma_squared) = computeNormalAndErrorForWindow(
            map.block(i-pad, j-pad, kernel_size, kernel_size), kernel_size, resolution);
        inclination(i, j) = n.z();
        error(i, j) = sigma_squared;
      }
    }

    return {inclination, error};
  }, py::arg("map"), py::arg("kernel_size"), py::arg("resolution"));
}

}
}