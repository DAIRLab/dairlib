#pragma once
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

using drake::MatrixX;
using drake::VectorX;

/// CopyVectorXdToStdVector returns an std::vector<double> which is converted
/// from an Eigen::VectorXd.
std::vector<double> CopyVectorXdToStdVector(const Eigen::VectorXd& eigen_vec);
std::vector<float> CopyVectorXdToStdFloatVector(
    const Eigen::VectorXd& eigen_vec);

Eigen::VectorXd eigen_clamp(const Eigen::VectorXd& value,
                            const Eigen::VectorXd& lb,
                            const Eigen::VectorXd& ub);
template <typename Scalar>
inline MatrixX<Scalar> KronDiag(
    const MatrixX<Scalar>& diagonal_matrix, const MatrixX<Scalar>& X) {

  MatrixX<Scalar> P(
      diagonal_matrix.rows() * X.rows(), diagonal_matrix.cols() * X.cols()
  );
  P.setZero();

  for(int i = 0; i < diagonal_matrix.rows(); ++i) {
    P.block(i * X.rows(), i * X.cols(), X.rows(), X.cols()) = diagonal_matrix(i,i) * X;
  }
  return P;
}

template <typename Scalar>
inline MatrixX<Scalar> BlockDiagonalRepeat(MatrixX<Scalar> X, int n) {
  MatrixX<Scalar> P(n * X.rows(), n * X.cols());
  P.setZero();

  for (int i = 0; i < n; ++i) {
    P.block(i * X.rows(), i * X.cols(), X.rows(), X.cols()) = X;
  }
  return P;
}

template<typename Scalar>
inline VectorX<Scalar> stack(const std::vector<VectorX<Scalar>>& v) {
  VectorX<Scalar> out(v.size() * v.front().rows());

  Eigen::Index start = 0;
  for (const auto& e: v) {
    if (out.rows() - start < e.rows()) {
      out.conservativeResize(start + e.rows());
    }
    out.segment(start, e.rows()) = e;
  }
  return out;
}