#include "legendre.h"
#include <cassert>

namespace dairlib::polynomials {

namespace {
long factorial(long i) {
  long f = 1;
  for (int j = 1; j <= i; ++j) {
    f *= j;
  }
  return f;
}
}

Eigen::MatrixXd MakeChangeOfBasisOperatorFromLegendreToMonomials(int order) {
  int N = order + 1;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(N, N);

  for (int col = 0; col < N; ++col) {
//    int row_offset = (col % 2 == 0) ? 0 : 1;
    int M = (col % 2 == 0) ? col / 2 : (col - 1) / 2;
    for (int m = 0; m <= M; ++m) {
      // the ith column contains the power series coefficients for the ith
      // legendre polynomial
      double pow_neg_1 = (m % 2 == 0) ? 1 : -1;
      long n = col;
      long double num = factorial(2*n - 2*m);
      long double denom = powl(2, n) * factorial(m) * factorial(n - m) * factorial(n - 2 * m);

      int row_idx = n - 2 * m ;//+ row_offset;
      B(row_idx, col) = pow_neg_1 * num / denom;
    }
  }
  return B;
}

Eigen::MatrixXi LegendreBasisDerivativeOperator(int order) {
  int N = order + 1;
  Eigen::MatrixXi D = Eigen::MatrixXi::Zero(N, N);

  for (int i  = 0;  i < order; ++i) {
    int row = i+1;
    int o = 0;
    for (int j = i; j >= 0; j -= 2) {
      D(row, j) = 2 * (i - o) + 1;
      o += 2;
    }
  }
  return D;
}

double LegendreBasisInner(const Eigen::VectorXd &u, const Eigen::VectorXd& v) {
  double dot = 0;
  assert(u.rows() == v.rows());
  int n = u.rows();
  for (int i = 0; i < n; ++i) {
    dot += u(i) * v(i) * static_cast<double>(2.0 * i) / static_cast<double>(2 * n + 1);
  }
  return dot;
}

Eigen::VectorXd EvalLegendreBasis(int order, double t) {
  assert(fabs(t) <= 1);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(order + 1);
  for (int i = 0; i <= order; ++i) {
    b(i) = std::legendre(i, t);
  }
  return b;
}

Eigen::VectorXd EvalLegendreBasisDerivative(int order, int deriv, double t) {
  assert(fabs(t) <= 1);

  int N = order + 1;

  Eigen::VectorXd d = Eigen::VectorXd::Zero(N);
  Eigen::MatrixXi D = LegendreBasisDerivativeOperator(order);
  Eigen::MatrixXi op = Eigen::MatrixXi::Identity(N, N);

  for (int i = 0; i < deriv; ++i) {
    op = D * op;
  }

  for (int i = 0; i <= order; ++i) {
    double sum = 0;
    for (int j = 0; j < N; ++j) {
      if (op(i, j) != 0) {
        sum += op(i, j) * std::legendre(j, t);
      }
    }
    d(i) = sum;
  }
  return d;
}

Eigen::MatrixXd MakeCostMatrixForMinimizingPathDerivativeSquaredWithLegendreBasis(
    int poly_order, int deriv_order) {
  assert(deriv_order >= 1);
  assert(poly_order >= deriv_order);

  int N = poly_order + 1;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(N, N);

  Eigen::MatrixXi B = Eigen::MatrixXi::Identity(N, N);
  Eigen::MatrixXi diff_operator = LegendreBasisDerivativeOperator(poly_order);

  for (int i = 0; i < deriv_order; ++i) {
    B = diff_operator * B;
  }

  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      Q(i, j) = LegendreBasisInner(
          B.col(i).cast<double>(), B.col(j).cast<double>());
    }
  }
  return Q;
}

}