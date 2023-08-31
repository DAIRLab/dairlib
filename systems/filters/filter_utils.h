#pragma once

#include <complex>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include "drake/common/drake_assert.h"

namespace dairlib::systems::filter_utils {

/// Container for a state space representation of a discrete time
/// SISO butterworth filter with the output as the last element of the state,
/// e.g. C = [0 0 0 ... 0 1], D = 0
struct DiscreteSISOButterworthFilter {
  Eigen::MatrixXd A_;
  Eigen::VectorXd B_;
  Eigen::VectorXd UpdateFilter(const Eigen::VectorXd &x, double u) const {
    return A_ * x + B_ * u;
  };
  static double GetFilterOutput(const Eigen::VectorXd &x) {
    return x.tail(1)(0);
  }
  static void ValidateA(const Eigen::MatrixXd& A) {
    Eigen::VectorXcd eig = A.eigenvalues();
    for (int i = 0; i < eig.size(); i++) {
      DRAKE_DEMAND(norm(eig(i)) < 1.0);
    }
  }
};

/// Generates the butterworth filter - Note that this implementation is
/// implemented naively and therefore:
/// - is likely not numerically stable for high-order filters
/// - only supports an even filter order
DiscreteSISOButterworthFilter butter(int order, double w_c);
inline DiscreteSISOButterworthFilter butter(int order, double f_s, double f_c) {
  DRAKE_DEMAND(f_s > f_c);
  return butter(order, 0.5 * f_c / f_s);
}

}