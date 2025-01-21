#pragma once

#include "nonlinear_least_squares_cost.h"

#include "drake/math/quaternion.h"

namespace dairlib::systems::controllers::id_mpc {

template <typename T>
class OrientationErrorCost : public NonlinearLeastSquaresCost<T> {
 public:
  OrientationErrorCost(const Eigen::MatrixXd& Q, const Eigen::Vector4d& q_ref,
                       const std::string& description="");

  void EvaluateInnerTerm(const Eigen::Ref<const drake::AutoDiffVecXd> &x,
                         drake::AutoDiffVecXd *y) const override;

  void EvaluateInnerTerm(const Eigen::Ref<const Eigen::VectorXd> &x,
                         Eigen::VectorXd *y) const override;

  void UpdateReference(const Eigen::VectorXd& q) override {
    DRAKE_ASSERT(q.rows() == 4);
    q_ = q;
  }

 private:
  Eigen::Vector4d q_;
};

}

