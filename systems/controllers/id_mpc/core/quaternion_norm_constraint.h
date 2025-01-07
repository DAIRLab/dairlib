#pragma once
#include "solvers/nonlinear_constraint.h"

namespace dairlib::systems::controllers::id_mpc {

template<typename T>
class QuaternionNormConstraint : public solvers::NonlinearConstraint<T> {
 public:
  QuaternionNormConstraint();
  ~QuaternionNormConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>> &x,
                          drake::VectorX<T> *y) const override;
};
}