#pragma once
#include "systems/controllers/id_mpc/core/knot_point.h"
#include "solvers/nonlinear_constraint.h"

namespace dairlib::systems::controllers::id_mpc {

template <typename T>
class KinematicConstraint : public solvers::NonlinearConstraint<T> {
 public:
  explicit KinematicConstraint(const KnotPoint& k, KnotPointState* x);

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

 private:
  const KnotPoint& k_;
  KnotPointState* x_;

};

}