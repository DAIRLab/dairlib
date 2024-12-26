#pragma once
#include "knot_point_state.h"
#include "solvers/nonlinear_constraint.h"

namespace dairlib::systems::controllers::id_mpc {

template <typename T>
class KinematicConstraint : public solvers::NonlinearConstraint<T> {
 public:
  explicit KinematicConstraint(KnotPointState* x);

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

 private:
  KnotPointState* x_;

};

}