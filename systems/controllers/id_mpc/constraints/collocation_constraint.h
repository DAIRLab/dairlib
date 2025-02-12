#pragma once

#include "systems/controllers/id_mpc/core/knot_point.h"
#include "solvers/nonlinear_constraint.h"

namespace dairlib::systems::controllers::id_mpc {

template <typename T>
 class CollocationConstraint : public
     solvers::NonlinearConstraint<T> {
  public:
   CollocationConstraint(
       const KnotPoint& k0, const KnotPoint& k1,
       KnotPointState* x0, KnotPointState* x1);

   void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                           drake::VectorX<T>* y) const override;

  private:
   const KnotPoint& k0_;
   const KnotPoint& k1_;
   KnotPointState* x0_;
   KnotPointState* x1_;
 };
}