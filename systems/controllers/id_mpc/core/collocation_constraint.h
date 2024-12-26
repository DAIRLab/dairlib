#pragma once

#include "timeline.h"
#include "solvers/nonlinear_constraint.h"

namespace dairlib::systems::controllers::id_mpc {

template <typename T>
 class CollocationConstraint : public
     solvers::NonlinearConstraint<T> {
  public:
   CollocationConstraint(KnotPointState* x0, KnotPointState* x1,
                                    double dt);

   void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                           drake::VectorX<T>* y) const override;

  private:
   Timeline* knots_and_breaks_;
   double dt_;
 };
}