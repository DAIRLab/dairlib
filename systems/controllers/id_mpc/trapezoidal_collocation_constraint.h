#pragma once

#include "knot_point_state.h"
#include "solvers/nonlinear_constraint.h"

namespace dairlib::systems::controllers::id_mpc {

template <typename T>
 class TrapezoidalCollocationConstraint : public
     solvers::NonlinearConstraint<T> {
  public:
   TrapezoidalCollocationConstraint(KnotPointState* x0, KnotPointState* x1,
                                    double dt);

   void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                           drake::VectorX<T>* y) const override;

   void set_dt(double dt) { dt_ = dt;}
   double dt() {return dt_; }

  private:
   KnotPointState* x0_state_;
   KnotPointState* x1_state_;
   double dt_;
 };
}