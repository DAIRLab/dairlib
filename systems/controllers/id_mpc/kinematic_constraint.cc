#include "kinematic_constraint.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::VectorX;
using Eigen::VectorXd;

template <typename T>
KinematicConstraint<T>::KinematicConstraint(KnotPointState *x) :
solvers::NonlinearConstraint<T>(
    x->get_dynamics().n_constraint_total(),
    x->get_dynamics().variable_count(),
    VectorXd::Zero(3 * x->get_dynamics().n_constraint_total()),
    VectorXd::Zero(3 * x->get_dynamics().n_constraint_total())), x_(x){}


template <typename T>
void KinematicConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>> &x, VectorX<T> *y) const {
  DRAKE_ASSERT(x.rows() == x_->get_dynamics().variable_count());
  x_->Update<T>(x);
  *y = x_->GetKinematicConstraints<T>();
}

template class KinematicConstraint<double>;
template class KinematicConstraint<AutoDiffXd>;

}