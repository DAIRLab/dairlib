#include "kinematic_constraint.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::VectorX;
using Eigen::VectorXd;

template <typename T>
KinematicConstraint<T>::KinematicConstraint(
    const KnotPoint& k, KnotPointState *x) :
solvers::NonlinearConstraint<T>(
    k.kinematic_constraint_dimension(),
    k.num_state_variables(),
    VectorXd::Zero(k.kinematic_constraint_dimension()),
    VectorXd::Zero(k.kinematic_constraint_dimension())),
    k_(k), x_(x){}


template <typename T>
void KinematicConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>> &x, VectorX<T> *y) const {
  DRAKE_ASSERT(x.rows() == k_.num_state_variables());
  *y = k_.EvalKinematicConstraints<T>(x_, x);
}

template class KinematicConstraint<double>;
template class KinematicConstraint<AutoDiffXd>;

}