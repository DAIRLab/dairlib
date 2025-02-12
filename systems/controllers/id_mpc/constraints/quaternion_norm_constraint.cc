#include "quaternion_norm_constraint.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::VectorXd;
using drake::VectorX;
using drake::AutoDiffXd;

template<typename T>
QuaternionNormConstraint<T>::QuaternionNormConstraint()
    : solvers::NonlinearConstraint<T>(
        1, 4, VectorXd::Zero(1), VectorXd::Zero(1),
        "quaternion_norm_constraint") {}

template<typename T>
void QuaternionNormConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<T>> &x, drake::VectorX<T> *y) const {
  VectorX<T> output(1);
  output << x.norm() - 1;
  *y = output;
}

template class QuaternionNormConstraint<double>;
template class QuaternionNormConstraint<AutoDiffXd>;

}

