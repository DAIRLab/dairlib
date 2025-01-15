#include "quadratic_error_cost.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::VectorX;
using drake::AutoDiffXd;

using Eigen::VectorXd;
using Eigen::MatrixXd;

template <typename T>
QuadraticErrorCost<T>::QuadraticErrorCost(const MatrixXd& Q, const VectorXd& x,
                                          const std::string& description) :
    NonlinearLeastSquaresCost<T>(x.rows(), x.rows(), Q, description), x_ref_(x) {
  DRAKE_DEMAND(x_ref_.rows() == Q.rows());
}

template<typename T>
void QuadraticErrorCost<T>::EvaluateInnerTerm(
    const Eigen::Ref<const VectorX<AutoDiffXd>>& x, VectorX<AutoDiffXd>* y) const {
  *y = x - x_ref_;
}

template<typename T>
void QuadraticErrorCost<T>::EvaluateInnerTerm(
    const Eigen::Ref<const VectorX<double>>& x, VectorX<double>* y) const {
  *y = x - x_ref_;
}

template <typename T>
GaussNewtonApproximation QuadraticErrorCost<T>::CalcGaussNewtonApproximation(
    const Eigen::Ref<const Eigen::VectorXd>& x) const {
  VectorXd y = x - x_ref_;
  return {
    2 * this->Q_,
    2 * this->Q_ * y,
    y.dot(this->Q_ * y)
  };
}
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::systems::controllers::id_mpc::QuadraticErrorCost)