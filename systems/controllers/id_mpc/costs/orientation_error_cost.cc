#include "orientation_error_cost.h"
#include "drake/math/differentiable_norm.h"
#include "drake/math/rotation_matrix.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Quaternion;
using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::math::InitializeAutoDiff;
using drake::math::DifferentiableNorm;
using drake::math::RotationMatrix;

template <typename T>
OrientationErrorCost<T>::OrientationErrorCost(
    const MatrixXd &Q, const Vector4d &q_ref, const std::string& description) :
    NonlinearLeastSquaresCost<T>(4, 3, Q, description), q_(q_ref) {}

template<typename T>
void OrientationErrorCost<T>::EvaluateInnerTerm(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  Quaternion<AutoDiffXd> qd(q_(0), q_(1), q_(2), q_(3));
  Quaternion<AutoDiffXd> q(x(0), x(1), x(2), x(3));
  Eigen::AngleAxis<AutoDiffXd> diff(qd * q.inverse());
  *y = diff.angle() * diff.axis();
}

template<typename T>
void OrientationErrorCost<T>::EvaluateInnerTerm(
    const Eigen::Ref<const VectorXd>& x, VectorXd* y) const {
  Quaternion<double> qd(q_(0), q_(1), q_(2), q_(3));
  Quaternion<double> q(x(0), x(1), x(2), x(3));
  Eigen::AngleAxis<double> diff(qd * q.inverse());
  *y = diff.angle() * diff.axis();
}

}
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::systems::controllers::id_mpc::OrientationErrorCost)