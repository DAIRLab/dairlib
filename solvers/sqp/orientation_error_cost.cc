#include "orientation_error_cost.h"
#include "drake/math/differentiable_norm.h"
#include "drake/math/rotation_matrix.h"

namespace dairlib::solvers::sqp {

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
using drake::VectorX;

namespace {

// Convert a quaternion (split into the scalar and vector parts)
// into the angle axis representation
// Copie dfrom the CERES solver
// https://github.com/ceres-solver/ceres-solver/blob/c29b5257e23f91d6a47c4db9d57350ed4985ea46/include/ceres/rotation.h#L345
template <typename T>
void QuaternionToAngleAxis(
    const Quaternion<T>& quaternion, VectorX<T>* angle_axis) {
  const T& q1 = quaternion.x();
  const T& q2 = quaternion.y();
  const T& q3 = quaternion.z();
  const T sin_theta_squared = q1 * q1 + q2 * q2 + q3 * q3;

  // For quaternions representing non-zero rotation, the conversion
  // is numerically stable.
  if (sin_theta_squared > 0) {
    T sin_theta = sqrt(sin_theta_squared);
    const T& cos_theta = quaternion.w();

    // If cos_theta is negative, theta is greater than pi/2, which
    // means that angle for the angle_axis vector which is 2 * theta
    // would be greater than pi.
    //
    // While this will result in the correct rotation, it does not
    // result in a normalized angle-axis vector.
    //
    // In that case we observe that 2 * theta ~ 2 * theta - 2 * pi,
    // which is equivalent saying
    //
    //   theta - pi = atan(sin(theta - pi), cos(theta - pi))
    //              = atan(-sin(theta), -cos(theta))
    //
    const T two_theta =
        T(2.0) * ((cos_theta < T(0.0)) ? atan2(-sin_theta, -cos_theta)
                                       : atan2(sin_theta, cos_theta));
    const T k = two_theta / sin_theta;
    (*angle_axis)[0] = q1 * k;
    (*angle_axis)[1] = q2 * k;
    (*angle_axis)[2] = q3 * k;
  } else {
    // For zero rotation, sqrt() will produce NaN in the derivative since
    // the argument is zero.  By approximating with a Taylor series,
    // and truncating at one term, the value and first derivatives will be
    // computed correctly when Jets are used.
    const T k(2.0);
    (*angle_axis)[0] = q1 * k;
    (*angle_axis)[1] = q2 * k;
    (*angle_axis)[2] = q3 * k;
  }
}
template void QuaternionToAngleAxis(
    const Quaternion<double>&, VectorX<double>*);
template void QuaternionToAngleAxis(
    const Quaternion<AutoDiffXd>&, VectorX<AutoDiffXd>*);
}

template <typename T>
OrientationErrorCost<T>::OrientationErrorCost(
    const MatrixXd &Q, const Vector4d &q_ref, const std::string& description) :
    NonlinearLeastSquaresCost<T>(4, 3, Q, description), q_(q_ref) {}

template<typename T>
void OrientationErrorCost<T>::EvaluateInnerTerm(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  Quaternion<double> qd(q_(0), q_(1), q_(2), q_(3));
  Quaternion<AutoDiffXd> q(x(0), x(1), x(2), x(3));
  Quaternion<AutoDiffXd> qrel = qd.template cast<AutoDiffXd>() * q.inverse();
  *y = AutoDiffVecXd::Zero(3);
  QuaternionToAngleAxis(qrel, y);
}

template<typename T>
void OrientationErrorCost<T>::EvaluateInnerTerm(
    const Eigen::Ref<const VectorXd>& x, VectorXd* y) const {
  Quaternion<double> qd(q_(0), q_(1), q_(2), q_(3));
  Quaternion<double> q(x(0), x(1), x(2), x(3));
  Quaternion<double> qrel = qd * q.inverse();
  *y = Vector3d::Zero();
  QuaternionToAngleAxis(qrel, y);
}

}
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::solvers::sqp::OrientationErrorCost)