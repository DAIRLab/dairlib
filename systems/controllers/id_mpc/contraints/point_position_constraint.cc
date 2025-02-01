#include "point_position_constraint.h"
#include "multibody/multibody_utils.h"

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::VectorX;
using drake::AutoDiffXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;

using drake::math::ExtractValue;
using drake::math::ExtractGradient;
using drake::math::InitializeAutoDiff;

template <typename T>
PointPositionConstraint<T>::PointPositionConstraint(
    const ConstrainedDynamicsInfo &dynamics, std::string frame,
    const Vector3d &point_in_frame) : solvers::NonlinearConstraint<T>(
        3, dynamics.nq() + 3, Vector3d::Zero(), Vector3d::Zero()),
                                      plant_(dynamics.get_plant()),
                                      body_name_(frame),
                                      point_(point_in_frame) {
  context_ = plant_.CreateDefaultContext();
}

template<>
void PointPositionConstraint<double>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<double>>& x, VectorX<double>* y) const {
  if (not body_name_.empty()) {
    const VectorXd& q = x.head(plant_.num_positions());
    const Vector3d& p = x.tail<3>();
    Vector3d phi;
    multibody::SetPositionsIfNew<double>(plant_, q, context_.get());
    plant_.CalcPointsPositions(
        *context_, plant_.GetBodyByName(body_name_).body_frame(),
        point_, plant_.world_frame(),&phi);
    *y = phi - p;
  } else {
    *y = VectorXd::Zero(3);
  }
}

template <>
void PointPositionConstraint<AutoDiffXd>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<AutoDiffXd>>& x, VectorX<AutoDiffXd>* y) const {
  VectorXd xd = ExtractValue(x);
  MatrixXd ddx = ExtractGradient(x);

  if (not body_name_.empty()) {
    const VectorXd& q = xd.head(plant_.num_positions());
    const Vector3d& p = xd.tail<3>();
    Vector3d phi;

    multibody::SetPositionsIfNew<double>(plant_, q, context_.get());
    plant_.CalcPointsPositions(
        *context_, plant_.GetBodyByName(body_name_).body_frame(),
        point_, plant_.world_frame(), &phi);
    VectorXd yd = phi - p;

    MatrixXd J = MatrixXd::Zero(3, plant_.num_positions());
    plant_.CalcJacobianTranslationalVelocity(
        *context_, drake::multibody::JacobianWrtVariable::kQDot,
        plant_.GetBodyByName(body_name_).body_frame(), point_,
        plant_.world_frame(), plant_.world_frame(), &J);

    MatrixXd grad = MatrixXd::Zero(3, plant_.num_positions() + 3);
    grad.leftCols(plant_.num_positions()) = J;
    grad.rightCols<3>() = -Matrix3d::Identity();
    if (ddx.isIdentity(1e-16)) {
      *y = InitializeAutoDiff(yd, grad);
    } else {
      *y = InitializeAutoDiff(yd, ddx * grad);
    }
  } else {
    VectorXd yd = Vector3d::Zero();
    MatrixXd grad = MatrixXd::Zero(3, plant_.num_positions() + 3);
    *y = InitializeAutoDiff(yd, grad);
  }
}

template class PointPositionConstraint<double>;
template class PointPositionConstraint<AutoDiffXd>;

}