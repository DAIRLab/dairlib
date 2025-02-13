#include "relative_position_cost.h"
#include "multibody/multibody_utils.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::VectorXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;

using drake::VectorX;
using drake::math::ExtractValue;
using drake::math::ExtractGradient;
using drake::math::InitializeAutoDiff;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;

RelativePositionCost::RelativePositionCost(
    const MatrixXd &Q, const Vector3d& yref,
    const MultibodyPlant<double>& plant,
    const std::string& from_frame, const std::string& to_frame,
    const Vector3d& from_point, const Vector3d& to_point,
    const std::string& description) : NonlinearLeastSquaresCost<double>(
        plant.num_positions(), 3, Q, description),
        plant_(plant),
        frame_from_(plant.GetBodyByName(from_frame).body_frame()),
        frame_to_(plant.GetBodyByName(to_frame).body_frame()),
        point_from_(from_point),
        point_to_(to_point) {
  context_ = plant_.CreateDefaultContext();
}

void RelativePositionCost::EvaluateInnerTerm(
    const Eigen::Ref<const drake::AutoDiffVecXd> &x,
    drake::AutoDiffVecXd *y) const{

  VectorXd q = ExtractValue(x);
  multibody::SetPositionsIfNew<double>(plant_, q, context_.get());

  Vector3d p_from;
  Vector3d p_to;
  MatrixXd J_from = MatrixXd::Zero(3, plant_.num_positions());
  MatrixXd J_to = MatrixXd::Zero(3, plant_.num_positions());

  plant_.CalcPointsPositions(
      *context_, frame_from_, point_from_, plant_.world_frame(), &p_from);
  plant_.CalcPointsPositions(
      *context_, frame_to_, point_to_, plant_.world_frame(), &p_to);

  plant_.CalcJacobianTranslationalVelocity(
      *context_, drake::multibody::JacobianWrtVariable::kQDot, frame_from_,
      point_from_, plant_.world_frame(), plant_.world_frame(), &J_from);

  plant_.CalcJacobianTranslationalVelocity(
      *context_, drake::multibody::JacobianWrtVariable::kQDot, frame_to_,
      point_to_, plant_.world_frame(), plant_.world_frame(), &J_to);

  VectorXd yd = (p_to - p_from) - y_;
  MatrixXd grad = J_to - J_from;
  MatrixXd ddx = ExtractGradient(x);
  if (ddx.isIdentity(1e-16)) {
    *y = InitializeAutoDiff(yd, grad);
  } else {
    *y = InitializeAutoDiff(yd, ddx * grad);
  }
}

void RelativePositionCost::EvaluateInnerTerm(
    const Eigen::Ref<const VectorXd> &x, VectorXd *y) const{

  Vector3d p_from;
  Vector3d p_to;

  plant_.CalcPointsPositions(
      *context_, frame_from_, point_from_, plant_.world_frame(), &p_from);
  plant_.CalcPointsPositions(
      *context_, frame_to_, point_to_, plant_.world_frame(), &p_to);

  *y = (p_to - p_from) - y_;
}

}