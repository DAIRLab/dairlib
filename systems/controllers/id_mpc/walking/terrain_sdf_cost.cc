#include <iostream>
#include "terrain_sdf_cost.h"
#include "multibody/multibody_utils.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace dairlib::solvers::sqp {

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

namespace {

template<typename T>
T smooth_min(T a, T b, double lambda) {
  T exp_a = exp(a / lambda);
  T exp_b = exp(b / lambda);

  return (exp_b * a  + exp_a * b) / (exp_a + exp_b);
}

}

TerrainSDFCost::TerrainSDFCost(
    const MatrixXd &Q, const VectorXd& yref,
    const MultibodyPlant<double>& plant,
    const std::string& frame,
    const Vector3d& point,
    grid_map::SignedDistanceField* sdf,
    const std::string& description) : NonlinearLeastSquaresCost<double>(
    plant.num_positions(), 1, Q, description),
    plant_(plant),
    frame_(&plant.GetBodyByName(frame).body_frame()),
    point_(point),
    sdf_(sdf){
  context_ = plant_.CreateDefaultContext();
  DRAKE_DEMAND(sdf_ != nullptr);
}

void TerrainSDFCost::EvaluateInnerTerm(
    const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::VectorXd *y) const {

  DRAKE_DEMAND(sdf_ != nullptr);

  multibody::SetPositionsIfNew<double>(plant_, x, context_.get());

  Vector3d p;

  plant_.CalcPointsPositions(
      *context_, *frame_, point_, plant_.world_frame(), &p);

  double signed_distance = sdf_->value(p);
  *y =  VectorXd::Constant(1, signed_distance) - y_;
}

void TerrainSDFCost::EvaluateInnerTerm(const Eigen::Ref<const drake::AutoDiffVecXd> &x,
                                       drake::AutoDiffVecXd *y) const {
  DRAKE_DEMAND(not y_.hasNaN());
  DRAKE_DEMAND(sdf_ != nullptr);

  VectorXd q = ExtractValue(x);
  multibody::SetPositionsIfNew<double>(plant_, q, context_.get());

  Vector3d p;
  MatrixXd J = MatrixXd::Zero(3, plant_.num_positions());

  plant_.CalcPointsPositions(
      *context_, *frame_, point_, plant_.world_frame(), &p);

  plant_.CalcJacobianTranslationalVelocity(
      *context_, drake::multibody::JacobianWrtVariable::kQDot, *frame_,
      point_, plant_.world_frame(), plant_.world_frame(), &J);

  const auto [phi, dphi_dp] = sdf_->valueAndDerivative(p);

  //dphi_dq = dphi_dp * dp_dq

  VectorXd yd = VectorXd::Constant(1, phi);
  MatrixXd grad = dphi_dp.transpose() * J;
  MatrixXd ddx = ExtractGradient(x);

  drake::AutoDiffXd y_ad(y_(0));

  if (ddx.isIdentity(1e-16)) {
    *y = InitializeAutoDiff(yd, grad);
  } else {
    *y = InitializeAutoDiff(yd, ddx * grad);
  }
  (*y)(0) = smooth_min((*y)(0), y_ad, 10.0) - y_ad;
}

}