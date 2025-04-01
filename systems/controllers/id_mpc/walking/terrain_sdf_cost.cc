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

constexpr double lambda = 3.0;

template<typename T>
T smooth_min(T a, T b) {
  T exp_a = exp(a * lambda);
  T exp_b = exp(b * lambda);

  return (exp_b * a  + exp_a * b) / (exp_a + exp_b);
}

}

TerrainSDFCost::TerrainSDFCost(
    const MatrixXd &Q, const VectorXd& yref,
    const MultibodyPlant<double>& plant,
    const std::string& frame,
    const Vector3d& point,
    multibody::BoxSet* box_set,
    const std::string& description) : NonlinearLeastSquaresCost<double>(
    plant.num_positions(), 1, Q, description),
    plant_(plant),
    frame_(&plant.GetBodyByName(frame).body_frame()),
    front_point_(point),
    rear_point_(point),
    box_set_(box_set){
  context_ = plant_.CreateDefaultContext();
  DRAKE_DEMAND(box_set_ != nullptr);
}

void TerrainSDFCost::EvaluateInnerTerm(
    const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::VectorXd *y) const {

  DRAKE_DEMAND(box_set_ != nullptr);

  multibody::SetPositionsIfNew<double>(plant_, x, context_.get());

  Vector3d pf;
  Vector3d pr;

  plant_.CalcPointsPositions(
      *context_, *frame_, front_point_, plant_.world_frame(), &pf);

  const auto [phi_f, gf] = box_set_->CalcSDF(pf);
  const auto [phi_r, gr] = box_set_->CalcSDF(pr);

  double smooth_min_y = smooth_min(std::min(phi_f, phi_r), y_(0));

  *y =  VectorXd::Constant(1, smooth_min_y) - y_;
}

void TerrainSDFCost::EvaluateInnerTerm(const Eigen::Ref<const drake::AutoDiffVecXd> &x,
                                       drake::AutoDiffVecXd *y) const {
  DRAKE_DEMAND(not y_.hasNaN());
  DRAKE_DEMAND(box_set_ != nullptr);

  VectorXd q = ExtractValue(x);
  multibody::SetPositionsIfNew<double>(plant_, q, context_.get());

  Vector3d pf;
  Vector3d pr;

  plant_.CalcPointsPositions(
      *context_, *frame_, front_point_, plant_.world_frame(), &pf);
  plant_.CalcPointsPositions(
      *context_, *frame_, rear_point_, plant_.world_frame(), &pr);

  const auto [phi_f, dphi_f_dp] = box_set_->CalcSDF(pf);
  const auto [phi_r, dphi_r_dp] = box_set_->CalcSDF(pr);

  double phi;
  Vector3d dphi_dp;
  MatrixXd J = MatrixXd::Zero(3, plant_.num_positions());

  if (phi_f < phi_r) {
    phi = phi_f;
    dphi_dp = dphi_f_dp;
    plant_.CalcJacobianTranslationalVelocity(
        *context_, drake::multibody::JacobianWrtVariable::kQDot, *frame_,
        front_point_, plant_.world_frame(), plant_.world_frame(), &J);
  } else {
    phi = phi_r;
    dphi_dp = dphi_r_dp;
    plant_.CalcJacobianTranslationalVelocity(
        *context_, drake::multibody::JacobianWrtVariable::kQDot, *frame_,
        rear_point_, plant_.world_frame(), plant_.world_frame(), &J);
  }

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
  (*y)(0) = smooth_min((*y)(0), y_ad) - y_ad;
}

}