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

constexpr double lambda = 10.0;

template<typename T>
T smooth_min(T a, T b) {
  T exp_a = exp(-a * lambda);
  T exp_b = exp(-b * lambda);
  return -1.0 / (lambda) * log(0.5 * exp_a + 0.5 * exp_b);
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

  double smooth_min_y = smooth_min(phi_f, phi_r);

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

  MatrixXd Jf = MatrixXd::Zero(3, plant_.num_positions());
  MatrixXd Jr = MatrixXd::Zero(3, plant_.num_positions());

  plant_.CalcJacobianTranslationalVelocity(
      *context_, drake::multibody::JacobianWrtVariable::kQDot, *frame_,
      front_point_, plant_.world_frame(), plant_.world_frame(), &Jf);

  plant_.CalcJacobianTranslationalVelocity(
      *context_, drake::multibody::JacobianWrtVariable::kQDot, *frame_,
      rear_point_, plant_.world_frame(), plant_.world_frame(), &Jf);

  MatrixXd dphi_f_dq = dphi_f_dp.transpose() * Jf;
  MatrixXd dphi_r_dq = dphi_r_dp.transpose() * Jr;
  drake::AutoDiffVecXd phi_ad_f = InitializeAutoDiff(
      VectorXd::Constant(1, phi_f), dphi_f_dq);
  drake::AutoDiffVecXd phi_ad_r = InitializeAutoDiff(
      VectorXd::Constant(1, phi_r), dphi_r_dq);

  //dphi_dq = dphi_dp * dp_dq
  drake::AutoDiffXd phi_ad = smooth_min(phi_ad_f(0), phi_ad_r(0));
  *y = drake::AutoDiffVecXd::Zero(1);
  (*y)(0) = phi_ad - y_(0);
}

}