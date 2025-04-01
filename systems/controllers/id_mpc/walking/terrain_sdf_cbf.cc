#include "terrain_sdf_cbf.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;

TerrainSDFCBF::TerrainSDFCBF(
    const drake::multibody::MultibodyPlant<double> &plant,
    multibody::BoxSet *box_set) :
    plant_(plant),
    context_(plant_.CreateDefaultContext()),
    box_set_(box_set){
  DRAKE_DEMAND(box_set != nullptr);
}

std::pair<MatrixXd, VectorXd> TerrainSDFCBF::GetConstraintMatrices(
    const std::string &frame, const Vector3d &point, const VectorXd &q,
    const VectorXd &v, double a1, double a2, double dt)
    const {

  plant_.SetPositions(context_.get(), q);
  plant_.SetVelocities(context_.get(), v);

  // calc h and Jh
  MatrixXd J = MatrixXd::Zero(3, plant_.num_velocities());
  Vector3d p;
  Vector3d JdotV;
  const auto& body_frame = plant_.GetBodyByName(frame).body_frame();

  plant_.CalcPointsPositions(
      *context_, body_frame, point, plant_.world_frame(), &p);

  plant_.CalcJacobianTranslationalVelocity(
      *context_, drake::multibody::JacobianWrtVariable::kV, body_frame,
      point, plant_.world_frame(), plant_.world_frame(), &J);
  JdotV = plant_.CalcBiasTranslationalAcceleration(
      *context_, drake::multibody::JacobianWrtVariable::kV, body_frame,
      point, plant_.world_frame(), plant_.world_frame());

  const auto [phi, dphi_dp] = box_set_->CalcSDF(p);

  MatrixXd Jh = dphi_dp.transpose() * J;
  VectorXd JhdotV = dphi_dp.transpose() * JdotV;
  VectorXd h = phi * VectorXd::Ones(1);

  // Jh * dv + JhdotV + (a1 + a2)Jh * v + (a1a2) h >= 0

  //  Jh * v1 - Jh * v + dt * (JhdotV  + (a1 + a2)Jh * v + (a1a2) h) >= 0

  VectorXd b = -
      (JhdotV + (a1 + a2) * Jh * v + (a1 * a2) * h);

  return {Jh, b};
}


}