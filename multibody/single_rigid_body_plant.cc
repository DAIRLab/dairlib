#include "single_rigid_body_plant.h"

using dairlib::multibody::SetPositionsAndVelocitiesIfNew;
using dairlib::multibody::SetPositionsIfNew;
using drake::multibody::MultibodyPlant;
using drake::multibody::JacobianWrtVariable;
using drake::systems::Context;

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;

namespace dairlib::multibody {

SingleRigidBodyPlant::SingleRigidBodyPlant(
    const MultibodyPlant<double>& plant,
    Context<double>* plant_context,
    const bool use_com) :
    use_com_(use_com),
    plant_(plant),
    world_frame_(plant.world_frame()),
    rpy_(drake::math::RollPitchYaw<double>(0, 0, 0)),
    plant_context_(plant_context){
  nq_ = plant_.num_positions();
  nv_ = plant_.num_velocities();
}

VectorXd SingleRigidBodyPlant::CalcSRBStateFromPlantState(const VectorXd& x) {
  Vector3d com_pos;
  Vector3d com_vel;
  Vector3d omega;
  SetPositionsAndVelocitiesIfNew<double>(plant_, x, plant_context_);

  if (use_com_) {
    MatrixXd J_CoM_v = MatrixXd::Zero(3, nv_);
    com_pos = plant_.CalcCenterOfMassPositionInWorld(*plant_context_);
    plant_.CalcJacobianCenterOfMassTranslationalVelocity(
        *plant_context_, JacobianWrtVariable::kV,
        world_frame_, world_frame_, &J_CoM_v);
    com_vel = J_CoM_v * x.tail(nv_);
  } else {
    plant_.CalcPointsPositions(
        *plant_context_,
        plant_.GetBodyByName(base_).body_frame(),
        com_offset_, world_frame_, &com_pos);
    com_vel = plant_
              .GetBodyByName(base_)
              .EvalSpatialVelocityInWorld(*plant_context_).translational();
  }
  auto base_transform = plant_.EvalBodyPoseInWorld(
      *plant_context_, plant_.GetBodyByName(base_));
  rpy_.SetFromRotationMatrix(base_transform.rotation());

  omega = plant_
          .GetBodyByName(base_)
          .EvalSpatialVelocityInWorld(*plant_context_).rotational();

  VectorXd x_srbd(nx_);

  x_srbd << com_pos, rpy_.vector(), com_vel, omega;
  return x_srbd;
}

std::vector<Vector3d> SingleRigidBodyPlant::CalcFootPositions(const Eigen::VectorXd &x) {
  SetPositionsIfNew<double>(plant_, x.head(nq_), plant_context_);
  std::vector<Vector3d> foot_positions;

  for (int i = 0; i < 2; i++) {
    Vector3d pos  = Vector3d::Zero();
    plant_.CalcPointsPositions(
        *plant_context_,
        contact_points_.at(i).second,
        contact_points_.at(i).first, world_frame_, &pos);
    foot_positions.push_back(pos);
  }
  return foot_positions;
}

double SingleRigidBodyPlant::CalcMassFromListOfBodies(
    const std::vector<std::string> &bodies) {
  double mass = 0;
  for (auto & name : bodies) {
    mass += plant_.GetBodyByName(name).get_mass(*plant_context_);
  }
  return mass;
}

double SingleRigidBodyPlant::SetMassFromListOfBodies(
    const std::vector<std::string> &bodies) {
  mass_ = CalcMassFromListOfBodies(bodies);
  return mass_;
}

void SingleRigidBodyPlant::AddBaseFrame(
    const std::string &body_name,
    const Eigen::Vector3d &offset) {
  com_offset_ = offset;
  base_ = body_name;
}

void SingleRigidBodyPlant::AddContactPoint(
    std::pair<Eigen::Vector3d, const drake::multibody::BodyFrame<double>&> pt,
    BipedStance stance) {
  DRAKE_ASSERT(contact_points_.size() == stance);
  contact_points_.push_back(pt);
}

} // <\dairlib::multibody>
