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

Matrix3d HatOperator3x3(const Vector3d& v){
  Eigen::Matrix3d v_hat = Eigen::Matrix3d::Zero();
  v_hat << 0, -v(2), v(1),
           v(2), 0, -v(0),
           -v(1), v(0), 0;
  return v_hat;
}

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
  nu_ = plant.num_actuators();
}

VectorXd SingleRigidBodyPlant::CalcSRBStateFromPlantState(const VectorXd& x) const {
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
        Vector3d::Zero(), world_frame_, &com_pos);
    com_pos += com_offset_;
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

std::vector<Vector3d> SingleRigidBodyPlant::CalcFootPositions(
    const Eigen::VectorXd &x) const {
  return {CalcFootPosition(x, kLeft), CalcFootPosition(x, kRight)};
}

Vector3d SingleRigidBodyPlant::CalcFootPosition(
    const Eigen::VectorXd &x, const BipedStance& stance) const {
  SetPositionsIfNew<double>(plant_, x.head(nq_), plant_context_);
  Vector3d pos  = Vector3d::Zero();
  plant_.CalcPointsPositions(
      *plant_context_,
      contact_points_.at(stance).second,
      contact_points_.at(stance).first, world_frame_, &pos);
  return pos;
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
    const std::pair<
        Eigen::Vector3d,
        const drake::multibody::BodyFrame<double>&>& pt,
    BipedStance stance) {
  DRAKE_ASSERT(contact_points_.size() == stance);
  contact_points_.push_back(pt);
}

void SingleRigidBodyPlant::CopyContinuousLinearized3dSrbDynamicsForMPC(
    double m, double yaw, BipedStance stance,
    const Eigen::MatrixXd &b_I,
    const Eigen::Vector3d &eq_com_pos,
    const Eigen::Vector3d &eq_foot_pos,
    const drake::EigenPtr<MatrixXd> &Ad,
    const drake::EigenPtr<MatrixXd> &Bd,
    const drake::EigenPtr<VectorXd> &bd) {

  const Eigen::Vector3d g = {0.0, 0.0, 9.81};
  drake::math::RollPitchYaw rpy(0.0, 0.0, yaw);
  Matrix3d R_yaw = rpy.ToMatrix3ViaRotationMatrix();
  Vector3d tau = {0.0,0.0,1.0};
  Vector3d mg = {0.0, 0.0, m * 9.81};
  Matrix3d lambda_hat = HatOperator3x3(mg);
  Matrix3d g_I_inv = (R_yaw * b_I * R_yaw.transpose()).inverse();


  MatrixXd A = MatrixXd::Zero(12,15);
  MatrixXd B = MatrixXd::Zero(12,4);
  VectorXd b = VectorXd::Zero(12);


  // Continuous A matrix
  A.block(0, 6, 3, 3) = Matrix3d::Identity();
  A.block(3, 9, 3, 3) = R_yaw;
  A.block(9, 0, 3, 3) = g_I_inv * lambda_hat;
  A.block(9, 12, 3, 3) = -g_I_inv * lambda_hat;

  // Continuous B matrix
  B.block(6, 0, 3, 3) = (1.0 / m) * Matrix3d::Identity();
  B.block(9, 0, 3, 1) = g_I_inv *
      HatOperator3x3(R_yaw * (eq_foot_pos - eq_com_pos));
  B.block(9, 9, 3, 1) = g_I_inv * Vector3d(0.0, 0.0, 1.0);

  // Continuous Affine portion (b)
  b.segment(6, 3) = -g;
  b.segment(9, 3) = g_I_inv *
      HatOperator3x3(R_yaw * (eq_foot_pos - eq_com_pos)) * mg;

  *Ad = A;
  *Bd = B;
  *bd = b;
}

void SingleRigidBodyPlant::CopyDiscreteLinearizedSrbDynamicsForMPC(
    double dt, double m, double yaw, BipedStance stance,
    const Eigen::MatrixXd &b_I,
    const Eigen::Vector3d &eq_com_pos,
    const Eigen::Vector3d &eq_foot_pos,
    const drake::EigenPtr<MatrixXd> &Ad,
    const drake::EigenPtr<MatrixXd> &Bd,
    const drake::EigenPtr<VectorXd> &bd) {

  MatrixXd A = MatrixXd::Zero(12,15);
  MatrixXd B = MatrixXd::Zero(12,4);
  VectorXd b = VectorXd::Zero(12);

  CopyContinuousLinearized3dSrbDynamicsForMPC(
      m, yaw, stance, b_I, eq_com_pos, eq_foot_pos, &A, &B, &b);

  MatrixXd A_accel = MatrixXd::Zero(12, 15);
  MatrixXd B_accel = MatrixXd::Zero(12, 4);
  VectorXd b_accel = VectorXd::Zero(12);
  A_accel.block(0, 0, 6, A_accel.cols()) = A.block(6, 0, 6, A_accel.cols());
  B_accel.block(0, 0, 6, B_accel.cols()) = B.block(6, 0, 6, B_accel.cols());
  b_accel.head(6) = b.tail(6);
  A = MatrixXd::Identity(12, 15) + A*dt + (0.5 * dt*dt)*A_accel;
  B = B*dt + 0.5*dt*dt*B_accel;
  b = b*dt + 0.5*dt*dt*b_accel;

  *Ad = A;
  *Bd = B;
  *bd = b;
}

} // <\dairlib::multibody>
