#include "multibody/pinocchio_plant.h"

#include <iostream>

#include "multibody/multibody_utils.h"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/centroidal-derivatives.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/frame.hpp"
#include "pinocchio/multibody/joint/joint-free-flyer.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace dairlib {
namespace multibody {

using drake::AutoDiffXd;
using drake::Matrix3X;
using drake::Matrix6X;
using drake::MatrixX;
using drake::Vector3;
using drake::Matrix3;
using drake::VectorX;
using drake::Vector6;
using drake::math::ExtractValue;
using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialMomentum;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using multibody::HasQuaternion;
using std::cout;
using std::endl;
using std::map;
using std::string;



template <>
PinocchioPlant<AutoDiffXd>::PinocchioPlant(const MultibodyPlant<double>& plant,
                                           const std::string& urdf)
    : MultibodyPlant<AutoDiffXd>(plant), urdf_(urdf) {
  interface_ = std::make_unique<PinocchioInterface>(plant, urdf_);
  J_work_ = Matrix6X<AutoDiffXd>::Zero(6, plant.num_velocities());
}

template <>
PinocchioPlant<double>::PinocchioPlant(double time_step,
                                       const std::string& urdf)
    : MultibodyPlant<double>(time_step), urdf_(urdf) {}


template <>
void PinocchioPlant<double>::FinalizePlant() {
  interface_ = std::make_unique<PinocchioInterface>(*this, urdf_);
  this->DoFinalizePinocchioPlant();
  J_work_ = Matrix6X<double>::Zero(6, this->num_velocities());
}

template <>
void PinocchioPlant<AutoDiffXd>::FinalizePlant() {
  this->DoFinalizePinocchioPlant();
}

template <typename T>
void PinocchioPlant<T>::DoFinalizePinocchioPlant() {
  DRAKE_DEMAND(this->is_finalized());

  is_floating_base_ = HasQuaternion(*this);
  if (is_floating_base_) {
    pinocchio::urdf::buildModel(urdf_, pinocchio::JointModelFreeFlyer(),
                                pinocchio_model_);
  } else {
    pinocchio::urdf::buildModel(urdf_, pinocchio_model_);
  }

  pinocchio_model_.rotorInertia = interface_->get_model().rotorInertia;
  pinocchio_model_.rotorGearRatio = interface_->get_model().rotorGearRatio;
  pinocchio_model_.armature = interface_->get_model().armature;

  pinocchio_data_ = pinocchio::Data(pinocchio_model_);

  // Check that models match
  int nq = this->num_positions();
  int nv = this->num_velocities();

  n_q_ = nq;
  n_v_ = nv;
}

template <typename T>
Matrix3<T> PinocchioPlant<T>::skew(const Vector3<T>& v) const {
  Matrix3<T> ret = Matrix3<T>::Zero();
  ret(0, 1) = v(2);
  ret(0, 2) = -v(1);
  ret(1, 2) = v(0);
  ret(1, 0) = -v(2);
  ret(2, 0) = v(1);
  ret(2, 1) = -v(0);
  return ret;
}

// To spatial accel (featherstone 2008, eq 2.47) then rotate
template <typename T>
Vector6<T> PinocchioPlant<T>::MapVDotToBodyFrame(
    const drake::VectorX<T>& quat, const drake::VectorX<T>& v,
    const drake::VectorX<T>& vdot) const {
  Matrix3<T> R_BW =
      Eigen::Quaternion<T>(quat(0), quat(1), quat(2), quat(3))
          .toRotationMatrix().transpose();

  Vector3<T> offset = -v.template head<3>().cross(v.template segment<3>(3));

  Vector6<T> ret = Vector6<T>::Zero();
  ret.template head<3>() = R_BW * vdot.template head<3>();
  ret.template tail<3>() = R_BW * (vdot.template tail<3>() + offset);
  return ret;
}

template <typename T>
drake::MatrixX<T> PinocchioPlant<T>::GetVelocityMapFromDrakeToPinocchio(
    const drake::VectorX<T>& quat) const {
  drake::MatrixX<T> ret = drake::MatrixX<T>::Identity(this->num_velocities(),
                                                      this->num_velocities());
  if (is_floating_base_) {
    drake::MatrixX<T> rot =
        Eigen::Quaternion<T>(quat(0), quat(1), quat(2), quat(3))
            .toRotationMatrix()
            .transpose();
    ret.template block<3, 3>(0, 0) = rot;
    ret.template block<3, 3>(3, 3) = rot;
  }
  return interface_->v_perm().inverse() * ret;
}

template <typename T>
drake::MatrixX<double> PinocchioPlant<T>::GetVelocityMapFromPinocchioToDrake(
    const drake::VectorX<double>& quat) const {
  drake::MatrixX<double> ret = drake::MatrixX<double>::Identity(
      this->num_velocities(), this->num_velocities());
  if (is_floating_base_) {
    drake::MatrixX<double> rot =
        Eigen::Quaternion<double>(quat(0), quat(1), quat(2), quat(3))
            .toRotationMatrix();
    ret.template block<3, 3>(0, 0) = rot;
    ret.template block<3, 3>(3, 3) = rot;
  }
  return interface_->v_perm() * ret;
}

// Copied from Drake QuaternionFloatingMobilizer, since the
// relevant methods are now private
namespace {
Eigen::Matrix<double, 4, 3> CalcLMatrix(const Eigen::Quaternion<double>& q_FM) {
  // This L matrix helps us compute both N(q) and N⁺(q) since it turns out that:
  //   N(q) = L(q_FM/2)
  // and:
  //   N⁺(q) = L(2 q_FM)ᵀ
  // See Eqs. 5 and 6 in Section 9.2 of Paul's book
  // [Mitiguy (August 7) 2017, §9.2], for the time derivative of the vector
  // component of the quaternion (Euler parameters). Notice however here we use
  // qs and qv for the "scalar" and "vector" components of the quaternion q_FM,
  // respectively, while Mitiguy uses ε₀ and ε (in bold), respectively.
  // This mobilizer is parameterized by the angular velocity w_FM, i.e. time
  // derivatives of the vector component of the quaternion are taken in the F
  // frame. If you are confused by this, notice that the vector component of a
  // quaternion IS a vector, and therefore you must specify in what frame time
  // derivatives are taken.
  //
  // Notice this is equivalent to:
  // Dt_F(q) = 1/2 * w_FM⋅q_FM, where ⋅ denotes the "quaternion product" and
  // both the vector component qv_FM of q_FM and w_FM are expressed in frame F.
  // Dt_F(q) is short for [Dt_F(q)]_F.
  // The expression above can be written as:
  // Dt_F(q) = 1/2 * (-w_FM.dot(qv_F); qs * w_FM + w_FM.cross(qv_F))
  //         = 1/2 * (-w_FM.dot(qv_F); qs * w_FM - qv_F.cross(w_FM))
  //         = 1/2 * (-w_FM.dot(qv_F); (qs * Id - [qv_F]x) * w_FM)
  //         = L(q_FM/2) * w_FM
  // That is:
  //        |         -qv_Fᵀ    |
  // L(q) = | qs * Id - [qv_F]x |

  const double qs = q_FM.w();             // The scalar component.
  const Vector3<double> qv = q_FM.vec();  // The vector component.
  const Vector3<double> mqv = -qv;        // minus qv  .

  // NOTE: the rows of this matrix are in an order consistent with the order
  // in which we store the quaternion in the state, with the scalar component
  // first followed by the vector component.
  return (Eigen::Matrix<double, 4, 3>() << mqv.transpose(), qs, qv.z(), mqv.y(),
      mqv.z(), qs, qv.x(), qv.y(), mqv.x(), qs)
      .finished();
}

Eigen::Matrix<double, 4, 3> AngularVelocityToQuaternionRateMatrix(
    const Eigen::Quaternion<double>& q_FM) {
  // With L given by CalcLMatrix we have:
  // N(q) = L(q_FM/2)
  return CalcLMatrix(
      {q_FM.w() / 2.0, q_FM.x() / 2.0, q_FM.y() / 2.0, q_FM.z() / 2.0}
  );
}
}

template <>
VectorXd PinocchioPlant<double>::CalcInverseDynamics(
    const drake::systems::Context<double>& context, const VectorXd& known_vdot,
    const drake::multibody::MultibodyForces<double>& external_forces) const {

  for (const auto& f : external_forces.body_forces()) {
    // We don't support body forces in PinocchioPlant CalcInverseDynamics yet
    DRAKE_ASSERT(f.get_coeffs() == Vector6<double>::Zero());
  }

  const VectorXd& q = GetPositions(context);
  const VectorXd& v = GetVelocities(context);

  VectorXd fp = pinocchio::rnea(
      pinocchio_model_, pinocchio_data_,
      interface_->MapPositionsToPinocchio(q),
      interface_->MapVelocitiesToPinocchio(q, v),
      interface_->MapVDotToPinocchio(q, v, known_vdot));

  // At this point, f_pin = M vdot + C + g
  // Drake doesn't include gravity, so we will subtract it out.
  VectorXd g = pinocchio::computeGeneralizedGravity(
      pinocchio_model_, pinocchio_data_,
      interface_->MapPositionsToPinocchio(q));
  // subract gravity
  VectorXd fnet = fp - g;
  fp = interface_->MapVelocitiesToDrake(q, fnet);
  fp = fp - external_forces.generalized_forces();
  return fp;
}

template <>
void PinocchioPlant<double>::CalcJacobianTranslationalVelocity(
    const drake::systems::Context<double>& context,
    drake::multibody::JacobianWrtVariable with_respect_to,
    const drake::multibody::Frame<double>& frame_B,
    const Eigen::Ref<const drake::Matrix3X<double>>& p_BoBi_B,
    const drake::multibody::Frame<double>& frame_A, const
    drake::multibody::Frame<double>& frame_E,
    drake::EigenPtr<drake::MatrixX<double>> Js_v_ABi_E) const {

  bool wrt_q =
      (with_respect_to == drake::multibody::JacobianWrtVariable::kQDot);
  int cols = wrt_q ? n_q_ : n_v_;

  DRAKE_DEMAND(Js_v_ABi_E->cols() == cols and
               Js_v_ABi_E->rows() == 3 * p_BoBi_B.cols());

  DRAKE_DEMAND(frame_A.is_world_frame());

  pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;

  pinocchio::FrameIndex frame_id = pinocchio_model_.getFrameId(
      frame_B.name(), pinocchio::BODY);


  J_work_.setZero();

  const VectorXd& q = GetPositions(context);
  pinocchio::computeFrameJacobian(
      pinocchio_model_, pinocchio_data_, interface_->MapPositionsToPinocchio(q),
      frame_id, rf, J_work_);

  // Hack to deal with return type issues by maxing this fully dynamic sized
  interface_->MapJvToDrake(q, &J_work_);

  const Matrix3X<double>& J_translation = J_work_.topRows<3>();
  const Matrix3X<double>& J_rotation = J_work_.bottomRows<3>();

  Eigen::Quaternion<double> quat(
      2.0 * q(0), 2.0 * q(1), 2.0 * q(2), 2.0 * q(3));
  MatrixX<double> Nplus = CalcLMatrix(quat).transpose();

  for (int i = 0; i < p_BoBi_B.cols(); ++i) {
    Vector3d p_w = pinocchio_data_.oMf[frame_id].rotation() * p_BoBi_B.col(i);
    Js_v_ABi_E->block(i * 3, cols - n_v_, 3, n_v_) =
        J_translation + J_rotation.colwise().cross(p_w);
    if (wrt_q and is_floating_base_) {
      Js_v_ABi_E->block(i*3, 0, 3, 4) = Js_v_ABi_E->block(i*3, 1, 3, 3) * Nplus;
    }
  }
}

template <>
void PinocchioPlant<AutoDiffXd>::CalcJacobianTranslationalVelocity(
    const drake::systems::Context<AutoDiffXd>& context,
    drake::multibody::JacobianWrtVariable with_respect_to,
    const drake::multibody::Frame<AutoDiffXd>& frame_B,
    const Eigen::Ref<const drake::Matrix3X<AutoDiffXd>>& p_BoBi_B,
    const drake::multibody::Frame<AutoDiffXd>& frame_A, const
    drake::multibody::Frame<AutoDiffXd>& frame_E,
    drake::EigenPtr<drake::MatrixX<AutoDiffXd>> Js_v_ABi_E) const {
  throw std::runtime_error("not implemented yet");
}

template <>
VectorXd PinocchioPlant<double>::CalcInverseDynamicsWithGravity(
    const drake::systems::Context<double>& context, const VectorXd& known_vdot,
    const drake::multibody::MultibodyForces<double>& external_forces) const {

  for (const auto& f : external_forces.body_forces()) {
    // We don't support body forces in PinocchioPlant CalcInverseDynamics yet
    DRAKE_ASSERT(f.get_coeffs() == Vector6<double>::Zero());
  }

  const VectorXd& q = GetPositions(context);
  const VectorXd& v = GetVelocities(context);

  VectorXd fp = pinocchio::rnea(
      pinocchio_model_, pinocchio_data_,
      interface_->MapPositionsToPinocchio(q),
      interface_->MapVelocitiesToPinocchio(q, v),
      interface_->MapVDotToPinocchio(q, v, known_vdot));

  fp = interface_->MapVelocitiesToDrake(q, fp);
  fp = fp - external_forces.generalized_forces();
  return fp;
}

template<>
VectorX<AutoDiffXd> PinocchioPlant<AutoDiffXd>::CalcInverseDynamicsWithGravity(
    const drake::systems::Context<AutoDiffXd> &context,
    const drake::VectorX<AutoDiffXd> &known_vdot,
    const drake::multibody::MultibodyForces<AutoDiffXd> &external_forces) const {
  throw std::runtime_error("CalcInverseDynamicsWithGravity not implemented "
                           "for AutoDiffXd yet.");
  return VectorX<AutoDiffXd>::Zero(known_vdot.rows());
}

template <>
void PinocchioPlant<double>::CalcMassMatrix(
    const Context<double>& context, drake::EigenPtr<Eigen::MatrixXd> M) const {

  const VectorXd& q = GetPositions(context);
  const VectorXd& v = GetVelocities(context);

  pinocchio::crba(pinocchio_model_, pinocchio_data_,
                  interface_->MapPositionsToPinocchio(q));

  // Pinocchio builds an upper triangular matrix, skipping the parts
  // below the diagonal. Fill those in here.
  *M = pinocchio_data_.M;
  for (int i = 0; i < M->cols(); i++) {
    for (int j = i + 1; j < M->rows(); j++) {
      (*M)(j, i) = (*M)(i, j);
    }
  }
  // TODO: we can speed this up by not doing full matrix multiplication here.
  // Similar to RightMultiplicationFromDrakeToPinocchio
  *M = GetVelocityMapFromPinocchioToDrake(GetPositions(context).head<4>()) *
       (*M) *
       GetVelocityMapFromDrakeToPinocchio(GetPositions(context).head<4>());
}

template <>
void PinocchioPlant<AutoDiffXd>::CalcMassMatrix(
    const Context<AutoDiffXd>& context,
    drake::EigenPtr<drake::MatrixX<AutoDiffXd>> M) const {
  throw std::domain_error("CalcMassMatrix not implemented with AutoDiffXd");
}

template <>
drake::Vector3<double> PinocchioPlant<double>::CalcCenterOfMassPositionInWorld(
    const Context<double>& context) const {

  const VectorXd& q = GetPositions(context);

  pinocchio::centerOfMass(
      pinocchio_model_, pinocchio_data_,
      interface_->MapPositionsToPinocchio(q));

  return pinocchio_data_.com[0];
}

template <>
drake::Vector3<AutoDiffXd>
PinocchioPlant<AutoDiffXd>::CalcCenterOfMassPositionInWorld(
    const Context<AutoDiffXd>& context) const {
  VectorXd q_drake = ExtractValue(GetPositions(context));
  VectorXd q_pin = interface_->MapPositionsToPinocchio(q_drake);
  auto drake_quat =
      Eigen::Quaternion<double>(q_drake(0), q_drake(1), q_drake(2), q_drake(3));
  drake::MatrixX<double> rot = drake_quat.toRotationMatrix().transpose();
  Matrix3X<double> gradient = MatrixXd(3, n_q_ + n_v_);
  Vector3<double> gradient_qw = Vector3d::Zero();
  Matrix3X<double> gradient_q =
      pinocchio::jacobianCenterOfMass(pinocchio_model_, pinocchio_data_, q_pin);
  Matrix3X<double> gradient_v = MatrixXd::Zero(3, n_v_);
  auto map = AngularVelocityToQuaternionRateMatrix(drake_quat);
  MatrixXd gradient_quat =
      4 * map * rot.transpose() * gradient_q.block<3, 3>(0, 3).transpose();
  Matrix3X<double> gradient_pos = gradient_q.block<3, 3>(0, 0) * rot;
  gradient_quat.transposeInPlace();
  Matrix3X<double> drake_gradient = Matrix3X<double>::Zero(3, n_q_);
  drake_gradient << gradient_quat, gradient_pos, gradient_q.rightCols(n_q_ - 7);
  gradient << drake_gradient, gradient_v;
  return drake::math::InitializeAutoDiff(pinocchio_data_.com[0], gradient);
}

template <>
drake::Vector3<double>
PinocchioPlant<double>::CalcCenterOfMassTranslationalVelocityInWorld(
    const Context<double>& context) const {
  const VectorXd& q = GetPositions(context);
  const VectorXd& v = GetVelocities(context);
  pinocchio::centerOfMass(
      pinocchio_model_, pinocchio_data_,
      interface_->MapPositionsToPinocchio(q),
      interface_->MapVelocitiesToPinocchio(q, v));
  return pinocchio_data_.vcom[0];
}

template <>
drake::Vector3<AutoDiffXd>
PinocchioPlant<AutoDiffXd>::CalcCenterOfMassTranslationalVelocityInWorld(
    const Context<AutoDiffXd>& context) const {
  throw std::domain_error(
      "CalcCenterOfMassTranslationalVelocityInWorld not implemented with "
      "AutoDiffXd");
}

template <>
void PinocchioPlant<double>::CalcJacobianCenterOfMassTranslationalVelocity(
    const Context<double>& context, JacobianWrtVariable with_respect_to,
    const Frame<double>& frame_A, const Frame<double>& frame_E,
    drake::EigenPtr<drake::Matrix3X<double>> J) const {
  DRAKE_DEMAND(frame_A.is_world_frame());
  DRAKE_DEMAND(frame_E.is_world_frame());
  const VectorXd& q = GetPositions(context);
  *J = pinocchio::jacobianCenterOfMass(
      pinocchio_model_, pinocchio_data_,
      interface_->MapPositionsToPinocchio(q));
  interface_->MapJvToDrake(q, J);
}

template <>
void PinocchioPlant<AutoDiffXd>::CalcJacobianCenterOfMassTranslationalVelocity(
    const Context<AutoDiffXd>& context, JacobianWrtVariable with_respect_to,
    const Frame<AutoDiffXd>& frame_A, const Frame<AutoDiffXd>& frame_E,
    drake::EigenPtr<drake::Matrix3X<AutoDiffXd>> J) const {
  throw std::domain_error("CalcMassMatrix not implemented with AutoDiffXd");
}

template <>
void PinocchioPlant<double>::CalcPointsPositions(
    const drake::systems::Context<double>& context,
    const drake::multibody::Frame<double>& frame_B,
    const Eigen::Ref<const drake::MatrixX<double>>& p_BQi,
    const drake::multibody::Frame<double>& frame_A,
    drake::EigenPtr<MatrixX<double>> p_AQi) const {
  pinocchio::ReferenceFrame rf;
  if (frame_A.is_world_frame()) {
    rf = pinocchio_world_;
  } else {
    rf = pinocchio::ReferenceFrame::LOCAL;
  }
  const VectorXd& q = GetPositions(context);

  pinocchio::framesForwardKinematics(pinocchio_model_, pinocchio_data_,
                                     interface_->MapPositionsToPinocchio(q));
  pinocchio::FrameIndex frame_id = pinocchio_model_.getFrameId(
      frame_B.name(), pinocchio::BODY);
  DRAKE_DEMAND(p_AQi);
  *p_AQi = pinocchio_data_.oMf[frame_id].actOnEigenObject(p_BQi);
}

template <>
void PinocchioPlant<AutoDiffXd>::CalcPointsPositions(
    const drake::systems::Context<AutoDiffXd>& context,
    const drake::multibody::Frame<AutoDiffXd>& frame_B,
    const Eigen::Ref<const drake::MatrixX<AutoDiffXd>>& p_BQi,
    const drake::multibody::Frame<AutoDiffXd>& frame_A,
    drake::EigenPtr<MatrixX<AutoDiffXd>> p_AQi) const {
  pinocchio::ReferenceFrame rf;
  if (frame_A.is_world_frame()) {
    rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  } else {
    rf = pinocchio::ReferenceFrame::LOCAL;
  }
  pinocchio::FrameIndex frame_id = pinocchio_model_.getFrameId(
      frame_B.name(), pinocchio::BODY);
  Matrix6X<double> J = MatrixXd::Zero(6, n_v_);

  const VectorXd& q = ExtractValue(GetPositions(context));
  pinocchio::computeFrameJacobian(
      pinocchio_model_, pinocchio_data_, interface_->MapPositionsToPinocchio(q),
      frame_id, rf, J);

  Matrix3X<double> J_translation = J.topRows<3>();
  Matrix3X<double> J_rotation = J.bottomRows<3>();

  Vector3d p_BQi_double = ExtractValue(p_BQi);
  J_translation = J_translation + J_rotation.colwise().cross(p_BQi_double);

  Vector3d position = pinocchio_data_.oMf[frame_id].actOnEigenObject(p_BQi_double);
//  DRAKE_DEMAND(p_AQi);
  *p_AQi = drake::math::InitializeAutoDiff(position, J_translation);
//  *p_AQi = drake::math::InitializeAutoDiff(position);
}

}  // namespace multibody
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class dairlib::multibody::PinocchioPlant)
