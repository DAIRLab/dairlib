#include "systems/trajectory_optimization/dircon_distance_data.h"

namespace dairlib {

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Matrix2d;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::multibody::Body;
using drake::VectorX;
using drake::Vector3;
using drake::MatrixX;

template <typename T>
DirconDistanceData<T>::DirconDistanceData(const MultibodyPlant<T>& plant,
      const Body<T>& body1, const Vector3d pt1, const Body<T>& body2,
      const Vector3d pt2, const double distance) :
    DirconKinematicData<T>(plant, 1),
    body1_(body1),
    body2_(body2),
    pt1_(pt1),
    pt2_(pt2),
    distance_(distance) {}

template <typename T>
DirconDistanceData<T>::~DirconDistanceData() {
}

template <typename T>
void DirconDistanceData<T>::updateConstraint(const Context<T>& context) {
  Vector3<T> pt1_transform(3);
  MatrixX<T> J1(3, this->plant_.num_velocities());
  const auto x =
      dynamic_cast<const drake::systems::BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value();
  const auto v = x.tail(this->plant_.num_velocities());

  const drake::multibody::Frame<T>& world = this->plant_.world_frame();

  Vector3<T> pt1_cast = pt1_.template cast<T>();

  this->plant_.CalcPointsPositions(context, body1_.body_frame(), pt1_cast,
      world, &pt1_transform);
  this->plant_.CalcJacobianTranslationalVelocity(
      context, drake::multibody::JacobianWrtVariable::kV,
      body1_.body_frame(), pt1_cast, world, world, &J1);

  MatrixX<T> J1_times_v =
      this->plant_.CalcBiasForJacobianSpatialVelocity(
          context, drake::multibody::JacobianWrtVariable::kV,
          body1_.body_frame(), pt1_cast, world, world).tail(3);

  Vector3<T> pt2_transform(3);
  MatrixX<T> J2(3, this->plant_.num_velocities());

  VectorX<T> pt2_cast = pt2_.template cast<T>();

  this->plant_.CalcPointsPositions(context, body2_.body_frame(), pt2_cast,
      world, &pt2_transform);
  this->plant_.CalcJacobianTranslationalVelocity(
      context, drake::multibody::JacobianWrtVariable::kV,
      body2_.body_frame(), pt2_cast, world, world, &J2);


  MatrixX<T> J2dot_times_v =
      this->plant_.CalcBiasForJacobianSpatialVelocity(
          context, drake::multibody::JacobianWrtVariable::kV,
          body2_.body_frame(), pt2_cast, world, world).tail(3);

  // Constraint is ||r1-r2||^2  - d^2, to keep it differentiable everywhere
  // J is then 2*(r1-r2)^T * (J1 - J2)
  // Jdot*v can then be computed as
  //    2*(r1dot-r2dot)^T * (J1 - J2)*v + 2*(r1-r2)^T * (J1dot - J2dot)*v
  //  = 2*(J1*v - J2*v)^T * (J1 - J2)*v + 2*(r1-r2)^T * (J1dot - J2dot)*v
  //  = 2*||(J1-J2)*v||^2 + 2*(r1-r2)^T * (J1dot - J2dot)*v
  // And cdot = J*v (as usual)
  VectorX<T> pt_delta = pt1_transform - pt2_transform;
  this->c_(0) = pt_delta.squaredNorm() - distance_ * distance_;
  this->J_ = 2*pt_delta.transpose()*(J1 - J2);
  this->Jdotv_ = 2*pt_delta.transpose()*(J1_times_v - J2dot_times_v);
  this->Jdotv_(0) += 2*(((J1 - J2)*v)).squaredNorm();
  this->cdot_ = this->J_*v;
}

// Explicitly instantiates on the most common scalar types.
template class DirconDistanceData<double>;
template class DirconDistanceData<drake::AutoDiffXd>;
}  // namespace dairlib
