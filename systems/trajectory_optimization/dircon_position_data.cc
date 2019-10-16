#include <limits>
#include <vector>

#include "systems/trajectory_optimization/dircon_position_data.h"
#include "drake/math/orthonormal_basis.h"

namespace dairlib {

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Matrix2d;
using Eigen::MatrixXd;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::multibody::Body;
using drake::VectorX;
using drake::MatrixX;

template <typename T>
DirconPositionData<T>::DirconPositionData(const MultibodyPlant<T>& plant,
    const Body<T>& body, Vector3d pt, bool isXZ) :
    DirconKinematicData<T>(plant, isXZ ? 2 : 3),
    body_(body),
    pt_(pt),
    isXZ_(isXZ) {
  TXZ_ << 1, 0, 0,
          0, 0, 1;
}

template <typename T>
DirconPositionData<T>::~DirconPositionData() {
}

template <typename T>
void DirconPositionData<T>::updateConstraint(const Context<T>& context) {
  VectorX<T> pt_transform(3);
  MatrixX<T> J3d(3, this->plant_.num_velocities());
  const auto x =
      dynamic_cast<const drake::systems::BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value();
  const auto v = x.tail(this->plant_.num_velocities());

  VectorX<T> pt_cast = pt_.template cast<T>();
  const drake::multibody::Frame<T>& world = this->plant_.world_frame();

  this->plant_.CalcPointsPositions(context, body_.body_frame(), pt_cast,
      world, &pt_transform);
  this->plant_.CalcJacobianTranslationalVelocity(
      context, drake::multibody::JacobianWrtVariable::kV,
      body_.body_frame(), pt_cast, world, world, &J3d);

  MatrixX<T> J3d_times_v =
      this->plant_.CalcBiasForJacobianSpatialVelocity(
          context, drake::multibody::JacobianWrtVariable::kV,
          body_.body_frame(), pt_cast,
          world, world).tail(3);

  if (isXZ_) {
    this->c_ = TXZ_*pt_transform;
    this->J_ = TXZ_*J3d;
    this->Jdotv_ = TXZ_*J3d_times_v;
  } else {
    this->c_ = pt_transform;
    this->J_ = J3d;
    this->Jdotv_ = J3d_times_v;
  }
  this->cdot_ = this->J_*v;
}

template <typename T>
void DirconPositionData<T>::addFixedNormalFrictionConstraints(Vector3d normal,
                                                              double mu) {
  if (isXZ_) {
    // specifically builds the basis for the x-axis
    Vector2d normal_xz, d_xz;
    double L = sqrt(normal(0)*normal(0) + normal(2)*normal(2));
    normal_xz << normal(0)/L, normal(2)/L;
    d_xz << -normal_xz(1), normal_xz(0);

    Matrix2d A_fric;
    A_fric << (mu*normal_xz + d_xz).transpose(),
              (mu*normal_xz - d_xz).transpose();

    // Adding one more row for positive normal force constraint
    MatrixXd A = MatrixXd::Zero(3,2);
    A.block(0,0,2,2) = A_fric;
    A(2,1) = 1;
    Vector3d lb = Vector3d::Zero();
    Vector3d ub = Vector3d::Constant(
        std::numeric_limits<double>::infinity());
    auto force_constraint = std::make_shared<drake::solvers::LinearConstraint>(
        A, lb, ub);
    this->force_constraints_.push_back(force_constraint);

  } else {
    // Linear friction cone constraint with positive normal force
    ///     mu_*lambda_c(3*i+2) - lambda_c(3*i+0) >= 0
    ///     mu_*lambda_c(3*i+2) + lambda_c(3*i+0) >= 0
    ///     mu_*lambda_c(3*i+2) - lambda_c(3*i+1) >= 0
    ///     mu_*lambda_c(3*i+2) + lambda_c(3*i+1) >= 0
    ///                           lambda_c(3*i+2) >= 0
    MatrixXd A = MatrixXd::Zero(5,3);
    A.block(0,2,4,1) = mu * VectorXd::Ones(4,1);
    A(0,0) = -1;
    A(1,0) = 1;
    A(2,1) = -1;
    A(3,1) = 1;
    A(4,2) = 1;
    VectorXd lb = VectorXd::Zero(5);
    VectorXd ub = VectorXd::Ones(5) * std::numeric_limits<double>::infinity();
    auto force_constraint = std::make_shared<drake::solvers::LinearConstraint>(
        A, lb, ub);
    this->force_constraints_.push_back(force_constraint);
  }
}


// Explicitly instantiates on the most common scalar types.
template class DirconPositionData<double>;
template class DirconPositionData<drake::AutoDiffXd>;
}  // namespace dairlib
