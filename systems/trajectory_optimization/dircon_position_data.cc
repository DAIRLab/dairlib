#include <limits>
#include <vector>

#include "systems/trajectory_optimization/dircon_position_data.h"
#include "drake/math/orthonormal_basis.h"

namespace dairlib {

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Matrix2d;
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
  this->plant_.CalcPointsGeometricJacobianExpressedInWorld(context,
      body_.body_frame(), pt_cast, &pt_transform, &J3d);

  MatrixX<T> J3d_times_v =
      this->plant_.CalcBiasForFrameGeometricJacobianExpressedInWorld(
          context, body_.body_frame(), pt_cast).tail(3);
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
    Vector2d lb_fric = Vector2d::Zero();
    Vector2d ub_fric = Vector2d::Constant(
        std::numeric_limits<double>::infinity());

    auto force_constraint = std::make_shared<drake::solvers::LinearConstraint>(
        A_fric, lb_fric, ub_fric);
    this->force_constraints_.push_back(force_constraint);
  } else {
    // builds a basis from the normal
    const Matrix3d basis = drake::math::ComputeBasisFromAxis(2, normal);
    Matrix3d A_fric;
    A_fric << mu*normal.transpose(), basis.block(0, 1, 3, 2).transpose();
    Vector3d b_fric = Vector3d::Zero();
    auto force_constraint =
        std::make_shared<drake::solvers::LorentzConeConstraint>(A_fric, b_fric);
    this->force_constraints_.push_back(force_constraint);
  }
}


// Explicitly instantiates on the most common scalar types.
template class DirconPositionData<double>;
template class DirconPositionData<drake::AutoDiffXd>;
}  // namespace dairlib
