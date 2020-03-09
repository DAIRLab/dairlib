#include "multibody_distance_constraint.h"

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
namespace dairlib::multibody {

MultibodyDistanceConstraint::MultibodyDistanceConstraint(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::multibody::Body<double>& body_1,
    const drake::multibody::Body<double>& body_2, const Vector3d& pt_1,
    const Vector3d& pt_2, double distance)
    : plant_(plant),
      body1_(body_1),
      body2_(body_2),
      pt1_(pt_1),
      pt2_(pt_2),
      distance_(distance) {}

void MultibodyDistanceConstraint::updateConstraint(
    const drake::systems::Context<double>& context) {
  Vector3d pt1_wrt_w(3);
  MatrixXd J1(3, plant_.num_velocities());
  const auto x = dynamic_cast<const drake::systems::BasicVector<double>&>(
                     context.get_continuous_state_vector())
                     .get_value();
  const auto v = x.tail(this->plant_.num_velocities());

  const drake::multibody::Frame<double>& world = this->plant_.world_frame();

  Vector3d pt1_cast = pt1_.template cast<double>();

  this->plant_.CalcPointsPositions(context, body1_.body_frame(), pt1_cast,
                                   world, &pt1_wrt_w);
  this->plant_.CalcJacobianTranslationalVelocity(
      context, drake::multibody::JacobianWrtVariable::kV, body1_.body_frame(),
      pt1_cast, world, world, &J1);

  MatrixXd J1_times_v =
      this->plant_
          .CalcBiasForJacobianSpatialVelocity(
              context, drake::multibody::JacobianWrtVariable::kV,
              body1_.body_frame(), pt1_cast, world, world)
          .tail(3);



  Vector3d pt2_wrt_w(3);
  MatrixXd J2(3, this->plant_.num_velocities());

  VectorXd pt2_cast = pt2_.template cast<double>();

  this->plant_.CalcPointsPositions(context, body2_.body_frame(), pt2_cast,
                                   world, &pt2_wrt_w);
  this->plant_.CalcJacobianTranslationalVelocity(
      context, drake::multibody::JacobianWrtVariable::kV, body2_.body_frame(),
      pt2_cast, world, world, &J2);

  MatrixXd J2dot_times_v =
      this->plant_
          .CalcBiasForJacobianSpatialVelocity(
              context, drake::multibody::JacobianWrtVariable::kV,
              body2_.body_frame(), pt2_cast, world, world)
          .tail(3);

  // Constraint is ||r1-r2||^2  - d^2, to keep it differentiable everywhere
  // J is then 2*(r1-r2)^T * (J1 - J2)
  // Jdot*v can then be computed as
  //    2*(r1dot-r2dot)^T * (J1 - J2)*v + 2*(r1-r2)^T * (J1dot - J2dot)*v
  //  = 2*(J1*v - J2*v)^T * (J1 - J2)*v + 2*(r1-r2)^T * (J1dot - J2dot)*v
  //  = 2*||(J1-J2)*v||^2 + 2*(r1-r2)^T * (J1dot - J2dot)*v
  // And cdot = J*v (as usual)
  VectorXd pt_delta = pt1_wrt_w - pt2_wrt_w;
  this->c_(0) = pt_delta.squaredNorm() - distance_ * distance_;
  this->J_ = 2 * pt_delta.transpose() * (J1 - J2);
  this->Jdotv_ = 2 * pt_delta.transpose() * (J1_times_v - J2dot_times_v);
  this->Jdotv_(0) += 2 * (((J1 - J2) * v)).squaredNorm();
  this->cdot_ = this->J_ * v;
}
Eigen::VectorXd MultibodyDistanceConstraint::getC() const { return this->c_; }
Eigen::VectorXd MultibodyDistanceConstraint::getCDot() const {
  return this->cdot_;
}
Eigen::MatrixXd MultibodyDistanceConstraint::getJ() const { return this->J_; }
Eigen::VectorXd MultibodyDistanceConstraint::getJdotv() const {
  return this->Jdotv_;
}

}  // namespace dairlib::multibody