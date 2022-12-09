#include <drake/math/quaternion.h>
#include <iostream>
#include "systems/trajectory_optimization/kinematic_centroidal_planner/kinematic_centroidal_constraints.h"
#include "multibody/multibody_utils.h"

template<typename T>
CentroidalDynamicsConstraint<T>::CentroidalDynamicsConstraint(const drake::multibody::MultibodyPlant<T> &plant,
                                                              drake::systems::Context<T> *context,
                                                              int n_contact,
                                                              double dt,
                                                              int knot_index): dairlib::solvers::NonlinearConstraint<T>(
    6,  2 * (6  + 3 + 2 * 3 * n_contact), // Number of inputs = 2 * (momentum + com + contact_pos + contact_vel)
    Eigen::VectorXd::Zero(6),
    Eigen::VectorXd::Zero(6),
    "momentum_collocation[" +
        std::to_string(knot_index) + "]"),
                                                                               plant_(plant),
                                                                               context_(context),
                                                                               n_contact_(n_contact),
                                                                               dt_(dt),
                                                                               m_(plant_.CalcTotalMass(*context)){}


/// The format of the input to the eval() function is in the order
///   - momentum0, momentum state at time k
///   - momentum1, momentum state at time k+1
///   - com0, location of com at time k
///   - x_contact0, contact locations time k
///   - f0, contact forces at time k
///   - com1, location of com at time k+1
///   - x_contact1, contact locations time k+1
///   - f1, contact forces at time k+1

template <typename T>
void CentroidalDynamicsConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<T>>& x, drake::VectorX<T>* y) const {
  // Extract decision variables
  const auto& momentum0 = x.segment(0, n_mom_);
  const auto& momentum1 = x.segment(n_mom_, n_mom_);
  const auto& com0 = x.segment(2 * n_mom_, 3);
  const auto& x_contact0 = x.segment(2 * n_mom_ + 3, 3 * n_contact_);
  const auto& f0 = x.segment(2 * n_mom_ + 3 + 3 * n_contact_, 3 * n_contact_);
  const auto& com1 = x.segment(2 * n_mom_ + 3 + 3 * n_contact_ + 3 * n_contact_, 3);
  const auto& x_contact1 = x.segment(2 * n_mom_ + 3 + 3 * n_contact_ + 3 * n_contact_+ 3, 3 * n_contact_);
  const auto& f1 = x.segment(2 * n_mom_ + 3 + 3 * n_contact_ + 3 * n_contact_+ 3 + 3 * n_contact_, 3 * n_contact_);

  drake::Vector<T, 6> xdot0Mom = CalcTimeDerivativesWithForce(com0, x_contact0, f0);
  drake::Vector<T, 6> xdot1Mom = CalcTimeDerivativesWithForce(com1, x_contact1, f1);

  // Predict state and return error
  const auto x1Predict = momentum0 + 0.5 * dt_ * (xdot0Mom + xdot1Mom);
  *y = momentum1 - x1Predict;
}

template<typename T>
drake::VectorX<T> CentroidalDynamicsConstraint<T>::CalcTimeDerivativesWithForce(const drake::VectorX<T>& com_position,
                                                                                const drake::VectorX<T>& contact_locations,
                                                                                const drake::VectorX<T>& contact_forces) const {
  drake::Vector3<T> sum_moments(0,0,0);
  drake::Vector3<T> sum_forces = - m_ * drake::Vector3<T>(0, 0, 9.81);
  for(int contact = 0; contact < n_contact_; contact ++){
    const drake::Vector3<T>& location = contact_locations.segment(contact * 3, 3);
    const drake::Vector3<T>& force = contact_forces.segment(contact * 3, 3);

    sum_moments = sum_moments + (location - com_position).cross(force);
    sum_forces = sum_forces + force;
  }

  drake::Vector<T, 6> rv;
  rv.head(3) = sum_moments;
  rv.tail(3) = sum_forces;
  return rv;
}

template<typename T>
KinematicIntegratorConstraint<T>::KinematicIntegratorConstraint(const drake::multibody::MultibodyPlant<T> &plant,
                                                                  drake::systems::Context<T> *context0,
                                                                  drake::systems::Context<T>* context1,
                                                                  double dt,
                                                                  int knot_index): dairlib::solvers::NonlinearConstraint<T>(
    plant.num_positions(),  2 * plant.num_positions() + 2 * plant.num_velocities(),
    Eigen::VectorXd::Zero(plant.num_positions()),
    Eigen::VectorXd::Zero(plant.num_positions()),
    "generalized_velocity_integrator[" +
        std::to_string(knot_index) + "]"),
                                                                                   plant_(plant),
                                                                                   context0_(context0),
                                                                                   context1_(context1),
                                                                                   n_q_(plant_.num_positions()),
                                                                                   n_v_(plant_.num_velocities()),
                                                                                   dt_(dt) {}

/// The format of the input to the eval() function is in the order
///   - q0, generalized position at time k
///   - q1, generalized position at time k + 1
///   - v0, generalized velocity at time k
///   - v1, generalized velocity at time k + 1
template<typename T>
void KinematicIntegratorConstraint<T>::EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>> &x,
                                                           drake::VectorX<T> *y) const {
  const auto& q0 = x.head(n_q_);
  const auto& q1 = x.segment(n_q_, n_q_);
  const auto& v0 = x.segment(n_q_ + n_q_, n_v_);
  const auto& v1 = x.segment(n_q_ + n_q_ + n_v_, n_v_);

  dairlib::multibody::SetPositionsIfNew<T>(plant_, q0, context0_);
  dairlib::multibody::SetPositionsIfNew<T>(plant_, q1, context1_);

  drake::VectorX<T> qdot0(n_q_);
  drake::VectorX<T> qdot1(n_q_);
  plant_.MapVelocityToQDot(*context0_, v0, &qdot0);
  plant_.MapVelocityToQDot(*context1_, v1, &qdot1);
  *y = 0.5 * dt_ * (qdot0 + qdot1) + q0 - q1;
}

template<typename T>
CenterofMassPositionConstraint<T>::CenterofMassPositionConstraint(const drake::multibody::MultibodyPlant<T> &plant,
                                                                  drake::systems::Context<T> *context,
                                                                  int knot_index): dairlib::solvers::NonlinearConstraint<T>(
    3,  plant.num_positions()+ plant.num_velocities()+3,
    Eigen::VectorXd::Zero(3),
    Eigen::VectorXd::Zero(3),
    "center_of_mass_position[" +
        std::to_string(knot_index) + "]"),
                                                                                   plant_(plant),
                                                                                   context_(context),
                                                                                   n_q_(plant.num_positions()){}

/// The format of the input to the eval() function is in the order
///   - rCOM, location of the center of mass
///   - x0, state
template<typename T>
void CenterofMassPositionConstraint<T>::EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>> &x,
                                                           drake::VectorX<T> *y) const {
  const auto& rCom = x.segment(0, 3);
  const auto& x0 = x.segment(3, n_q_);
  dairlib::multibody::SetPositionsIfNew<T>(plant_, x0,  context_);
  *y = rCom - plant_.CalcCenterOfMassPositionInWorld(*context_);
}

template<typename T>
CentroidalMomentumConstraint<T>::CentroidalMomentumConstraint(const drake::multibody::MultibodyPlant<T> &plant,
                                                                  drake::systems::Context<T> *context,
                                                                  int knot_index): dairlib::solvers::NonlinearConstraint<T>(
    6,  plant.num_positions()+ plant.num_velocities()+6 + 3,
    Eigen::VectorXd::Zero(6),
    Eigen::VectorXd::Zero(6),
    "centroidal_momentum[" +
        std::to_string(knot_index) + "]"),
                                                                                   plant_(plant),
                                                                                   context_(context),
                                                                                   n_x_(plant.num_positions()
                                                                                            + plant.num_velocities()){}

/// The format of the input to the eval() function is in the order
///   - q, generalized positions
///   - v, generalized velocities
///   - com, location of the com
///   - h_WC, angular momentum, linear momentum in the wf about the com
template<typename T>
void CentroidalMomentumConstraint<T>::EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>> &x,
                                                           drake::VectorX<T> *y) const {
  const auto& x0 = x.head(n_x_);
  const auto& com = x.segment(n_x_, 3);
  const auto& h_WC = x.segment(n_x_ + 3, 6);
  dairlib::multibody::SetPositionsAndVelocitiesIfNew<T>(plant_, x0,  context_);
  const auto& spatial_momentum = plant_.CalcSpatialMomentumInWorldAboutPoint(*context_, com);
  *y = spatial_momentum.get_coeffs() - h_WC;
}

template<typename T>
CenterofMassVelocityConstraint<T>::CenterofMassVelocityConstraint(const drake::multibody::MultibodyPlant<T> &plant,
                                                                  drake::systems::Context<T> *context,
                                                                  int knot_index): dairlib::solvers::NonlinearConstraint<T>(
    3,  plant.num_positions()+ plant.num_velocities()+3,
    Eigen::VectorXd::Zero(3),
    Eigen::VectorXd::Zero(3),
    "center_of_mass_velocity[" +
        std::to_string(knot_index) + "]"),
                                                                                   plant_(plant),
                                                                                   context_(context),
                                                                                   n_x_(plant.num_positions()
                                                                                            + plant.num_velocities()){}

/// The format of the input to the eval() function is in the order
///   - drCOM, location of the center of mass
///   - x0, state
template<typename T>
void CenterofMassVelocityConstraint<T>::EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>> &x,
                                                           drake::VectorX<T> *y) const{
  const auto& drCom = x.segment(0, 3);
  const auto& x0 = x.segment(3, n_x_);

  dairlib::multibody::SetPositionsAndVelocitiesIfNew<T>(plant_, x0, context_);
  *y = drCom - plant_.CalcCenterOfMassTranslationalVelocityInWorld(*context_);
}

template<typename T>
AngularVelocityConstraint<T>::AngularVelocityConstraint(int knot_index): dairlib::solvers::NonlinearConstraint<T>(
    3, 4 + 3 + 3,
    Eigen::VectorXd::Zero(3),
    Eigen::VectorXd::Zero(3),
    "angular_velocity_constraint[" +
        std::to_string(knot_index) + "]") {}

/// The format of the input to the eval() function is in the order
///   - w_Q_b, quaternion describing transform from body to world
///   - omega_ewrt_b, body frame angular velocity
///   - omega_ewrt_w, world frame angular velocity
template<typename T>
void AngularVelocityConstraint<T>::EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>> &x,
                                                      drake::VectorX<T> *y) const {
  // We use this constructor since passing in a vector has the order [x,y,z,w], while x is [w,x,y,z]
  const Eigen::Quaternion<T> w_Q_b(x[0], x[1], x[2], x[3]);
  const auto& omega_ewrt_b = x.segment(4, 3);
  const auto& omega_ewrt_w = x.segment(7, 3);
  *y = omega_ewrt_w - w_Q_b * omega_ewrt_b;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS (class CentroidalDynamicsConstraint);
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS (class KinematicIntegratorConstraint);
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS (class CenterofMassPositionConstraint);
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS (class CentroidalMomentumConstraint);
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS (class CenterofMassVelocityConstraint);
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS (class AngularVelocityConstraint);
