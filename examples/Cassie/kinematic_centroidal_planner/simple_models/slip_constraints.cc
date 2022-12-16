#include "slip_constraints.h"

#include <iostream>
#include <utility>

#include "examples/Cassie/kinematic_centroidal_planner/simple_models/slip_utils.h"
#include "multibody/multibody_utils.h"

double SlipGrf(double k, double r0, double b, double r, double dr,
               double force) {}

SlipReductionConstraint::SlipReductionConstraint(
    const drake::multibody::MultibodyPlant<double>& plant,
    std::shared_ptr<SlipReducer> reducing_function, int n_slip_feet,
    int n_complex_feet, int knot_index)
    : dairlib::solvers::NonlinearConstraint<double>(
          3 + 3 + 3 * n_slip_feet + 3 * n_slip_feet + n_slip_feet,
          3 + 3 + 3 * n_slip_feet + 3 * n_slip_feet + n_slip_feet + 6 + 3 +
              3 * 3 * n_complex_feet + plant.num_positions() +
              plant.num_velocities(),
          Eigen::VectorXd::Zero(3 + 3 + 3 * 2 * n_slip_feet + n_slip_feet),
          Eigen::VectorXd::Zero(3 + 3 + 3 * 2 * n_slip_feet + n_slip_feet),
          "SlipReductionConstraint[" + std::to_string(knot_index) + "]"),
      reducing_function_(reducing_function),
      slip_dim_(3 + 3 + 2 * 3 * n_slip_feet + n_slip_feet),
      complex_dim_(6 + 3 + 3 * 3 * n_complex_feet + plant.num_positions() +
                   plant.num_velocities()) {}

/// Input is of the form:
///     slip_com
///     slip_velocity
///     slip_contact_pos
///     slip_contact_vel
///     slip_force
///     complex_com
///     complex_ang_momentum
///     complex_lin_momentum
///     complex_contact_pos
///     complex_contact_vel
///     complex_contact_force
///     complex_gen_pos
///     complex_gen_vel
void SlipReductionConstraint::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<double>>& x,
    drake::VectorX<double>* y) const {
  const auto& slip_state = x.head(slip_dim_);
  const auto& complex_state = x.tail(complex_dim_);
  *y = reducing_function_->Reduce(complex_state) - slip_state;
}

SlipGrfReductionConstrain::SlipGrfReductionConstrain(
    const drake::multibody::MultibodyPlant<double>& plant,
    std::shared_ptr<SlipReducer> reducing_function, int n_slip_feet,
    int n_complex_feet, int knot_index)
    : dairlib::solvers::NonlinearConstraint<double>(
          n_slip_feet,
          3 + 3 * n_slip_feet + 3 * n_complex_feet + n_slip_feet + 3,
          Eigen::VectorXd::Zero(n_slip_feet),
          Eigen::VectorXd::Zero(n_slip_feet),
          "SlipGrfReductionConstraint[" + std::to_string(knot_index) + "]"),
      reducing_function_(reducing_function),
      n_slip_feet_(n_slip_feet),
      n_complex_feet_(n_complex_feet) {}
/// Input is of the form:
///     complex_com
///     slip com velocity
///     slip_contact_pos
///     complex_grf
///     slip_contact_force
void SlipGrfReductionConstrain::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<double>>& x,
    drake::VectorX<double>* y) const {
  const auto& complex_com = x.head(3);
  const auto& slip_vel = x.segment(3, 3);
  const auto& slip_contact_pos = x.segment(3 + 3, 3 * n_slip_feet_);
  const auto& complex_grf =
      x.segment(3 + 3 + 3 * n_slip_feet_, 3 * n_complex_feet_);
  const auto& slip_contact_force =
      x.segment(3 + 3 + 3 * n_slip_feet_ + 3 * n_complex_feet_, n_slip_feet_);
  *y = reducing_function_->ReduceGrf(complex_com, slip_vel, slip_contact_pos,
                                     complex_grf) -
       slip_contact_force;
}

template <typename T>
SlipDynamicsConstraint<T>::SlipDynamicsConstraint(
    double r0, double k, double b, T m, int n_feet,
    std::vector<bool> contact_mask0, std::vector<bool> contact_mask1, double dt,
    int knot_index)
    : dairlib::solvers::NonlinearConstraint<T>(
          3 + 3, 2 * (3 + 3 + 3 * n_feet + n_feet), Eigen::VectorXd::Zero(6),
          Eigen::VectorXd::Zero(6),
          "SlipDynamicsConstraint[" + std::to_string(knot_index) + "]"),
      r0_(r0),
      k_(k),
      b_(b),
      m_(m),
      n_feet_(n_feet),
      contact_mask0_(std::move(contact_mask0)),
      contact_mask1_(std::move(contact_mask1)),
      dt_(dt) {}

/// The format of the input to the eval() function is in the order
///   - com0, center of mass at time k
///   - vel0, center of mass velocity at time k
///   - contact_pos0, active contact positions at time k
///   - force0, slip force in parallel with spring at time k
///   - com1, center of mass at time k+1
///   - vel1, center of mass velocity at time k+1
///   - contact_pos1, active contact positions at time k+1
///   - force1, slip force in parallel with spring at time k+1
template <typename T>
void SlipDynamicsConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<T>>& x, drake::VectorX<T>* y) const {
  const auto& com0 = x.head(3);
  const auto& vel0 = x.segment(3, 3);
  const auto& contact_pos0 = x.segment(3 + 3, n_feet_ * 3);
  const auto& force0 = x.segment(3 + 3 + n_feet_ * 3, n_feet_);
  const auto& com1 = x.segment(3 + 3 + n_feet_ * 3 + n_feet_, 3);
  const auto& vel1 = x.segment(3 + 3 + n_feet_ * 3 + 3 + n_feet_, 3);
  const auto& contact_pos1 =
      x.segment(3 + 3 + n_feet_ * 3 + 3 + 3 + n_feet_, n_feet_ * 3);
  const auto& force1 =
      x.segment(3 + 3 + n_feet_ * 3 + 3 + 3 + n_feet_ + n_feet_ * 3, n_feet_);

  const auto& x0 = x.head(6);
  const auto& x1 = x.segment(3 + 3 + n_feet_ * 3 + n_feet_, 6);

  drake::Vector<T, 6> xdot0 = CalcTimeDerivativesWithForce(
      com0, vel0, contact_pos0, force0, contact_mask0_);
  drake::Vector<T, 6> xdot1 = CalcTimeDerivativesWithForce(
      com1, vel1, contact_pos1, force1, contact_mask1_);

  // Predict state and return error
  const auto x1Predict = x0 + 0.5 * dt_ * (xdot0 + xdot1);
  *y = x1 - x1Predict;
}

template <typename T>
drake::VectorX<T> SlipDynamicsConstraint<T>::CalcTimeDerivativesWithForce(
    const drake::VectorX<T>& com_position, const drake::VectorX<T>& com_vel,
    const drake::VectorX<T>& contact_loc, const drake::VectorX<T>& slip_force,
    const std::vector<bool>& contact_mask) const {
  drake::Vector3<T> ddcom = {0, 0, -9.81};
  for (int foot = 0; foot < n_feet_; foot++) {
    if (contact_mask[foot]) {
      drake::Vector3<T> com_rt_foot =
          com_position - contact_loc.segment(3 * foot, 3);
      const auto r = com_rt_foot.norm();
      const auto unit_vec = com_rt_foot / r;
      const auto dr = com_vel.dot(unit_vec);
      auto F = SlipGrf<T>(k_, r0_, b_, r, dr, slip_force[foot]);
      ddcom = ddcom + F * unit_vec / m_;
    }
  }
  drake::Vector6<T> derivative;
  derivative << com_vel, ddcom;
  return derivative;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class SlipDynamicsConstraint);
