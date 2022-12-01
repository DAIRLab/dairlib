#include <iostream>
#include <utility>
#include "planar_slip_constraints.h"
#include "multibody/multibody_utils.h"

PlanarSlipReductionConstraint::PlanarSlipReductionConstraint(const drake::multibody::MultibodyPlant<double> &plant,
                                                                std::shared_ptr<PlanarSlipReducer> reducing_function,
                                                                int n_slip_feet,
                                                                int n_complex_feet,
                                                                int knot_index)
    :dairlib::solvers::NonlinearConstraint<double>(
    3 + 3 + 3 * n_slip_feet + 3 * n_slip_feet + n_slip_feet,
    3 + 3 + 3 * n_slip_feet + 3 * n_slip_feet + n_slip_feet + 6 + 3 + 3 * 3 * n_complex_feet + plant.num_positions()
        + plant.num_velocities(),
    Eigen::VectorXd::Zero(3 + 3 + 3 * 2 * n_slip_feet + n_slip_feet),
    Eigen::VectorXd::Zero(3 + 3 + 3 * 2 * n_slip_feet + n_slip_feet),
    "PlanarSlipReductionConstraint[" +
        std::to_string(knot_index) + "]"),
     reducing_function_(reducing_function),
     slip_dim_(2 + 2 + 2 * 2 * n_slip_feet + n_slip_feet),
     complex_dim_(6 + 3 + 3 * 3 * n_complex_feet + plant.num_positions() + plant.num_velocities()) {}


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
void PlanarSlipReductionConstraint::EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>> &x,
                                                          drake::VectorX<double> *y) const {
  const auto& slip_state = x.head(slip_dim_);
  const auto& complex_state = x.tail(complex_dim_);
  *y = reducing_function_->Reduce(complex_state) - slip_state;
}


SlipGrfReductionConstrain::SlipGrfReductionConstrain(const drake::multibody::MultibodyPlant<double> &plant,
                                                     std::shared_ptr<PlanarSlipReducer> reducing_function,
                                                     int n_slip_feet,
                                                     int n_complex_feet,
                                                     int knot_index):dairlib::solvers::NonlinearConstraint<double>(
    n_slip_feet,
    3 + 3 * n_slip_feet + 3 * n_complex_feet + n_slip_feet,
    Eigen::VectorXd::Zero(n_slip_feet),
    Eigen::VectorXd::Zero(n_slip_feet),
    "SlipGrfReductionConstraint[" +
        std::to_string(knot_index) + "]"),
                                                                     reducing_function_(reducing_function),
                                                                     n_slip_feet_(n_slip_feet),
                                                                     n_complex_feet_(n_complex_feet) {}
/// Input is of the form:
///     complex_com
///     slip_contact_pos
///     complex_grf
///     slip_contact_force
void SlipGrfReductionConstrain::EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>> &x,
                                                   drake::VectorX<double> *y) const {
  const auto& complex_com = x.head(3);
  const auto& slip_contact_pos = x.segment(3,3*n_slip_feet_);
  const auto& complex_grf = x.segment(3 + 3*n_slip_feet_,3*n_complex_feet_);
  const auto& slip_contact_force = x.segment(3 + 3*n_slip_feet_ + 3*n_complex_feet_,n_slip_feet_);
  *y = reducing_function_->ReduceGrf(complex_com, slip_contact_pos, complex_grf) - slip_contact_force;

}

PlanarSlipLiftingConstraint::PlanarSlipLiftingConstraint(const drake::multibody::MultibodyPlant<double> &plant,
                                                         std::shared_ptr<PlanarSlipLifter> lifting_function,
                                                         int n_slip_feet,
                                                         int n_complex_feet,
                                                         int knot_index)
    : dairlib::solvers::NonlinearConstraint<double>(
    6 + 3 + 3 * 3 * n_complex_feet + plant.num_positions() + plant.num_velocities(),
    3 + 3 + 3 * 2 * n_slip_feet + 6 + 3 + 3 * 3 * n_complex_feet + plant.num_positions() + n_slip_feet
        + plant.num_velocities(),
    Eigen::VectorXd::Zero(
        6 + 3 + 3 * 3 * n_complex_feet + plant.num_positions() + plant.num_velocities()),
    Eigen::VectorXd::Zero(
        6 + 3 + 3 * 3 * n_complex_feet + plant.num_positions() + plant.num_velocities()),
    "PlanarSlipLiftingConstraint[" +
        std::to_string(knot_index) + "]"),
      lifting_function_(std::move(lifting_function)),
      slip_dim_(2 + 2 + 2 * 2 * n_slip_feet + n_slip_feet),
      complex_dim_(6 + 3 + 3 * 3 * n_complex_feet + plant.num_positions() + plant.num_velocities()){

}

/// Input is of the form and should match the lifting function input and output:
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
void PlanarSlipLiftingConstraint::EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>> &x,
                                                     drake::VectorX<double> *y) const {
  const auto& slip_state = x.head(slip_dim_);
  const auto& complex_state = x.tail(complex_dim_);
  *y = lifting_function_->Lift(slip_state) - complex_state;
}

template<typename T>
PlanarSlipDynamicsConstraint<T>::PlanarSlipDynamicsConstraint(double r0,
                                                              double k,
                                                              T m,
                                                              int n_feet,
                                                              std::vector<bool> contact_mask,
                                                              double dt,
                                                              int knot_index):dairlib::solvers::NonlinearConstraint<T>(
    3 + 3, 2 * (3 + 3 + 3 * n_feet + n_feet),
    Eigen::VectorXd::Zero(6),
    Eigen::VectorXd::Zero(6),
    "PlanarSlipDynamicsConstraint[" +
        std::to_string(knot_index) + "]"),
                                                                              r0_(r0),
                                                                              k_(k),
                                                                              m_(m),
                                                                              n_feet_(n_feet),
                                                                              contact_mask_(std::move(contact_mask)),
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
template<typename T>
void PlanarSlipDynamicsConstraint<T>::EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>> &x,
                                                         drake::VectorX<T> *y) const {
  const auto& com0 = x.head(3);
  const auto& vel0 = x.segment(3, 3);
  const auto& contact_pos0 = x.segment(3 + 3, n_feet_ * 3);
  const auto& force0 = x.segment(3 + 3 + n_feet_ * 3, n_feet_);
  const auto& com1 = x.segment(3 + 3 + n_feet_ * 3 + n_feet_, 3);
  const auto& vel1 = x.segment(3 + 3 + n_feet_ * 3 + 3 + n_feet_, 3);
  const auto& contact_pos1 = x.segment(3 + 3 + n_feet_ * 3 + 3 + 3 + n_feet_, n_feet_ * 3);
  const auto& force1 = x.segment(3 + 3 + n_feet_ * 3 + 3 + 3 + n_feet_ + n_feet_ * 3, n_feet_);

  const auto& x0 = x.head(6);
  const auto& x1 = x.segment(3 + 3 + n_feet_ * 3 + n_feet_, 6);

  drake::Vector<T, 6> xdot0 = CalcTimeDerivativesWithForce(com0, vel0,contact_pos0, force0);
  drake::Vector<T, 6> xdot1 = CalcTimeDerivativesWithForce(com1, vel1,contact_pos1, force1);

  // Predict state and return error
  const auto x1Predict = x0 + 0.5 * dt_ * (xdot0 + xdot1);
  *y = x1 - x1Predict;
}

template<typename T>
drake::VectorX<T> PlanarSlipDynamicsConstraint<T>::CalcTimeDerivativesWithForce(const drake::VectorX<T> &com_position,
                                                                                const drake::VectorX<T> &com_vel,
                                                                                const drake::VectorX<T> &contact_loc,
                                                                                const drake::VectorX<T> &slip_force) const {
  drake::Vector3<T> ddcom = {0, 0, -9.81};
  for(int foot = 0; foot < n_feet_; foot ++){
    if(contact_mask_[foot]){
      drake::Vector3<T> com_rt_foot = com_position - contact_loc.segment(3 * foot, 3);
      const auto r = com_rt_foot.norm();
      const auto unit_vec = com_rt_foot/r;
      const auto F = k_ * (r0_ - r) + slip_force[foot];
      ddcom = ddcom + F * unit_vec / m_;
    }
  }
  drake::Vector6<T> derivative;
  derivative << com_vel,ddcom;
  return derivative;
}

QuadraticLiftedCost::QuadraticLiftedCost(std::shared_ptr<PlanarSlipLifter> lifting_function,
                                            QuadraticLiftedCost::cost_element com_cost,
                                            QuadraticLiftedCost::cost_element momentum_cost,
                                            QuadraticLiftedCost::cost_element contact_cost,
                                            QuadraticLiftedCost::cost_element grf_cost,
                                            QuadraticLiftedCost::cost_element q_cost,
                                            QuadraticLiftedCost::cost_element v_cost,
                                            double terminal_gain,
                                            int n_slip_feet,
                                            int knot_point):dairlib::solvers::NonlinearCost<double>(
    3 + 3 + 3 * 2 * n_slip_feet + n_slip_feet, "LiftedCost[" +
        std::to_string(knot_point) + "]"),
                                                            lifting_function_(std::move(lifting_function)),
                                                            com_cost_(std::move(com_cost)),
                                                            momentum_cost_(std::move(momentum_cost)),
                                                            contact_cost_(std::move(contact_cost)),
                                                            grf_cost_(std::move(grf_cost)),
                                                            q_cost_(std::move(q_cost)),
                                                            v_cost_(std::move(v_cost)),
                                                            terminal_gain_(terminal_gain) {}

void QuadraticLiftedCost::EvaluateCost(const Eigen::Ref<const drake::VectorX<double>> &x, drake::VectorX<double> *y) const {
  const auto lifted_state = lifting_function_->Lift(x);

  const auto& com = lifted_state.head(3);
  const auto& momentum = lifted_state.segment(3,6);
  const auto& contact_info = lifted_state.segment(3 + 6, contact_cost_.ref.size());
  const auto& grf = lifted_state.segment(3 + 6 + contact_cost_.ref.size(), grf_cost_.ref.size());
  const auto& q = lifted_state.segment(3 + 6 + contact_cost_.ref.size() + grf_cost_.ref.size(), q_cost_.ref.size());
  const auto& v = lifted_state.segment(3 + 6 + contact_cost_.ref.size() + grf_cost_.ref.size() + q_cost_.ref.size(), v_cost_.ref.size());

  *y = CalcCost(com_cost_, com) + CalcCost(momentum_cost_, momentum) + CalcCost(contact_cost_, contact_info)
      + CalcCost(grf_cost_, grf) + CalcCost(q_cost_, q) + CalcCost(v_cost_, v);
}

Eigen::Matrix<double, -1, 1, 0> QuadraticLiftedCost::CalcCost(const QuadraticLiftedCost::cost_element &cost,
                                                                    const Eigen::Ref<const drake::VectorX<double>> &x)const {
  auto error = x-cost.ref;
  return terminal_gain_ * error.transpose() * cost.Q * error;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS (class PlanarSlipDynamicsConstraint);
