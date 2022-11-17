#include <iostream>
#include <utility>
#include "planar_slip_constraints.h"
#include "multibody/multibody_utils.h"

template<typename T>
PlanarSlipReductionConstraint<T>::PlanarSlipReductionConstraint(const drake::multibody::MultibodyPlant<T>& plant,
                                                                drake::systems::Context<T> *context,
                                                                const std::vector<dairlib::multibody::WorldPointEvaluator<
                                                                    T>> &slip_feet,
                                                                int complex_state_size,
                                                                int knot_index):dairlib::solvers::NonlinearConstraint<T>(
                                                                    2+2+2*2*slip_feet.size(), 2+2+2*2*slip_feet.size() + complex_state_size,
                                                                    Eigen::VectorXd::Zero(2+2+2*2*slip_feet.size()),
                                                                    Eigen::VectorXd::Zero(2+2+2*2*slip_feet.size()),
                                                                    "PlanarSlipReductionConstraint[" +
                                                                        std::to_string(knot_index) + "]"),
                                                                        m_(plant.CalcTotalMass(*context)),
                                                                        context_(context),
                                                                        plant_(plant),
                                                                        nx_(plant.num_positions()+plant.num_velocities()),
                                                                        slip_feet_(slip_feet)
                                                                    {}


/// Input is of the form:
///     slip_com
///     slip_velocity
///     slip_contact_pos
///     slip_contact_vel
///     complex_com
///     complex_ang_momentum
///     complex_lin_momentum
///     complex_contact_pos
///     complex_contact_vel
///     complex_contact_force
///     complex_gen_pos
///     complex_gen_vel
template<typename T>
void PlanarSlipReductionConstraint<T>::EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>> &x,
                                                          drake::VectorX<T> *y) const {
  const auto& slip_com = x.head(kSLIP_DIM);
  const auto& slip_vel = x.segment(kSLIP_DIM, kSLIP_DIM);
  const auto& slip_contact_pos = x.segment(kSLIP_DIM + kSLIP_DIM, kSLIP_DIM * slip_feet_.size());
  const auto& slip_contact_vel = x.segment(kSLIP_DIM + kSLIP_DIM + kSLIP_DIM * slip_feet_.size(), kSLIP_DIM * slip_feet_.size());
  const auto& complex_com = x.segment(kSLIP_DIM + kSLIP_DIM + kSLIP_DIM * slip_feet_.size() + kSLIP_DIM * slip_feet_.size(), 3);
  const auto& complex_ang_momentum = x.segment(kSLIP_DIM + kSLIP_DIM + kSLIP_DIM * slip_feet_.size() + kSLIP_DIM * slip_feet_.size() + 3, 3);
  const auto& complex_lin_momentum = x.segment(kSLIP_DIM + kSLIP_DIM + kSLIP_DIM * slip_feet_.size() + kSLIP_DIM * slip_feet_.size() + 3 + 3, 3);
  const auto& complex_gen_state = x.tail(nx_);

  dairlib::multibody::SetPositionsAndVelocitiesIfNew<T>(plant_, complex_gen_state,  context_);

  *y = drake::VectorX<T>::Zero(2+2+2*kSLIP_DIM*slip_feet_.size());
  y->head(kSLIP_DIM) = slip_com - slip_index_.unaryExpr(complex_com);

  y->segment(kSLIP_DIM, kSLIP_DIM) = slip_vel * m_ - slip_index_.unaryExpr(complex_lin_momentum);
  for(int i = 0; i<slip_feet_.size(); i++){
    y->segment(kSLIP_DIM+kSLIP_DIM+kSLIP_DIM * i, kSLIP_DIM) = slip_contact_pos.segment(kSLIP_DIM * i, kSLIP_DIM) - slip_index_.unaryExpr(slip_feet_[i].EvalFull(*context_));
    y->segment(kSLIP_DIM+kSLIP_DIM+kSLIP_DIM * slip_feet_.size()+kSLIP_DIM * i, kSLIP_DIM) = slip_contact_vel.segment(kSLIP_DIM * i, kSLIP_DIM) - slip_index_.unaryExpr(slip_feet_[i].EvalFullTimeDerivative(*context_));
  }
}

PlanarSlipLiftingConstraint::PlanarSlipLiftingConstraint(const drake::multibody::MultibodyPlant<double> &plant,
                                                         std::shared_ptr<PlanarSlipLifter> lifting_function,
                                                         int n_slip_feet,
                                                         int n_complex_feet,
                                                         int knot_index)
    : dairlib::solvers::NonlinearConstraint<double>(
    6 + 3 + 3 * 3 * n_complex_feet + plant.num_positions() + plant.num_velocities(),
    2 + 2 + 2 * 2 * n_slip_feet + 6 + 3 + 3 * 3 * n_complex_feet + plant.num_positions()
        + plant.num_velocities(),
    Eigen::VectorXd::Zero(
        6 + 3 + 3 * 3 * n_complex_feet + plant.num_positions() + plant.num_velocities()),
    Eigen::VectorXd::Zero(
        6 + 3 + 3 * 3 * n_complex_feet + plant.num_positions() + plant.num_velocities()),
    "PlanarSlipLiftingConstraint[" +
        std::to_string(knot_index) + "]"),
      lifting_function_(std::move(lifting_function)),
      slip_dim_(2 + 2 + 2 * 2 * n_slip_feet),
      complex_dim_(6 + 3 + 3 * 3 * n_complex_feet + plant.num_positions() + plant.num_velocities()){

}

/// Input is of the form and should match the lifting function input and output:
///     slip_com
///     slip_velocity
///     slip_contact_pos
///     slip_contact_vel
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
  *y = lifting_function_->Lift(slip_state);
  *y = *y - complex_state;
}

template<typename T>
PlanarSlipDynamicsConstraint<T>::PlanarSlipDynamicsConstraint(double r0,
                                                              double k,
                                                              T m,
                                                              int n_feet,
                                                              std::vector<bool> contact_mask,
                                                              double dt,
                                                              int knot_index):dairlib::solvers::NonlinearConstraint<T>(
    2 + 2, 2 * (2 + 2 + 2 * n_feet),
    Eigen::VectorXd::Zero(2 + 2),
    Eigen::VectorXd::Zero(2 + 2),
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
///   - com1, center of mass at time k+1
///   - vel1, center of mass velocity at time k+1
///   - contact_pos1, active contact positions at time k+1
template<typename T>
void PlanarSlipDynamicsConstraint<T>::EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>> &x,
                                                         drake::VectorX<T> *y) const {
  const auto& com0 = x.head(2);
  const auto& vel0 = x.segment(2, 2);
  const auto& contact_pos0 = x.segment(2 + 2, n_feet_ * 2);
  const auto& com1 = x.segment(2 + 2 + n_feet_ * 2, 2);
  const auto& vel1 = x.segment(2 + 2 + n_feet_ * 2 + 2, 2);
  const auto& contact_pos1 = x.segment(2 + 2 + n_feet_ * 2 + 2 + 2, n_feet_ * 2);

  const auto& x0 = x.head(4);
  const auto& x1 = x.segment(2 + 2 + n_feet_ * 2, 4);

  drake::Vector<T, 4> xdot0 = CalcTimeDerivativesWithForce(com0, vel0,contact_pos0);
  drake::Vector<T, 4> xdot1 = CalcTimeDerivativesWithForce(com1, vel1,contact_pos1);

  // Predict state and return error
  const auto x1Predict = x0 + 0.5 * dt_ * (xdot0 + xdot1);
  *y = x1 - x1Predict;
}

template<typename T>
drake::VectorX<T> PlanarSlipDynamicsConstraint<T>::CalcTimeDerivativesWithForce(const drake::VectorX<T> &com_position,
                                                                                const drake::VectorX<T> &com_vel,
                                                                                const drake::VectorX<T> &contact_loc) const {
  drake::Vector2<T> ddcom = {0, -9.81};
  for(int foot = 0; foot < n_feet_; foot ++){
    if(contact_mask_[foot]){
      drake::Vector2<T> com_rt_foot = com_position - contact_loc.segment(2 * foot, 2);
      const auto r = com_rt_foot.norm();
      const auto unit_vec = com_rt_foot/r;
      const auto F = k_ * (r0_ - r);
      ddcom = ddcom + F * unit_vec / m_;
    }
  }
  drake::Vector4<T> derivative;
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
    2 + 2 + 2 * 2 * n_slip_feet, "LiftedCost[" +
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

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS (class PlanarSlipReductionConstraint);
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS (class PlanarSlipDynamicsConstraint);
