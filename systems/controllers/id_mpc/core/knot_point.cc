#include "knot_point.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::VectorX;
using Eigen::MatrixXd;

KnotPoint::KnotPoint(
    const ConstrainedDynamicsInfo &dynamics, knot_config config):
    dynamics_info_(dynamics), config_(config){
  nq_ = dynamics.nq();
  nv_ = dynamics.nv();
  nh_ = config_.state_only ? 0: dynamics.nh();
  nc_ = config_.state_only ? 0: dynamics.nc();
  nu_ = (config_.state_only || (!config_.include_torques)) ? 0 : dynamics.nu();
  if (config_.state_only) {
    config_.include_torques = false;
  }
  actuation_matrix_ = dynamics.get_plant().MakeActuationMatrix();
  for (int i = 0; i < dynamics.nv(); ++i) {
    if (actuation_matrix_.row(i).squaredNorm() == 0) {
      unactuated_vdot_coords_.push_back(i);
    }
  }
  DRAKE_DEMAND(unactuated_vdot_coords_.size() == (dynamics.nv() - dynamics.nu()));

  // TODO (@Brian-Acosta) Validate active constraint and constraint-dot indices
}

template<typename T>
const VectorX<T> KnotPoint::get_q(
    const VectorX<T>& full_vars) const {
  return full_vars.head(nq_);
}

template<typename T>
const VectorX<T> KnotPoint::get_v(
    const VectorX<T>& full_vars) const {
  return full_vars.segment(nq_, nv_);
}

template<typename T>
const VectorX<T> KnotPoint::get_x(
    const VectorX<T>& full_vars) const {
  return full_vars.segment(0, nq_ + nv_);
}

template<typename T>
const VectorX<T> KnotPoint::get_lh(
    const VectorX<T>& full_vars) const {
  return full_vars.segment(nq_ + nv_, nh_);
}

template<typename T>
const VectorX<T> KnotPoint::get_lc(
    const VectorX<T>& full_vars) const {
  return full_vars.segment(nq_ + nv_ + nh_, nc_);
}

template<typename T>
const VectorX<T> KnotPoint::get_lambda(
    const VectorX<T>& full_vars) const {
  return full_vars.segment(nq_ + nv_, nh_ + nc_);
}


template<typename T>
const VectorX<T> KnotPoint::get_u(
    const VectorX<T>& full_vars) const {
  return full_vars.segment(nq_ + nv_ + nh_ + nc_, nu_);
}

template <typename T>
VectorX<T> KnotPoint::EvalInverseDynamicsDefect(
    KnotPointState *cache, const VectorX<T>& x,
    const VectorX<T>& lambdas_and_maybe_us, const VectorX<T>& vdot) const {

  VectorX<T> lambda = lambdas_and_maybe_us.head(dynamics_info_.nh() + dynamics_info_.nc());
  cache->UpdateDynamics(x, vdot, lambda);

  const VectorX<T> tau = cache->GetTau<T>();

  if (config_.include_torques) {
    return
      actuation_matrix_ * lambdas_and_maybe_us.tail(dynamics_info_.nu()) - tau;
  }

  VectorX<T> out = VectorX<T>::Zero(unactuated_vdot_coords_.size());
  for (int i = 0; i < out.rows(); ++i) {
    out(i) = tau(unactuated_vdot_coords_[i]);
  }
  return out;
}

template <typename T>
VectorX<T> KnotPoint::EvalKinematicConstraints(
    KnotPointState *cache, const VectorX<T> &x) const {
  cache->UpdateKinematics(x);
  VectorX<T> out = VectorX<T>::Zero(
      config_.active_constraint_indices.size() +
      config_.active_constraint_dot_indices.size());

  size_t nca = config_.active_constraint_indices.size();
  for (size_t i = 0; i < nca ; ++i) {
    out(i) = cache->GetKinematicConstraints<T>()(
        config_.active_constraint_indices.at(i));
  }
  for (size_t i = 0; i < config_.active_constraint_dot_indices.size(); ++i) {
    out(nca + i) = cache->GetKinematicConstraintsDot<T>()(
        config_.active_constraint_dot_indices.at(i));
  }
  return out;
}

template const VectorX<double> KnotPoint::get_q(const 
    VectorX<double> &full_vars) const;
template const VectorX<double> KnotPoint::get_v(const 
    VectorX<double> &full_vars) const;
template const VectorX<double> KnotPoint::get_x(const 
    VectorX<double> &full_vars) const;
template const VectorX<double> KnotPoint::get_u(const 
    VectorX<double> &full_vars) const;
template const VectorX<double> KnotPoint::get_lh(const 
    VectorX<double> &full_vars) const;
template const VectorX<double> KnotPoint::get_lc(const 
    VectorX<double> &full_vars) const;

template const VectorX<AutoDiffXd> KnotPoint::get_q(const 
    VectorX<AutoDiffXd> &full_vars) const;
template const VectorX<AutoDiffXd> KnotPoint::get_v(const 
    VectorX<AutoDiffXd> &full_vars) const;
template const VectorX<AutoDiffXd> KnotPoint::get_x(const 
    VectorX<AutoDiffXd> &full_vars) const;
template const VectorX<AutoDiffXd> KnotPoint::get_u(const 
    VectorX<AutoDiffXd> &full_vars) const;
template const VectorX<AutoDiffXd> KnotPoint::get_lh(const 
    VectorX<AutoDiffXd> &full_vars) const;
template const VectorX<AutoDiffXd> KnotPoint::get_lc(const 
    VectorX<AutoDiffXd> &full_vars) const;

template VectorX<double> KnotPoint::EvalInverseDynamicsDefect(
    KnotPointState *cache, const VectorX<double>& x,
    const VectorX<double>& lambdas_and_maybe_us,
    const VectorX<double>& vdot) const;

template VectorX<AutoDiffXd> KnotPoint::EvalInverseDynamicsDefect(
    KnotPointState *cache, const VectorX<AutoDiffXd>& x,
    const VectorX<AutoDiffXd>& lambdas_and_maybe_us,
    const VectorX<AutoDiffXd>& vdot) const;

template VectorX<double> KnotPoint::EvalKinematicConstraints(
    KnotPointState *cache, const VectorX<double> &x) const;

template VectorX<AutoDiffXd> KnotPoint::EvalKinematicConstraints(
    KnotPointState *cache, const VectorX<AutoDiffXd> &x) const;


}