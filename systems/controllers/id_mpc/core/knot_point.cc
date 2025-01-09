#include "knot_point.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::VectorX;

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
const VectorX<T> KnotPoint::get_u(
    const VectorX<T>& full_vars) const {
  return full_vars.segment(nq_ + nv_ + nh_ + nc_, nu_);
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


}