#include "knot_point_state.h"
#include "common/eigen_utils.h"
#include "multibody/multibody_utils.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::VectorX;
using drake::AutoDiffVecXd;
using drake::math::AreAutoDiffVecXdEqual;

using Eigen::VectorXd;

namespace {
bool AreVectorsEqual(const Eigen::Ref<const AutoDiffVecXd> &a,
                     const Eigen::Ref<const AutoDiffVecXd> &b) {
  return AreAutoDiffVecXdEqual(a, b);
}

bool AreVectorsEqual(const Eigen::Ref<const VectorXd> &a,
                     const Eigen::Ref<const VectorXd> &b) {
  return a == b;
}
}

KnotPointState::KnotPointState(const ConstrainedDynamicsInfo &dynamics) :
      dynamics_(dynamics) {
  k_cache_.x = VectorXd::Zero(dynamics.nx());
  k_cache_.kinematics = dynamics.MakeEmptyKinematicsResults<double>();
  k_cache_.context = dynamics.MakeContext<double>();
  d_cache_.vdot = VectorXd::Zero(dynamics.nv());
  d_cache_.tau - VectorXd::Zero(dynamics.nv());

  k_cache_ad_.x = AutoDiffVecXd::Zero(dynamics.nx());
  k_cache_ad_.kinematics = dynamics.MakeEmptyKinematicsResults<AutoDiffXd>();
  k_cache_ad_.context = dynamics.MakeContext<AutoDiffXd>();
  d_cache_ad_.vdot = AutoDiffVecXd::Zero(dynamics.nv());
  d_cache_ad_.tau - AutoDiffVecXd::Zero(dynamics.nv());
}

template<>
const VectorX<double>& KnotPointState::GetKinematicConstraints() const {
    return k_cache_.kinematics.c;
}

template<>
const VectorX<AutoDiffXd>& KnotPointState::GetKinematicConstraints() const {
  return k_cache_ad_.kinematics.c;
}

template<>
const VectorX<double>& KnotPointState::GetKinematicConstraintsDot() const {
  return k_cache_.kinematics.cdot;
}

template<>
const VectorX<AutoDiffXd>& KnotPointState::GetKinematicConstraintsDot() const {
  return k_cache_ad_.kinematics.cdot;
}

template<>
const VectorX<double>& KnotPointState::GetQDot() const {
  DRAKE_DEMAND(not k_cache_.dirty);
  return k_cache_.kinematics.qdot;
}

template<>
const VectorX<AutoDiffXd>& KnotPointState::GetQDot() const {
  DRAKE_DEMAND(not k_cache_ad_.dirty);
  return k_cache_ad_.kinematics.qdot;
}

template<>
const VectorX<double>& KnotPointState::GetTau() const {
  DRAKE_DEMAND(not d_cache_.dirty);
  return d_cache_.tau;
}

template<>
const VectorX<AutoDiffXd>& KnotPointState::GetTau() const {
  DRAKE_DEMAND(not d_cache_ad_.dirty);
  return d_cache_ad_.tau;
}


void KnotPointState::UpdateActiveContacts(
    const std::vector<std::string>& active_contacts) {
  k_cache_.dirty = active_contacts != k_cache_.active_contacts;
  k_cache_.active_contacts = active_contacts;

  k_cache_ad_.dirty = k_cache_ad_.active_contacts != active_contacts;
  k_cache_ad_.active_contacts = active_contacts;

  d_cache_.dirty = k_cache_.dirty;
  d_cache_ad_.dirty = k_cache_ad_.dirty;
}

template<>
void KnotPointState::UpdateKinematics(const VectorXd& x) {
  if (k_cache_.dirty || !AreVectorsEqual(x, k_cache_.x)) {
    k_cache_.x = x;
    dynamics_.SetPlantStateIfNew(x, k_cache_.context.get());
    k_cache_.kinematics = dynamics_.EvaluateKinematics<double>(
        *k_cache_.context, k_cache_.active_contacts);
    k_cache_.dirty = false;
    d_cache_.dirty = true;
  }
}

template<>
void KnotPointState::UpdateKinematics(const AutoDiffVecXd& x) {
  if (k_cache_ad_.dirty || !AreVectorsEqual(x, k_cache_ad_.x)) {
    k_cache_ad_.x = x;
    dynamics_.SetPlantStateIfNew(x, k_cache_ad_.context.get());
    k_cache_ad_.kinematics = dynamics_.EvaluateKinematics<AutoDiffXd>(
        *k_cache_ad_.context, k_cache_ad_.active_contacts);
    k_cache_ad_.dirty = false;
    d_cache_ad_.dirty = true;
  }
}

template<>
void KnotPointState::UpdateDynamics(
    const VectorXd& x, const VectorXd& vdot, const VectorXd& lambda) {
  UpdateKinematics(x);
  if (d_cache_.dirty or !AreVectorsEqual(vdot, d_cache_.vdot) or
                        !AreVectorsEqual(lambda, d_cache_.lambda)) {
    d_cache_.vdot = vdot;
    d_cache_.lambda = lambda;
    d_cache_.tau = dynamics_.EvaluateInverseDynamics(
        *k_cache_.context, k_cache_.kinematics, vdot, lambda);
  }
}

template<>
void KnotPointState::UpdateDynamics(
    const AutoDiffVecXd& x, const AutoDiffVecXd& vdot, const AutoDiffVecXd& lambda) {
  UpdateKinematics(x);
  if (d_cache_ad_.dirty or !AreVectorsEqual(vdot, d_cache_ad_.vdot) or
                           !AreVectorsEqual(lambda, d_cache_ad_.lambda)) {
    d_cache_ad_.vdot = vdot;
    d_cache_ad_.lambda = lambda;
    d_cache_ad_.tau = dynamics_.EvaluateInverseDynamics(
        *k_cache_ad_.context, k_cache_ad_.kinematics, vdot, lambda);
  }
}

}