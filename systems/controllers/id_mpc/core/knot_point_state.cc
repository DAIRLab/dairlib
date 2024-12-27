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

KnotPointState::KnotPointState(
    const ConstrainedDynamicsInfo &dynamics) : dynamics_(dynamics) {
  cache_.vdot = VectorXd::Zero(dynamics.nv());
  cache_.decision_vars = VectorXd::Zero(dynamics.variable_count());
  cache_.dynamics_results = dynamics.MakeEmptyDynamicsEvaluation<double>();
  cache_.context = dynamics.MakeContext<double>();

  cache_ad_.decision_vars = AutoDiffVecXd::Zero(dynamics.nv());
  cache_ad_.decision_vars = AutoDiffVecXd::Zero(dynamics.variable_count());
  cache_ad_.dynamics_results = dynamics.MakeEmptyDynamicsEvaluation<AutoDiffXd>();
  cache_ad_.context = dynamics.MakeContext<AutoDiffXd>();
}

template<>
VectorX<double> KnotPointState::GetKinematicConstraints() const {
  return stack<double>({
    cache_.dynamics_results.c_.head(dynamics_.nh()),
    cache_.dynamics_results.cdot_
  });
}

template<>
VectorX<AutoDiffXd> KnotPointState::GetKinematicConstraints() const {
  return stack<AutoDiffXd>({
    cache_ad_.dynamics_results.c_.head(dynamics_.nh()),
    cache_ad_.dynamics_results.cdot_
  });
}

template<>
VectorX<double> KnotPointState::GetQDot() const {
  return cache_.dynamics_results.qdot_;
}

template<>
VectorX<AutoDiffXd> KnotPointState::GetTau() const {
  return cache_ad_.dynamics_results.tau_;
}

template<>
VectorX<double> KnotPointState::GetTau() const {
  return cache_.dynamics_results.tau_;
}

template<>
VectorX<AutoDiffXd> KnotPointState::GetQDot() const {
  return cache_ad_.dynamics_results.qdot_;
}

void KnotPointState::UpdateActiveContacts(
    const std::vector<std::string>& active_contacts) {
  cache_.dirty = active_contacts != cache_.active_contacts;
  cache_.active_contacts = active_contacts;

  cache_ad_.dirty = cache_ad_.active_contacts != active_contacts;
  cache_ad_.active_contacts = active_contacts;
}

template<>
void KnotPointState::SetVDot(const drake::VectorX<double> &vdot) {
  cache_.dirty = AreVectorsEqual(cache_.vdot, vdot);
  cache_.vdot = vdot;
}

template<>
void KnotPointState::SetVDot(const drake::VectorX<AutoDiffXd> &vdot) {
  cache_ad_.dirty = AreVectorsEqual(cache_ad_.vdot, vdot);
  cache_ad_.vdot = vdot;
}

template<>
void KnotPointState::Update(const VectorXd& vars) {
  if (cache_.dirty || !AreVectorsEqual(vars, cache_.decision_vars)) {
    cache_.decision_vars = vars;
    dynamics_.SetPlantStateIfNew(dynamics_.get_x(vars), cache_.context.get());
    cache_.dynamics_results = dynamics_.EvaluateDynamics<double>(
        *cache_.context,
        cache_.vdot,
        dynamics_.get_lh(vars),
        dynamics_.get_lc(vars),
        cache_.active_contacts);
    cache_.dirty = false;
  }
}

template<>
void KnotPointState::Update(
    const VectorX<AutoDiffXd>& vars) {
  if (cache_ad_.dirty || !AreVectorsEqual(vars, cache_ad_.decision_vars)) {
    cache_ad_.decision_vars = vars;
    dynamics_.SetPlantStateIfNew(
        dynamics_.get_x(vars), cache_ad_.context.get());
    cache_ad_.dynamics_results = dynamics_.EvaluateDynamics<AutoDiffXd>(
        *cache_ad_.context,
        cache_ad_.vdot,
        dynamics_.get_lh(vars),
        dynamics_.get_lc(vars),
        cache_ad_.active_contacts);
    cache_ad_.dirty = false;
  }
}

}