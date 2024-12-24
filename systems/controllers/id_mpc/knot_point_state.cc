#include "knot_point_state.h"
#include "common/eigen_utils.h"

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
  cache_.all_vars = VectorXd::Zero(dynamics.variable_count());
  cache_.dynamics_results = dynamics.MakeEmptyDynamicsEvaluation<double>();
  cache_.context = dynamics.MakeContext<double>();

  cache_ad_.all_vars = AutoDiffVecXd::Zero(dynamics.variable_count());
  cache_ad_.dynamics_results = dynamics.MakeEmptyDynamicsEvaluation<AutoDiffXd>();
  cache_ad_.context = dynamics.MakeContext<AutoDiffXd>();
}

template<>
VectorX<double> KnotPointState::GetKinematicConstraints() const {
  return stack<double>({
    cache_.dynamics_results.c_,
    cache_.dynamics_results.cdot_,
    cache_.dynamics_results.cddot_
  });
}

template<>
VectorX<AutoDiffXd> KnotPointState::GetKinematicConstraints() const {
  return stack<AutoDiffXd>({
    cache_ad_.dynamics_results.c_,
    cache_ad_.dynamics_results.cdot_,
    cache_ad_.dynamics_results.cddot_
  });
}

template<>
VectorX<double> KnotPointState::GetXDot() const {
  return stack<double>(
      {cache_.dynamics_results.qdot_, cache_.dynamics_results.vdot_});
}

template<>
VectorX<AutoDiffXd> KnotPointState::GetXDot() const {
  return stack<AutoDiffXd>(
      {cache_ad_.dynamics_results.qdot_, cache_ad_.dynamics_results.vdot_});
}

void KnotPointState::UpdateActiveContacts(
    const std::vector<std::string>& active_contacts) {
  cache_.dirty = active_contacts != cache_.active_contacts;
  cache_.active_contacts = active_contacts;

  cache_ad_.dirty = cache_ad_.active_contacts != active_contacts;
  cache_ad_.active_contacts = active_contacts;
}

template<>
void KnotPointState::Update(const VectorXd& vars) {
  if (cache_.dirty || !AreVectorsEqual(vars, cache_.all_vars)) {
    cache_.all_vars = vars;
    cache_.dynamics_results = dynamics_.EvaluateDynamics(
        cache_.context.get(), cache_.all_vars, cache_.active_contacts);
    cache_.dirty = false;
  }
}

template<>
void KnotPointState::Update(
    const VectorX<AutoDiffXd>& vars) {
  if (cache_ad_.dirty || !AreVectorsEqual(vars, cache_ad_.all_vars)) {
    cache_ad_.all_vars = vars;
    cache_ad_.dynamics_results = dynamics_.EvaluateDynamics(
        cache_ad_.context.get(), cache_ad_.all_vars, cache_ad_.active_contacts);
    cache_ad_.dirty = false;
  }
}

}