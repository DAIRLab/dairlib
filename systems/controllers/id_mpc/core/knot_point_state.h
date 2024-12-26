#pragma once

#include "constrained_dynamics_info.h"
#include "solvers/nonlinear_constraint.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib::systems::controllers::id_mpc {

/*!
 * Wrapper class meant to hold mutable state for a single knot point
 *
 * holds the appropriate plant contexts and a cache of the most recently
 * evaluated dynamics values
 */
class KnotPointState {
 public:
  explicit KnotPointState(const ConstrainedDynamicsInfo& dynamics);

  const ConstrainedDynamicsInfo& get_dynamics() const {
    return dynamics_;
  }

  template<typename T>
  void Update(const drake::VectorX<T>& vars);

  template<typename T>
  drake::VectorX<T> GetXDot() const;

  template<typename T>
  drake::VectorX<T> GetKinematicConstraints() const;

  void UpdateActiveContacts(const std::vector<std::string>& active_contacts);

 private:

  template<typename T>
  struct cache {
    bool dirty = true;
    drake::VectorX<T> all_vars;
    std::unique_ptr<drake::systems::Context<T>> context;
    ConstrainedDynamicsInfo::DynamicsEvaluation<T> dynamics_results;
    std::vector<std::string> active_contacts;
  };

  cache<double> cache_;
  cache<AutoDiffXd> cache_ad_;

  const ConstrainedDynamicsInfo& dynamics_;

};

}
