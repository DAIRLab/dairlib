#pragma once

#include "constrained_inverse_dynamics_info.h"

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

  double time() {return timestamp;}

  template<typename T>
  void UpdateKinematics(const drake::VectorX<T>& x);

  template<typename T>
  void UpdateDynamics(const drake::VectorX<T>&x,
                      const drake::VectorX<T>& vdot,
                      const drake::VectorX<T>& lambda);

  template<typename T>
  const drake::VectorX<T>& GetQDot() const;

  template<typename T>
  const drake::VectorX<T>& GetTau() const;

  template<typename T>
  const drake::VectorX<T>& GetKinematicConstraints() const;

  template<typename T>
  const drake::VectorX<T>& GetKinematicConstraintsDot() const;

  void UpdateActiveContacts(const std::vector<std::string>& active_contacts);
  void UpdateTimestamp(double t) {
    timestamp = t;
  }

 private:
  double timestamp;

  template<typename T>
  struct KinematicsCache {
    bool dirty = true;
    drake::VectorX<T> x;
    std::unique_ptr<drake::systems::Context<T>> context;
    ConstrainedDynamicsInfo::KinematicsResults<T> kinematics;
    std::vector<std::string> active_contacts;
  };

  template<typename T>
  struct DynamicsCache {
    bool dirty = true;
    drake::VectorX<T> vdot;
    drake::VectorX<T> lambda;
    drake::VectorX<T> tau;
  };

  KinematicsCache<double> k_cache_;
  KinematicsCache<AutoDiffXd> k_cache_ad_;

  DynamicsCache<double> d_cache_;
  DynamicsCache<AutoDiffXd> d_cache_ad_;

  const ConstrainedDynamicsInfo& dynamics_;

};

}
