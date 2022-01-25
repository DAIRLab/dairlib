#include "systems/trajectory_optimization/dircon/dynamics_cache.h"

#include "drake/math/autodiff_gradient.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

using drake::AutoDiffXd;
using drake::AutoDiffVecXd;

inline void hash_combine(std::size_t& seed, const double& v) {
    seed ^= std::hash<double>{}(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
}

inline void hash_combine(std::size_t& seed,
    const Eigen::Ref<const Eigen::VectorXd>& v) {
  for (int i = 0; i < v.size(); i++) {
    hash_combine(seed, v(i)); 
  }
}

inline void hash_combine(std::size_t& seed,
    const Eigen::Ref<const AutoDiffVecXd>& v) {
  for (int i = 0; i < v.size(); i++) {
    hash_combine(seed, v(i).value());
    hash_combine(seed, v(i).derivatives());
  }
}

template <typename T>
DynamicsCache<T>::DynamicsCache(
    const multibody::KinematicEvaluatorSet<T>& evaluators, int max_size)
  : evaluators_(evaluators),
    max_size_(max_size) {}

template <typename T>
drake::VectorX<T> DynamicsCache<T>::CalcTimeDerivativesWithForce(
    drake::systems::Context<T>* context,
    const drake::VectorX<T>& forces) {
  CacheKey<T> key{evaluators_.plant().GetPositionsAndVelocities(*context), 
                  evaluators_.plant().get_actuation_input_port().Eval(*context),
                  forces};
  auto it = map_.find(key);
  if (it == map_.end()) {
    auto xdot = evaluators_.CalcTimeDerivativesWithForce(context, forces);
    map_[key] = xdot;

    // Add to the queue_
    if (map_.size() >= max_size_) {
      map_.erase(queue_.front());
      queue_.pop_front();
    }
    queue_.push_back(key);
    return xdot;
  } else {
    return it->second;
  }
}

bool AreVectorsEqual(const Eigen::Ref<const AutoDiffVecXd>& a,
                     const Eigen::Ref<const AutoDiffVecXd>& b) {
  if (a.value() != b.value()) {
    return false;
  }
  const Eigen::MatrixXd& a_gradient = drake::math::ExtractGradient(a);
  const Eigen::MatrixXd& b_gradient = drake::math::ExtractGradient(b);

  return a_gradient == b_gradient;
}

template <typename T>
std::size_t CacheHasher<T>::operator()(const CacheKey<T>& key) const {
  std::size_t seed = 0;
  hash_combine(seed, key.state);
  hash_combine(seed, key.input);
  hash_combine(seed, key.forces);
  return seed;
}

template <>
bool CacheComparer<double>::operator()(const CacheKey<double>& a,
    const CacheKey<double>& b) const {
  return (a.state == b.state) && (a.forces == b.forces) &&
      (a.input == b.input);
}

template <>
bool CacheComparer<AutoDiffXd>::operator()(const CacheKey<AutoDiffXd>& a,
    const CacheKey<AutoDiffXd>& b) const {
  return AreVectorsEqual(a.state, b.state) &&
         AreVectorsEqual(a.input, b.input) &&
        AreVectorsEqual(a.forces, b.forces);
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::systems::trajectory_optimization::DynamicsCache)
