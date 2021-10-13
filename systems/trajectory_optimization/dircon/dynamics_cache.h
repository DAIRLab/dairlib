#pragma once

#include <unordered_map>

#include "multibody/kinematic/kinematic_evaluator_set.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

template <typename T>
struct CacheKey {
  const drake::VectorX<T> state;
  const drake::VectorX<T> input;
  const drake::VectorX<T> forces;
};

// == operation for two CacheKeys
template <typename T>
class CacheComparer {
 public:
  bool operator()(const CacheKey<T>& a, const CacheKey<T>& b) const;
};

// Hashes a CacheKey by bit shifting and combining hashes of the double
// elements
template <typename T>
class CacheHasher {
 public:
  std::size_t operator()(const CacheKey<T>& key) const;
};

template <typename T>
class DynamicsCache {
 public:
  DynamicsCache(const multibody::KinematicEvaluatorSet<T>& constraints,
      int max_size);

  drake::VectorX<T> CalcTimeDerivativesWithForce(
      drake::systems::Context<T>* context,
      const drake::VectorX<T>& forces);

 private:
  const multibody::KinematicEvaluatorSet<T>& evaluators_;
  size_t max_size_;
  std::unordered_map<CacheKey<T>, drake::VectorX<T>, CacheHasher<T>,
      CacheComparer<T>> map_;
  std::list<CacheKey<T>> queue_;
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib
