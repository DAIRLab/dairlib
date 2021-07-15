#include "multibody/kinematic/jacobian_cache.h"

#include "drake/common/default_scalars.h"
#include "drake/math/autodiff_gradient.h"

namespace dairlib {
namespace multibody {

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::VectorX;

inline void hash_combine(std::size_t& seed, const double& v) {
  seed ^= std::hash<double>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
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
JacobianCache<T>::JacobianCache(int max_size)
    : max_size_(max_size) {}

template <typename T>
bool JacobianCache<T>::SetIfCached(const drake::VectorX<T>& q,
                                   drake::EigenPtr<MatrixX<T>> J) {
  JacCacheKey<T> key{q};
  auto it = map_.find(key);
  if (it != map_.end()) {
    *J = it->second;
    return true;
  }
  return false;
}

template <typename T>
void JacobianCache<T>::Add(const VectorX<T>& q, const MatrixX<T>& J) {
  JacCacheKey<T> key{q};
  map_[key] = J;

  // Add to the queue_
  if (map_.size() >= max_size_) {
    map_.erase(queue_.front());
    queue_.pop_front();
  }
  queue_.push_back(key);
}

bool AreADVectorsEqual(const Eigen::Ref<const AutoDiffVecXd>& a,
                       const Eigen::Ref<const AutoDiffVecXd>& b) {
  if (a.value() != b.value()) {
    return false;
  }
  const Eigen::MatrixXd& a_gradient = drake::math::autoDiffToGradientMatrix(a);
  const Eigen::MatrixXd& b_gradient = drake::math::autoDiffToGradientMatrix(b);

  return a_gradient == b_gradient;
}

template <typename T>
std::size_t JacCacheHasher<T>::operator()(const JacCacheKey<T>& key) const {
  std::size_t seed = 0;
  hash_combine(seed, key.q);
  return seed;
}

template <>
bool JacCacheComparer<double>::operator()(const JacCacheKey<double>& a,
                                          const JacCacheKey<double>& b) const {
  return (a.q == b.q);
}

template <>
bool JacCacheComparer<AutoDiffXd>::operator()(
    const JacCacheKey<AutoDiffXd>& a, const JacCacheKey<AutoDiffXd>& b) const {
  return AreADVectorsEqual(a.q, b.q);
}

}  // namespace multibody
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::JacobianCache)
