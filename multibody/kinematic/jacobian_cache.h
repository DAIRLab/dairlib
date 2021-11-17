#pragma once

#include <list>
#include <unordered_map>

#include "drake/common/eigen_types.h"

namespace dairlib {
namespace multibody {

template <typename T>
struct JacCacheKey {
  const drake::VectorX<T> q;
};

// == operation for two JacCacheKey
template <typename T>
class JacCacheComparer {
 public:
  bool operator()(const JacCacheKey<T>& a, const JacCacheKey<T>& b) const;
};

// Hashes a JacCacheKey by bit shifting and combining hashes of the double
// elements
template <typename T>
class JacCacheHasher {
 public:
  std::size_t operator()(const JacCacheKey<T>& key) const;
};

template <typename T>
class JacobianCache {
 public:
  JacobianCache(int max_size);

  bool SetIfCached(const drake::VectorX<T>& q,
                   drake::EigenPtr<drake::MatrixX<T>> J);

  void Add(const drake::VectorX<T>& q, const drake::MatrixX<T>& J);

 private:
  size_t max_size_;
  std::unordered_map<JacCacheKey<T>, drake::MatrixX<T>, JacCacheHasher<T>,
                     JacCacheComparer<T>>
      map_;
  std::list<JacCacheKey<T>> queue_;
};

}  // namespace multibody
}  // namespace dairlib
