#pragma once

#include <memory>
#include <vector>
#include <list>

#include "drake/multibody/plant/multibody_plant.h"
#include "systems/trajectory_optimization/dircon_kinematic_data.h"

namespace dairlib {

template <typename T>
class DirconKinematicDataSet {
 public:
  DirconKinematicDataSet(const drake::multibody::MultibodyPlant<T>& plant,
                         std::vector<DirconKinematicData<T>*>* constraints);

  void updateData(const drake::systems::Context<T>& context,
                  const drake::VectorX<T>& forces);

  drake::VectorX<T> getC();
  drake::VectorX<T> getCDot();
  drake::MatrixX<T> getJ();
  drake::VectorX<T> getJdotv();
  drake::VectorX<T> getCDDot();
  drake::VectorX<T> getVDot();
  drake::VectorX<T> getXDot();

  DirconKinematicData<T>* getConstraint(int index);

  int getNumConstraintObjects();
  int countConstraints();

 private:
  // Key for the data cache--note that these are VectorXd, and not VectorX<T>
  struct CacheKey {
    const Eigen::VectorXd state;
    const Eigen::VectorXd input;
    const Eigen::VectorXd forces;
  };

  // Copy of a data entry for the cache
  struct CacheData {
    drake::VectorX<T> c_;
    drake::VectorX<T> cdot_;
    drake::MatrixX<T> J_;
    drake::VectorX<T> Jdotv_;
    drake::VectorX<T> cddot_;
    drake::VectorX<T> vdot_;
    drake::VectorX<T> xdot_;
  };

  // Hashes a CacheKey by bit shifting and combining hashes of the double
  // elements
  class CacheHasher {
   public:
    std::size_t operator()(const CacheKey& key) const {
      std::size_t hash = 0;
      for (int i = 0; i < key.state.rows(); i++) {
        hash = (hash << 1) ^ std::hash<double>{}(key.state(i));
      }
      for (int i = 0; i < key.input.rows(); i++) {
        hash = (hash << 1) ^ std::hash<double>{}(key.input(i));
      }
      for (int i = 0; i < key.forces.rows(); i++) {
        hash = (hash << 1) ^ std::hash<double>{}(key.forces(i));
      }
      return hash;
    }
  };

  // == operation for two CacheKeys
  class CacheComparer {
   public:
    bool operator()(const CacheKey& a, const CacheKey& b) const {
      auto ret = (a.state.isApprox(b.state)) && (a.forces.isApprox(b.forces)) &&
          (a.input.isApprox(b.input));
      return ret;
    }
  };

  // Simple wrapper of an unordered_map that manages a maximum size
  // At max size, the oldest objects are removed from the map
  class Cache {
   public:
    explicit Cache(unsigned int max_size) : max_size_(max_size) {}

    // Check if the cache contains the given key
    bool Contains(const CacheKey& key) const {
      return map_.find(key) != map_.end();
    }

    // Get the data associated with a key element
    const CacheData& GetData(const CacheKey& key) {
      return map_[key];
    }

    // Adds an entry to storage. If at max size, removes the oldest element
    // For speed, this assumes that the map does not contain the key already!!
    void AddData(const CacheKey& key, const CacheData& data) {
      if (map_.size() >= max_size_) {
        map_.erase(queue_.front());
        queue_.pop_front();
      }
      map_[key] = data;
      queue_.push_back(key);
    }

   private:
    unsigned int max_size_;
    std::unordered_map<CacheKey, CacheData, CacheHasher, CacheComparer> map_;
    std::list<CacheKey> queue_;
  };

  const drake::multibody::MultibodyPlant<T>& plant_;
  std::vector<DirconKinematicData<T>*>* constraints_;
  int num_positions_;
  int num_velocities_;
  int constraint_count_;
  drake::VectorX<T> c_;
  drake::VectorX<T> cdot_;
  drake::MatrixX<T> J_;
  drake::VectorX<T> Jdotv_;


  drake::VectorX<T> cddot_;
  drake::VectorX<T> vdot_;
  drake::VectorX<T> xdot_;
  drake::MatrixX<T> M_;
  drake::VectorX<T> right_hand_side_;

  Cache cache_;
};
}  // namespace dairlib
