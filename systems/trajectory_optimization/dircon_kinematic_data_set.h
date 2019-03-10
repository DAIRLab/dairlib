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
  struct CacheEntry {
    const Eigen::VectorXd state;
    const Eigen::VectorXd input;
    const Eigen::VectorXd forces;
  };

  struct CacheData {
    drake::VectorX<T> c_;
    drake::VectorX<T> cdot_;
    drake::MatrixX<T> J_;
    drake::VectorX<T> Jdotv_;
    drake::VectorX<T> cddot_;
    drake::VectorX<T> vdot_;
    drake::VectorX<T> xdot_;
  };

  class CacheHasher {
   public:
    std::size_t operator()(const CacheEntry& entry) const {
      std::size_t hash = 0;
      for (int i = 0; i < entry.state.rows(); i++) {
        hash = (hash << 1) ^ std::hash<double>{}(entry.state(i));
      }
      for (int i = 0; i < entry.input.rows(); i++) {
        hash = (hash << 1) ^ std::hash<double>{}(entry.input(i));
      }
      for (int i = 0; i < entry.forces.rows(); i++) {
        hash = (hash << 1) ^ std::hash<double>{}(entry.forces(i));
      }
      return hash;
    }
  };

  class CacheComparer {
   public:
    bool operator()(const CacheEntry& a, const CacheEntry& b) const {
      auto ret = (a.state.isApprox(b.state)) && (a.forces.isApprox(b.forces)) &&
          (a.input.isApprox(b.input));
      return ret;
    }
  };

  class Cache {
   public:
    explicit Cache(unsigned int max_size) : max_size_(max_size) {}

    bool Contains(const CacheEntry& entry) const {
      return map_.find(entry) != map_.end();
    }

    const CacheData& GetData(const CacheEntry& entry) {
      return map_[entry];
    }

    void AddData(const CacheEntry& entry, const CacheData& data) {
      if (map_.size() >= max_size_) {
        map_.erase(queue_.front());
        queue_.pop_front();
      }
      map_[entry] = data;
      queue_.push_back(entry);
    }

   private:
    unsigned int max_size_;
    std::unordered_map<CacheEntry, CacheData, CacheHasher, CacheComparer> map_;
    std::list<CacheEntry> queue_;
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
