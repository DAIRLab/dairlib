#pragma once

#include <memory>
#include <vector>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/kinematics_cache.h"
#include "attic/systems/trajectory_optimization/dircon_kinematic_data.h"

namespace dairlib {

template <typename T>
class DirconKinematicDataSet {
 public:
  DirconKinematicDataSet(const RigidBodyTree<double>& tree,
                         std::vector<DirconKinematicData<T>*>* constraints);

  void updateData(const drake::VectorX<T>& state,
                  const drake::VectorX<T>& input,
                  const drake::VectorX<T>& forces);

  drake::VectorX<T> getC();
  drake::VectorX<T> getCDot();
  drake::MatrixX<T> getJ();
  drake::VectorX<T> getJdotv();
  drake::VectorX<T> getCDDot();
  drake::VectorX<T> getVDot();
  drake::VectorX<T> getXDot();

  DirconKinematicData<T>* getConstraint(int index);

  KinematicsCache<T>* getCache() { return &cache_; }

  int getNumConstraintObjects();
  int countConstraints();

 private:
  DirconKinematicDataSet(const RigidBodyTree<double>& tree,
                         std::vector<DirconKinematicData<T>*>* constraints,
                         int num_positions, int num_velocities);

    const RigidBodyTree<double>* tree_;
    int num_positions_;
    int num_velocities_;
    int constraint_count_;
    std::vector<DirconKinematicData<T>*>* constraints_;
    drake::VectorX<T> c_;
    drake::VectorX<T> cdot_;
    drake::MatrixX<T> J_;
    drake::VectorX<T> Jdotv_;
    drake::VectorX<T> cddot_;
    drake::VectorX<T> vdot_;
    drake::VectorX<T> xdot_;
    KinematicsCache<T> cache_;
};
}  // namespace dairlib
