#pragma once

#include <memory>

#include "drake/solvers/constraint.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/kinematics_cache.h"
#include "attic/systems/trajectory_optimization/dircon_kinematic_data.h"

namespace dairlib {

template <typename T>
class DirconPositionData : public DirconKinematicData<T> {
 public:
  DirconPositionData(const RigidBodyTree<double>& tree, int bodyIdx,
                     Eigen::Vector3d pt, bool isXZ = false);
  ~DirconPositionData();

  // The workhorse function, updates and caches everything needed by the
  // outside world
  void updateConstraint(const KinematicsCache<T>& cache);

  void addFixedNormalFrictionConstraints(Eigen::Vector3d normal, double mu);

 private:
    int bodyIdx_;
    Eigen::Vector3d pt_;
    bool isXZ_;
    Eigen::Matrix<double, 2, 3> TXZ_;
};
}  // namespace dairlib
