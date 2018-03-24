#pragma once

#include <memory>

#include <gflags/gflags.h>
#include "drake/solvers/constraint.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/kinematics_cache.h"
#include "dircon_kinematic_data.h"

using Eigen::Vector3d;

namespace drake {
template <typename T>
class DirconPositionData : public DirconKinematicData<T> {
  public:
    DirconPositionData(const RigidBodyTree<double>& tree, int bodyIdx, Vector3d pt, bool isXZ = false);
    ~DirconPositionData();

    //The workhorse function, updates and caches everything needed by the outside world
    void updateConstraint(KinematicsCache<T>& cache);

  private:
    int bodyIdx_;
    Vector3d pt_;
    bool isXZ_;
    Eigen::Matrix<double,2,3> TXZ_;
};
}