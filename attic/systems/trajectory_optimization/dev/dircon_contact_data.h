#pragma once

#include <memory>

#include <gflags/gflags.h>
#include "drake/solvers/constraint.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/kinematics_cache.h"
#include "dircon_kinematic_data.h"

namespace dairlib {
template <typename T>
class DirconContactData : public DirconKinematicData<T> {
  public:
    DirconContactData(RigidBodyTree<double>& tree, std::vector<int>& contact_indices,
                      double mu, bool isXZ = false);
    ~DirconContactData();

    //The workhorse function, updates and caches everything needed by the outside world
    void updateConstraint(KinematicsCache<T>& cache);

  private:
    double mu_;
    std::vector<int> contact_indices_;
    bool isXZ_;
    Eigen::Matrix<double,2,3> TXZ_;


    Eigen::Matrix3Xd xA_, xB_, normal_, d_data_;
    std::vector<Eigen::Map<Matrix3Xd>> d_world_;
    std::vector<int> idxA_;
    std::vector<int> idxB_;
    Eigen::VectorXd phi_;
};
}