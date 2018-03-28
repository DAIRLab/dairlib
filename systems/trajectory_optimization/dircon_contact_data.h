#pragma once

#include <memory>

#include <gflags/gflags.h>
#include "drake/solvers/constraint.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/kinematics_cache.h"
#include "dircon_kinematic_data.h"

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;


namespace drake {
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

    Matrix3Xd xA_, xB_, normal_;
    std::vector<int> idxA_;
    std::vector<int> idxB_;
    VectorXd phi_;
};
}