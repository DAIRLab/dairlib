#pragma once

#include <memory>
#include <vector>

#include "drake/solvers/constraint.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/kinematics_cache.h"

namespace dairlib {

template <typename T>
class DirconKinematicData {
 public:
    DirconKinematicData(const RigidBodyTree<double>& tree, int length);
    ~DirconKinematicData();

    // The workhorse function, updates and caches everything needed by the
    // outside world
    virtual void updateConstraint(const KinematicsCache<T>& cache) = 0;

    drake::VectorX<T> getC();
    drake::VectorX<T> getCDot();
    drake::MatrixX<T> getJ();
    drake::VectorX<T> getJdotv();
    int getLength();
    int numForceConstraints();
    std::shared_ptr<drake::solvers::Constraint> getForceConstraint(int index);

 protected:
    const RigidBodyTree<double>* tree_;
    std::vector<std::shared_ptr<drake::solvers::Constraint>> force_constraints_;
    drake::VectorX<T> c_;
    drake::VectorX<T> cdot_;
    drake::MatrixX<T> J_;
    drake::VectorX<T> Jdotv_;
    int length_;
};

}  // namespace dairlib
