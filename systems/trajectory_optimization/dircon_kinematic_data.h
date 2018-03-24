#pragma once

#include <memory>

#include <gflags/gflags.h>
#include "drake/solvers/constraint.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/kinematics_cache.h"
namespace drake {

template <typename T>
class DirconKinematicData {
  public:
    DirconKinematicData(const RigidBodyTree<double>& tree, int length);
    ~DirconKinematicData();

    //The workhorse function, updates and caches everything needed by the outside world
    virtual void updateConstraint(KinematicsCache<T>& cache) = 0;

    VectorX<T> getC();
    VectorX<T> getCDot();
    MatrixX<T> getJ();
    VectorX<T> getJdotv();
    int getLength();

  protected:
    const RigidBodyTree<double>* tree_;
    //things like friction cone constraints
    std::vector<std::shared_ptr<solvers::Constraint>> force_constraints_;
    VectorX<T> c_;
    VectorX<T> cdot_;
    MatrixX<T> J_;
    VectorX<T> Jdotv_;
    int length_;
};

template <typename T>
class DirconKinematicDataSet {
  public:
    DirconKinematicDataSet(RigidBodyTree<double>& tree, std::vector<DirconKinematicData<T>> constraints, int num_positions, int num_velocities);
    ~DirconKinematicDataSet();

    void updateData(const VectorX<T>& state, const VectorX<T>& input, const VectorX<T>& forces);

    VectorX<T> getC();
    VectorX<T> getCDot();
    MatrixX<T> getJ();
    VectorX<T> getJdotv();
    VectorX<T> getCDDot();
    VectorX<T> getVDot();
    VectorX<T> getXDot();

    DirconKinematicData<T> getConstraint(int index);

    int getNumConstraints();

  private:
    void updateVdot(const VectorX<T>& state, const VectorX<T>& input, const VectorX<T>& forces);

    const RigidBodyTree<double>* tree_;
    int num_positions_;
    int num_velocities_;
    int num_constraints_;
    std::vector<DirconKinematicData<T>> constraints_;
    VectorX<T> c_;
    VectorX<T> cdot_;
    MatrixX<T> J_;
    VectorX<T> Jdotv_;
    VectorX<T> cddot_;
    VectorX<T> vdot_;
    VectorX<T> xdot_;
};
}