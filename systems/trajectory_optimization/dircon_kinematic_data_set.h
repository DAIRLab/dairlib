#pragma once

#include <memory>

#include "dircon_kinematic_data.h"
#include <gflags/gflags.h>
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/kinematics_cache.h"

namespace drake{
template <typename T>
class DirconKinematicDataSet {
  public:
    DirconKinematicDataSet(const RigidBodyTree<double>& tree, std::vector<DirconKinematicData<T>*>* constraints);

    void updateData(const VectorX<T>& state, const VectorX<T>& input, const VectorX<T>& forces);

    VectorX<T> getC();
    VectorX<T> getCDot();
    MatrixX<T> getJ();
    VectorX<T> getJdotv();
    VectorX<T> getCDDot();
    VectorX<T> getVDot();
    VectorX<T> getXDot();

    DirconKinematicData<T>* getConstraint(int index);

    int getNumConstraintObjects();
    int countConstraints();

  private:
    DirconKinematicDataSet(const RigidBodyTree<double>& tree, std::vector<DirconKinematicData<T>*>* constraints, int num_positions, int num_velocities);

    const RigidBodyTree<double>* tree_;
    int num_positions_;
    int num_velocities_;
    int constraint_count_;
    std::vector<DirconKinematicData<T>*>* constraints_;
    VectorX<T> c_;
    VectorX<T> cdot_;
    MatrixX<T> J_;
    VectorX<T> Jdotv_;
    VectorX<T> cddot_;
    VectorX<T> vdot_;
    VectorX<T> xdot_;
};
}