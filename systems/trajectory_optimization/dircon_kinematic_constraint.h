#include <memory>

#include <gflags/gflags.h>
#include "drake/solvers/constraint.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/kinematics_cache.h"
namespace drake {
template <typename T>
class DirconKinematicConstraint {
  public:
    DirconKinematicConstraint(RigidBodyTree<double>* tree, int length);
    ~DirconKinematicConstraint();

    //The workhorse function, updates and caches everything needed by the outside world
    virtual void updateConstraint(KinematicsCache<T>& cache) = 0;

    VectorX<T> getC();
    VectorX<T> getCDot();
    MatrixX<T> getJ();
    VectorX<T> getJdotv();

  protected:
    RigidBodyTree<double>* tree_;
    //std::vector<solvers::Constraint> force_constraints_; //things like friction cone constraints
    VectorX<T> c_;
    VectorX<T> cdot_;
    MatrixX<T> J_;
    VectorX<T> Jdotv_;
};
}