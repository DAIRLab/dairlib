#pragma once
#include "solvers/nonlinear_constraint.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"


namespace dairlib {
namespace centroidal_to {

const int kNLinearVars = 6;
const int kNAngularVars = 7;
const int kNForceVars = 9;


class RigidBodyDynamicsConstraint : public dairlib::solvers::NonlinearConstraint<drake::AutoDiffXd> {
 public:
  RigidBodyDynamicsConstraint(const Eigen::Matrix3d& Inertia, const double mass,
                              const double h, const int n_c);
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<drake::AutoDiffXd>>& x,
                          drake::VectorX<drake::AutoDiffXd>* y) const;

 private:
  drake::VectorX<drake::AutoDiffXd> F(drake::VectorX<drake::AutoDiffXd> x,
              std::vector<drake::Vector3<drake::AutoDiffXd>> forces,
              std::vector<drake::Vector3<drake::AutoDiffXd>> p) const;

  Eigen::Matrix3d Inertia_;
  double mass_;
  double h_;
  int n_c_;
};

}
}

