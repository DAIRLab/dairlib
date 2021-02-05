//
// Created by brian on 1/31/21.
//
#pragma once
#include "solvers/nonlinear_constraint.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace dairlib {
namespace centroidal_to {
namespace planar {

const int kLinearVars = 4;
const int kAngularVars = 2;
const int kStateVars = kLinearVars + kAngularVars;
const int kLinearDim = 2;
const int kAngularDim = 1;
const int kForceVars = 4;
const int kForceDim = 2;
const int kStanceVars = 2;


class PlanarRigidBodyDynamicsConstraint
    : public dairlib::solvers::NonlinearConstraint<drake::AutoDiffXd> {
 public:
  PlanarRigidBodyDynamicsConstraint(double I, double mass,
                                    double h, int n_c);

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<drake::AutoDiffXd>> &x,
                          drake::VectorX<drake::AutoDiffXd> *y) const;

 private:
  drake::VectorX<drake::AutoDiffXd> F(drake::VectorX<drake::AutoDiffXd> x,
                                      std::vector<drake::Vector2<drake::AutoDiffXd>> forces) const;

  double I_;
  double mass_;
  double h_;
  int n_c_;
};

}
}
}