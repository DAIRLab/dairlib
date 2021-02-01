//
// Created by brian on 1/31/21.
//

#include "planar_rigid_body_dynamics_constraint.h"


using dairlib::solvers::NonlinearConstraint;

using drake::AutoDiffXd;

using Eigen::Matrix3d;
using Eigen::VectorXd;
using VectorX = drake::VectorX<AutoDiffXd>;
using Quat = drake::Quaternion<AutoDiffXd>;
using Vector2 = drake::Vector2<AutoDiffXd>;
using Vector1 = drake::Vector1<AutoDiffXd>;

namespace dairlib {
namespace centroidal_to {
namespace planar {
PlanarRigidBodyDynamicsConstraint::PlanarRigidBodyDynamicsConstraint(
    const double I,
    const double mass,
    const double h,
    const int n_c
) : NonlinearConstraint<AutoDiffXd>(kLinearVars + kAngularVars,
                                    2*(kLinearVars + kAngularVars) + n_c * (kForceVars + kStanceVars),
                                    VectorXd::Zero(
                                        kLinearVars + kAngularVars),
                                    VectorXd(kLinearVars + kAngularVars)),
    I_(I),
    mass_(mass),
    n_c_(n_c),
    h_(h) {}

/// Variable Ordering: X0, X1, {F0 Fc F1}_1, ... {F0 Fc F1}_nc, P_1, ... P_nc

void PlanarRigidBodyDynamicsConstraint::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<drake::AutoDiffXd>> &x,
    drake::VectorX<drake::AutoDiffXd> *y) const {

  int n_x = kLinearVars + kAngularVars;
  VectorX x0 = x.head(n_x);
  VectorX x1 = x.segment(n_x, n_x);

  std::vector<Vector2> f0;
  std::vector<Vector2> fc;
  std::vector<Vector2> f1;
  std::vector<Vector2> p;

  for (int i = 0; i < n_c_; i++) {
    f0.push_back(x.segment(2 * n_x + kForceVars * i, kForceDim));
    fc.push_back(x.segment(2 * n_x + kForceVars * i + kForceDim, kForceDim));
    f1.push_back(x.segment(2 * n_x + kForceVars * i + 2*kForceDim, kForceDim));
    p.push_back(x.segment(2 * n_x + kForceVars * n_c_ + kStanceVars * i, kStanceVars));
  }

  // compact form of collocation constraint
  VectorX F0 = F(x0, f0, p);
  VectorX F1 = F(x1, f1, p);
  VectorX xc = 0.5 * (x0 + x1) - (h_ / 8) * (F1 - F0);
  VectorX Fc = (3 / (2 * h_)) * (x1 - x0) - (1 / 4) * (F0 + F1);
  *y = Fc - F(xc, fc, p);
}

VectorX PlanarRigidBodyDynamicsConstraint::F(VectorX x, std::vector<Vector2> forces,
                                       std::vector<Vector2> p) const {
  Eigen::Vector2d g;
  g << 0, -9.81;

  Vector2 force_sum = Vector2::Zero();
  Vector1 pxf_sum = Vector1::Zero();
  for (int i = 0; i < n_c_; i++) {
    force_sum += forces[i];
    pxf_sum += p[i].head(1)*forces[i].tail(1) - p[i].tail(1) * forces[i].head(1);
  }

  VectorX f = VectorX::Zero(kLinearVars + kAngularVars);
  f.head(kLinearDim + kAngularDim) = x.segment(kLinearDim + kAngularDim,
                                               kLinearDim + kAngularDim);

  f.segment(kLinearDim + kAngularDim, kLinearDim) = (1/mass_) * force_sum + g;
  f.tail(kAngularDim) = (1/I_)*pxf_sum;

  return f;
}

}
}
}