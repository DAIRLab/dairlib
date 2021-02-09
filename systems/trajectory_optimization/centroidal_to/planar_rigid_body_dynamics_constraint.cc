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
) : NonlinearConstraint<AutoDiffXd>(kStateVars,
                                    2*(kLinearVars + kAngularVars) + n_c * (2 * kForceVars + kStanceVars),
                                    VectorXd::Zero(kStateVars),
                                    VectorXd::Zero(kStateVars )),
    I_(I),
    mass_(mass),
    n_c_(n_c),
    h_(h) {}

/// Variable Ordering: X0, X1 ,P_1, ... P_nc {F0 F1}_1, ... {F0 F1}_nc,

void PlanarRigidBodyDynamicsConstraint::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<drake::AutoDiffXd>> &x,
    drake::VectorX<drake::AutoDiffXd> *y) const {

  int n_x = kStateVars;
  VectorX x0 = x.head(n_x);
  VectorX x1 = x.segment(n_x, n_x);

  std::vector<Vector2> f0;
  std::vector<Vector2> fc;
  std::vector<Vector2> f1;
  std::vector<Vector2> p;

  for (int i = 0; i < n_c_; i++) {
    p.push_back(x.segment(2 * n_x + kStanceVars * i, kStanceVars));
    f0.push_back(x.segment(2 * n_x + kStanceVars * n_c_ + kForceDim * i, kForceDim));
    f1.push_back(x.segment(2 * n_x + kStanceVars * n_c_ +  kForceDim * n_c_ + kForceDim * i, kForceDim));
    fc.push_back(0.5 * f0.back() + 0.5 * f1.back());
  }

  // compact form of collocation constraint
  VectorX F0 = F(x0, f0, p);
  VectorX F1 = F(x1, f1, p);
  VectorX xc = 0.5 * (x0 + x1) - (h_ / 8.0) * (F1 - F0);
  VectorX Fc = (3 / (2 * h_)) * (x1 - x0) - (1 / 4) * (F0 + F1);

  *y = F(xc, fc, p) - Fc;
}

VectorX PlanarRigidBodyDynamicsConstraint::F(VectorX x, std::vector<Vector2> forces, std::vector<Vector2> P) const {
  Eigen::Vector2d g;
  g << 0, -9.81;

  Vector2 force_sum = Vector2::Zero();
  Vector1 pxf_sum = Vector1::Zero();

  for (int i = 0; i < n_c_; i++) {
    Vector2 p = P[i] - x.head(kLinearDim);
    force_sum += forces[i];
    pxf_sum += p.head(1)*forces[i].tail(1) - p.tail(1) * forces[i].head(1);
  }

  VectorX f = VectorX::Zero(kStateVars);

  f.head(kLinearDim + kAngularDim) = x.segment(kLinearDim + kAngularDim,
                                               kLinearDim + kAngularDim);

  f.segment(kLinearDim + kAngularDim, kLinearDim) = (1/mass_) * force_sum + g;
  f.segment(kStateVars - kAngularDim, kAngularDim) = (1/I_)*pxf_sum;


  return f;
}

}
}
}