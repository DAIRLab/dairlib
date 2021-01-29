//
// Created by brian on 1/26/21.
//

#include "rigid_body_dynamics_constraint.h"
#include "drake/math/autodiff.h"


using dairlib::solvers::NonlinearConstraint;

using drake::AutoDiffXd;

using Eigen::Matrix3d;
using Eigen::VectorXd;
using VectorX = drake::VectorX<AutoDiffXd>;
using Quat = drake::Quaternion<AutoDiffXd>;
using Vector3 = drake::Vector3<AutoDiffXd>;

namespace dairlib {
namespace centroidal_to {
  RigidBodyDynamicsConstraint::RigidBodyDynamicsConstraint(
      const Matrix3d& Inertia,
      const double mass,
      const double h,
      const int n_c
      ) : NonlinearConstraint<AutoDiffXd> (kNLinearVars + kNAngularVars,
          2*(kNLinearVars + kNAngularVars) + kNForceVars * n_c,
          VectorXd::Zero(kNLinearVars + kNAngularVars),
          VectorXd(kNLinearVars + kNAngularVars)),
          Inertia_(Inertia),
          mass_(mass),
          n_c_(n_c),
          h_(h){}

  /// using variable ordering {q0, v0}, {q1, v1}, {f0_1, fc_1, f1_1} ... {f0_nc....}
  /// {p_1}, {P_2}
  /// Total decision variables =  2*(7 + 6) + n_c_ * (9 + 3) = 26 + 12 * n_c_

  void RigidBodyDynamicsConstraint::EvaluateConstraint(const Eigen::Ref<const drake::VectorX<drake::AutoDiffXd> > &x,
      drake::VectorX<drake::AutoDiffXd> *y) const {
    // TODO: Implement dynamics constraint - Remember to include x0 and x1 in the decision variables
    int n_x = kNLinearVars + kNLinearVars;
    VectorX x0 = x.head(n_x);
    VectorX x1 = x.segment(n_x,n_x);
    std::vector<Vector3> f0;
    std::vector<Vector3> fc;
    std::vector<Vector3> f1;
    std::vector<Vector3> p;

    for (int i = 0; i < n_c_; i++) {
      f0.push_back(x.segment(2*n_x + n_c_ * kNForceVars * 3*i, kNForceVars));
      fc.push_back(x.segment(2*n_x + kNForceVars + n_c_ * kNForceVars * 3*i, kNForceVars));
      f1.push_back(x.segment(2*n_x + 2*kNForceVars + n_c_ * kNForceVars * 3*i, kNForceVars));
      p.push_back(x.segment(2*n_x + 3*kNForceVars*n_c_ + 3*i, 3));
    }

    VectorX F0 = F(x0, f0, p);
    VectorX F1 = F(x1, f1, p);
    VectorX xc = 0.5*(x0 + x1) - (h_/8)*(F1 -  F0);
    VectorX Fc = (3/(2*h_))*(x1 -x0) - (1/4)*(F0+F1);
    *y = Fc - F(xc, fc, p);
  }

  VectorX RigidBodyDynamicsConstraint::F(VectorX x, std::vector<Vector3> forces,
                                         std::vector<Vector3> p) const {
    Eigen::Vector3d g;
    g << 0, 0, -9.81;

    Vector3 force;
    Vector3 pxf;
    for(int i = 0; i < n_c_; i++) {
      force += forces[i];
      pxf += p[i].cross(forces[i]);
    }
    VectorX f = VectorX::Zero(kNLinearVars + kNLinearVars);
    f.head(3) = x.segment(7, 3);
    Quat q = Quat(x(3), x(4), x(5), x(6));
    Quat omega_q = Quat(0, x(10), x(11), x(12));
    Vector3 omega = omega_q.vec();
    Quat qdot = omega_q*q;
    VectorX qd;
    qd << qdot.w(), qdot.vec();
    f.segment(3, 4) = qd;
    f.segment(7, 3) = (1/mass_) * force + g;
    f.segment(10, 3) = Inertia_.inverse() * (pxf - omega.cross(Inertia_ * omega));
    return f;
  }

}
}
