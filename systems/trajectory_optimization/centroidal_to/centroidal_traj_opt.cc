//
// Created by brian on 1/26/21.
//

#include "centroidal_traj_opt.h"
#include "rigid_body_dynamics_constraint.h"

namespace dairlib {
namespace centroidal_to {

CentroidalTrajOpt::CentroidalTrajOpt(Eigen::Matrix3d inertia_tensor,
                                     double mass,
                                     double h,
                                     double T_ss,
                                     double T_ds,
                                     double mu) :
    inertia_tensor_(inertia_tensor),
    mass_(mass),
    h_(h),
    T_ss_(T_ss),
    T_ds_(T_ds),
    mu_(mu) {}

void CentroidalTrajOpt::SetModeSequence(std::vector<stance> sequence,
                                        std::vector<double> times) {
  DRAKE_ASSERT(sequence.size() == times.size())
  int n_modes = sequence.size();
  for (int i = 0; i < n_modes; i++) {
    int n_knot_f = std::round(times[i] / h_) + 1;
    int n_knot_s = (i == 0) ? n_knot_f : n_knot_f - 1;
    int n_c = (sequence[i] == stance::kDouble) ? 2 : 1;


    force_vars_.push_back(NewContinuousVariables(
        n_knot_f * 3 * kNForceVars * n_c,
        "forces[" + std::to_string(i) + "]"));

    stance_vars_.push_back(NewContinuousVariables(n_c * 3,
            "stance_pos[" + std::to_string(i) + "]"));

    state_vars_.push_back(NewContinuousVariables(
              kNLinearVars + kNForceVars * n_knot_s,
              "x[" + std::to_string(i) + "]"));

    if (i != 0 && sequence[i-1] != stance::kDouble) {
      has_impact_.push_back(true);
      impulse_vars_.push_back(NewContinuousVariables(
                kNForceVars, "impulse[" + std::to_string(i) + "]"));
      post_impact_vars_.push_back(NewContinuousVariables(
              6, "delta_v[" + std::to_string(i) + "]"));
    } else { has_impact_.push_back(false); }

    for (int j = 0; j < n_knot_s - 1 ; j++ ) {
      auto rbd_constraint = std::make_shared<RigidBodyDynamicsConstraint>(
              inertia_tensor_, mass_, h_, n_c);
      AddConstraint(rbd_constraint, {
          state_vars_[i].segment((kNLinearVars + kNAngularVars) * j, kNLinearVars + kNAngularVars),
          force_vars_[i].segment(3*kNForceVars*n_c*j, 3*kNForceVars*n_c),
          stance_vars_[i]});
    }


  }
}

void CentroidalTrajOpt::SetNominalStance(Eigen::Vector3d left,
                                         Eigen::Vector3d right) {
  nominal_stance_.clear();
  nominal_stance_.push_back(left);
  nominal_stance_.push_back(right);
}

}
}