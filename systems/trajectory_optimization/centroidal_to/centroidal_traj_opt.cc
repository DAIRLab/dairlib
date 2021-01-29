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

      state_vars_.push_back(NewContinuousVariables(
          n_knot_f * 3 * kNForceVars,
          "forces[" + std::to_string(i) + "]"));
      


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