//
// Created by brian on 1/26/21.
//

#include "centroidal_traj_opt.h"
#include "rigid_body_dynamics_constraint.h"
#include "solvers/constraint_factory.h"

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
  DRAKE_ASSERT(sequence.size() == times.size());

  sequence_ = sequence;
  times_ = times;
  Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(6,6);
  Aeq.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3,3);
  Aeq.block(0, 3, 3, 3) = -1*Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd beq = Eigen::VectorXd::Zero(6);

  int n_modes = sequence.size();
  for (int i = 0; i < n_modes; i++) {
    int n_knot_f = std::round(times[i] / h_);
    int n_knot_s = n_knot_f + 1;
    int n_c = (sequence[i] == stance::kDouble) ? 2 : 1;

    CentroidalMode mode;
    mode.n_c_ = n_c;

    // create 3 forces per foot per node
    for (int j = 0; j < n_knot_f; j++) {
      mode.force_vars_.push_back(NewContinuousVariables(
           kNForceVars * n_c,
          "forces[" + std::to_string(i) + "," + std::to_string(j) + "]"));
    }


    for (int j = 0; j < n_c; j++) {
      mode.stance_vars_.push_back(NewContinuousVariables(3,
          "stance_pos[" + std::to_string(i) + "," + std::to_string(j) + "]"));
    }

    for (int j = 0; j < n_knot_s; j ++) {
      mode.state_vars_.push_back(NewContinuousVariables(
          kNLinearVars + kNAngularVars,
          "x[" + std::to_string(i) + "]"));

    }


    for (int j = 0; j < n_knot_s - 1 ; j++ ) {
      auto rbd_constraint = std::make_shared<RigidBodyDynamicsConstraint>(
              inertia_tensor_, mass_, h_, n_c);
      switch(n_c) {
        case 0:
          AddConstraint(rbd_constraint, {
              mode.state_vars_[j],
              mode.state_vars_[j+1],
              mode.force_vars_[j]});
          break;
        case 1:
          AddConstraint(rbd_constraint, {
              mode.state_vars_[j],
              mode.state_vars_[j+1],
              mode.force_vars_[j],
              mode.stance_vars_[0] });
        case 2:
          AddConstraint(rbd_constraint, {
              mode.state_vars_[j],
              mode.state_vars_[j+1],
              mode.force_vars_[i],
              mode.stance_vars_[0],
              mode.stance_vars_[1]});
      }
      for (int k = 0; k < n_c; k++) {
        AddLinearEqualityConstraint(Aeq, beq,
                                    {mode.force_vars_[j].segment(kNForceVars*k + 6, 3),
                                     mode.force_vars_[j + 1].segment(kNForceVars*k, 3)});
        for(int z = 0; z < 3; z++) {
          AddConstraint(solvers::CreateLinearFrictionConstraint(mu_),
              mode.force_vars_[j+1].segment(kNForceVars*k + 3*z, 3));
        }
      }
    }

    // zero impact force for incoming swing leg
    if (i != 0 && sequence_[i] == stance::kDouble) {
      AddBoundingBoxConstraint(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
          modes_[i].force_vars_[0].segment(kNForceVars*(1-(int)sequence[i-1]), 3));
    }
    // zero force for outgoing swing leg at end of mode
    if (i != n_modes - 1 && sequence_[i] == stance::kDouble) {
      AddBoundingBoxConstraint(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
          modes_[i].force_vars_.end()->segment(kNForceVars*(int)sequence[i+1], 3));
    }
  }
}


void CentroidalTrajOpt::SetNominalStance(Eigen::Vector3d left,
                                         Eigen::Vector3d right) {
  nominal_stance_.clear();
  nominal_stance_.push_back(left);
  nominal_stance_.push_back(right);
}


void CentroidalTrajOpt::SetMaxDeviationConstraint(Eigen::Vector3d max) {

  // Assume flat ground for now
  max.z() = 0;

  std::vector<Eigen::Vector3d> lbs;
  std::vector<Eigen::Vector3d> ubs;

  for (auto & pose : nominal_stance_) {
    lbs.push_back(pose - max);
    ubs.push_back(pose + max);
  }

  for (int i = 0; i < sequence_.size(); i++) {
    for (int j = 0; j < modes_[i].n_c_; j ++) {
      AddBoundingBoxConstraint(lbs[j], ubs[j],
          modes_[i].stance_vars_[j]);
    }
  }
}
}
}
