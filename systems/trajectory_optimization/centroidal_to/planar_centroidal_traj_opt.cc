//
// Created by brian on 1/31/21.
//

#include "planar_centroidal_traj_opt.h"
#include "planar_rigid_body_dynamics_constraint.h"
#include "solvers/constraint_factory.h"

namespace dairlib {
namespace centroidal_to {
namespace planar {

PlanarCentroidalTrajOpt::PlanarCentroidalTrajOpt(double I,
                                     double mass,
                                     double h,
                                     double T_ss,
                                     double T_ds,
                                     double mu) :
    I_(I),
    mass_(mass),
    h_(h),
    T_ss_(T_ss),
    T_ds_(T_ds),
    mu_(mu) {}

void PlanarCentroidalTrajOpt::SetModeSequence(std::vector<stance> sequence,
                                        std::vector<double> times) {
  DRAKE_ASSERT(sequence.size() == times.size())

  sequence_ = sequence;
  times_ = times;

  Eigen::MatrixXd AeqForceKnots = Eigen::MatrixXd::Zero(kForceDim, 2*kForceDim);
  AeqForceKnots.block(0, 0, kForceDim, kForceDim) = Eigen::MatrixXd::Identity(kForceDim, kForceDim);
  AeqForceKnots.block(0, kForceDim, kForceDim, kForceDim) = -1 * Eigen::MatrixXd::Identity(kForceDim, kForceDim);

  Eigen::MatrixXd AeqModeTransition = Eigen::MatrixXd::Zero(kStateVars, 2* (kStateVars));
  AeqModeTransition.block(0, 0, kStateVars, kStateVars) =
      Eigen::MatrixXd::Identity(kStateVars, kStateVars);
  AeqModeTransition.block(0, kStateVars, kStateVars, kStateVars) =
      -1 * Eigen::MatrixXd::Identity(kStateVars, kStateVars);

  Eigen::Matrix2d friction_cone;
  friction_cone << mu_, -1, 0, 1;
  Eigen::Vector2d zero = Eigen::Vector2d::Zero();
  Eigen::Vector2d inf;
  inf << std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity();
  Eigen::VectorXd beq = Eigen::VectorXd::Zero(6);

  int n_modes = sequence.size();
  for (int i = 0; i < n_modes; i++) {
    int n_knot_f = std::round(times[i] / h_);
    int n_knot_s = n_knot_f + 1;
    int n_c = (sequence[i] == stance::D) ? 2 : 1;

    CentroidalMode mode;
    mode.n_c_ = n_c;

    // create 3 forces per foot per node
    for (int j = 0; j < n_knot_f; j++) {
      mode.force_vars_.push_back(NewContinuousVariables(
          kForceVars * n_c,
          "forces[" + std::to_string(i) + "," + std::to_string(j) + "]"));
    }

    for (int j = 0; j < n_c; j++) {
      mode.stance_vars_.push_back(NewContinuousVariables(kStanceVars,
                                                         "stance_pos["
                                                             + std::to_string(i)
                                                             + ","
                                                             + std::to_string(j)
                                                             + "]"));
    }

    for (int j = 0; j < n_knot_s; j++) {
      mode.state_vars_.push_back(NewContinuousVariables(
          kStateVars,
          "x[" + std::to_string(i) + "]"));

    }

    for (int j = 0; j < n_knot_s - 1; j++) {
      auto rbd_constraint = std::make_shared<PlanarRigidBodyDynamicsConstraint>(
          I_, mass_, h_, n_c);
      switch (n_c) {
        case 0:
          AddConstraint(rbd_constraint, {
              mode.state_vars_[j],
              mode.state_vars_[j + 1],
              mode.force_vars_[j]});
          break;
        case 1:
          AddConstraint(rbd_constraint, {
              mode.state_vars_[j],
              mode.state_vars_[j + 1],
              mode.force_vars_[j],
              mode.stance_vars_[0]});
        case 2:
          AddConstraint(rbd_constraint, {
              mode.state_vars_[j],
              mode.state_vars_[j + 1],
              mode.force_vars_[j],
              mode.stance_vars_[0],
              mode.stance_vars_[1]});
      }
      for (int k = 0; k < n_c; k++) {
        AddLinearEqualityConstraint(AeqForceKnots, beq,
                                    {mode.force_vars_[j].segment(
                                        kForceVars * k + 2 * kForceDim,
                                        kForceDim),
                                     mode.force_vars_[j + 1].segment(
                                         kForceVars * k, kForceDim)});

        for (int z = 0; z < kForceVars / kForceDim; z++) {
          AddLinearConstraint(friction_cone, zero, inf, mode.force_vars_[j+1].segment(k*kForceVars + kForceDim*z, kForceDim));
        }
      }
    }

    // Add Friction cone constraint for first mode
    if (i == 0) {
      for (int k = 0; k < n_c; k++ ) {
        for (int z = 0; z < kForceVars / kForceDim; z++) {
          AddLinearConstraint(friction_cone, zero, inf, mode.force_vars_[0].segment(k*kForceVars + kForceDim*z, kForceDim));
        }
      }
    }

    // zero impact force for incoming swing leg and friction cone constraint
    // for stance foot
    if (i != 0 && sequence_[i] == stance::D) {
      AddBoundingBoxConstraint(
          Eigen::VectorXd::Zero(kForceDim),
          Eigen::VectorXd::Zero(kForceDim),
          mode.force_vars_[0].segment(kForceVars * (1 - sequence_[i - 1]),
                                   kForceDim));
    }

    // zero force for outgoing swing leg at end of mode
    if (i != n_modes - 1 && sequence_[i] == stance::D) {
      AddBoundingBoxConstraint(
          Eigen::VectorXd::Zero(kForceDim),
          Eigen::VectorXd::Zero(kForceDim),
          mode.force_vars_[0].segment(kForceVars * (sequence_[i - 1]),
                                                           kForceDim));
    }

    modes_.push_back(mode);
  }

  // Continuous state between modes
  for (int i = 1; i < n_modes; i++) {
    AddLinearEqualityConstraint(AeqModeTransition, Eigen::VectorXd::Zero(kStateVars),
                                {modes_[i-1].state_vars_.end(), modes_[i].state_vars_.begin()});
  }
}

void PlanarCentroidalTrajOpt::SetNominalStance(Eigen::Vector3d left,
                                         Eigen::Vector3d right) {
  nominal_stance_.clear();
  nominal_stance_.push_back(left);
  nominal_stance_.push_back(right);
}

void PlanarCentroidalTrajOpt::SetMaxDeviationConstraint(Eigen::Vector3d max) {

  // Assume flat ground for now
  max.z() = 0;

  std::vector<Eigen::Vector3d> lbs;
  std::vector<Eigen::Vector3d> ubs;

  for (auto &pose : nominal_stance_) {
    lbs.push_back(pose - max);
    ubs.push_back(pose + max);
  }

  for (int i = 0; i < sequence_.size(); i++) {
    for (int j = 0; j < modes_[i].n_c_; j++) {
      AddBoundingBoxConstraint(lbs[j], ubs[j], // this wont work
                               modes_[i].stance_vars_[j]);
    }
  }
}
}
}
}
