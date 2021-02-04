//
// Created by brian on 1/31/21.
//

#include "planar_centroidal_traj_opt.h"
#include "planar_rigid_body_dynamics_constraint.h"
#include "solvers/constraint_factory.h"
#include "drake/solvers/solve.h"

namespace dairlib {
namespace centroidal_to {
namespace planar {

PlanarCentroidalTrajOpt::PlanarCentroidalTrajOpt(double I,
                                     double mass,
                                     double h,
                                     double mu) :
    I_(I),
    mass_(mass),
    h_(h),
    mu_(mu) {
  x0_ = Eigen::VectorXd::Zero(kStateVars);
  xf_ = Eigen::VectorXd::Zero(kStateVars);
}


void PlanarCentroidalTrajOpt::SetFinalPose(Eigen::Vector2d com, double theta,
    double eps) {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  pos.head(kLinearDim) = com;
  pos.tail(kAngularDim) = theta * Eigen::VectorXd::Ones(1);

  Eigen::Vector3d tol = eps* Eigen::Vector3d::Ones();

  AddBoundingBoxConstraint(pos-tol, pos+tol,
      modes_.back().state_vars_.back().head(kLinearDim + kAngularDim));

  SetInitialGuess(
      modes_.back().state_vars_.back().head(kLinearDim + kAngularDim),
      pos);

  xf_.head(kLinearDim + kAngularDim) = pos;
}

void PlanarCentroidalTrajOpt::SetFinalVel(Eigen::Vector2d v, double omega, double eps) {
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  vel.head(kLinearDim) = v;
  vel.tail(kAngularDim) = omega * Eigen::VectorXd::Ones(1);

  Eigen::Vector3d tol = eps* Eigen::Vector3d::Ones();

  AddBoundingBoxConstraint(vel-tol, vel+tol,
                           modes_.back().state_vars_.back().segment(
                               kLinearDim + kAngularDim,
                               kLinearDim + kAngularDim));
  SetInitialGuess(modes_.back().state_vars_.back().segment(
      kLinearDim + kAngularDim,
      kLinearDim + kAngularDim), vel);

  xf_.tail(kLinearDim + kAngularDim) = vel;
}

void PlanarCentroidalTrajOpt::SetFinalState(Eigen::VectorXd state) {
  AddBoundingBoxConstraint(state, state,
      modes_.back().state_vars_.back().head(kStateVars));
  SetInitialGuess(modes_.back().state_vars_.back().head(kStateVars),
      state);
  xf_ = state;
}

void PlanarCentroidalTrajOpt::SetInitialPose(Eigen::Vector2d com, double theta) {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  pos.head(kLinearDim) = com;
  pos.tail(kAngularDim) = theta * Eigen::VectorXd::Ones(1);

  AddBoundingBoxConstraint(pos, pos,
                           modes_.front().state_vars_.front().head(kLinearDim + kAngularDim));

  SetInitialGuess(modes_.front().state_vars_.front().head(kLinearDim + kAngularDim),
      pos);
  x0_.head(kLinearDim + kAngularDim) = pos;
}

void PlanarCentroidalTrajOpt::SetInitialVel(Eigen::Vector2d v, double omega) {
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  vel.head(kLinearDim) = v;
  vel.tail(kAngularDim) = omega * Eigen::VectorXd::Ones(1);


  AddBoundingBoxConstraint(vel, vel,
                           modes_.front().state_vars_.front().segment(
                               kLinearDim + kAngularDim,
                               kLinearDim + kAngularDim));
  SetInitialGuess(modes_.front().state_vars_.front().segment(
      kLinearDim + kAngularDim,
      kLinearDim + kAngularDim), vel);
  x0_.tail(kLinearDim + kAngularDim) = vel;
}



void PlanarCentroidalTrajOpt::SetModeSequence(std::vector<stance> sequence,
                                        std::vector<double> times) {
  DRAKE_ASSERT(sequence.size() == times.size())

  sequence_ = sequence;
  times_ = times;

  Eigen::MatrixXd AeqForceKnots = Eigen::MatrixXd::Zero(kForceDim, 2*kForceDim);
  AeqForceKnots.block(0, 0, kForceDim, kForceDim) =
      Eigen::MatrixXd::Identity(kForceDim, kForceDim);
  AeqForceKnots.block(0, kForceDim, kForceDim, kForceDim) =
      -1 * Eigen::MatrixXd::Identity(kForceDim, kForceDim);

  Eigen::MatrixXd AeqStanceFoot = Eigen::MatrixXd::Zero(kStanceVars, 2*kStanceVars);
  AeqStanceFoot.block(0, 0, kStanceVars, kStanceVars) =
      Eigen::MatrixXd::Identity(kStanceVars, kStanceVars);
  AeqStanceFoot.block(0, kStanceVars, kStanceVars, kStanceVars) =
      -1 * Eigen::MatrixXd::Identity(kStanceVars, kStanceVars);

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

  Eigen::VectorXd beq = Eigen::VectorXd::Zero(kForceDim);

  int n_modes = sequence.size();

  for (int i = 0; i < n_modes; i++) {
    std::cout << "Mode " << i << std::endl;

    int n_knot_f = std::round(times[i] / h_);
    int n_knot_s = n_knot_f + 1;
    int n_c = (sequence[i] == stance::D) ? 2 : 1;
    std::cout << "NC: " << n_c << std::endl;
    std::cout << "State Knot Points: " << n_knot_s << std::endl;
    std::cout << "Force Knot Points: " << n_knot_f << std::endl;

    CentroidalMode mode;
    mode.n_c_ = n_c;

    // create 3 forces per foot per node
    for (int j = 0; j < n_knot_f; j++) {
      mode.force_vars_.push_back(NewContinuousVariables(
          kForceVars * n_c,
          "forces[" + std::to_string(i) + "," + std::to_string(j) + "]"));
    }

    for (int j = 0; j < n_knot_s; j++) {
      mode.stance_vars_.push_back(NewContinuousVariables(kStanceVars * n_c,
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
      if(n_c == 0) {
        AddConstraint(rbd_constraint, {
            mode.state_vars_[j],
            mode.state_vars_[j + 1],
            mode.force_vars_[j]});
      } else {
        AddConstraint(rbd_constraint, {
            mode.state_vars_[j],
            mode.state_vars_[j + 1],
            mode.stance_vars_[j],
            mode.stance_vars_[j + 1],
            mode.force_vars_[j]});
      }
      for (int k = 0; k < n_c; k++) {
        if (j > 0) {
          AddLinearEqualityConstraint(AeqForceKnots, beq,
                                      {mode.force_vars_[j-1].segment(2 * kForceDim,
                                                                   kForceDim),
                                       mode.force_vars_[j].segment(
                                           0, kForceDim)});

        }
        for (int z = 0; z < kForceVars / kForceDim; z++) {
          AddLinearConstraint(friction_cone, zero, inf, mode.force_vars_[j].segment(k*kForceVars + kForceDim*z, kForceDim));
        }
      }
    }

    // zero impact force for incoming swing leg and friction cone constraint
    // for stance foot
    // Stance foot cannot change
    if (i != 0 && sequence_[i] == stance::D) {
      AddBoundingBoxConstraint(
          Eigen::VectorXd::Zero(kForceDim),
          Eigen::VectorXd::Zero(kForceDim),
          mode.force_vars_[0].segment(kForceVars * (1 - sequence_[i - 1]),
                                   kForceDim));
      AddLinearEqualityConstraint(AeqStanceFoot,
                               Eigen::VectorXd::Zero(kStanceVars),
                                  { mode.stance_vars_.front().segment(
                                      kStanceVars * (sequence_[i - 1]),
                                                           kStanceVars),
                                    modes_[i-1].stance_vars_.back()});
    }

    // zero force for outgoing swing leg at end of mode
    if (i != n_modes - 1 && sequence_[i] == stance::D) {
      AddBoundingBoxConstraint(
          Eigen::VectorXd::Zero(kForceDim),
          Eigen::VectorXd::Zero(kForceDim),
          mode.force_vars_[0].segment(kForceVars * (1-sequence_[i + 1]),
                                                           kForceDim));
    }

    if (i > 0 && sequence_[i-1] == stance::D) {
      AddLinearEqualityConstraint(AeqStanceFoot,
                                  Eigen::VectorXd::Zero(kStanceVars),
                                  {
        mode.stance_vars_.front(),
        modes_[i-1].stance_vars_.back().segment(
            kStanceVars * sequence_[i], kStanceVars)});
    }
    modes_.push_back(mode);
  }

  // Continuous state between modes
  for (int i = 1; i < n_modes; i++) {
    AddLinearEqualityConstraint(AeqModeTransition, Eigen::VectorXd::Zero(kStateVars),
                                {modes_[i-1].state_vars_.back(), modes_[i].state_vars_.front()});
  }
}

void PlanarCentroidalTrajOpt::SetNominalStance(Eigen::Vector2d left,
                                         Eigen::Vector2d right) {
  nominal_stance_.clear();
  nominal_stance_.push_back(left);
  nominal_stance_.push_back(right);
  for(int i = 0; i< modes_.size(); i++) {
    for (int j = 0; j < modes_[i].stance_vars_.size(); j++) {
      switch(sequence_[i]) {
        case (stance::L):
          SetInitialGuess(modes_[i].stance_vars_[j],
              nominal_stance_[stance::L]);
          break;
        case(stance::R):
          SetInitialGuess(modes_[i].stance_vars_[j],
                          nominal_stance_[stance::R]);
          break;
        case(stance::D):
          SetInitialGuess(modes_[i].stance_vars_[j].head(kStanceVars),
              nominal_stance_[stance::L]);
          SetInitialGuess(modes_[i].stance_vars_[j].tail(kStanceVars),
                          nominal_stance_[stance::R]);
      }
    }
  }
}

void PlanarCentroidalTrajOpt::SetMaxDeviationConstraint(Eigen::Vector2d max) {

  // Assume flat ground for now
  max[1] = 0;

  std::vector<Eigen::Vector2d> lbs;
  std::vector<Eigen::Vector2d> ubs;

  for (auto &pose : nominal_stance_) {
    lbs.push_back(pose - max);
    ubs.push_back(pose + max);
  }

  for (int i = 0; i < sequence_.size(); i++) {
    switch (sequence_[i]) {
      case (stance::L):
        for (int j = 0; j < modes_[i].stance_vars_.size(); j++) {
          AddBoundingBoxConstraint(lbs[stance::L], ubs[stance::L],
                                   modes_[i].stance_vars_[j]);
        }
        break;
      case (stance::R) :
        for (int j = 0; j < modes_[i].stance_vars_.size(); j++) {
          AddBoundingBoxConstraint(lbs[stance::R], ubs[stance::R],
                                   modes_[i].stance_vars_[j]);
        }
        break;
      case (stance::D) :
        for (int j = 0; j < modes_[i].stance_vars_.size(); j++) {
          AddBoundingBoxConstraint(lbs[stance::L], ubs[stance::L],
                                   modes_[i].stance_vars_[j].head(kStanceVars));
          AddBoundingBoxConstraint(lbs[stance::R], ubs[stance::R],
                                   modes_[i].stance_vars_[j].tail(kStanceVars));
        }
        break;
    }
  }
}

void PlanarCentroidalTrajOpt::SetInitialStateGuess() {
  int n_knot_s = 0;
  double t = 0;
  Eigen::VectorXd delta_pos = (x0_ - xf_).head(kLinearDim + kAngularDim);

  for (int i = 0; i < modes_.size(); i++) {
    n_knot_s += modes_[i].state_vars_.size() - 1;
    t+=times_[i];
  }


  Eigen::VectorXd delta_avg = (1.0/n_knot_s) * delta_pos;
  Eigen::VectorXd v_avg = (1.0/t) * delta_pos;
  Eigen::VectorXd pos = Eigen::VectorXd::Zero(kLinearDim+kAngularDim);

  for (int i = 0; i < modes_.size(); i++){
    for (int j = 0; j < modes_[i].state_vars_.size(); j++){
      if ((i != 0 || j!= 0) && i != (modes_.size()-1) && j!= (modes_.back().state_vars_.size() - 1)){
        SetInitialGuess(modes_[i].state_vars_[j].tail(kLinearDim + kAngularDim),
                        v_avg);
        pos += delta_avg;
        SetInitialGuess(modes_[i].state_vars_[j].head(kLinearDim + kAngularDim),
            pos);
      }
    }
  }
}

void PlanarCentroidalTrajOpt::SetInitialForceGuess() {
  Eigen::VectorXd f = Eigen::VectorXd::Zero(kForceDim);
  f.tail(1) = 9.81 * mass_ * Eigen::VectorXd::Ones(1);

  for (int i = 0; i < modes_.size(); i++){
    for (int j = 0; j < modes_[i].force_vars_.size(); j++) {
      for (int k = 0; k < kForceVars; k+=kForceDim) {
        SetInitialGuess(modes_[i].force_vars_[j].segment(k, kForceDim),f);
      }
    }
  }
}

drake::solvers::MathematicalProgramResult PlanarCentroidalTrajOpt::SolveProg() {
  auto result = drake::solvers::Solve(*this);
  return result;
}

}
}
}

