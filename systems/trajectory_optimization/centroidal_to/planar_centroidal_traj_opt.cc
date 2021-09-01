//
// Created by brian on 1/31/21.
//

#include <drake/solvers/snopt_solver.h>
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

  Eigen::Vector3d tol = eps * Eigen::Vector3d::Ones();

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
  DRAKE_ASSERT(sequence.size() == times.size());

  sequence_ = sequence;
  times_ = times;
  n_modes_ = sequence_.size();

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
  friction_cone << -mu_, 1, mu_, 1;
  Eigen::Vector2d zero = Eigen::Vector2d::Zero();
  Eigen::Vector2d inf = 3 * 9.81 * mass_ * Eigen::Vector2d::Ones();
  /*inf << std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity();
*/

  int n_modes = sequence.size();

  for (int i = 0; i < n_modes; i++) {

    int n_knot_f = std::round(times[i] / h_) + 1;
    int n_knot_s = n_knot_f;
    int n_c = (sequence[i] == stance::D) ? 2 : 1;

    CentroidalMode mode;
    mode.n_c_ = n_c;

    // create 3 forces per foot per node
    for (int j = 0; j < n_knot_f; j++) {
      mode.force_vars_.push_back(NewContinuousVariables(
          kForceVars * n_c,
          "forces[" + std::to_string(i) + "," + std::to_string(j) + "]"));
    }


     mode.stance_vars_ = NewContinuousVariables(kStanceVars * n_c,
                                                         "stance_pos["
                                                             + std::to_string(i)
                                                             + "]");


    for (int j = 0; j < n_knot_s; j++) {
      mode.state_vars_.push_back(NewContinuousVariables(
          kStateVars,
          "x[" + std::to_string(i) + "]"));

    }

    for (int j = 0; j < n_knot_s - 1; j++) {
      auto rbd_constraint = std::make_shared<PlanarRigidBodyDynamicsConstraint>(
          I_, mass_, h_, n_c);
      if (n_c == 0) {
        AddConstraint(rbd_constraint, {
            mode.state_vars_[j],
            mode.state_vars_[j + 1]});
      } else {
        AddConstraint(rbd_constraint, {
            mode.state_vars_[j],
            mode.state_vars_[j + 1],
            mode.stance_vars_,
            mode.force_vars_[j],
            mode.force_vars_[j + 1]});
      }
    }
    for (int j = 0; j < n_knot_f; j++) {
      for (int k = 0; k < n_c; k++) {
        AddLinearConstraint(friction_cone, zero, inf, mode.force_vars_[j].segment(kForceDim * k, kForceDim));
      }
    }

    // zero impact force for incoming swing leg and friction cone constraint
    // for stance foot
    // Stance foot cannot change

    if (i != 0 && sequence_[i] == stance::D) {
      AddBoundingBoxConstraint(
          Eigen::VectorXd::Zero(kForceDim),
          Eigen::VectorXd::Zero(kForceDim),
          mode.force_vars_[0].segment(kForceDim * (1 - sequence_[i - 1]),
                                   kForceDim));
      AddLinearEqualityConstraint(AeqStanceFoot,
                               Eigen::VectorXd::Zero(kStanceVars),
                                  { mode.stance_vars_.segment(
                                      kStanceVars * (sequence_[i - 1]),
                                                           kStanceVars),
                                    modes_[i-1].stance_vars_});
      AddLinearEqualityConstraint(AeqStanceFoot,
                                  Eigen::VectorXd::Zero(kStanceVars),
                                  { mode.force_vars_.front().segment(
                                      kStanceVars * (sequence_[i - 1]),
                                      kStanceVars),
                                    modes_[i-1].force_vars_.back()});
    }

    // zero force for outgoing swing leg at end of mode

    if (i != n_modes - 1 && sequence_[i] == stance::D) {
      AddBoundingBoxConstraint(
          Eigen::VectorXd::Zero(kForceDim),
          Eigen::VectorXd::Zero(kForceDim),
          mode.force_vars_.back().segment(kForceVars * (1-sequence_[i + 1]),
                                                           kForceDim));
    }

    if (i > 0 && sequence_[i-1] == stance::D) {
      AddLinearEqualityConstraint(AeqStanceFoot,
                                  Eigen::VectorXd::Zero(kStanceVars),
                                  {
        mode.stance_vars_,
        modes_[i-1].stance_vars_.segment(
            kStanceVars * sequence_[i], kStanceVars)});
      AddLinearEqualityConstraint(AeqStanceFoot,
                                  Eigen::VectorXd::Zero(kStanceVars),
                                  {
                                      mode.force_vars_.front(),
                                      modes_[i-1].force_vars_.back().segment(
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
}

void PlanarCentroidalTrajOpt::SetMaxDeviationConstraint(Eigen::Vector2d max) {

  std::vector<Eigen::Vector2d> lbs;
  std::vector<Eigen::Vector2d> ubs;

  for (auto &pose : nominal_stance_) {
    lbs.push_back(pose - max);
    ubs.push_back(pose + max);
  }

  for (int i = 0; i < sequence_.size(); i++) {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(kLinearDim, 2 * kLinearDim);
    A.block(0, 0, kLinearDim, kLinearDim) =
        Eigen::MatrixXd::Identity(kLinearDim, kLinearDim);
    A.block(0, kLinearDim, kLinearDim, kLinearDim) =
        -1.0 * Eigen::MatrixXd::Identity(kLinearDim, kLinearDim);

    switch (sequence_[i]) {
      case (stance::L):
        for (int j = 0; j < modes_[i].state_vars_.size(); j++) {
          AddLinearConstraint(A, lbs[stance::L], ubs[stance::L],
                              {modes_[i].stance_vars_,
                               modes_[i].state_vars_[j].head(kLinearDim)});
        }
        break;
      case (stance::R) :
        for (int j = 0; j < modes_[i].state_vars_.size(); j++) {
          AddLinearConstraint(A, lbs[stance::R], ubs[stance::R],
                              {modes_[i].stance_vars_,
                               modes_[i].state_vars_[j].head(kLinearDim)});
        }
        break;
      case (stance::D) :
        for (int j = 0; j < modes_[i].state_vars_.size(); j++) {
          AddLinearConstraint(A, lbs[stance::L], ubs[stance::L],
                              {modes_[i].stance_vars_.head(kStanceVars),
                               modes_[i].state_vars_[j].head(kLinearDim)});
          AddLinearConstraint(A, lbs[stance::R], ubs[stance::R],
                              {modes_[i].stance_vars_.tail(kStanceVars),
                               modes_[i].state_vars_[j].head(kLinearDim)});
        }
        break;
    }
  }
  SetFlatGroundTerrainConstraint();
}

void PlanarCentroidalTrajOpt::SetFlatGroundTerrainConstraint() {
  for (int i = 0; i < n_modes_; i++) {
    AddBoundingBoxConstraint(Eigen::VectorXd::Zero(1),
                             Eigen::VectorXd::Zero(1),
                             modes_[i].stance_vars_.tail(1));
    if (sequence_[i] == stance::D) {
      AddBoundingBoxConstraint(Eigen::VectorXd::Zero(1),
                               Eigen::VectorXd::Zero(1),
                               modes_[i].stance_vars_.segment(kStanceVars - 1, 1));
    }
  }
}

void PlanarCentroidalTrajOpt::SetInitialStateGuess() {
  double T = MapKnotPointToTime(n_modes_ - 1,
      modes_.back().state_vars_.size() - 1);

  Eigen::VectorXd delta_pos = (xf_ - x0_).head(kLinearDim + kAngularDim);
  Eigen::VectorXd v_coeff = (-6.0 / pow(T, 3)) * delta_pos;

  for (int i = 0; i < n_modes_; i++){
    for (int j = 0; j < modes_[i].state_vars_.size(); j++){
      if ((i != 0 || j!= 0) &&
      ! (i == (n_modes_-1) && j == (modes_.back().state_vars_.size() - 1))){

        double t = MapKnotPointToTime(i, j);
        Eigen::VectorXd pos = x0_.head(kLinearDim + kAngularDim) +
            v_coeff * ((pow(t, 3) / 3.0 - T * (pow(t, 2) / 2.0)));

        SetInitialGuess(modes_[i].state_vars_[j].tail(kLinearDim + kAngularDim),
                        t * (t - T) * v_coeff);

        SetInitialGuess(modes_[i].state_vars_[j].head(kLinearDim + kAngularDim),
            pos);
      }
    }
  }
}

void PlanarCentroidalTrajOpt::SetInitialForceGuess() {
  Eigen::VectorXd f = Eigen::VectorXd::Zero(kForceDim);
  f.tail(1) = 9.81 * mass_ * Eigen::VectorXd::Ones(1);

  for (int i = 0; i < n_modes_; i++){
    int n_c = (sequence_[i] == stance::D)? 2 : 1;
    for (int j = 0; j < modes_[i].force_vars_.size(); j++) {
      for (int k = 0; k < kForceVars * n_c; k+=kForceDim) {
        SetInitialGuess(modes_[i].force_vars_[j].segment(k, kForceDim),f);
      }
    }
  }
}

void PlanarCentroidalTrajOpt::SetInitialStanceGuess() {
  double n_ss = floor(sequence_.size() / 2.0);
  Eigen::VectorXd delta_pos = (xf_ - x0_).head(kLinearDim + kAngularDim);
  Eigen::Vector2d stance_delta = (1.0 / (2.0 * n_ss)) * delta_pos.head(kLinearDim);
  Eigen::Vector2d stance = Eigen::Vector2d::Zero();
  stance.head(1) = x0_.head(1) - stance_delta.head(1);

  for (int i = 0; i < n_modes_; i ++) {
    if (sequence_[i] == stance::D) {
      SetInitialGuess(modes_[i].stance_vars_.segment(kStanceVars * (1 - sequence_[i + 1]), kStanceVars), stance);
      stance  = stance + 2.0 * stance_delta;
      SetInitialGuess(modes_[i].stance_vars_.segment(kStanceVars * (sequence_[i + 1]), kStanceVars), stance);
    } else {
      SetInitialGuess(modes_[i].stance_vars_, stance);
    }
  }
}

drake::solvers::MathematicalProgramResult PlanarCentroidalTrajOpt::SolveProg(int iteration_limit) {

  std::string outfile = "/home/brian/workspace/dairlib/systems/trajectory_optimization/centroidal_to/snopt.out";
  SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file", outfile);
  SetSolverOption(drake::solvers::SnoptSolver::id(), "Major iterations limit", iteration_limit);

  drake::solvers::SnoptSolver solver;
  auto result = solver.Solve(*this, {}, {});
  return result;
}


double PlanarCentroidalTrajOpt::MapKnotPointToTime(int idx_mode, int idx_knot)  {
  double time = 0;
  int n_knot = modes_[idx_mode].state_vars_.size() - 1;

  for (int i = 0; i < idx_mode; i++) {
    time += times_[i];
  }

  return time + times_[idx_mode] * ((double) idx_knot )/((double) n_knot);
}

int PlanarCentroidalTrajOpt::NumStateKnots() {
  int n = 0;
  for (int i = 0; i < n_modes_; i++) {
    n += modes_[i].state_vars_.size() - 1;
  }
  return n+1;
}

LcmTrajectory PlanarCentroidalTrajOpt::GetStateTrajectories(
    drake::solvers::MathematicalProgramResult& result
    ) {

  std::vector<LcmTrajectory::Trajectory> state_traj;
  int n_knot = NumStateKnots();

  Eigen::MatrixXd state_knots = Eigen::MatrixXd::Zero(kStateVars, n_knot);
  Eigen::VectorXd time_knots = Eigen::VectorXd::Zero(n_knot);
  int time_idx = 0;

  for (int i = 0; i < n_modes_; i++) {
    for (int j = 0; j < modes_[i].state_vars_.size() - 1; j++) {
      time_knots[time_idx] = MapKnotPointToTime(i, j);
      state_knots.block(0, time_idx, kStateVars, 1) =
          result.GetSolution(modes_[i].state_vars_[j]);
      time_idx ++;
    }
  }

  time_knots[time_idx] = MapKnotPointToTime(n_modes_ - 1,
      modes_.back().state_vars_.size() -1);
  state_knots.block(0, time_idx, kStateVars, 1) =
      result.GetSolution(modes_.back().state_vars_.back());

  for (int i = 0; i < kStateVars; i ++) {
    auto traj = LcmTrajectory::Trajectory();
    traj.traj_name = state_var_names_[i];
    traj.datapoints = state_knots.block(i, 0, 1, n_knot);
    traj.time_vector = time_knots;
    traj.datatypes = {"double"};
    state_traj.push_back(traj);
  }

  return LcmTrajectory(state_traj, state_var_names_, "state_trajectories",
      "state trajectories");
}

LcmTrajectory PlanarCentroidalTrajOpt::GetFootTrajectories(
    drake::solvers::MathematicalProgramResult &result
    ) {

  std::vector<LcmTrajectory::Trajectory> foot_traj;
  std::vector<std::string> traj_names;


  for (int i = 0; i < n_modes_; i++) {

    Eigen::Vector2d t = {MapKnotPointToTime(i, 0),
         MapKnotPointToTime(i, modes_[i].state_vars_.size() -1)};

    if (sequence_[i] != stance::D) {

      auto traj = LcmTrajectory::Trajectory();
      char buf[10];
      sprintf(buf, "%02d", i);
      std::string b = buf;
      traj.traj_name = stance_var_names_[sequence_[i]] + ("[" + b + "]");
      Eigen::MatrixXd res = Eigen::MatrixXd::Zero(kStanceVars, 2);

      for (int j = 0; j < 2; j++) {
        res.block(0, j, kStanceVars, 1) =
            result.GetSolution(modes_[i].stance_vars_);
      }

      traj.datapoints = res;
      traj.time_vector = t;
      traj.datatypes = {"double", "double"};
      traj_names.push_back(traj.traj_name);
      foot_traj.push_back(traj);

    } else {

      for (int j = 0; j < 2; j++) {
        auto traj = LcmTrajectory::Trajectory();
        char buf[10];
        sprintf(buf, "%02d", i);
        std::string b = buf;
        traj.traj_name = stance_var_names_[j] + ("[" + b + "]");
        Eigen::MatrixXd res = Eigen::MatrixXd::Zero(kStanceVars, 2);

        for (int k = 0; k < 2; k++) {
          res.block(0, k, kStanceVars, 1) =
              result.GetSolution(modes_[i].stance_vars_.segment( j * kStanceVars, kStanceVars));
        }

        traj.datapoints = res;
        traj.time_vector = t;
        traj.datatypes = {"double", "double"};
        traj_names.push_back(traj.traj_name);
        foot_traj.push_back(traj);
      }
    }
  }

  return LcmTrajectory(foot_traj, traj_names, "Foot Traj", "Foot Placement Trajectories");
}

LcmTrajectory PlanarCentroidalTrajOpt::GetForceTrajectories(
    drake::solvers::MathematicalProgramResult &result
    ) {
  std::vector<LcmTrajectory::Trajectory> force_traj;
  int n_knot = NumStateKnots();
  LcmTrajectory::Trajectory l_force;
  LcmTrajectory::Trajectory r_force;

  Eigen::MatrixXd force_knots = Eigen::MatrixXd::Zero(2 * kForceDim, n_knot);
  Eigen::VectorXd time_knots = Eigen::VectorXd::Zero(n_knot);

  int time_idx = 0;
  for (int i = 0; i < n_modes_; i++) {
    for (int j = 0; j < modes_[i].force_vars_.size() - 1; j++) {
      time_knots[time_idx] = MapKnotPointToTime(i, j);

      if (sequence_[i] != stance::D) {
        force_knots.block(  kForceDim * sequence_[i], time_idx, kForceDim,1) =
            result.GetSolution(modes_[i].force_vars_[j]);
        force_knots.block(kForceDim * (1 - sequence_[i]), time_idx, kForceDim, 1) =
            Eigen::MatrixXd::Zero(kForceDim, 1);
      } else {
        force_knots.block(0, time_idx, 2 * kForceDim, 1) =
            result.GetSolution(modes_[i].force_vars_[j]);
      }
      time_idx ++;
    }
  }

  /// Assume double stance in final pose
  time_knots[time_idx] = MapKnotPointToTime(n_modes_ - 1,
                                            modes_.back().force_vars_.size() -1);
  force_knots.block(0, time_idx, 2 * kForceDim, 1) =
      result.GetSolution(modes_.back().force_vars_.back());

  l_force.time_vector = time_knots;
  l_force.datapoints = force_knots.block(0, kForceDim * stance::L, kForceDim, n_knot);
  l_force.traj_name = force_var_names_[stance::L];
  l_force.datatypes = {"double", "double"};

  r_force.time_vector = time_knots;
  r_force.datapoints = force_knots.block(kForceDim, kForceDim * stance::R, kForceDim, n_knot);
  r_force.traj_name = force_var_names_[stance::R];
  r_force.datatypes = {"double", "double"};

  force_traj.push_back(l_force);
  force_traj.push_back(r_force);

  return LcmTrajectory(force_traj, force_var_names_,  "Force Traj", "force Trajectories");
}

}
}
}

