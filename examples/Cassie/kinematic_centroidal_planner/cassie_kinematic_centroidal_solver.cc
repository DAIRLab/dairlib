#include "cassie_kinematic_centroidal_solver.h"

#include "common/find_resource.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/kinematic_centroidal_planner/simple_models/slip_constraints.h"

#include "drake/multibody/parsing/parser.h"

std::vector<dairlib::multibody::WorldPointEvaluator<double>>
CassieKinematicCentroidalSolver::CreateSlipContactPoints(
    const drake::multibody::MultibodyPlant<double>& plant, double mu) {
  auto left_toe_pair = dairlib::LeftToeFront(plant);
  auto right_toe_pair = dairlib::RightToeFront(plant);
  std::vector<int> active_inds{0, 1, 2};

  auto left_toe_eval = dairlib::multibody::WorldPointEvaluator<double>(
      plant, left_toe_pair.first, left_toe_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), active_inds);
  left_toe_eval.set_frictional();
  left_toe_eval.set_mu(mu);

  auto right_toe_eval = dairlib::multibody::WorldPointEvaluator<double>(
      plant, right_toe_pair.first, right_toe_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), active_inds);
  right_toe_eval.set_frictional();
  right_toe_eval.set_mu(mu);

  return {left_toe_eval, right_toe_eval};
}

void CassieKinematicCentroidalSolver::AddLoopClosure() {
  loop_closure_evaluators.add_evaluator(&l_loop_evaluator_);
  loop_closure_evaluators.add_evaluator(&r_loop_evaluator_);
  auto loop_closure =
      std::make_shared<dairlib::multibody::KinematicPositionConstraint<double>>(
          Plant(), loop_closure_evaluators, Eigen::VectorXd::Zero(2),
          Eigen::VectorXd::Zero(2));
  for (int knot_point = 1; knot_point < num_knot_points(); knot_point++) {
    AddKinematicConstraint(
        loop_closure, state_vars(knot_point).head(Plant().num_positions()));
  }
}

void CassieKinematicCentroidalSolver::AddSlipConstraints(int knot_point) {
  for (int contact_index = 0; contact_index < slip_contact_points_.size();
       contact_index++) {
    prog_->AddConstraint((slip_contact_pos_vars(knot_point, contact_index) -
                          slip_com_vars_[knot_point])
                             .squaredNorm() <= 1.0);

    if (slip_contact_sequence_[knot_point][contact_index]) {
      // Foot isn't moving
      prog_->AddBoundingBoxConstraint(
          Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3),
          slip_contact_vel_vars(knot_point, contact_index));
      // Foot is on the ground
      prog_->AddBoundingBoxConstraint(
          slip_ground_offset_, slip_ground_offset_,
          slip_contact_pos_vars(knot_point, contact_index)[2]);
    } else {
      prog_->AddBoundingBoxConstraint(
          0, 0, slip_force_vars_[knot_point][contact_index]);
      // Feet are above the ground
      double lb = 0;
      // Check if at least one of the time points before or after is also in
      // flight before restricting the foot to be in the air to limit over
      // constraining the optimization problem
      if (!is_first_knot(knot_point) and !is_last_knot(knot_point) and
          (!slip_contact_sequence_[knot_point - 1][contact_index] or
           !slip_contact_sequence_[knot_point + 1][contact_index])) {
        lb = swing_foot_minimum_height_;
      }
      lb += slip_ground_offset_;
      prog_->AddBoundingBoxConstraint(
          lb, 0.5, slip_contact_pos_vars(knot_point, contact_index)[2]);
    }
  }
}

void CassieKinematicCentroidalSolver::AddSlipCost(int knot_point,
                                                  double terminal_gain) {
  const double t = dt_ * knot_point;
  if (com_ref_traj_) {
    prog_->AddQuadraticErrorCost(terminal_gain * Q_com_,
                                 com_ref_traj_->value(t),
                                 slip_com_vars_[knot_point]);
  }

  // Project linear momentum
  if (mom_ref_traj_) {
    const Eigen::MatrixXd Q_vel = gains_.lin_momentum.asDiagonal() * m_;
    prog_->AddQuadraticErrorCost(
        terminal_gain * Q_vel, Eigen::VectorXd(mom_ref_traj_->value(t)).tail(3),
        slip_vel_vars_[knot_point]);
  }
  // Project com velocity
  if (v_ref_traj_) {
    const Eigen::MatrixXd Q_vel = Q_v_.block(3, 3, 3, 3);
    prog_->AddQuadraticErrorCost(
        terminal_gain * Q_vel,
        Eigen::VectorXd(v_ref_traj_->value(t)).segment(4, 3),
        slip_vel_vars_[knot_point]);
  }

  if (contact_ref_traj_) {
    const Eigen::MatrixXd Q_contact_pos = 2 * gains_.contact_pos.asDiagonal();
    prog_->AddQuadraticErrorCost(
        terminal_gain * Q_contact_pos,
        Eigen::VectorXd(contact_ref_traj_->value(t)).segment(0, 3),
        slip_contact_pos_vars(knot_point, 0));
    prog_->AddQuadraticErrorCost(
        terminal_gain * Q_contact_pos,
        Eigen::VectorXd(contact_ref_traj_->value(t)).segment(6, 3),
        slip_contact_pos_vars(knot_point, 1));

    const Eigen::MatrixXd Q_contact_vel = 2 * gains_.contact_vel.asDiagonal();
    prog_->AddQuadraticErrorCost(
        terminal_gain * Q_contact_vel,
        Eigen::VectorXd(contact_ref_traj_->value(t)).segment(12, 3),
        slip_contact_vel_vars(knot_point, 0));
    prog_->AddQuadraticErrorCost(
        terminal_gain * Q_contact_vel,
        Eigen::VectorXd(contact_ref_traj_->value(t)).segment(18, 3),
        slip_contact_vel_vars(knot_point, 1));
  }
  if (force_ref_traj_) {
    const Eigen::MatrixXd Q_contact_pos =
        Eigen::Vector2d(gains_.contact_force[2], gains_.contact_force[2])
            .asDiagonal();
    prog_->AddQuadraticErrorCost(terminal_gain * Q_contact_pos,
                                 Eigen::Vector2d::Zero(2),
                                 slip_force_vars_[knot_point]);
  }
}

void CassieKinematicCentroidalSolver::SetModeSequence(
    const std::vector<std::vector<bool>>& contact_sequence) {
  KinematicCentroidalSolver::SetModeSequence(contact_sequence);
  MapModeSequence();
}
void CassieKinematicCentroidalSolver::SetModeSequence(
    const drake::trajectories::PiecewisePolynomial<double>& contact_sequence) {
  KinematicCentroidalSolver::SetModeSequence(contact_sequence);
  MapModeSequence();
}

void CassieKinematicCentroidalSolver::MapModeSequence() {
  for (int knot_point = 0; knot_point < num_knot_points(); knot_point++) {
    slip_contact_sequence_[knot_point] =
        complex_mode_to_slip_mode_.at(contact_sequence_[knot_point]);
  }
}
void CassieKinematicCentroidalSolver::AddSlipEqualityConstraint(
    int knot_point) {
  prog_->AddConstraint(slip_com_vars_[knot_point] == com_pos_vars(knot_point));
  prog_->AddConstraint(slip_vel_vars_[knot_point] * m_ ==
                       momentum_vars(knot_point).tail(3));
  prog_->AddConstraint(slip_contact_pos_vars(knot_point, 0) ==
                       contact_pos_vars(knot_point, 0));
  prog_->AddConstraint(slip_contact_pos_vars(knot_point, 1) ==
                       contact_pos_vars(knot_point, 2));
  prog_->AddConstraint(slip_contact_vel_vars(knot_point, 0) ==
                       contact_vel_vars(knot_point, 0));
  prog_->AddConstraint(slip_contact_vel_vars(knot_point, 1) ==
                       contact_vel_vars(knot_point, 2));
  auto grf_constraint = std::make_shared<SlipGrfReductionConstrain>(
      plant_, reducers[knot_point], 2, 4, knot_point);
  prog_->AddConstraint(
      grf_constraint,
      {com_pos_vars(knot_point), slip_vel_vars_[knot_point],
       slip_contact_pos_vars_[knot_point], contact_force_[knot_point],
       slip_force_vars_[knot_point]});

  AddSlipPosturePrincipleConstraint(knot_point);
}

void CassieKinematicCentroidalSolver::AddSlipDynamics(int knot_point) {
  if (!is_last_knot(knot_point)) {
    auto slip_com_dynamics = std::make_shared<SlipDynamicsConstraint<double>>(
        r0_, k_, b_, m_, 2, slip_contact_sequence_[knot_point],
        slip_contact_sequence_[knot_point + 1], dt_, knot_point);

    slip_dynamics_binding_.push_back(prog_->AddConstraint(
        slip_com_dynamics,
        {slip_com_vars_[knot_point], slip_vel_vars_[knot_point],
         slip_contact_pos_vars_[knot_point], slip_force_vars_[knot_point],
         slip_com_vars_[knot_point + 1], slip_vel_vars_[knot_point + 1],
         slip_contact_pos_vars_[knot_point + 1],
         slip_force_vars_[knot_point + 1]}));
    prog_->AddConstraint(slip_contact_pos_vars_[knot_point + 1] ==
                         slip_contact_pos_vars_[knot_point] +
                             0.5 * dt_ *
                                 (slip_contact_vel_vars_[knot_point] +
                                  slip_contact_vel_vars_[knot_point + 1]));
  }
}

drake::solvers::VectorXDecisionVariable
CassieKinematicCentroidalSolver::slip_contact_pos_vars(int knot_point_index,
                                                       int slip_foot_index) {
  return slip_contact_pos_vars_[knot_point_index].segment(3 * slip_foot_index,
                                                          3);
}
drake::solvers::VectorXDecisionVariable
CassieKinematicCentroidalSolver::slip_contact_vel_vars(int knot_point_index,
                                                       int slip_foot_index) {
  return slip_contact_vel_vars_[knot_point_index].segment(3 * slip_foot_index,
                                                          3);
}

void CassieKinematicCentroidalSolver::SetComPositionGuess(
    const drake::trajectories::PiecewisePolynomial<double>& com_trajectory) {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    prog_->SetInitialGuess(slip_com_vars_[knot_point],
                           com_trajectory.value(dt_ * knot_point));
  }
  KinematicCentroidalSolver::SetComPositionGuess(com_trajectory);
}

void CassieKinematicCentroidalSolver::SetRobotStateGuess(
    const drake::trajectories::PiecewisePolynomial<double>& q_traj,
    const drake::trajectories::PiecewisePolynomial<double>& v_traj) {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    dairlib::multibody::SetPositionsIfNew<double>(
        plant_, q_traj.value(dt_ * knot_point), contexts_[knot_point].get());
    dairlib::multibody::SetVelocitiesIfNew<double>(
        plant_, v_traj.value(dt_ * knot_point), contexts_[knot_point].get());
    for (int contact = 0; contact < slip_contact_points_.size(); contact++) {
      prog_->SetInitialGuess(
          slip_contact_pos_vars(knot_point, contact),
          slip_contact_points_[contact].EvalFull(*contexts_[knot_point]));
      prog_->SetInitialGuess(
          slip_contact_vel_vars(knot_point, contact),
          slip_contact_points_[contact].EvalFullTimeDerivative(
              *contexts_[knot_point]));
    }
  }
  KinematicCentroidalSolver::SetRobotStateGuess(q_traj, v_traj);
}

void CassieKinematicCentroidalSolver::SetMomentumGuess(
    const drake::trajectories::PiecewisePolynomial<double>&
        momentum_trajectory) {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    prog_->SetInitialGuess(
        slip_vel_vars_[knot_point],
        drake::VectorX<double>(momentum_trajectory.value(dt_ * knot_point))
                .tail(3) /
            m_);
  }
  KinematicCentroidalSolver::SetMomentumGuess(momentum_trajectory);
}

drake::VectorX<double> CassieKinematicCentroidalSolver::LiftSlipSolution(
    int knot_point) {
  drake::VectorX<double> slip_state(3 + 3 + 6 * slip_contact_points_.size() +
                                    slip_contact_points_.size());
  slip_state << result_->GetSolution(slip_com_vars_[knot_point]),
      result_->GetSolution(slip_vel_vars_[knot_point]),
      result_->GetSolution(slip_contact_pos_vars_[knot_point]),
      result_->GetSolution(slip_contact_vel_vars_[knot_point]),
      result_->GetSolution(slip_force_vars_[knot_point]);
  return lifters_[knot_point]->Lift(slip_state).tail(n_q_ + n_v_);
}

void CassieKinematicCentroidalSolver::SetForceGuess(
    const drake::trajectories::PiecewisePolynomial<double>& force_trajectory) {
  for (const auto& force_vars : slip_force_vars_) {
    // TODO find better initial guess
    prog_->SetInitialGuess(force_vars,
                           0 * drake::VectorX<double>::Ones(force_vars.size()));
  }
  KinematicCentroidalSolver::SetForceGuess(force_trajectory);
}
void CassieKinematicCentroidalSolver::Build(
    const drake::solvers::SolverOptions& solver_options) {
  for (int knot = 0; knot < n_knot_points_; knot++) {
    lifters_.emplace_back(new SlipLifter(
        plant_, contexts_[knot].get(), slip_contact_points_,
        CreateContactPoints(plant_, 0), {{0, {0, 1}}, {1, {2, 3}}},
        nominal_stand_, k_, b_, r0_, slip_contact_sequence_[knot]));
    reducers.emplace_back(new SlipReducer(
        plant_, contexts_[knot].get(), slip_contact_points_,
        CreateContactPoints(plant_, 0), {{0, {0, 1}}, {1, {2, 3}}}, k_, b_, r0_,
        slip_contact_sequence_[knot]));
  }
  KinematicCentroidalSolver::Build(solver_options);
}
void CassieKinematicCentroidalSolver::AddSlipPosturePrincipleConstraint(
    int knot_point) {
  const double eps = 1e-2;
  // Zero hip yaw
  prog_->AddBoundingBoxConstraint(
      -eps, eps, x_vars_[knot_point][positions_map_.at("hip_yaw_left")]);
  prog_->AddBoundingBoxConstraint(
      -eps, eps, x_vars_[knot_point][positions_map_.at("hip_yaw_right")]);
  //  // Zero hip yaw dot
  prog_->AddBoundingBoxConstraint(
      -eps, eps,
      state_vars(knot_point)[n_q_ + velocity_map_.at("hip_yaw_leftdot")]);
  prog_->AddBoundingBoxConstraint(
      -eps, eps,
      state_vars(knot_point)[n_q_ + velocity_map_.at("hip_yaw_rightdot")]);

  // Identity orientation
  // TODO figure out why this identity quaternion constraint is hard for the mpc
  prog_->AddBoundingBoxConstraint(
      1 - eps, 1 + eps, state_vars(knot_point)[positions_map_.at("base_qw")]);
  //  prog_->AddBoundingBoxConstraint(
  //      -eps, eps, state_vars(knot_point)[positions_map_.at("base_qx")]);
  //  prog_->AddBoundingBoxConstraint(
  //      -eps, eps, state_vars(knot_point)[positions_map_.at("base_qy")]);
  prog_->AddBoundingBoxConstraint(
      -eps, eps, state_vars(knot_point)[positions_map_.at("base_qz")]);

  // Zero angular velocity
  prog_->AddBoundingBoxConstraint(
      -eps, eps, state_vars(knot_point)[n_q_ + velocity_map_.at("base_wx")]);
  prog_->AddBoundingBoxConstraint(
      -eps, eps, state_vars(knot_point)[n_q_ + velocity_map_.at("base_wy")]);
  prog_->AddBoundingBoxConstraint(
      -eps, eps, state_vars(knot_point)[n_q_ + velocity_map_.at("base_wz")]);

  // Toe and heel same velocity
  prog_->AddConstraint(contact_vel_vars(knot_point, 0) ==
                       contact_vel_vars(knot_point, 1));
  prog_->AddConstraint(contact_vel_vars(knot_point, 2) ==
                       contact_vel_vars(knot_point, 3));

  // Toe and heel same height
  prog_->AddConstraint(contact_pos_vars(knot_point, 0)[2] ==
                       contact_pos_vars(knot_point, 1)[2]);
  prog_->AddConstraint(contact_pos_vars(knot_point, 2)[2] ==
                       contact_pos_vars(knot_point, 3)[2]);

  //  // GRF per foot equal
  prog_->AddConstraint(contact_force_vars(knot_point, 0) ==
                       contact_force_vars(knot_point, 1));
  prog_->AddConstraint(contact_force_vars(knot_point, 2) ==
                       contact_force_vars(knot_point, 3));

  // TODO figure out this constraint (nonlinear constraint)
  // GRF per foot parallel to vector from foot center to com
  //  prog_->AddConstraint((contact_force_vars(knot_point, 0)
  //  / 1.0).normalized() ==
  //                       (com_pos_vars(knot_point) -
  //                        contact_pos_vars(knot_point, 0) / 2.0 -
  //                        contact_pos_vars(knot_point, 1) / 2.0)
  //                           .normalized());
  //
  //  prog_->AddConstraint((contact_force_vars(knot_point, 2)
  //  / 1.0).normalized() ==
  //                       (com_pos_vars(knot_point) -
  //                        contact_pos_vars(knot_point, 2) / 2.0 -
  //                        contact_pos_vars(knot_point, 3) / 2.0)
  //                           .normalized());
}
