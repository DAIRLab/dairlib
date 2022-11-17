#include "cassie_kinematic_centroidal_mpc.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/multibody/parsing/parser.h"
#include "common/find_resource.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/kinematic_centroidal_mpc/simple_models/planar_slip_constraints.h"

std::vector<dairlib::multibody::WorldPointEvaluator<double>> CassieKinematicCentroidalMPC::CreateContactPoints(const drake::multibody::MultibodyPlant<
    double> &plant,
                                                                                                               double mu) {
  auto left_toe_pair = dairlib::LeftToeFront(plant);
  auto left_heel_pair = dairlib::LeftToeRear(plant);
  auto right_toe_pair = dairlib::RightToeFront(plant);
  auto right_heel_pair = dairlib::RightToeRear(plant);

  std::vector<int> active_inds{0, 1, 2};

  auto left_toe_eval = dairlib::multibody::WorldPointEvaluator<double>(
      plant, left_toe_pair.first, left_toe_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), active_inds);
  left_toe_eval.set_frictional();
  left_toe_eval.set_mu(mu);

  auto left_heel_eval = dairlib::multibody::WorldPointEvaluator<double>(
      plant, left_heel_pair.first, left_heel_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), active_inds);
  left_heel_eval.set_frictional();
  left_heel_eval.set_mu(mu);

  auto right_toe_eval = dairlib::multibody::WorldPointEvaluator<double>(
      plant, right_toe_pair.first, right_toe_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), active_inds);
  right_toe_eval.set_frictional();
  right_toe_eval.set_mu(mu);

  auto right_heel_eval = dairlib::multibody::WorldPointEvaluator<double>(
      plant, right_heel_pair.first, right_heel_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), active_inds);
  right_heel_eval.set_frictional();
  right_heel_eval.set_mu(mu);

  return {left_toe_eval, left_heel_eval, right_toe_eval, right_heel_eval};
}

std::vector<dairlib::multibody::WorldPointEvaluator<double>> CassieKinematicCentroidalMPC::CreateSlipContactPoints(const drake::multibody::MultibodyPlant<
    double> &plant, double mu) {
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

void CassieKinematicCentroidalMPC::AddLoopClosure() {
  loop_closure_evaluators.add_evaluator(&l_loop_evaluator_);
  loop_closure_evaluators.add_evaluator(&r_loop_evaluator_);
  auto loop_closure =
      std::make_shared<dairlib::multibody::KinematicPositionConstraint<double>>(
          Plant(),
          loop_closure_evaluators,
          Eigen::VectorXd::Zero(2),
          Eigen::VectorXd::Zero(2));
  for (int knot_point = 1; knot_point < num_knot_points(); knot_point++) {
    AddKinematicConstraint(loop_closure, state_vars(knot_point).head(Plant().num_positions()));
  }

}

void CassieKinematicCentroidalMPC::AddPlanarSlipConstraints(int knot_point) {
  for(int contact_index = 0; contact_index < slip_contact_points_.size(); contact_index ++){
    if(slip_contact_sequence_[knot_point][contact_index]){
      // Foot isn't moving
      prog_->AddBoundingBoxConstraint(
          Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2),
          slip_contact_vel_vars(knot_point, contact_index));
      // Foot is on the ground
      prog_->AddBoundingBoxConstraint(
          slip_ground_offset_, slip_ground_offset_, slip_contact_pos_vars(knot_point, contact_index)[1]);

      prog_->AddConstraint((slip_com_vars_[knot_point]-slip_contact_pos_vars(knot_point,contact_index)).norm() <= r0_);
    }else{
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
      lb +=slip_ground_offset_;
      prog_->AddBoundingBoxConstraint(
          lb, 10, slip_contact_pos_vars(knot_point, contact_index)[1]);

    }
  }
}

void CassieKinematicCentroidalMPC::AddPlanarSlipCost(int knot_point, double terminal_gain) {
  const double t = dt_ * knot_point;
  std::shared_ptr<QuadraticLiftedCost> lifting_cost(new QuadraticLiftedCost(lifters_[knot_point],
                                                            {Q_com_, com_ref_traj_->value(t)},
                                                            {Q_mom_, mom_ref_traj_->value(t)},
                                                            {Q_contact_, contact_ref_traj_->value(t)},
                                                            {Q_force_, force_ref_traj_->value(t)},
                                                            {Q_q_,q_ref_traj_->value(t)},
                                                            {Q_v_,v_ref_traj_->value(t)},
                                                            terminal_gain,
                                                            slip_contact_points_.size(),
                                                            knot_point));
  prog_->AddCost(lifting_cost, {slip_com_vars_[knot_point], slip_vel_vars_[knot_point], slip_contact_pos_vars_[knot_point],slip_contact_vel_vars_[knot_point]});
}

void CassieKinematicCentroidalMPC::SetModeSequence(const std::vector<std::vector<bool>> &contact_sequence) {
  KinematicCentroidalMPC::SetModeSequence(contact_sequence);
  MapModeSequence();
}
void CassieKinematicCentroidalMPC::SetModeSequence(const drake::trajectories::PiecewisePolynomial<double> &contact_sequence) {
  KinematicCentroidalMPC::SetModeSequence(contact_sequence);
  MapModeSequence();
}

void CassieKinematicCentroidalMPC::MapModeSequence() {
  for (int knot_point = 0; knot_point < num_knot_points(); knot_point++) {
    slip_contact_sequence_[knot_point] = complex_mode_to_slip_mode_.at(contact_sequence_[knot_point]);
  }
}
void CassieKinematicCentroidalMPC::AddSlipReductionConstraint(int knot_point) {
  auto reduction_constraint =
      std::make_shared<PlanarSlipReductionConstraint<double>>(
          plant_, contexts_[knot_point].get(), slip_contact_points_,6 + 3 + 3 * 3 * n_contact_points_ + n_q_ + n_v_,knot_point);
  prog_->AddConstraint(reduction_constraint,
                       {slip_com_vars_[knot_point], slip_vel_vars_[knot_point], slip_contact_pos_vars_[knot_point],slip_contact_vel_vars_[knot_point],
                        com_pos_vars(knot_point), momentum_vars(knot_point), contact_pos_[knot_point], contact_vel_[knot_point], contact_force_[knot_point],
                        state_vars(knot_point)});
}

void CassieKinematicCentroidalMPC::AddSlipLiftingConstraint(int knot_point) {
  auto lifting_constraint =
      std::make_shared<PlanarSlipLiftingConstraint>(
          plant_, lifters_[knot_point], 2, n_contact_points_,knot_point);
  prog_->AddConstraint(lifting_constraint,
                       {slip_com_vars_[knot_point], slip_vel_vars_[knot_point], slip_contact_pos_vars_[knot_point],slip_contact_vel_vars_[knot_point],
                        com_pos_vars(knot_point), momentum_vars(knot_point), contact_pos_[knot_point], contact_vel_[knot_point], contact_force_[knot_point],
                        state_vars(knot_point)});
}

void CassieKinematicCentroidalMPC::AddSlipDynamics(int knot_point) {
  if(!is_last_knot(knot_point)) {
    auto slip_com_dynamics =
        std::make_shared<PlanarSlipDynamicsConstraint<double>>(
            r0_, k_, m_, 2, slip_contact_sequence_[knot_point], dt_, knot_point);

    prog_->AddConstraint(slip_com_dynamics,
                         {slip_com_vars_[knot_point], slip_vel_vars_[knot_point], slip_contact_pos_vars_[knot_point],
                          slip_com_vars_[knot_point+1], slip_vel_vars_[knot_point+1], slip_contact_pos_vars_[knot_point+1]});
    prog_->AddConstraint(
        slip_contact_pos_vars_[knot_point + 1] ==
            slip_contact_pos_vars_[knot_point] +
                0.5 * dt_ *
                    (slip_contact_vel_vars_[knot_point] +
                        slip_contact_vel_vars_[knot_point + 1]));

  }
}

drake::solvers::VectorXDecisionVariable CassieKinematicCentroidalMPC::slip_contact_pos_vars(int knot_point_index,
                                                                                            int slip_foot_index) {
  return slip_contact_pos_vars_[knot_point_index].segment(2 * slip_foot_index, 2);
}
drake::solvers::VectorXDecisionVariable CassieKinematicCentroidalMPC::slip_contact_vel_vars(int knot_point_index,
                                                                                            int slip_foot_index) {
  return slip_contact_vel_vars_[knot_point_index].segment(2 * slip_foot_index, 2);
}

void CassieKinematicCentroidalMPC::SetComPositionGuess(const drake::trajectories::PiecewisePolynomial<double> &com_trajectory) {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    prog_->SetInitialGuess(slip_com_vars_[knot_point],
                           slip_index_.unaryExpr(com_trajectory.value(dt_ * knot_point)));
  }
  KinematicCentroidalMPC::SetComPositionGuess(com_trajectory);
}

void CassieKinematicCentroidalMPC::SetRobotStateGuess(const drake::trajectories::PiecewisePolynomial<double> &q_traj,
                                                      const drake::trajectories::PiecewisePolynomial<double> &v_traj) {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    dairlib::multibody::SetPositionsIfNew<double>(plant_, q_traj.value(dt_*knot_point),contexts_[knot_point].get());
    dairlib::multibody::SetVelocitiesIfNew<double>(plant_, v_traj.value(dt_*knot_point),contexts_[knot_point].get());
    for(int contact = 0; contact < slip_contact_points_.size(); contact++){
      prog_->SetInitialGuess(slip_contact_pos_vars(knot_point, contact),
                             slip_index_.unaryExpr(slip_contact_points_[contact].EvalFull(*contexts_[knot_point])));
      prog_->SetInitialGuess(slip_contact_vel_vars(knot_point, contact),
                             slip_index_.unaryExpr(slip_contact_points_[contact].EvalFullTimeDerivative(*contexts_[knot_point])));
    }
  }
  KinematicCentroidalMPC::SetRobotStateGuess(q_traj, v_traj);
}
void CassieKinematicCentroidalMPC::SetMomentumGuess(const drake::trajectories::PiecewisePolynomial<double> &momentum_trajectory) {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    prog_->SetInitialGuess(slip_vel_vars_[knot_point],
                           slip_index_.unaryExpr(momentum_trajectory.value(dt_ * knot_point))/m_);
  }
  KinematicCentroidalMPC::SetMomentumGuess(momentum_trajectory);
}
drake::VectorX<double> CassieKinematicCentroidalMPC::LiftSlipSolution(int knot_point) {
  drake::VectorX<double> slip_state(2 + 2 + 4 * slip_contact_points_.size());
  slip_state<<result_->GetSolution(slip_com_vars_[knot_point]),
      result_->GetSolution(slip_vel_vars_[knot_point]),
      result_->GetSolution(slip_contact_pos_vars_[knot_point]),
      result_->GetSolution(slip_contact_vel_vars_[knot_point]);

 return lifters_[knot_point]->Lift(slip_state).tail(n_q_+n_v_);
}
