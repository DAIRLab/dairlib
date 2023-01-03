#include "systems/trajectory_optimization/kinematic_centroidal_planner/kinematic_centroidal_solver.h"

#include <iostream>
#include <utility>

#include "lcm/lcm_trajectory.h"
#include "multibody/kinematic/kinematic_constraints.h"
#include "multibody/multibody_utils.h"
#include "systems/trajectory_optimization/kinematic_centroidal_planner/kinematic_centroidal_constraints.h"

#include "drake/common/trajectories/piecewise_polynomial.h"

using dairlib::LcmTrajectory;
using drake::AutoDiffXd;
using drake::multibody::MultibodyPlant;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KinematicCentroidalSolver::KinematicCentroidalSolver(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::multibody::MultibodyPlant<AutoDiffXd>& plant_ad,
    int n_knot_points, double dt,
    const std::vector<dairlib::multibody::WorldPointEvaluator<double>>&
        contact_points)
    : plant_(plant),
      plant_ad_(plant_ad),
      n_knot_points_(n_knot_points),
      dt_(dt),
      contact_points_(contact_points),
      n_q_(plant.num_positions()),
      n_v_(plant.num_velocities()),
      n_contact_points_(contact_points.size()),
      Q_q_(Eigen::MatrixXd::Zero(n_q_, n_q_)),
      Q_v_(Eigen::MatrixXd::Zero(n_v_, n_v_)),
      Q_com_(Eigen::MatrixXd::Zero(3, 3)),
      Q_mom_(Eigen::MatrixXd::Zero(6, 6)),
      Q_contact_(
          Eigen::MatrixXd::Zero(6 * n_contact_points_, 6 * n_contact_points_)),
      Q_force_(
          Eigen::MatrixXd::Zero(3 * n_contact_points_, 3 * n_contact_points_)),
      contact_sequence_(n_knot_points),
      contexts_(n_knot_points),
      contexts_ad_(n_knot_points) {
  n_joint_q_ = n_q_ - kCentroidalPosDim;
  n_joint_v_ = n_v_ - kCentroidalVelDim;
  prog_ = std::make_unique<drake::solvers::MathematicalProgram>();
  solver_ = std::make_unique<drake::solvers::IpoptSolver>();
  result_ = std::make_unique<drake::solvers::MathematicalProgramResult>();
  for (int contact_index = 0; contact_index < n_contact_points_;
       contact_index++) {
    contact_sets_.emplace_back(plant_);
    contact_sets_[contact_index].add_evaluator(&contact_points_[contact_index]);
  }

  for (int knot = 0; knot < n_knot_points; knot++) {
    contexts_[knot] = plant_.CreateDefaultContext();
    contexts_ad_[knot] = plant_ad_.CreateDefaultContext();
    x_vars_.push_back(prog_->NewContinuousVariables(
        n_q_ + n_v_, "x_vars_" + std::to_string(knot)));
    mom_vars_.push_back(
        prog_->NewContinuousVariables(6, "mom_vars_" + std::to_string(knot)));
    com_vars_.push_back(
        prog_->NewContinuousVariables(3, "com_vars_" + std::to_string(knot)));
    contact_pos_.push_back(prog_->NewContinuousVariables(
        3 * n_contact_points_, "contact_pos_" + std::to_string(knot)));
    contact_vel_.push_back(prog_->NewContinuousVariables(
        3 * n_contact_points_, "contact_vel_" + std::to_string(knot)));
    contact_force_.push_back(prog_->NewContinuousVariables(
        3 * n_contact_points_, "contact_force_" + std::to_string(knot)));
  }
  std::vector<bool> stance_mode(n_contact_points_);
  std::fill(stance_mode.begin(), stance_mode.end(), true);
  std::fill(contact_sequence_.begin(), contact_sequence_.end(), stance_mode);
}

void KinematicCentroidalSolver::SetGenPosReference(
    std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj) {
  q_ref_traj_ = std::move(ref_traj);
}

void KinematicCentroidalSolver::SetGenVelReference(
    std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj) {
  v_ref_traj_ = std::move(ref_traj);
}

void KinematicCentroidalSolver::SetComReference(
    std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj) {
  com_ref_traj_ = std::move(ref_traj);
}

void KinematicCentroidalSolver::SetContactTrackingReference(
    std::unique_ptr<drake::trajectories::Trajectory<double>> contact_ref_traj) {
  contact_ref_traj_ = std::move(contact_ref_traj);
}

void KinematicCentroidalSolver::SetForceTrackingReference(
    std::unique_ptr<drake::trajectories::Trajectory<double>> force_ref_traj) {
  force_ref_traj_ = std::move(force_ref_traj);
}

void KinematicCentroidalSolver::SetMomentumReference(
    std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj) {
  mom_ref_traj_ = std::move(ref_traj);
}

void KinematicCentroidalSolver::AddCentroidalDynamics() {
  for (int knot_point = 0; knot_point < n_knot_points_ - 1; knot_point++) {
    auto constraint = std::make_shared<CentroidalDynamicsConstraint<double>>(
        plant_, contexts_[knot_point].get(), n_contact_points_, dt_,
        knot_point);
    //    auto constraint =
    //    std::make_shared<CentroidalDynamicsConstraint<AutoDiffXd>>(
    //        *plant_ad_, contexts_ad_[knot_point].get(), n_contact_points_,
    //        dt_, knot_point);
    centroidal_dynamics_binding_.push_back(prog_->AddConstraint(
        constraint,
        {momentum_vars(knot_point), momentum_vars(knot_point + 1),
         com_pos_vars(knot_point), contact_pos_[knot_point],
         contact_force_[knot_point], com_pos_vars(knot_point + 1),
         contact_pos_[knot_point + 1], contact_force_[knot_point + 1]}));
  }
}

void KinematicCentroidalSolver::AddKinematicsIntegrator() {
  for (int knot_point = 0; knot_point < n_knot_points_ - 1; knot_point++) {
    // Integrate generalized velocities to get generalized positions
    auto constraint = std::make_shared<KinematicIntegratorConstraint<double>>(
        plant_, contexts_[knot_point].get(), contexts_[knot_point + 1].get(),
        dt_, knot_point);
    prog_->AddConstraint(constraint, {state_vars(knot_point).head(n_q_),
                                      state_vars(knot_point + 1).head(n_q_),
                                      state_vars(knot_point).tail(n_v_),
                                      state_vars(knot_point + 1).tail(n_v_)});

    // Integrate foot states
    for (int contact_index = 0; contact_index < n_contact_points_;
         contact_index++) {
      prog_->AddConstraint(
          contact_pos_vars(knot_point + 1, contact_index) ==
          contact_pos_vars(knot_point, contact_index) +
              0.5 * dt_ *
                  (contact_vel_vars(knot_point, contact_index) +
                   contact_vel_vars(knot_point + 1, contact_index)));
    }
  }
}

void KinematicCentroidalSolver::AddContactConstraints() {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    for (int contact_index = 0; contact_index < n_contact_points_;
         contact_index++) {
      // Make sure feet in stance are not moving and on the ground
      if (contact_sequence_[knot_point][contact_index]) {
        prog_->AddBoundingBoxConstraint(
            Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3),
            contact_vel_vars(knot_point, contact_index));
        if (knot_point != 0) {
          prog_->AddBoundingBoxConstraint(
              0, 0, contact_pos_vars(knot_point, contact_index)[2]);
        }
      } else {
        // Feet are above the ground
        double lb = 0;
        // Check if at least one of the time points before or after is also in
        // flight before restricting the foot to be in the air to limit over
        // constraining the optimization problem
        if (!is_first_knot(knot_point) and !is_last_knot(knot_point) and
            (!contact_sequence_[knot_point - 1][contact_index] or
             !contact_sequence_[knot_point + 1][contact_index])) {
          lb = swing_foot_minimum_height_;
        }
        prog_->AddBoundingBoxConstraint(
            lb, 10, contact_pos_vars(knot_point, contact_index)[2]);
      }
    }
  }
}

void KinematicCentroidalSolver::AddCentroidalKinematicConsistency() {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    // Ensure linear and angular momentum line up
//    auto centroidal_momentum =
//        std::make_shared<CentroidalMomentumConstraint<double>>(
//            plant_, contexts_[knot_point].get(), knot_point);
    auto centroidal_momentum =
        std::make_shared<CentroidalMomentumConstraint<AutoDiffXd>>(
            plant_ad_, contexts_ad_[knot_point].get(), knot_point);
    prog_->AddConstraint(centroidal_momentum,
                         {state_vars(knot_point), momentum_vars(knot_point)});
    for (int contact_index = 0; contact_index < n_contact_points_;
         contact_index++) {
      // Ensure foot position line up with kinematics
      auto foot_position_constraint = std::make_shared<
          dairlib::multibody::KinematicPositionConstraint<double>>(
          plant_, contact_sets_[contact_index], Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero(), full_constraint_relative_);
      prog_->AddConstraint(foot_position_constraint,
                           {state_vars(knot_point).head(n_q_),
                            contact_pos_vars(knot_point, contact_index)});
    }
    // Constrain com position
    //    auto com_position =
    //        std::make_shared<CenterofMassPositionConstraint<double>>(
    //            plant_, contexts_[knot_point].get(), knot_point);
    //    auto com_position =
    //        std::make_shared<CenterofMassPositionConstraint<drake::AutoDiffXd>>(
    //            plant_, contexts_[knot_point].get(), knot_point);
    auto com_position =
        std::make_shared<CenterofMassPositionConstraint<drake::AutoDiffXd>>(
            plant_ad_, contexts_ad_[knot_point].get(), knot_point);
    prog_->AddConstraint(com_position,
                         {com_pos_vars(knot_point), state_vars(knot_point)});
  }
}

void KinematicCentroidalSolver::AddFrictionConeConstraints() {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    for (int contact_index = 0; contact_index < n_contact_points_;
         contact_index++) {
      auto force_constraints_vec =
          contact_points_[contact_index].CreateLinearFrictionConstraints();
      for (const auto& constraint : force_constraints_vec) {
        prog_->AddConstraint(constraint,
                             contact_force_vars(knot_point, contact_index));
      }
    }
  }
}

void KinematicCentroidalSolver::AddFlightContactForceConstraints() {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    for (int contact_index = 0; contact_index < n_contact_points_;
         contact_index++) {
      // Feet in flight produce no force
      if (!contact_sequence_[knot_point][contact_index]) {
        prog_->AddBoundingBoxConstraint(
            Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3),
            contact_force_vars(knot_point, contact_index));
      }
    }
  }
}

drake::solvers::VectorXDecisionVariable KinematicCentroidalSolver::state_vars(
    int knotpoint_index) const {
  return x_vars_[knotpoint_index];
}
drake::solvers::VectorXDecisionVariable
KinematicCentroidalSolver::joint_pos_vars(int knotpoint_index) const {
  return x_vars_[knotpoint_index].segment(kCentroidalPosDim, n_joint_q_);
}
drake::solvers::VectorXDecisionVariable
KinematicCentroidalSolver::joint_vel_vars(int knotpoint_index) const {
  return x_vars_[knotpoint_index].segment(n_q_ + kCentroidalVelDim, n_joint_v_);
}

void KinematicCentroidalSolver::Build(
    const drake::solvers::SolverOptions& solver_options) {
  AddCentroidalDynamics();
  AddKinematicsIntegrator();
  AddContactConstraints();
  AddCentroidalKinematicConsistency();
  AddFrictionConeConstraints();
  AddFlightContactForceConstraints();
  AddCosts();
  prog_->SetSolverOptions(solver_options);
}

void KinematicCentroidalSolver::SetConstantMomentumReference(
    const drake::VectorX<double>& value) {
  SetMomentumReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          value));
}

double KinematicCentroidalSolver::GetKnotpointGain(int knot_point) const {
  const double terminal_gain = is_last_knot(knot_point) ? 100 : 1;
  const double collocation_gain =
      (is_first_knot(knot_point) or is_last_knot(knot_point)) ? 0.5 : 1;
  return terminal_gain * collocation_gain;
}

void KinematicCentroidalSolver::AddCosts() {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    double knot_point_gain = GetKnotpointGain(knot_point);
    double t = dt_ * knot_point;
    if (q_ref_traj_) {
      q_ref_cost_.push_back(
          prog_
              ->AddQuadraticErrorCost(knot_point_gain * Q_q_,
                                      q_ref_traj_->value(t),
                                      state_vars(knot_point).head(n_q_))
              .evaluator());
    }
    if (v_ref_traj_) {
      v_ref_cost_.push_back(
          prog_
              ->AddQuadraticErrorCost(knot_point_gain * Q_v_,
                                      v_ref_traj_->value(t),
                                      state_vars(knot_point).tail(n_v_))
              .evaluator());
    }
    if (com_ref_traj_) {
      com_ref_cost_.push_back(
          prog_
              ->AddQuadraticErrorCost(knot_point_gain * Q_com_,
                                      com_ref_traj_->value(t),
                                      com_pos_vars(knot_point))
              .evaluator());
    }
    if (mom_ref_traj_) {
      mom_ref_cost_.push_back(
          prog_
              ->AddQuadraticErrorCost(knot_point_gain * Q_mom_,
                                      mom_ref_traj_->value(t),
                                      momentum_vars(knot_point))
              .evaluator());
    }
    if (contact_ref_traj_) {
      contact_ref_cost_.push_back(
          prog_
              ->AddQuadraticErrorCost(
                  knot_point_gain * Q_contact_, contact_ref_traj_->value(t),
                  {contact_pos_[knot_point], contact_vel_[knot_point]})
              .evaluator());
    }
    if (force_ref_traj_) {
      force_ref_cost_.push_back(
          prog_
              ->AddQuadraticErrorCost(knot_point_gain * Q_force_,
                                      force_ref_traj_->value(t),
                                      contact_force_[knot_point])
              .evaluator());
    }
  }
}

void KinematicCentroidalSolver::UpdateCosts() {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    const double knot_point_gain = GetKnotpointGain(knot_point);
    double t = dt_ * knot_point;
    if (q_ref_cost_[knot_point]) {
      q_ref_cost_[knot_point]->UpdateCoefficients(
          knot_point_gain * 2 * Q_q_,
          -knot_point_gain * 2 * Q_q_ * q_ref_traj_->value(t));
    }
    if (v_ref_cost_[knot_point]) {
      v_ref_cost_[knot_point]->UpdateCoefficients(
          knot_point_gain * 2 * Q_v_,
          -knot_point_gain * 2 * Q_v_ * v_ref_traj_->value(t));
    }
    if (com_ref_cost_[knot_point]) {
      com_ref_cost_[knot_point]->UpdateCoefficients(
          knot_point_gain * 2 * Q_com_,
          -knot_point_gain * 2 * Q_com_ * com_ref_traj_->value(t));
    }
    if (mom_ref_cost_[knot_point]) {
      mom_ref_cost_[knot_point]->UpdateCoefficients(
          knot_point_gain * 2 * Q_mom_,
          -knot_point_gain * 2 * Q_mom_ * mom_ref_traj_->value(t));
    }
    if (contact_ref_cost_[knot_point]) {
      contact_ref_cost_[knot_point]->UpdateCoefficients(
          knot_point_gain * 2 * Q_contact_,
          -knot_point_gain * 2 * Q_contact_ * contact_ref_traj_->value(t));
    }
    if (force_ref_cost_[knot_point]) {
      force_ref_cost_[knot_point]->UpdateCoefficients(
          knot_point_gain * 2 * Q_force_,
          -knot_point_gain * 2 * Q_force_ * force_ref_traj_->value(t));
    }
  }
}

void KinematicCentroidalSolver::SetZeroInitialGuess() {
  Eigen::VectorXd initialGuess =
      Eigen::VectorXd::Zero(n_q_ + n_v_ + n_contact_points_ * 9 + 6 + 3);
  prog_->SetInitialGuessForAllVariables(
      initialGuess.replicate(n_knot_points_, 1));
  // Make sure unit quaternions
  for (int i = 0; i < n_knot_points_; i++) {
    auto xi = state_vars(i);
    prog_->SetInitialGuess(xi(0), 1);
    prog_->SetInitialGuess(xi(1), 0);
    prog_->SetInitialGuess(xi(2), 0);
    prog_->SetInitialGuess(xi(3), 0);
  }
}

drake::trajectories::PiecewisePolynomial<double>
KinematicCentroidalSolver::Solve() {
  for (int i = 0; i < 1; i++) {
    auto start = std::chrono::high_resolution_clock::now();
    solver_->Solve(*prog_, prog_->initial_guess(), prog_->solver_options(),
                   result_.get());
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    solve_time_ = elapsed.count();
    std::cout << "Solve time:" << elapsed.count() << std::endl;
    std::cout << "Cost:" << result_->get_optimal_cost() << std::endl;
    prog_->SetInitialGuessForAllVariables(result_->GetSolution());
  }

  std::vector<double> time_points;
  std::vector<drake::MatrixX<double>> states;
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    time_points.emplace_back(dt_ * knot_point);
    states.emplace_back(result_->GetSolution(state_vars(knot_point)));
  }
  return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      time_points, states);
}

void KinematicCentroidalSolver::SerializeSolution(int n_knot_points,
                                                  double time_offset) {
  DRAKE_DEMAND(result_->is_success());

  Eigen::MatrixXd state_points;
  Eigen::MatrixXd contact_force_points;
  std::vector<double> time_samples;
  this->SetFromSolution(*result_, &state_points, &contact_force_points,
                        &time_samples);
  for (auto& t : time_samples) {
    t += time_offset;
  }

  int knot_points_to_serialize =
      n_knot_points == -1 ? time_samples.size() : n_knot_points;

  dairlib::LcmTrajectory::Trajectory state_traj;
  dairlib::LcmTrajectory::Trajectory contact_force_traj;
  state_traj.traj_name = "state_traj";
  state_traj.datapoints = state_points.leftCols(knot_points_to_serialize);
  state_traj.time_vector =
      Eigen::Map<VectorXd>(time_samples.data(), knot_points_to_serialize);
  state_traj.datatypes =
      dairlib::multibody::CreateStateNameVectorFromMap(plant_);

  contact_force_traj.traj_name = "contact_force_traj";
  contact_force_traj.datapoints =
      contact_force_points.leftCols(knot_points_to_serialize);
  contact_force_traj.time_vector =
      //      Eigen::Map<VectorXd>(time_samples.data(), time_samples.size());
      Eigen::Map<VectorXd>(time_samples.data(), knot_points_to_serialize);
  contact_force_traj.datatypes =
      std::vector<std::string>(contact_force_traj.datapoints.rows());

  std::vector<dairlib::LcmTrajectory::Trajectory> trajectories;
  trajectories.push_back(state_traj);
  trajectories.push_back(contact_force_traj);
  std::vector<std::string> trajectory_names = {state_traj.traj_name,
                                               contact_force_traj.traj_name};
  lcm_trajectory_ =
      LcmTrajectory(trajectories, trajectory_names, "centroidal_mpc_solution",
                    "centroidal_mpc_solution");
}

dairlib::lcmt_saved_traj KinematicCentroidalSolver::GenerateLcmTraj(
    int n_knot_points, double time_offset) {
  SerializeSolution(n_knot_points, time_offset);
  return lcm_trajectory_.GenerateLcmObject();
}

void KinematicCentroidalSolver::PublishSolution(const std::string& lcm_channel,
                                                int n_knot_points) {
  if (!lcm_) {
    lcm_ = std::make_unique<drake::lcm::DrakeLcm>();
  }
  auto lcm_msg = GenerateLcmTraj(n_knot_points);
  drake::lcm::Publish(lcm_.get(), lcm_channel, lcm_msg);
}

bool KinematicCentroidalSolver::SaveSolutionToFile(
    const std::string& filepath) {
  SerializeSolution(-1);
  lcm_trajectory_.WriteToFile(filepath);
  return true;
}

void KinematicCentroidalSolver::SetFromSolution(
    const drake::solvers::MathematicalProgramResult& result,
    Eigen::MatrixXd* state_samples, Eigen::MatrixXd* contact_force_samples,
    std::vector<double>* time_samples) const {
  DRAKE_ASSERT(state_samples != nullptr);
  DRAKE_ASSERT(contact_force_samples != nullptr);
  DRAKE_ASSERT(time_samples->empty());

  *state_samples = MatrixXd(n_q_ + n_v_, n_knot_points_);
  *contact_force_samples = MatrixXd(3 * n_contact_points_, n_knot_points_);
  time_samples->resize(n_knot_points_);

  for (int knot_point = 0; knot_point < num_knot_points(); knot_point++) {
    time_samples->at(knot_point) = knot_point * dt_;

    VectorXd x = result.GetSolution(state_vars(knot_point));
    VectorXd contact_force = result.GetSolution(contact_force_.at(knot_point));
    state_samples->col(knot_point) = x;
    contact_force_samples->col(knot_point) = contact_force;
  }
}

void KinematicCentroidalSolver::CreateVisualizationCallback(
    const std::string& model_file, double alpha,
    const std::string& weld_frame_to_world) {
  DRAKE_DEMAND(!callback_visualizer_);  // Cannot be set twice

  // Assemble variable list
  drake::solvers::VectorXDecisionVariable vars(n_knot_points_ / 2 * n_q_);
  for (int knot_point = 0; knot_point < n_knot_points_ / 2; knot_point++) {
    vars.segment(knot_point * n_q_, n_q_) =
        state_vars(knot_point * 2).head(n_q_);
  }
  Eigen::VectorXd alpha_vec =
      Eigen::VectorXd::Constant(n_knot_points_ / 2, alpha);
  alpha_vec(0) = 1;
  alpha_vec(n_knot_points_ / 2 - 1) = 1;

  // Create visualizer
  callback_visualizer_ =
      std::make_unique<dairlib::multibody::MultiposeVisualizer>(
          model_file, n_knot_points_ / 2, alpha_vec, weld_frame_to_world);

  // Callback lambda function
  auto my_callback = [this](const Eigen::Ref<const Eigen::VectorXd>& vars) {
    Eigen::VectorXd vars_copy = vars;
    Eigen::Map<Eigen::MatrixXd> states(vars_copy.data(), this->n_q_,
                                       this->n_knot_points_ / 2);
    this->callback_visualizer_->DrawPoses(states);
  };

  prog_->AddVisualizationCallback(my_callback, vars);
}
void KinematicCentroidalSolver::AddContactPointPositionConstraint(
    int contact_index, const Eigen::Vector3d& lb, const Eigen::Vector3d& ub) {
  //{TODO} eventually make in body frame
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    prog_->AddBoundingBoxConstraint(
        lb, ub, contact_pos_vars(knot_point, contact_index));
  }
}
void KinematicCentroidalSolver::AddPlantJointLimits(
    const std::vector<std::string>& joint_to_constrain) {
  std::map<std::string, int> positions_map =
      dairlib::multibody::MakeNameToPositionsMap(plant_);
  for (const auto& member : joint_to_constrain) {
    for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
      prog_->AddBoundingBoxConstraint(
          plant_.GetJointByName(member).position_lower_limits()(0),
          plant_.GetJointByName(member).position_upper_limits()(0),
          state_vars(knot_point)[positions_map.at(member)]);
    }
  }
}

drake::solvers::VectorXDecisionVariable KinematicCentroidalSolver::com_pos_vars(
    int knotpoint_index) const {
  return com_vars_[knotpoint_index];
}
drake::solvers::VectorXDecisionVariable
KinematicCentroidalSolver::contact_pos_vars(int knotpoint_index,
                                            int contact_index) const {
  return contact_pos_[knotpoint_index].segment(contact_index * 3, 3);
}
drake::solvers::VectorXDecisionVariable
KinematicCentroidalSolver::contact_vel_vars(int knotpoint_index,
                                            int contact_index) const {
  return contact_vel_[knotpoint_index].segment(contact_index * 3, 3);
}
drake::solvers::VectorXDecisionVariable
KinematicCentroidalSolver::contact_force_vars(int knotpoint_index,
                                              int contact_index) const {
  return contact_force_[knotpoint_index].segment(contact_index * 3, 3);
}
void KinematicCentroidalSolver::AddKinematicConstraint(
    std::shared_ptr<dairlib::multibody::KinematicPositionConstraint<double>>
        con,
    const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& vars) {
  prog_->AddConstraint(std::move(con), vars);
}
void KinematicCentroidalSolver::SetRobotStateGuess(
    const drake::VectorX<double>& state) {
  DRAKE_DEMAND(state.size() == n_q_ + n_v_);

  for (const auto& state_vars : x_vars_) {
    prog_->SetInitialGuess(state_vars, state);
  }
}

void KinematicCentroidalSolver::SetRobotStateGuess(
    const drake::trajectories::PiecewisePolynomial<double>& state_trajectory) {
  DRAKE_DEMAND(state_trajectory.rows() == n_q_ + n_v_);
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    prog_->SetInitialGuess(state_vars(knot_point),
                           state_trajectory.value(dt_ * knot_point));
  }
}

void KinematicCentroidalSolver::SetRobotStateGuess(
    const drake::trajectories::PiecewisePolynomial<double>& q_traj,
    const drake::trajectories::PiecewisePolynomial<double>& v_traj) {
  DRAKE_DEMAND(q_traj.rows() == n_q_);
  DRAKE_DEMAND(v_traj.rows() == n_v_);
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    prog_->SetInitialGuess(state_vars(knot_point).head(n_q_),
                           q_traj.value(dt_ * knot_point));
    prog_->SetInitialGuess(state_vars(knot_point).tail(n_v_),
                           v_traj.value(dt_ * knot_point));
  }
}

drake::solvers::VectorXDecisionVariable
KinematicCentroidalSolver::momentum_vars(int knotpoint_index) const {
  return mom_vars_[knotpoint_index];
}
void KinematicCentroidalSolver::AddComHeightBoundingConstraint(double lb,
                                                               double ub) {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    prog_->AddBoundingBoxConstraint(lb, ub, com_pos_vars(knot_point)[2]);
  }
}
void KinematicCentroidalSolver::SetComPositionGuess(
    const drake::Vector3<double>& state) {
  for (const auto& com_pos : com_vars_) {
    prog_->SetInitialGuess(com_pos, state);
  }
}

void KinematicCentroidalSolver::SetComPositionGuess(
    const drake::trajectories::PiecewisePolynomial<double>& com_trajectory) {
  DRAKE_DEMAND(com_trajectory.rows() == 3);
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    prog_->SetInitialGuess(com_pos_vars(knot_point),
                           com_trajectory.value(dt_ * knot_point));
  }
}

void KinematicCentroidalSolver::SetContactGuess(
    const drake::trajectories::PiecewisePolynomial<double>&
        contact_trajectory) {
  DRAKE_DEMAND(contact_trajectory.rows() == 6 * n_contact_points_);
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    prog_->SetInitialGuess(
        contact_pos_[knot_point],
        drake::VectorX<double>(contact_trajectory.value(dt_ * knot_point))
            .head(3 * n_contact_points_));
    prog_->SetInitialGuess(
        contact_vel_[knot_point],
        drake::VectorX<double>(contact_trajectory.value(dt_ * knot_point))
            .tail(3 * n_contact_points_));
  }
}
void KinematicCentroidalSolver::SetForceGuess(
    const drake::trajectories::PiecewisePolynomial<double>& force_trajectory) {
  DRAKE_DEMAND(force_trajectory.rows() == 3 * n_contact_points_);
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    prog_->SetInitialGuess(contact_force_[knot_point],
                           force_trajectory.value(dt_ * knot_point));
  }
}

void KinematicCentroidalSolver::SetModeSequence(
    const std::vector<std::vector<bool>>& contact_sequence) {
  contact_sequence_ = contact_sequence;
}
void KinematicCentroidalSolver::SetModeSequence(
    const drake::trajectories::PiecewisePolynomial<double>& contact_sequence) {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    for (int contact_index = 0; contact_index < 4; contact_index++) {
      contact_sequence_[knot_point][contact_index] =
          contact_sequence.value(dt_ * knot_point).coeff(contact_index);
    }
  }
}

void KinematicCentroidalSolver::AddInitialStateConstraint(
    const Eigen::VectorXd& state) {
  DRAKE_DEMAND(state.size() == state_vars(0).size());
  init_state_constraint_ =
      prog_->AddBoundingBoxConstraint(state, state, state_vars((0)))
          .evaluator()
          .get();
}

void KinematicCentroidalSolver::UpdateInitialStateConstraint(
    const Eigen::VectorXd& state) {
  DRAKE_DEMAND(state.size() == state_vars(0).size());
  init_state_constraint_->set_bounds(state, state);
}

void KinematicCentroidalSolver::SetGains(
    const KinematicCentroidalGains& gains) {
  std::map<std::string, int> positions_map =
      dairlib::multibody::MakeNameToPositionsMap(plant_);
  std::map<std::string, int> velocities_map =
      dairlib::multibody::MakeNameToVelocitiesMap(plant_);

  // Generalize state
  for (const auto& [name, index] : positions_map) {
    const auto it = gains.generalized_positions.find(name);
    if (it != gains.generalized_positions.end()) {
      Q_q_(index, index) = it->second;
    } else if (name.find("left") != std::string::npos) {
      const auto it_left =
          gains.generalized_positions.find(name.substr(0, name.size() - 4));
      if (it_left != gains.generalized_positions.end()) {
        Q_q_(index, index) = it_left->second;
      }
    } else if (name.find("right") != std::string::npos) {
      const auto it_right =
          gains.generalized_positions.find(name.substr(0, name.size() - 5));
      if (it_right != gains.generalized_positions.end()) {
        Q_q_(index, index) = it_right->second;
      }
    }
  }

  for (const auto& [name, index] : velocities_map) {
    const auto it = gains.generalized_velocities.find(name);
    if (it != gains.generalized_velocities.end()) {
      Q_v_(index, index) = it->second;
    } else if (name.find("left") != std::string::npos) {
      const auto it_left =
          gains.generalized_velocities.find(name.substr(0, name.size() - 7));
      if (it_left != gains.generalized_velocities.end()) {
        Q_v_(index, index) = it_left->second;
      }
    } else if (name.find("right") != std::string::npos) {
      const auto it_right =
          gains.generalized_velocities.find(name.substr(0, name.size() - 8));
      if (it_right != gains.generalized_velocities.end()) {
        Q_v_(index, index) = it_right->second;
      }
    }
  }

  // com
  Q_com_ = gains.com_position.asDiagonal();

  // momentum
  Q_mom_.block<3, 3>(0, 0, 3, 3) = gains.ang_momentum.asDiagonal();
  Q_mom_.block<3, 3>(3, 3, 3, 3) = gains.lin_momentum.asDiagonal();

  // contact pos, contact vel
  Q_contact_.block<Eigen::Dynamic, Eigen::Dynamic>(0, 0, 3 * n_contact_points_,
                                                   3 * n_contact_points_) =
      gains.contact_pos.replicate(n_contact_points_, 1).asDiagonal();
  Q_contact_.block<Eigen::Dynamic, Eigen::Dynamic>(
      3 * n_contact_points_, 3 * n_contact_points_, 3 * n_contact_points_,
      3 * n_contact_points_) =
      gains.contact_vel.replicate(n_contact_points_, 1).asDiagonal();

  // contact force
  Q_force_ = gains.contact_force.replicate(n_contact_points_, 1).asDiagonal();
}
