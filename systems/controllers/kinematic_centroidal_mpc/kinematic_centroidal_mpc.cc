#include "kinematic_centroidal_mpc.h"

#include <drake/common/yaml/yaml_io.h>

//#include "multibody/multibody_utils.h"
#include "common/eigen_utils.h"
#include "examples/Cassie/kinematic_centroidal_planner/cassie_kinematic_centroidal_solver.h"
#include "examples/Cassie/kinematic_centroidal_planner/cassie_reference_utils.h"
#include "systems/framework/output_vector.h"
#include "systems/trajectory_optimization/kinematic_centroidal_planner/gait.h"
#include "systems/trajectory_optimization/kinematic_centroidal_planner/kcmpc_reference_generator.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"
//#include <iostream>

using dairlib::systems::BasicVector;
using dairlib::systems::OutputVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::Trajectory;
using Eigen::VectorXd;

namespace dairlib {

KinematicCentroidalMPC::KinematicCentroidalMPC(
    const drake::multibody::MultibodyPlant<double>& plant_w_spr,
    const drake::multibody::MultibodyPlant<double>& plant_wo_spr,
    drake::systems::Context<double>* context, const TrajectoryParameters& motion,
    const KinematicCentroidalGains& gains)
    : plant_w_spr_(plant_w_spr),
      plant_wo_spr_(plant_wo_spr),
      context_wo_spr_(context),
      world_(plant_wo_spr.world_frame()),
      n_q_(plant_wo_spr.num_positions()),
      n_v_(plant_wo_spr.num_velocities()) {
  this->set_name("kinematic_centroidal_planner");

  // Create gaits
  auto stand = drake::yaml::LoadYamlFile<Gait>(
      "examples/Cassie/kinematic_centroidal_planner/gaits/stand.yaml");
  auto walk = drake::yaml::LoadYamlFile<Gait>(
      "examples/Cassie/kinematic_centroidal_planner/gaits/walk.yaml");
  auto right_step = drake::yaml::LoadYamlFile<Gait>(
      "examples/Cassie/kinematic_centroidal_planner/gaits/right_step.yaml");
  auto left_step = drake::yaml::LoadYamlFile<Gait>(
      "examples/Cassie/kinematic_centroidal_planner/gaits/left_step.yaml");
  auto jump = drake::yaml::LoadYamlFile<Gait>(
      "examples/Cassie/kinematic_centroidal_planner/gaits/jump.yaml");

  gait_library_["stand"] = stand;
  gait_library_["walk"] = walk;
  gait_library_["right_step"] = right_step;
  gait_library_["left_step"] = left_step;
  gait_library_["jump"] = jump;

  for (const auto& gait : motion.gait_sequence) {
    gait_samples_.push_back(gait_library_.at(gait));
  }
  DRAKE_DEMAND(gait_samples_.size() == motion.duration_scaling.size());

  auto time_vector = KcmpcReferenceGenerator::GenerateTimePoints(
      motion.duration_scaling, gait_samples_);

  time_points_ = Eigen::Vector4d(time_vector.data());

  n_knot_points_ = motion.n_knot_points;
  // Create MPC and set gains
  solver_ = std::make_unique<CassieKinematicCentroidalSolver>(
      plant_wo_spr_, n_knot_points_, time_vector.back() / (n_knot_points_ - 1),
      0.4);
  solver_->SetGains(gains);
  solver_->SetMinimumFootClearance(motion.swing_foot_minimum_height);

  reference_generator_ = std::make_unique<KcmpcReferenceGenerator>(
      plant_wo_spr_, context_wo_spr_, CreateContactPoints(plant_wo_spr_, 0));
  reference_state_ = GenerateNominalStand(
      plant_wo_spr_, motion.target_com_height, motion.stance_width, false);
  reference_generator_->SetNominalReferenceConfiguration(
      reference_state_.head(plant_wo_spr_.num_positions()));
  reference_generator_->SetComKnotPoints({time_vector, motion.com_vel_vector});
  reference_generator_->SetGaitSequence({time_vector, gait_samples_});
  reference_generator_->Generate();

  solver_->SetForceTrackingReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          reference_generator_->grf_traj_));
  solver_->SetGenPosReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          reference_generator_->q_trajectory_));
  solver_->SetGenVelReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          reference_generator_->v_trajectory_));
  solver_->SetComReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          reference_generator_->com_trajectory_));
  solver_->SetContactTrackingReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          reference_generator_->contact_traj_));
  solver_->SetConstantMomentumReference(Eigen::VectorXd::Zero(6));
  solver_->SetModeSequence(reference_generator_->contact_sequence_);

  solver_->AddInitialStateConstraint(reference_state_);
  solver_->SetRobotStateGuess(reference_generator_->q_trajectory_,
                              reference_generator_->v_trajectory_);
  solver_->SetComPositionGuess(reference_generator_->com_trajectory_);
  solver_->SetContactGuess(reference_generator_->contact_traj_);
  solver_->SetForceGuess(reference_generator_->grf_traj_);

  {
    drake::solvers::SolverOptions options;
    auto id = drake::solvers::IpoptSolver::id();
    options.SetOption(id, "tol", ipopt_tol_);
    options.SetOption(id, "dual_inf_tol", ipopt_tol_);
    options.SetOption(id, "constr_viol_tol", ipopt_tol_);
    options.SetOption(id, "compl_inf_tol", ipopt_tol_);
    options.SetOption(id, "max_iter", 200);
    options.SetOption(id, "nlp_lower_bound_inf", -1e6);
    options.SetOption(id, "nlp_upper_bound_inf", 1e6);
    options.SetOption(id, "print_timing_statistics", "yes");
    //    options.SetOption(id, "print_level", 5);

    // Set to ignore overall tolerance/dual infeasibility, but terminate when
    // primal feasible and objective fails to increase over 5 iterations.
    options.SetOption(id, "acceptable_compl_inf_tol", ipopt_tol_);
    options.SetOption(id, "acceptable_constr_viol_tol", ipopt_tol_);
    options.SetOption(id, "acceptable_obj_change_tol", 1e-3);
    options.SetOption(id, "acceptable_tol", 1e2);
    options.SetOption(id, "acceptable_iter", 1);

    options.SetOption(id, "warm_start_init_point", "no");
    solver_->Build(options);
  }

  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(
              "x, u, t", OutputVector<double>(plant_w_spr_.num_positions(),
                                              plant_w_spr_.num_velocities(),
                                              plant_w_spr_.num_actuators()))
          .get_index();
  clock_port_ = this->DeclareVectorInputPort("t_clock", BasicVector<double>(1))
                    .get_index();
  target_vel_port_ = this->DeclareVectorInputPort(
                             "v_des", BasicVector<double>(VectorXd::Zero(2)))
                         .get_index();

  DeclarePerStepDiscreteUpdateEvent(
      &KinematicCentroidalMPC::DiscreteVariableUpdate);

  this->DeclareAbstractOutputPort("lcmt_target_trajectory",
                                  &KinematicCentroidalMPC::CalcTraj);
}

EventStatus KinematicCentroidalMPC::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  return EventStatus::Succeeded();
}

void KinematicCentroidalMPC::CalcTraj(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* traj) const {
  const auto robot_output =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  const auto desired_pelvis_vel_xy =
      this->EvalVectorInput(context, target_vel_port_)->get_value();

  VectorXd q_w_spr = robot_output->GetPositions();
  VectorXd v_w_spr = robot_output->GetVelocities();

  VectorXd x_wo_spr(n_q_ + n_v_);
  x_wo_spr << map_position_from_spring_to_no_spring_ * q_w_spr,
      map_velocity_from_spring_to_no_spring_ * v_w_spr;

  std::vector<Eigen::Vector3d> com_vel = {
      {{0, 0, 0},
       {desired_pelvis_vel_xy[0], -desired_pelvis_vel_xy[1], 0},
       {0, 0, 0}}};

  reference_generator_->SetNominalReferenceConfiguration(
      reference_state_.head(plant_wo_spr_.num_positions()));
  reference_generator_->SetGaitSequence(
      {CopyVectorXdToStdVector(time_points_), gait_samples_});
  reference_generator_->SetComKnotPoints(
      {CopyVectorXdToStdVector(time_points_), com_vel});
  reference_generator_->Generate();

  solver_->SetGenPosReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          reference_generator_->q_trajectory_));
  solver_->SetGenVelReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          reference_generator_->v_trajectory_));
  solver_->SetComReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          reference_generator_->com_trajectory_));
  solver_->SetConstantMomentumReference(Eigen::VectorXd::Zero(6));
  solver_->SetContactTrackingReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          reference_generator_->contact_traj_));
  solver_->SetForceTrackingReference(
      std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(
          reference_generator_->grf_traj_));
  solver_->SetModeSequence(reference_generator_->contact_sequence_);

  solver_->UpdateInitialStateConstraint(x_wo_spr);
  solver_->SetRobotStateGuess(reference_generator_->q_trajectory_,
                              reference_generator_->v_trajectory_);
  solver_->SetComPositionGuess(reference_generator_->com_trajectory_);
  solver_->SetContactGuess(reference_generator_->contact_traj_);
  solver_->SetForceGuess(reference_generator_->grf_traj_);
  solver_->UpdateCosts();

  std::cout << "Solving optimization\n\n";
  solver_->Solve();
  traj->saved_traj = solver_->GenerateLcmTraj(n_knot_points_, robot_output->get_timestamp() + 1.0);
  traj->utime = robot_output->get_timestamp() * 1e6;
}

}  // namespace dairlib