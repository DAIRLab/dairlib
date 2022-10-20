#include <iostream>
#include <utility>
#include "kinematic_centroidal_mpc.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "systems/controllers/kinematic_centroidal_mpc/kinematic_centroidal_constraints.h"
#include "multibody/kinematic/kinematic_constraints.h"
#include "multibody/multibody_utils.h"
#include "lcm/lcm_trajectory.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using dairlib::LcmTrajectory;

KinematicCentroidalMPC::KinematicCentroidalMPC(const drake::multibody::MultibodyPlant<double> &plant,
                                               int n_knot_points,
                                               double dt,
                                               const std::vector<std::shared_ptr<dairlib::multibody::WorldPointEvaluator<double>>>& contact_points):
                                               plant_(plant),
                                               n_knot_points_(n_knot_points),
                                               dt_(dt),
                                               contact_points_(contact_points),
                                               n_q_(plant.num_positions()),
                                               n_v_(plant.num_velocities()),
                                               n_contact_points_(contact_points.size()),
                                               contexts_(n_knot_points){

  n_joint_q_ = n_q_ - kCentroidalPosDim;
  n_joint_v_ = n_v_ - kCentroidalVelDim;
  prog_ = std::make_unique<drake::solvers::MathematicalProgram>();
  solver_ = std::make_unique<drake::solvers::IpoptSolver>();
  result_ = std::make_unique<drake::solvers::MathematicalProgramResult>();
  for(int contact_index = 0; contact_index < n_contact_points_; contact_index ++){
    contact_sets_.emplace_back(plant_);
    contact_sets_[contact_index].add_evaluator(contact_points_[contact_index].get());
  }

  for(int knot = 0; knot < n_knot_points; knot ++){
    contexts_[knot] = plant_.CreateDefaultContext();
    x_vars_.push_back(prog_->NewContinuousVariables(n_q_ + n_v_, "x_vars_" + std::to_string(knot)));
    mom_vars_.push_back(prog_->NewContinuousVariables(6, "mom_vars_" + std::to_string(knot)));
    com_vars_.push_back(prog_->NewContinuousVariables(3, "com_vars_" + std::to_string(knot)));
    contact_pos_.push_back(prog_->NewContinuousVariables(3 * n_contact_points_, "contact_pos_" + std::to_string(knot)));
    contact_vel_.push_back(prog_->NewContinuousVariables(3 * n_contact_points_, "contact_vel_" + std::to_string(knot)));
    contact_force_.push_back(prog_->NewContinuousVariables(3 * n_contact_points_, "contact_force_" + std::to_string(knot)));
  }
}

void KinematicCentroidalMPC::AddStateReferenceCost(std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj,
                                                   const Eigen::MatrixXd &Q) {
  // Ensure matrix is square and has correct number of rows and columns
  DRAKE_DEMAND(Q.rows() == n_q_ + n_v_);
  DRAKE_DEMAND(Q.cols() == n_q_ + n_v_);

  ref_traj_ = std::move(ref_traj);
  Q_ = Q;
}

void KinematicCentroidalMPC::AddContactTrackingReferenceCost(std::unique_ptr<drake::trajectories::Trajectory<double>> contact_ref_traj,
                                                             const Eigen::MatrixXd &Q_contact) {
  DRAKE_DEMAND(Q_contact.rows() == 2 * 3 * n_contact_points_);
  DRAKE_DEMAND(Q_contact.cols() == 2 * 3 * n_contact_points_);

  contact_ref_traj_ = std::move(contact_ref_traj);
  Q_contact_ = Q_contact;
}

void KinematicCentroidalMPC::AddForceTrackingReferenceCost(std::unique_ptr<drake::trajectories::Trajectory<double>> force_ref_traj,
                                                           const Eigen::MatrixXd &Q_force) {
  DRAKE_DEMAND(Q_force.rows() == 3 * n_contact_points_);
  DRAKE_DEMAND(Q_force.cols() == 3 * n_contact_points_);

  force_ref_traj_ = std::move(force_ref_traj);
  Q_force_ = Q_force;
}

void KinematicCentroidalMPC::AddComReferenceCost(std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj,
                                                 const Eigen::MatrixXd &Q) {
  DRAKE_DEMAND(Q.rows() == 3);
  DRAKE_DEMAND(Q.cols() == 3);

  com_ref_traj_ = std::move(ref_traj);
  Q_com_ = Q;
}

void KinematicCentroidalMPC::AddMomentumReferenceCost(std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj,
                                                      const Eigen::MatrixXd &Q) {
  DRAKE_DEMAND(Q.rows() == 6);
  DRAKE_DEMAND(Q.cols() == 6);
  mom_ref_traj_ = std::move(ref_traj);
  Q_mom_ = Q;
}

void KinematicCentroidalMPC::AddCentroidalDynamics() {
  for(int knot_point = 0; knot_point < n_knot_points_ - 1; knot_point ++){
    auto constraint = std::make_shared<CentroidalDynamicsConstraint<double>>(
        plant_, contexts_[knot_point].get(), n_contact_points_, dt_,knot_point);
    centroidal_dynamics_binding_.push_back(prog_->AddConstraint(constraint,
                                                                {momentum_vars(knot_point),
                                                                 momentum_vars(knot_point + 1),
                                                                 com_pos_vars(knot_point),
                                                                contact_pos_[knot_point],
                                                                contact_force_[knot_point]}));
  }
}

void KinematicCentroidalMPC::AddKinematicsIntegrator() {
  for (int knot_point = 0; knot_point < n_knot_points_ - 1; knot_point++) {
    // Integrate generalized velocities to get generalized positions
    auto constraint = std::make_shared<KinematicIntegratorConstraint<double>>(
        plant_, contexts_[knot_point].get(), dt_,knot_point);
    centroidal_dynamics_binding_.push_back(prog_->AddConstraint(constraint,
                                                                {state_vars(knot_point).head(n_q_),
                                                                 state_vars(knot_point+1).head(n_q_),
                                                                 state_vars(knot_point).tail(n_v_)}));

    // Integrate foot states
    for (int contact_index = 0; contact_index < n_contact_points_; contact_index++) {
      prog_->AddConstraint(contact_pos_vars(knot_point + 1, contact_index)
                               == contact_pos_vars(knot_point, contact_index)
                                   + dt_ * contact_vel_vars(knot_point, contact_index));
    }
  }
}

void KinematicCentroidalMPC::AddContactConstraints() {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    for (int contact_index = 0; contact_index < n_contact_points_; contact_index++) {
      //Make sure feet in stance are not moving and on the ground
      //TODO replace with check to see if foot is in stance
      if(true){
        prog_->AddBoundingBoxConstraint(Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3), contact_vel_vars(knot_point,contact_index));
        prog_->AddBoundingBoxConstraint(0, 0, contact_pos_vars(knot_point,contact_index)[2]);
      } else {
        // Feet are above the ground
        prog_->AddBoundingBoxConstraint(0, 10, contact_pos_vars(knot_point,contact_index)[2]);
      }
    }
  }
}

void KinematicCentroidalMPC::AddCentroidalKinematicConsistency() {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    // Ensure linear and angular momentum line up
    auto centroidal_momentum =
        std::make_shared<CentroidalMomentumConstraint<double>>(plant_, contexts_[knot_point].get(), knot_point);
    prog_->AddConstraint(centroidal_momentum, {state_vars(knot_point),
                                               com_pos_vars(knot_point),
                                               momentum_vars(knot_point)});
    for (int contact_index = 0; contact_index < n_contact_points_; contact_index++) {
      // Ensure foot position line up with kinematics
      auto foot_position_constraint =
          std::make_shared<dairlib::multibody::KinematicPositionConstraint<double>>(
              plant_,
              contact_sets_[contact_index],
              Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero(),
              full_constraint_relative_);
      prog_->AddConstraint(foot_position_constraint,
                           {state_vars(knot_point).head(n_q_), contact_pos_vars(knot_point,contact_index)});
    }
    // Constrain com position
    auto com_position =
        std::make_shared<CenterofMassPositionConstraint<double>>(
            plant_, contexts_[knot_point].get(), knot_point);
    prog_->AddConstraint(com_position, {com_pos_vars(knot_point), state_vars(knot_point)});
  }
}

void KinematicCentroidalMPC::AddFrictionConeConstraints() {
  //{TODO} make this actual friction cone constraint
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    for (int contact_index = 0; contact_index < n_contact_points_; contact_index++) {
      prog_->AddBoundingBoxConstraint(0, std::numeric_limits<double>::max(), contact_force_vars(knot_point,contact_index)[2]);
    }
  }
}
drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::state_vars(int knotpoint_index) const {
  return x_vars_[knotpoint_index];
}
drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::joint_pos_vars(int knotpoint_index) const {
  return x_vars_[knotpoint_index].segment(kCentroidalPosDim, n_joint_q_);
}
drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::joint_vel_vars(int knotpoint_index) const {
  return x_vars_[knotpoint_index].segment(n_q_ + kCentroidalVelDim, n_joint_v_);
}


void KinematicCentroidalMPC::Build(const drake::solvers::SolverOptions &solver_options) {
  AddCentroidalDynamics();
  AddKinematicsIntegrator();
  AddContactConstraints();
  AddCentroidalKinematicConsistency();
  AddFrictionConeConstraints();
  AddCosts();
  prog_->SetSolverOptions(solver_options);
}

void KinematicCentroidalMPC::AddConstantStateReferenceCost(const drake::VectorX<double>& value, const Eigen::MatrixXd &Q) {
  AddStateReferenceCost(std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(value), Q);
}

void KinematicCentroidalMPC::AddConstantForceTrackingReferenceCost(const drake::VectorX<double> &value,
                                                                   const Eigen::MatrixXd &Q_force) {
  AddForceTrackingReferenceCost(std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(value), Q_force);
}

void KinematicCentroidalMPC::AddConstantComReferenceCost(const drake::VectorX<double> &value,
                                                         const Eigen::MatrixXd &Q) {
  AddComReferenceCost(std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(value), Q);
}

void KinematicCentroidalMPC::AddConstantMomentumReferenceCost(const drake::VectorX<double> &value,
                                                              const Eigen::MatrixXd &Q) {
  AddMomentumReferenceCost(std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(value), Q);
}

void KinematicCentroidalMPC::AddCosts() {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    double t = dt_ * knot_point;
    if(ref_traj_){
      prog_->AddQuadraticErrorCost(Q_, ref_traj_->value(t), state_vars(knot_point));
    }
    if(com_ref_traj_){
      prog_->AddQuadraticErrorCost(Q_com_,  com_ref_traj_->value(t) , com_pos_vars(knot_point));
    }
    if(mom_ref_traj_){
      prog_->AddQuadraticErrorCost(Q_mom_, mom_ref_traj_->value(t), momentum_vars(knot_point));
    }
    if(contact_ref_traj_){
      prog_->AddQuadraticErrorCost(Q_contact_,  contact_ref_traj_->value(t) , {contact_pos_[knot_point],contact_vel_[knot_point]});
    }
    if(force_ref_traj_){
      prog_->AddQuadraticErrorCost(Q_force_, force_ref_traj_->value(t), contact_force_[knot_point]);
    }
  }
}

void KinematicCentroidalMPC::SetZeroInitialGuess() {
  Eigen::VectorXd initialGuess= Eigen::VectorXd::Zero(n_q_+n_v_+n_contact_points_*9 + 6 + 3);
  prog_->SetInitialGuessForAllVariables(initialGuess.replicate(n_knot_points_, 1));
  // Make sure unit quaternions
  for (int i = 0; i < n_knot_points_; i++) {
    auto xi = state_vars(i);
    prog_->SetInitialGuess(xi(0), 1);
    prog_->SetInitialGuess(xi(1), 0);
    prog_->SetInitialGuess(xi(2), 0);
    prog_->SetInitialGuess(xi(3), 0);
  }
}

drake::trajectories::PiecewisePolynomial<double> KinematicCentroidalMPC::Solve() {
  auto start = std::chrono::high_resolution_clock::now();
  solver_->Solve(*prog_, prog_->initial_guess(),
                prog_->solver_options(),
                result_.get());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Solve time:" << elapsed.count() << std::endl;
  std::cout << "Cost:" << result_->get_optimal_cost() << std::endl;

  std::vector<double> time_points;
  std::vector<drake::MatrixX<double>> states;
  for(int knot_point = 0; knot_point < n_knot_points_; knot_point++ ){
    time_points.emplace_back(dt_*knot_point);
    states.emplace_back(result_->GetSolution(state_vars(knot_point)));
  }
  return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(time_points, states);
}

bool KinematicCentroidalMPC::SaveSolutionToFile(const std::string& filepath){
  // check if there is a solution
  DRAKE_DEMAND(result_ != nullptr);
  Eigen::MatrixXd state_points;
  Eigen::MatrixXd centroidal_state_points;
  Eigen::MatrixXd contact_pos_points;
  Eigen::MatrixXd contact_vel_points;
  Eigen::MatrixXd contact_force_points;
  std::vector<double> time_samples;
  this->SetFromSolution(*result_,
                       &state_points,
                       &centroidal_state_points,
                       &contact_pos_points,
                       &contact_vel_points,
                       &contact_force_points,
                       &time_samples);

  dairlib::LcmTrajectory::Trajectory state_traj;
  dairlib::LcmTrajectory::Trajectory centroidal_state_traj;
  dairlib::LcmTrajectory::Trajectory contact_force_traj;
  state_traj.traj_name = "state_traj";
  state_traj.datapoints = state_points;
  state_traj.time_vector = Eigen::Map<VectorXd>(time_samples.data(), time_samples.size());
  state_traj.datatypes = dairlib::multibody::CreateStateNameVectorFromMap(plant_);

  centroidal_state_traj.traj_name = "centroidal_state_traj";
  centroidal_state_traj.datapoints = centroidal_state_points;
  centroidal_state_traj.time_vector = Eigen::Map<VectorXd>(time_samples.data(), time_samples.size());
  centroidal_state_traj.datatypes = std::vector<std::string>(centroidal_state_traj.datapoints.rows());

  contact_force_traj.traj_name = "contact_force_traj";
  contact_force_traj.datapoints = contact_force_points;
  contact_force_traj.time_vector = Eigen::Map<VectorXd>(time_samples.data(), time_samples.size());
  contact_force_traj.datatypes = std::vector<std::string>(contact_force_traj.datapoints.rows());

  std::vector<dairlib::LcmTrajectory::Trajectory> trajectories;
  trajectories.push_back(state_traj);
  trajectories.push_back(centroidal_state_traj);
  trajectories.push_back(contact_force_traj);
  std::vector<std::string> trajectory_names = {state_traj.traj_name,
                                               centroidal_state_traj.traj_name,
                                               contact_force_traj.traj_name};
  LcmTrajectory lcm_trajectory = LcmTrajectory(trajectories,
                trajectory_names,
                "centroidal_mpc_solution",
                "centroidal_mpc_solution");

  lcm_trajectory.WriteToFile(filepath);
  return true;
}


void KinematicCentroidalMPC::SetFromSolution(
    const drake::solvers::MathematicalProgramResult& result,
    Eigen::MatrixXd* state_samples,
    Eigen::MatrixXd* centroidal_samples,
    Eigen::MatrixXd* contact_pos_samples,
    Eigen::MatrixXd* contact_vel_samples,
    Eigen::MatrixXd* contact_force_samples,
    std::vector<double>* time_samples) const {
  DRAKE_ASSERT(state_samples != nullptr);
  DRAKE_ASSERT(centroidal_samples != nullptr);
  DRAKE_ASSERT(contact_pos_samples != nullptr);
  DRAKE_ASSERT(contact_vel_samples != nullptr);
  DRAKE_ASSERT(contact_vel_samples != nullptr);
  DRAKE_ASSERT(time_samples->empty());

  *state_samples = MatrixXd(n_q_ + n_v_, n_knot_points_);
  *centroidal_samples = MatrixXd(kCentroidalPosDim + kCentroidalVelDim, n_knot_points_);
  *contact_force_samples = MatrixXd(3 * n_contact_points_, n_knot_points_);
  time_samples->resize(n_knot_points_);

  for (int knot_point = 0; knot_point < num_knot_points(); knot_point++) {
    time_samples->at(knot_point) = knot_point * dt_;

    VectorXd x = result.GetSolution(state_vars(knot_point));
    VectorXd x_cent = result.GetSolution(x_cent_vars_[knot_point]);
//    VectorXd contact_pos = result.GetSolution(state_vars(knot_point));
//    VectorXd contact_vel = result.GetSolution(input_vars(knot_point));
    VectorXd contact_force = result.GetSolution(contact_force_.at(knot_point));
    state_samples->col(knot_point) = x;
    centroidal_samples->col(knot_point) = x_cent;
    contact_force_samples->col(knot_point) = contact_force;
  }
}


void KinematicCentroidalMPC::CreateVisualizationCallback(std::string model_file,
                                                         double alpha,
                                                         std::string weld_frame_to_world) {
  DRAKE_DEMAND(!callback_visualizer_);  // Cannot be set twice

  // Assemble variable list
  drake::solvers::VectorXDecisionVariable vars(n_knot_points_ *
  plant_.num_positions());
  for(int knot_point = 0; knot_point < n_knot_points_; knot_point ++){
    vars.segment(knot_point * plant_.num_positions(), plant_.num_positions()) = state_vars(knot_point).head(plant_.num_positions());
  }

  Eigen::VectorXd alpha_vec = Eigen::VectorXd::Constant(n_knot_points_, alpha);
  alpha_vec(0) = 1;
  alpha_vec(n_knot_points_ - 1) = 1;

  // Create visualizer
  callback_visualizer_ = std::make_unique<dairlib::multibody::MultiposeVisualizer>(
      model_file, n_knot_points_, alpha_vec, weld_frame_to_world);


  // Callback lambda function
  auto my_callback = [this](const Eigen::Ref<const Eigen::VectorXd>& vars) {
    Eigen::VectorXd vars_copy = vars;
    Eigen::Map<Eigen::MatrixXd> states(vars_copy.data(), this->plant_.num_positions(),
                                       this->n_knot_points_);
    this->callback_visualizer_->DrawPoses(states);
  };

  prog_->AddVisualizationCallback(my_callback, vars);

}
void KinematicCentroidalMPC::AddContactPointPositionConstraint(int contact_index, const Eigen::Vector3d& lb, const Eigen::Vector3d& ub) {
  //{TODO} eventually make in body frame
  for(int knot_point = 0; knot_point < n_knot_points_; knot_point ++){
    prog_->AddBoundingBoxConstraint(lb, ub, contact_pos_vars(knot_point,contact_index));
  }
}
void KinematicCentroidalMPC::AddPlantJointLimits(const std::vector<std::string>& joint_to_constrain) {
  std::map<std::string, int> positions_map = dairlib::multibody::MakeNameToPositionsMap(plant_);
  for (const auto& member : joint_to_constrain) {
    for(int knot_point = 0; knot_point < n_knot_points_; knot_point ++) {
      prog_->AddBoundingBoxConstraint(plant_.GetJointByName(member).position_lower_limits()(0),
                                      plant_.GetJointByName(member).position_upper_limits()(0),
                                      state_vars(knot_point)[positions_map.at(member)]);
    }
  }

}

drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::com_pos_vars(int knotpoint_index) const {
  return com_vars_[knotpoint_index];
}
drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::contact_pos_vars(int knotpoint_index,
                                                                                 int contact_index) const {
  return contact_pos_[knotpoint_index].segment(contact_index * 3, 3);
}
drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::contact_vel_vars(int knotpoint_index,
                                                                                 int contact_index) const {
  return contact_vel_[knotpoint_index].segment(contact_index * 3, 3);
}
drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::contact_force_vars(int knotpoint_index,
                                                                                   int contact_index) const {
  return contact_force_[knotpoint_index].segment(contact_index * 3, 3);
}
void KinematicCentroidalMPC::AddKinematicConstraint(std::shared_ptr<dairlib::multibody::KinematicPositionConstraint<
    double>> con, const Eigen::Ref<const drake::solvers::VectorXDecisionVariable> &vars) {
  prog_->AddConstraint(std::move(con), vars);
}
void KinematicCentroidalMPC::SetRobotStateGuess(const drake::VectorX<double> &state) {
  DRAKE_DEMAND(state.size() == n_q_ + n_v_);

  for(const auto& state_vars : x_vars_){
   prog_->SetInitialGuess(state_vars, state);
  }
}
drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::momentum_vars(int knotpoint_index) const {
  return mom_vars_[knotpoint_index];
}
void KinematicCentroidalMPC::AddComHeightBoundingConstraint(double lb, double ub) {
  for(int knot_point = 0; knot_point < n_knot_points_; knot_point ++) {
    prog_->AddBoundingBoxConstraint(lb, ub, com_pos_vars(knot_point)[2]);
  }
}
void KinematicCentroidalMPC::SetComPositionGuess(const drake::Vector3<double> &state) {
  for(const auto& com_pos : com_vars_){
    prog_->SetInitialGuess(com_pos, state);
  }
}
