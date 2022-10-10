#include <iostream>
#include <utility>
#include "kinematic_centroidal_mpc.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "systems/controllers/kinematic_centroidal_mpc/kinematic_centroidal_constraints.h"
#include "multibody/kinematic/kinematic_constraints.h"
#include "multibody/multibody_utils.h"

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

  n_kinematic_q_ = n_q_ - n_centroidal_pos_;
  n_kinematic_v_ = n_v_ - n_centroidal_vel_;
  prog_ = std::make_unique<drake::solvers::MathematicalProgram>();
  solver_ = std::make_unique<drake::solvers::IpoptSolver>();
  for(int contact = 0; contact < n_contact_points_; contact ++){
    contact_sets_.emplace_back(plant_);
    contact_sets_[contact].add_evaluator(contact_points_[contact].get());
  }

  for(int knot = 0; knot < n_knot_points; knot ++){
    contexts_[knot] = plant_.CreateDefaultContext();
    x_vars_.push_back(prog_->NewContinuousVariables(n_q_ + n_v_, "x_vars_" + std::to_string(knot)));
    x_cent_vars_.push_back(prog_->NewContinuousVariables(n_centroidal_pos_ + n_centroidal_vel_, "x_cent_vars_" + std::to_string(knot)));
    std::vector<drake::solvers::VectorXDecisionVariable> knot_point_forces(n_contact_points_);
    std::vector<drake::solvers::VectorXDecisionVariable> knot_point_contact_pos(n_contact_points_);
    std::vector<drake::solvers::VectorXDecisionVariable> knot_point_contact_vel(n_contact_points_);

    for(int contact = 0; contact < n_contact_points_; contact ++){
      knot_point_contact_pos[contact] =prog_->NewContinuousVariables(3, "contact_pos_" + std::to_string(knot) + "_" + std::to_string(contact));
      knot_point_contact_vel[contact] =prog_->NewContinuousVariables(3, "contact_vel_" + std::to_string(knot) + "_" + std::to_string(contact));
      knot_point_forces[contact] = prog_->NewContinuousVariables(3, "contact_force_" + std::to_string(knot) + "_" + std::to_string(contact));
    }
    contact_pos_.push_back(knot_point_contact_pos);
    contact_vel_.push_back(knot_point_contact_vel);
    contact_force_.push_back(knot_point_forces);
  }
}

void KinematicCentroidalMPC::AddStateReference(std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj,
                                               const Eigen::MatrixXd &Q) {
  // Ensure matrix is square and has correct number of rows and columns
  DRAKE_DEMAND(Q.rows() == n_q_ + n_v_);
  DRAKE_DEMAND(Q.cols() == n_q_ + n_v_);

  ref_traj_ = std::move(ref_traj);
  Q_ = Q;
}

void KinematicCentroidalMPC::AddContactTrackingReference(std::unique_ptr<drake::trajectories::Trajectory<double>> contact_ref_traj,
                                                         const Eigen::MatrixXd &Q_contact) {
  DRAKE_DEMAND(Q_contact.rows() == 2 * 3 * n_contact_points_);
  DRAKE_DEMAND(Q_contact.cols() == 2 * 3 * n_contact_points_);

  contact_ref_traj_ = std::move(contact_ref_traj);
  Q_contact_ = Q_contact;
}

void KinematicCentroidalMPC::AddForceTrackingReference(std::unique_ptr<drake::trajectories::Trajectory<double>> force_ref_traj,
                                                       const Eigen::MatrixXd &Q_force) {
  DRAKE_DEMAND(Q_force.rows() == 3 * n_contact_points_);
  DRAKE_DEMAND(Q_force.cols() == 3 * n_contact_points_);

  force_ref_traj_ = std::move(force_ref_traj);
  Q_force_ = Q_force;
}

void KinematicCentroidalMPC::AddCentroidalReference(std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj,
                                                    const Eigen::MatrixXd &Q) {
  DRAKE_DEMAND(Q.rows() == n_centroidal_pos_ + n_centroidal_vel_);
  DRAKE_DEMAND(Q.cols() == n_centroidal_pos_ + n_centroidal_vel_);

  centroidal_ref_traj_ = std::move(ref_traj);
  Q_cent_ = Q;
}

void KinematicCentroidalMPC::AddCentroidalDynamics() {
  for(int knot_point = 0; knot_point < n_knot_points_ - 1; knot_point ++){
    auto constraint = std::make_shared<CentroidalDynamicsConstraint<double>>(
        plant_, contexts_[knot_point].get(), n_contact_points_, dt_,knot_point);
    drake::solvers::VariableRefList constraint_vars;
    constraint_vars.push_back(centroidal_pos_vars(knot_point));
    constraint_vars.push_back(centroidal_vel_vars(knot_point));
    constraint_vars.push_back(centroidal_pos_vars(knot_point+1));
    constraint_vars.push_back(centroidal_vel_vars(knot_point+1));
    constraint_vars.push_back(state_vars(knot_point));
    for(const auto& contact_pos : contact_pos_[knot_point]){
      constraint_vars.emplace_back(contact_pos);
    }
    for(const auto& contact_force : contact_force_[knot_point]){
      constraint_vars.emplace_back(contact_force);
    }
    // TODO make not hard coded
    centroidal_dynamics_binding_.push_back(prog_->AddConstraint(constraint,
                                                                {centroidal_pos_vars(knot_point),
                                                                centroidal_vel_vars(knot_point),
                                                                centroidal_pos_vars(knot_point + 1),
                                                                centroidal_vel_vars(knot_point + 1),
                                                                state_vars(knot_point),
                                                                contact_pos_[knot_point][0],
                                                                contact_pos_[knot_point][1],
                                                                contact_pos_[knot_point][2],
                                                                contact_pos_[knot_point][3],
                                                                contact_force_[knot_point][0],
                                                                contact_force_[knot_point][1],
                                                                contact_force_[knot_point][2],
                                                                contact_force_[knot_point][3]}));
  }
}

void KinematicCentroidalMPC::AddKinematicDynamics() {
  for (int knot_point = 0; knot_point < n_knot_points_ - 1; knot_point++) {
    // Integrate joint states
    {
      Eigen::MatrixXd A(n_kinematic_q_, 3 * n_kinematic_q_);
      A << Eigen::MatrixXd::Identity(n_kinematic_q_, n_kinematic_q_),
          -Eigen::MatrixXd::Identity(n_kinematic_q_, n_kinematic_q_),
          dt_ * Eigen::MatrixXd::Identity(n_kinematic_q_, n_kinematic_q_);

      prog_->AddLinearConstraint(A,
                                 Eigen::VectorXd::Zero(n_kinematic_q_),
                                 Eigen::VectorXd::Zero(n_kinematic_q_),
                                 {joint_pos_vars(knot_point), joint_pos_vars(knot_point + 1),
                                  joint_vel_vars(knot_point)});
    }
    // Integrate foot states
    {
      Eigen::MatrixXd A(3 * n_contact_points_, 3 * 3 * n_contact_points_);
      A << Eigen::MatrixXd::Identity(3 * n_contact_points_, 3 * n_contact_points_),
          -Eigen::MatrixXd::Identity(3 * n_contact_points_, 3 * n_contact_points_),
          dt_ * Eigen::MatrixXd::Identity(3 * n_contact_points_, 3 * n_contact_points_);

      drake::solvers::VariableRefList contact_vars;
      for(const auto& contact_pos : contact_pos_[knot_point]){
        contact_vars.emplace_back(contact_pos);
      }
      for(const auto& contact_pos : contact_pos_[knot_point+1]){
        contact_vars.emplace_back(contact_pos);
      }
      for(const auto& contact_vel : contact_vel_[knot_point]){
        contact_vars.emplace_back(contact_vel);
      }

      prog_->AddLinearConstraint(A,
                                 Eigen::VectorXd::Zero(3 * n_contact_points_),
                                 Eigen::VectorXd::Zero(3 * n_contact_points_),
                                 contact_vars);
    }
  }
}

void KinematicCentroidalMPC::AddContactConstraints() {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    for (int contact = 0; contact < n_contact_points_; contact++) {
      //Make sure feet in stance are not moving and on the ground
      if(true){
        prog_->AddBoundingBoxConstraint(Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3), contact_vel_vars(knot_point,contact));
        prog_->AddBoundingBoxConstraint(0, 0, contact_pos_vars(knot_point,contact)[2]);
      } else {
        // Feet are above the ground
        prog_->AddBoundingBoxConstraint(0, 10, contact_pos_vars(knot_point,contact)[2]);
      }
    }
  }
}

void KinematicCentroidalMPC::AddCentroidalKinematicConsistency() {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    for (int contact = 0; contact < n_contact_points_; contact++) {
      // Ensure foot position line up with kinematics
      std::set<int> full_constraint_relative = {0, 1, 2};
      auto foot_position_constraint =
          std::make_shared<dairlib::multibody::KinematicPositionConstraint<double>>(
              plant_,
              contact_sets_[contact],
              Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero(),
              full_constraint_relative);
      prog_->AddConstraint(foot_position_constraint,
                           {state_vars(knot_point).head(n_q_), contact_pos_vars(knot_point,contact)});
    }
    // Constrain com position
    auto com_position =
        std::make_shared<CenterofMassPositionConstraint<double>>(
            plant_, contexts_[knot_point].get(), knot_point);
    prog_->AddConstraint(com_position, {com_pos_vars(knot_point), state_vars(knot_point)});

    // Constrain com velocity
    auto com_velocity =
        std::make_shared<CenterofMassVelocityConstraint<double>>(
            plant_, contexts_[knot_point].get(), knot_point);
    prog_->AddConstraint(com_velocity, {com_vel_vars(knot_point), state_vars(knot_point)});

    // Constrain orientation and angular velocity to be equal
    // TODO make sure angular velocity are both in the same frame
    // SRL I suspect that centroidal angular velocity is in body frame, what is drake angular velocity in?
    Eigen::MatrixXd A(7, 2 * 7);
    A << Eigen::MatrixXd::Identity(7, 7),
        -Eigen::MatrixXd::Identity(7, 7);
    prog_->AddLinearConstraint(A,
                               Eigen::VectorXd::Zero(7),
                               Eigen::VectorXd::Zero(7),
                               {cent_quat_vars(knot_point), cent_omega_vars(knot_point),
                                state_vars(knot_point).head(4), state_vars(knot_point).segment(n_q_,3)});
  }

}
void KinematicCentroidalMPC::AddFrictionConeConstraints() {
  //{TODO} make this actual friction cone constraint
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    for (int contact = 0; contact < n_contact_points_; contact++) {
      prog_->AddBoundingBoxConstraint(0, std::numeric_limits<double>::max(), contact_force_vars(knot_point,contact)[2]);
    }
  }
}
drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::state_vars(int knotpoint_index) const {
  return x_vars_[knotpoint_index];
}

drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::centroidal_pos_vars(int knotpoint_index) const {
  return x_cent_vars_[knotpoint_index].segment(0,n_centroidal_pos_);
}
drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::centroidal_vel_vars(int knotpoint_index) const {
  return x_cent_vars_[knotpoint_index].segment(n_centroidal_pos_,n_centroidal_vel_);
}
drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::joint_pos_vars(int knotpoint_index) const {
  return x_vars_[knotpoint_index].segment(n_centroidal_pos_,n_kinematic_q_);
}
drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::joint_vel_vars(int knotpoint_index) const {
  return x_vars_[knotpoint_index].segment(n_q_ + n_centroidal_vel_,n_kinematic_v_);
}


void KinematicCentroidalMPC::Build(const drake::solvers::SolverOptions &solver_options) {
  AddCentroidalDynamics();
  AddKinematicDynamics();
  AddContactConstraints();
  AddCentroidalKinematicConsistency();
  AddFrictionConeConstraints();
  AddCosts();
  prog_->SetSolverOptions(solver_options);
}

void KinematicCentroidalMPC::AddConstantStateReference(const drake::VectorX<double>& value, const Eigen::MatrixXd &Q) {
  AddStateReference(std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(value), Q);
}

void KinematicCentroidalMPC::AddConstantForceTrackingReference(const drake::VectorX<double> &value,
                                                               const Eigen::MatrixXd &Q_force) {
  AddForceTrackingReference(std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(value), Q_force);
}

void KinematicCentroidalMPC::AddConstantCentroidalReference(const drake::VectorX<double> &value,
                                                            const Eigen::MatrixXd &Q) {
  AddCentroidalReference(std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(value), Q);
}

void KinematicCentroidalMPC::AddCosts() {
  for (int knot_point = 0; knot_point < n_knot_points_; knot_point++) {
    double t = dt_ * knot_point;
    if(ref_traj_){
      const auto& ref = ref_traj_->value(t);
      prog_->AddQuadraticCost(2 * Q_,  -Q_ * ref , state_vars(knot_point));
    }
    if(centroidal_ref_traj_){
      const auto& ref = centroidal_ref_traj_->value(t);
      prog_->AddQuadraticCost(2 * Q_cent_,  -Q_cent_ * ref , {centroidal_pos_vars(knot_point), centroidal_vel_vars(knot_point)});
    }
    if(contact_ref_traj_){
      const auto& ref = contact_ref_traj_->value(t);
      drake::solvers::VariableRefList contact_vars;
      for(const auto& contact_pos : contact_pos_[knot_point]){
        contact_vars.emplace_back(contact_pos);
      }
      for(const auto& contact_vel : contact_vel_[knot_point]){
        contact_vars.emplace_back(contact_vel);
      }
      prog_->AddQuadraticCost(2 * Q_contact_,  -Q_contact_ * ref , contact_vars);
    }
    if(force_ref_traj_){
      const auto& ref = force_ref_traj_->value(t);
      drake::solvers::VariableRefList force_vars;
      for(const auto& force : contact_force_[knot_point]){
        force_vars.emplace_back(force);
      }
      prog_->AddQuadraticCost(2 * Q_force_, -Q_force_ * ref, force_vars);
    }
  }
}

void KinematicCentroidalMPC::SetZeroInitialGuess() {
  Eigen::VectorXd initialGuess= Eigen::VectorXd::Zero(n_q_+n_v_+n_contact_points_*9 + n_centroidal_pos_ + n_centroidal_vel_);
  prog_->SetInitialGuessForAllVariables(initialGuess.replicate(n_knot_points_, 1));
  // Make sure unit quaternions
  for (int i = 0; i < n_knot_points_; i++) {
    auto xi = state_vars(i);
    prog_->SetInitialGuess(xi(0), 1);
    prog_->SetInitialGuess(xi(1), 0);
    prog_->SetInitialGuess(xi(2), 0);
    prog_->SetInitialGuess(xi(3), 0);

    auto qi = centroidal_pos_vars(i);
    prog_->SetInitialGuess(qi(0), 1);
    prog_->SetInitialGuess(qi(1), 0);
    prog_->SetInitialGuess(qi(2), 0);
    prog_->SetInitialGuess(qi(3), 0);

  }
}

drake::trajectories::PiecewisePolynomial<double> KinematicCentroidalMPC::Solve() {
  drake::solvers::MathematicalProgramResult result;
  auto start = std::chrono::high_resolution_clock::now();
  solver_->Solve(*prog_, prog_->initial_guess(),
                prog_->solver_options(),
                &result);
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Solve time:" << elapsed.count() << std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() << std::endl;

  std::vector<double> time_points;
  std::vector<drake::MatrixX<double>> states;
  for(int knot_point = 0; knot_point < n_knot_points_; knot_point++ ){
    time_points.emplace_back(dt_*knot_point);
    states.emplace_back(result.GetSolution(state_vars(knot_point)));
  }
  return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(time_points, states);
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
  return centroidal_pos_vars(knotpoint_index).tail(3);
}
drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::com_vel_vars(int knotpoint_index) const {
  return centroidal_vel_vars(knotpoint_index).tail(3);
}
drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::cent_quat_vars(int knotpoint_index) const {
  return centroidal_pos_vars(knotpoint_index).head(4);
}
drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::cent_omega_vars(int knotpoint_index) const {
  return centroidal_vel_vars(knotpoint_index).head(3);
}
drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::contact_pos_vars(int knotpoint_index,
                                                                                 int contact) const {
  return contact_pos_[knotpoint_index][contact];
}
drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::contact_vel_vars(int knotpoint_index,
                                                                                 int contact) const {
  return contact_vel_[knotpoint_index][contact];
}
drake::solvers::VectorXDecisionVariable KinematicCentroidalMPC::contact_force_vars(int knotpoint_index,
                                                                                   int contact) const {
  return contact_force_[knotpoint_index][contact];
}
void KinematicCentroidalMPC::AddKinematicConstraint(std::shared_ptr<dairlib::multibody::KinematicPositionConstraint<
    double>> con, const Eigen::Ref<const drake::solvers::VectorXDecisionVariable> &vars) {
  prog_->AddConstraint(std::move(con), vars);
}
