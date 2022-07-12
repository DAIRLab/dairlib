#include "kinodynamic_planner.h"

using drake::AbstractValue;
using drake::multibody::BodyFrame;
using drake::multibody::JacobianWrtVariable;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;

using drake::EigenPtr;

using drake::multibody::CentroidalMomentumConstraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::OsqpSolver;
using drake::solvers::OsqpSolverDetails;
using drake::solvers::Solve;
using drake::solvers::VectorXDecisionVariable;

using drake::AutoDiffXd;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::LcmTrajectory;
using dairlib::multibody::MakeNameToPositionsMap;
using dairlib::multibody::MakeNameToVelocitiesMap;
using dairlib::multibody::SetPositionsAndVelocitiesIfNew;
using dairlib::systems::OutputVector;

namespace dairlib {

KinodynamicPlanner::KinodynamicPlanner(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>& context,
    const drake::multibody::MultibodyPlant<AutoDiffXd>& plant_ad,
    drake::systems::Context<AutoDiffXd>& context_ad)
    : plant_(plant),
      context_(context),
      plant_ad_(plant_ad),
      context_ad_(context_ad),
      n_q_(plant.num_positions()),
      n_v_(plant.num_velocities()),
      n_u_(plant.num_actuators()),
      m_(plant.CalcTotalMass(context)),
      g_(plant.gravity_field().gravity_vector()) {
  // Create Ports
  state_port_ = this->DeclareVectorInputPort(
                        "x, u, t", OutputVector<double>(n_q_, n_v_, n_u_))
                    .get_index();

  traj_out_port_ = this->DeclareAbstractOutputPort(
                           "y(t)", &KinodynamicPlanner::GetMostRecentMotionPlan,
                           {this->all_state_ticket()})
                       .get_index();

  // Discrete update
  DeclarePerStepDiscreteUpdateEvent(
      &KinodynamicPlanner::DiscreteVariableUpdate);
  //  DeclarePeriodicDiscreteUpdateEvent(dt_, 0,
  //                                     &KinodynamicPlanner::PeriodicUpdate);

  x_des_ = VectorXd ::Zero(n_r_ + n_h_);
}

// void KinodynamicPlanner::print_constraint(
//    const std::vector<drake::solvers::LinearConstraint*>& constraints) const {
//  for (auto& x0_const : constraints) {
//    std::cout << x0_const->get_description() << ":\n A:\n"
//              << x0_const->A() << "\nub:\n"
//              << x0_const->upper_bound() << "\nlb\n"
//              << x0_const->lower_bound() << std::endl;
//  }
//}
//
// void KinodynamicPlanner::print_constraint(
//    const std::vector<drake::solvers::LinearEqualityConstraint*>& constraints)
//    const {
//  for (auto& x0_const : constraints) {
//    std::cout << x0_const->get_description() << ":\n A:\n"
//              << x0_const->A() << "\nb:\n"
//              << x0_const->upper_bound() << std::endl;
//  }
//}
//
// void KinodynamicPlanner::print_constraint(
//    const std::vector<drake::solvers::Constraint*>& constraints) const {
//  for (auto& constraint : constraints) {
//    auto constr = dynamic_cast<drake::solvers::LinearConstraint*>(constraint);
//    std::cout << constr->get_description() << ":\n A:\n"
//              << constr->A() << "\nb:\n"
//              << constr->upper_bound() << std::endl;
//  }
//}

void KinodynamicPlanner::AddCoMDynamicsConstraint() {
  for (int i = 0; i < n_knot_points_; ++i) {
    prog_.AddLinearEqualityConstraint(
        m_ * ddr_[i] - F_[i][0] - F_[i][1] - F_[i][2] - F_[i][3], m_ * g_);
  }
}

void KinodynamicPlanner::AddCentroidalMomentumConstraint() {
  for (int i = 0; i < n_knot_points_; ++i) {
    centroidal_momentum_constraints_.push_back(
        std::make_shared<CentroidalMomentumConstraint>(&plant_ad_, std::nullopt,
                                                       &context_ad_, true));
//    VectorXDecisionVariable x;
    //    constraint.ComposeVariable(q_.at(i).eval(), v_.at(i).eval(),
    //    h_.at(i).eval(), x); constraint.ComposeVariable(q_.at(i), v_.at(i),
    //    h_.at(i), &x); constraint.ComposeVariable(q_.at(i), v_.at(i),
    //    h_.at(i), &q_.at(i));
    prog_.AddConstraint(centroidal_momentum_constraints_[i],
                        {q_.at(i), v_.at(i), h_.at(i)});
  }
}

void KinodynamicPlanner::AddAngularMomentumDynamicsConstraint() {
  for (int i = 0; i < n_knot_points_; ++i) {
    centroidal_momentum_constraints_.push_back(
        std::make_shared<CentroidalMomentumConstraint>(&plant_ad_, std::nullopt,
                                                       &context_ad_, true));
//    VectorXDecisionVariable x;
    //    constraint.ComposeVariable(q_.at(i).eval(), v_.at(i).eval(),
    //    h_.at(i).eval(), x); constraint.ComposeVariable(q_.at(i), v_.at(i),
    //    h_.at(i), &x); constraint.ComposeVariable(q_.at(i), v_.at(i),
    //    h_.at(i), &q_.at(i));
//    prog_.AddConstraint(dh_.at(i) == (c_[i][0] - r_[i][0]).cross(F_[i][0]),
//                        {q_.at(i), v_.at(i), h_.at(i)});
  }
}

void KinodynamicPlanner::AddIntegrationConstraints(){
  for (int i = 1; i < n_knot_points_; ++i) {
    // eq 7d
    prog_.AddLinearEqualityConstraint(q_[i] - q_[i-1] == v_[i] * dt_[i]);
    // eq 7e
    prog_.AddLinearEqualityConstraint(h_[i] - h_[i-1] == dh_[i] * dt_[i]);

    // eq 7f
//    prog_.AddLinearEqualityConstraint(r_[i] - r_[i-1] == (dr_[i] + dr_[i-1])/2 * dt_[i]);
    prog_.AddLinearEqualityConstraint(r_[i] - r_[i-1] == dr_[i] * dt_[i]);

    // eq 7g
    prog_.AddLinearEqualityConstraint(dr_[i] - dr_[i-1] == ddr_[i] * dt_[i]);
  }
}

void KinodynamicPlanner::AddDynamicsConstraint() {
  AddCoMDynamicsConstraint();
  AddCentroidalMomentumConstraint();
  AddIntegrationConstraints();
}

void KinodynamicPlanner::AddKinematicConstraints(){
  // Linearize the kinematic constraints about current state

}

void KinodynamicPlanner::CheckSquareMatrixDimensions(const MatrixXd& M,
                                                     const int n) const {
  DRAKE_DEMAND(M.rows() == n);
  DRAKE_DEMAND(M.cols() == n);
}

}  // namespace dairlib