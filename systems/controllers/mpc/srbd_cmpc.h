#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <set>
#include <dairlib/lcmt_saved_traj.hpp>

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "multibody/single_rigid_body_plant.h"
#include "solvers/constraint_factory.h"
#include "systems/framework/output_vector.h"
#include "solvers/fast_osqp_solver.h"

namespace dairlib {

enum BipedStance {
  kLeft=0,
  kRight=1
};

typedef struct LinearSrbdDynamics{
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::VectorXd b;
} LinearSrbdDynamics;

typedef struct SrbdMode {
  LinearSrbdDynamics dynamics;
  BipedStance stance;
  int N;

  drake::solvers::VectorXDecisionVariable pp;
  drake::solvers::VectorXDecisionVariable reachability_slack;
  drake::solvers::LinearConstraint* reachability_constraints;
  drake::solvers::LinearConstraint* terrain_constraint;

  std::vector<drake::solvers::VectorXDecisionVariable> xx;
  std::vector<drake::solvers::VectorXDecisionVariable> uu;
  std::vector<drake::solvers::QuadraticCost*> tracking_cost;
  std::vector<drake::solvers::QuadraticCost*> terminal_cost;
  std::vector<drake::solvers::QuadraticCost*> input_cost;
  std::vector<drake::solvers::LinearEqualityConstraint*> dynamics_constraints;
  std::vector<drake::solvers::LinearConstraint*> friction_constraints;
  std::vector<drake::solvers::LinearEqualityConstraint*> init_state_constraint;
} SrbdMode;

class SrbdCMPC : public drake::systems::LeafSystem<double> {
 public:
  SrbdCMPC(const multibody::SingleRigidBodyPlant& plant, double dt,
           double swing_ft_height, bool traj,
           bool used_with_finite_state_machine = true,
           bool use_com = false);


  /// Currently SrbdCMPC only supports the contact sequence
  /// [kLeft, kRight] so one should call AddMode(BipedStance::kLeft ...),
  /// AddMode(BipedStance::kRight ...)
  /// @param stance BipedStance::kLeft or BipedStance::kRight
  /// @param dynamics
  /// @param N
  void AddMode(const LinearSrbdDynamics&  dynamics, BipedStance stance, int N);

  void AddTrackingObjective(
      const Eigen::VectorXd& xdes, const Eigen::MatrixXd& Q);
  void SetTerminalCost(const Eigen::MatrixXd& Qf);
  void AddInputRegularization(const Eigen::MatrixXd& R);
  void Build();
  void CheckProblemDefinition();

  // Input ports
  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_x_des_input_port() const {
    return this->get_input_port(x_des_port_);
  }
  const drake::systems::OutputPort<double>& get_traj_out_port() const {
    return this->get_output_port(traj_out_port_);
  }

  void SetReachabilityBoundingBox(
      const Eigen::Vector3d& bounds,
      const std::vector<Eigen::VectorXd>& nominal_relative_pos,
      const Eigen::MatrixXd& kin_reach_soft_contraint_w);


  void SetMu(double mu) { DRAKE_DEMAND(mu > 0);  mu_ = mu; }

  std::vector<SrbdMode> get_modes() {return modes_;}

  drake::solvers::MathematicalProgramResult solve_problem_as_is() {
    return drake::solvers::Solve(prog_);
  }

  void print_constraint(
      const std::vector<drake::solvers::LinearConstraint*>& constraint) const;
  void print_constraint(
      const std::vector<drake::solvers::LinearEqualityConstraint*>& constraint) const;
  void print_initial_state_constraints() const;
  void print_state_knot_constraints() const;
  void print_dynamics_constraints() const;
  void print_current_init_state_constraint() const;

 private:

  void CheckSquareMatrixDimensions(const Eigen::MatrixXd& M, int n) const;
  void GetMostRecentMotionPlan(const drake::systems::Context<double>& context,
                               lcmt_saved_traj* traj_msg) const;

  Eigen::MatrixXd MakeSimpsonIntegratedTrackingAndInputCost(
      const Eigen::MatrixXd &A, const Eigen::MatrixXd &B) const;
  Eigen::VectorXd MakeSplineSegmentReferenceStateAndInput() const ;

  void MakeKinematicReachabilityConstraints();
  void MakeTerrainConstraints(
      const Eigen::Vector3d& normal = {0, 0, 1},
      const Eigen::Vector3d& origin = {0, 0, 0});
  void MakeDynamicsConstraints();
  void MakeFrictionConeConstraints();
  void MakeStateKnotConstraints();
  void MakeInitialStateConstraints();
  void MakeTrackingCost();

  Eigen::MatrixXd CalcSwingFootKnotPoints(
      const Eigen::VectorXd& x,
      const drake::solvers::MathematicalProgramResult& result,
      double time_since_last_touchdown) const;

  Eigen::MatrixXd MakeCollocationConstraintAMatrix(
      const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);

  lcmt_saved_traj MakeLcmTrajFromSol(
      const drake::solvers::MathematicalProgramResult& result,
      double time, double time_since_last_touchdown,
      const Eigen::VectorXd& state) const;

  void UpdateInitialStateConstraint(
      const Eigen::VectorXd& x0,
      int fsm_state, double t_since_last_switch) const;

  void UpdateTrackingObjective(const Eigen::VectorXd& xdes) const;

  void CopyDiscreteDynamicsConstraint(
      const SrbdMode& mode, bool current_stance,
      const Eigen::Vector3d& foot_pos,
      const drake::EigenPtr<Eigen::MatrixXd> &A,
      const drake::EigenPtr<Eigen::VectorXd> &b) const;

  std::vector<std::pair<int, int>> MapDecisionVariablesToKnots() const;

  std::pair<int,int> GetTerminalStepIdx() const;

  // discrete update
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::EventStatus PeriodicUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;


  // plant
  const multibody::SingleRigidBodyPlant& plant_;
  double mu_ = 0;

  // parameters and constants
  const bool use_fsm_;
  const bool use_com_;
  const bool traj_tracking_;
  const int nx_ = 12;
  const int nu_ = 4;
  const int np_ = 3;
  const int kLinearDim_ = 3;
  const int kAngularDim_ = 3;
  const double swing_ft_ht_;
  const double dt_;
  const double kMaxSolveDuration_ = 1.00;
  int nmodes_ = 0;

  // port indices
  int state_port_;
  int fsm_port_;
  int x_des_port_;
  int traj_out_port_;

  // discrete update indices
  int current_fsm_state_idx_;
  int prev_event_time_idx_;

  // Problem variables
  Eigen::MatrixXd Q_;     // For running cost x^TQx
  Eigen::MatrixXd R_;     // For running cost u^TRu
  Eigen::MatrixXd Qf_;    // For terminal cost x_{T}^{T}Q_{f}x_{T}
  std::vector<SrbdMode> modes_;
  int total_knots_ = 0;
  std::vector<Eigen::VectorXd> nominal_foot_pos_;
  Eigen::Vector3d kin_bounds_;
  Eigen::MatrixXd W_kin_reach_;
  mutable Eigen::VectorXd x_des_;
  mutable Eigen::MatrixXd x_des_mat_;

  // Solver
  drake::solvers::OsqpSolver solver_;
  mutable drake::solvers::MathematicalProgramResult result_;
  mutable drake::solvers::MathematicalProgram prog_;
  mutable int x0_mode_idx_;
  mutable int x0_knot_idx_;
  mutable lcmt_saved_traj most_recent_sol_;

  // constraints
  std::vector<drake::solvers::LinearEqualityConstraint*> state_knot_constraints_;
};
} // dairlib