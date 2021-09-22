//
// Created by brian on 3/8/21.
//

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <set>
#include <drake/multibody/plant/multibody_plant.h>
#include <dairlib/lcmt_saved_traj.hpp>

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

#include "lcm/lcm_trajectory.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_utils.h"
#include "solvers/constraint_factory.h"
#include "systems/framework/output_vector.h"
#include "solvers/fast_osqp_solver.h"

namespace dairlib {

enum BipedStance {
  kLeft=0,
  kRight=1
};

typedef struct SrbdDynamics{
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd b;
} SrbdDynamics;

typedef struct SrbdMode {
  SrbdDynamics dynamics;
  Eigen::MatrixXd A_collocation;
  Eigen::MatrixXd b_collocation;
  Eigen::VectorXd y_col_cost;
  Eigen::MatrixXd Q_total_col_cost;
  BipedStance stance;
  int N;
  std::vector<drake::solvers::VectorXDecisionVariable> xx;
  std::vector<drake::solvers::VectorXDecisionVariable> uu;
  std::vector<drake::solvers::VectorXDecisionVariable> kin_slack;
  std::vector<drake::solvers::QuadraticCost*> kin_reach_slack_cost;
  std::vector<drake::solvers::QuadraticCost*> flat_ground_soft_constraints;
  std::vector<drake::solvers::QuadraticCost*> tracking_cost;
  std::vector<drake::solvers::QuadraticCost*> terminal_cost;
  std::vector<drake::solvers::LinearEqualityConstraint*> stance_foot_constraints;
  std::vector<drake::solvers::LinearEqualityConstraint*> dynamics_constraints;
  std::vector<drake::solvers::LinearConstraint*> friction_constraints;
  std::vector<drake::solvers::LinearConstraint*> reachability_constraints;
  std::vector<drake::solvers::LinearEqualityConstraint*> init_state_constraint_;
} SrbdMode;

Eigen::Matrix3d HatOperator3x3(const Eigen::Vector3d& v);

class SrbdCMPC : public drake::systems::LeafSystem<double> {
 public:
  SrbdCMPC(const drake::multibody::MultibodyPlant<double>& plant,
           drake::systems::Context<double>* plant_context, double dt,
           double swing_ft_height, bool planar, bool traj,
           bool used_with_finite_state_machine = true,
           bool use_com = true);


  /// Appends a contact mode to the end of the planning horizon. Currently SrbdCMPC only supports the contact sequence
  /// [kLeft, kRight] so one should call AddMode(BipedStance::kLeft, BipedStance::kRight
  /// @param stance BipedStance::kLeft or BipedStance::kRight
  /// @param dynamics
  /// @param N
  void AddMode(const SrbdDynamics&  dynamics, BipedStance stance, int N);
  void AddTrackingObjective(const Eigen::VectorXd& xdes, const Eigen::MatrixXd& Q);
  void SetTerminalCost(const Eigen::MatrixXd& Qf);
  void AddInputRegularization(const Eigen::MatrixXd& R);
  void SetFlatGroundSoftConstraint(const Eigen::MatrixXd& W) {MakeFlatGroundConstraints(W);}

  int num_state() { return nx_;}
  int num_state_inflated() {return nxi_;}
  double SetMassFromListOfBodies(const std::vector<std::string>& bodies);

  void SetMass(double mass) {mass_ = mass;}

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


  void SetReachabilityLimit(const Eigen::Vector3d& kinematic_limit,
                            const std::vector<Eigen::VectorXd>& nominal_relative_pos,
                            const Eigen::MatrixXd& kin_reach_soft_contraint_w);


  void SetMu(double mu) { mu_ = mu; }
  int num_modes() { return nmodes_; }

  Eigen::Vector2d MakePlanarVectorFrom3d(Eigen::Vector3d vec) const;

  std::vector<SrbdMode> get_modes() {return modes_;}

  drake::solvers::MathematicalProgramResult solve_problem_as_is() {
    return drake::solvers::Solve(prog_);
  }

  static void CopyContinuous3dSrbDynamics(double m, double yaw, BipedStance stance,
                                      const Eigen::MatrixXd &b_I,
                                      const Eigen::Vector3d &eq_com_pos,
                                      const Eigen::Vector3d &eq_foot_pos,
                                      const drake::EigenPtr<Eigen::MatrixXd> &Ad,
                                      const drake::EigenPtr<Eigen::MatrixXd> &Bd,
                                      const drake::EigenPtr<Eigen::VectorXd> &bd);

  void print_initial_state_constraints() const;
  void print_state_knot_constraints() const;
  void print_dynamics_constraints() const;
  void print_current_init_state_constraint() const;


 private:

  void GetMostRecentMotionPlan(const drake::systems::Context<double>& context,
                               lcmt_saved_traj* traj_msg) const;

  Eigen::MatrixXd MakeSimpsonIntegratedTrackingAndInputCost(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B) const;
  Eigen::VectorXd MakeSplineSegmentReferenceStateAndInput() const ;
  void MakeStanceFootConstraints();
  void MakeKinematicReachabilityConstraints();
  void MakeDynamicsConstraints();
  void MakeFrictionConeConstraints();
  void MakeStateKnotConstraints();
  void MakeInitialStateConstraints();
  void MakeFlatGroundConstraints(const Eigen::MatrixXd& W);
  void MakeTrackingCost();
  void UpdateTrackingCost();

  Eigen::MatrixXd CalcSwingFootKnotPoints(const Eigen::VectorXd& x,
                                          const drake::solvers::MathematicalProgramResult& result,
                                          double time_since_last_touchdown) const;


  Eigen::MatrixXd MakeCollocationConstraintAMatrix(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);

  lcmt_saved_traj MakeLcmTrajFromSol(const drake::solvers::MathematicalProgramResult& result,
                                     double time, double time_since_last_touchdown,
                                     const Eigen::VectorXd& state) const;

  void UpdateInitialStateConstraint(const Eigen::VectorXd& x0,
                                    int fsm_state, double t_since_last_switch) const;


  void UpdateTrackingObjective(const Eigen::VectorXd& xdes) const;

  std::pair<int,int> GetTerminalStepIdx() const;

  // parameters
  bool use_fsm_;
  double dt_;

  // port indices
  int state_port_;
  int fsm_port_;
  int x_des_port_;
  int traj_out_port_;

  // discrete update indices
  int current_fsm_state_idx_;
  int prev_event_time_idx_;

  // discrete update
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::EventStatus PeriodicUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  // Problem variables
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::VectorXd xdes_;
  Eigen::MatrixXd Qf_;
  std::vector<SrbdMode> modes_;
  std::vector<int> mode_knot_counts_;
  int total_knots_ = 0;
  std::vector<Eigen::VectorXd> kin_nominal_;
  Eigen::VectorXd kin_lim_;
  Eigen::MatrixXd W_kin_reach_;


  // variable counts
  int nx_;  // number of floating base states
  int nxi_; // Floating base states + foot positions
  int nu_;
  int nmodes_ = 0;

  int nq_;    // number of plant gerenalized positions
  int nv_;    // number of plant generalized velocities
  int nu_p_;  // number of plant actuators (needed?)

  int kLinearDim_;
  int kAngularDim_;

  // Solver
  drake::solvers::OsqpSolver solver_;
  mutable drake::solvers::MathematicalProgramResult result_;
  mutable drake::solvers::MathematicalProgram prog_;
  mutable int x0_idx_[2] = {0, 0};
  mutable lcmt_saved_traj most_recent_sol_;

  // constraints
  std::vector<drake::solvers::LinearEqualityConstraint*> state_knot_constraints_;


  // drake boilerplate
  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_frame_;
  std::string base_;
  Eigen::Vector3d com_from_base_origin_;

  mutable drake::systems::Context<double>* plant_context_;
  mutable Eigen::VectorXd x_des_;
  mutable Eigen::MatrixXd x_des_mat_;

  // constants
  const double kMaxSolveDuration_ = 1.00;
  const double swing_ft_ht_;
  const bool use_com_;
  const bool planar_;
  const bool traj_tracking_;
  const int kNx3d = 12;
  const int kNu3d = 10;
  const Eigen::Vector3d gravity_ = {0.0, 0.0, -9.81};
  double mu_ = 0;
  double mass_ = 0;
  drake::multibody::RotationalInertia<double> rotational_inertia_;

};
} // dairlib