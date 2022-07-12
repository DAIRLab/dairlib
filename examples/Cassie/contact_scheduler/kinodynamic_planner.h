#pragma once

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <dairlib/lcmt_saved_traj.hpp>

#include "examples/Cassie/contact_scheduler/kinodynamic_settings.h"
#include "lcm/lcm_trajectory.h"

#include "multibody/multibody_utils.h"
#include "solvers/constraint_factory.h"
#include "solvers/fast_osqp_solver.h"
#include "systems/framework/output_vector.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

#include "drake/multibody/optimization/centroidal_momentum_constraint.h"
//#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

typedef struct LinearSrbdDynamics {
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::VectorXd b;
} LinearSrbdDynamics;

/// This kinodynamic planner is an implementation of the Dai 2014 paper
/// (Whole-body Motion Planning with Centroidal Dynamics and Full Kinematics)
/// This outputs: footstep timing
///               final footstep location
///               center of mass trajectory
///               swing foot trajectory (maybe)?

/// TODOs:
///

/// Template borrowed from SrbdCMPC written by Brian Acosta in centroidal_models
/// branch

class KinodynamicPlanner : public drake::systems::LeafSystem<double> {
 public:
  KinodynamicPlanner(const drake::multibody::MultibodyPlant<double>& plant,
                     drake::systems::Context<double>& context,
                     const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
                     drake::systems::Context<drake::AutoDiffXd>& context_ad);

  void SetMPCSettings(KinodynamicSettings& settings);
  void SetTerminalCost(const Eigen::MatrixXd& Qf);
  void AddInputRegularization(const Eigen::MatrixXd& R);
  void AddTrackingObjective(const Eigen::VectorXd& xdes,
                            const Eigen::MatrixXd& Q);
  //  void AddFootPlacementRegularization(const Eigen::MatrixXd& W);
  void Build();
  void CheckProblemDefinition();

  // Input ports
  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_target_input_port() const {
    return this->get_input_port(x_des_port_);
  }
  const drake::systems::InputPort<double>& get_warmstart_input_port() const {
    return this->get_input_port(srb_warmstart_port_);
  }
  //  const drake::systems::InputPort<double>& get_foot_target_input_port()
  //  const {
  //    return this->get_input_port(foot_target_port_);
  //  }
  const drake::systems::OutputPort<double>& get_traj_out_port() const {
    return this->get_output_port(traj_out_port_);
  }

  void SetKinematicConstraints();

  void SetContactConstraints();

  drake::solvers::MathematicalProgramResult solve_problem_as_is() {
    return drake::solvers::Solve(prog_);
  }

  void print_constraint(
      const std::vector<drake::solvers::LinearConstraint*>& constraint) const;
  void print_constraint(
      const std::vector<drake::solvers::Constraint*>& constraint) const;
  void print_constraint(
      const std::vector<drake::solvers::LinearEqualityConstraint*>& constraint)
      const;

 private:
  void CheckSquareMatrixDimensions(const Eigen::MatrixXd& M, int n) const;
  void GetMostRecentMotionPlan(const drake::systems::Context<double>& context,
                               lcmt_saved_traj* traj_msg) const;
  void MakeKinematicReachabilityConstraints();
  void MakeTerrainConstraints(const Eigen::Vector3d& normal = {0, 0, 1},
                              const Eigen::Vector3d& origin = {0, 0, 0});
  void MakeDynamicsConstraints();
  void MakeFrictionConeConstraints();
  void MakeInitialStateConstraint();
  void MakeCost();

  lcmt_saved_traj MakeLcmTrajFromSol(
      const drake::solvers::MathematicalProgramResult& result, double time,
      double time_since_last_touchdown, const Eigen::VectorXd& state,
      int fsm_state) const;

  void UpdateConstraints(const Eigen::VectorXd& x0, int fsm_state,
                         double t_since_last_switch) const;
  void UpdateTrackingObjective(const Eigen::VectorXd& xdes) const;
  void UpdateDynamicsConstraints(const Eigen::VectorXd& x,
                                 int n_until_next_stance, int fsm_state) const;
  void UpdateKinematicConstraints(int n_until_stance, int fsm_state) const;
  void SetInitialGuess(
      int fsm_state, double timestamp,
      const Eigen::VectorXd& up_next_stance_target,
      const Eigen::VectorXd& on_deck_stance_target,
      const drake::trajectories::Trajectory<double>& srb_traj) const;
//  void CopyDiscreteDynamicsConstraint(
//      const SrbdMode& mode, bool current_stance,
//      const Eigen::Vector3d& foot_pos,
//      const drake::EigenPtr<Eigen::MatrixXd>& A,
//      const drake::EigenPtr<Eigen::VectorXd>& b) const;
  void CopyCollocationDynamicsConstraint(
      const LinearSrbdDynamics& dyn, bool current_stance,
      const Eigen::Vector3d& foot_pos,
      const drake::EigenPtr<Eigen::MatrixXd>& A,
      const drake::EigenPtr<Eigen::VectorXd>& b) const;

  // discrete update
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::EventStatus PeriodicUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;


  void AddCentroidalMomentumConstraint();
  void AddAngularMomentumDynamicsConstraint();
  void AddCoMDynamicsConstraint();
  void AddDynamicsConstraint();
  void AddIntegrationConstraints();
  void AddKinematicConstraints();


  // plant
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>& context_;
  const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_;
  drake::systems::Context<drake::AutoDiffXd>& context_ad_;
  double mu_ = 0;
  int n_q_;
  int n_v_;
  int n_u_;

  // centroidal optimization parameters
  double m_;
  Eigen::Vector3d g_;
  int n_contacts_ = 4;
  int n_r_ = 3;
  int n_h_ = 3;

  // mpc parameters
  int n_knot_points_ = 4;








  // parameters and constants
  //  const bool use_fsm_;
  //  const bool use_residuals_;
  //  const bool traj_tracking_;
  //  static constexpr int nx_ = 12;
  //  static constexpr int nu_ = 5;
  static constexpr int kLinearDim_ = 3;
  static constexpr int kAngularDim_ = 3;
  //  const double dt_;
  int nmodes_ = 0;

  // port indices
  int state_port_;
  int fsm_port_;
  int x_des_port_;
  int srbd_residual_port_;
  int traj_out_port_;
  int foot_target_port_;
  int srb_warmstart_port_;

  // discrete update indices
  int current_fsm_state_idx_;
  int prev_event_time_idx_;

  // Problem variables
  Eigen::MatrixXd Q_;   // For running cost x^TQx
  Eigen::MatrixXd R_;   // For running cost u^TRu
  Eigen::MatrixXd Wp_;  // regularizing cost on footstep location
  Eigen::MatrixXd Qf_;  // For terminal cost x_{T}^{T}Q_{f}x_{T}
  Eigen::Vector3d kin_bounds_;
  mutable Eigen::VectorXd x_des_;
  mutable Eigen::MatrixXd x_des_mat_;
  int total_knots_ = 0;

  // Solver
  drake::solvers::OsqpSolver solver_;
  mutable drake::solvers::MathematicalProgramResult result_;
  mutable drake::solvers::MathematicalProgram prog_;
  mutable lcmt_saved_traj most_recent_sol_;

  // Constraints
  std::vector<drake::solvers::QuadraticCost*> tracking_cost_;
  std::vector<drake::solvers::QuadraticCost*> input_cost_;
  std::vector<drake::solvers::QuadraticCost*> foot_target_cost_;
  std::vector<drake::solvers::LinearConstraint*> friction_cone_;
  std::vector<drake::solvers::LinearEqualityConstraint*> terrain_angle_;
  drake::solvers::LinearEqualityConstraint* initial_state_;
  mutable std::vector<
      drake::solvers::Binding<drake::solvers::LinearEqualityConstraint>>
      dynamics_;
  mutable std::vector<drake::solvers::Binding<drake::solvers::LinearConstraint>>
      kinematic_constraint_;


  // Constraints
  std::vector<std::shared_ptr<drake::multibody::CentroidalMomentumConstraint>> centroidal_momentum_constraints_;


  // Decision Variables
  std::vector<drake::solvers::VectorXDecisionVariable> q_;
  std::vector<drake::solvers::VectorXDecisionVariable> v_;
  std::vector<drake::solvers::VectorXDecisionVariable> dt_;
  std::vector<drake::solvers::VectorXDecisionVariable> r_;
  std::vector<drake::solvers::VectorXDecisionVariable> dr_;
  std::vector<drake::solvers::VectorXDecisionVariable> ddr_;
  std::vector<std::vector<drake::solvers::VectorXDecisionVariable>> c_;
  std::vector<std::vector<drake::solvers::VectorXDecisionVariable>> F_;
  std::vector<std::vector<drake::solvers::VectorXDecisionVariable>> tau_;
  std::vector<drake::solvers::VectorXDecisionVariable> h_;
  std::vector<drake::solvers::VectorXDecisionVariable> dh_;

  std::vector<drake::solvers::VectorXDecisionVariable> xx;
  std::vector<drake::solvers::VectorXDecisionVariable> uu;
  std::vector<drake::solvers::VectorXDecisionVariable> pp;
};
}  // namespace dairlib