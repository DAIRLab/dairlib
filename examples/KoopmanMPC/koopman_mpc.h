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
#include <Eigen/src/Core/Matrix.h>

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

#include "lcm/lcm_trajectory.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_utils.h"
#include "solvers/constraint_factory.h"
#include "systems/controllers/control_utils.h"
#include "systems/controllers/osc/osc_tracking_data.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/basic_vector.h"

namespace dairlib {

enum koopMpcStance {
  kLeft=0,
  kRight=1
};

typedef Eigen::VectorXd (*KoopmanLiftingFunc)(const Eigen::VectorXd& x);

typedef struct KoopmanDynamics {
  KoopmanLiftingFunc x_basis_func;
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd b;
} KoopmanDynamics;

typedef struct KoopmanMpcMode {
  koopMpcStance stance;
  KoopmanDynamics dynamics;
  int N;
  std::vector<drake::solvers::VectorXDecisionVariable> zz;
  std::vector<drake::solvers::VectorXDecisionVariable> uu;
  std::vector<drake::solvers::LinearEqualityConstraint*> stance_foot_constraints;
  std::vector<drake::solvers::LinearEqualityConstraint*> flat_ground_constraints;
  std::vector<drake::solvers::LinearEqualityConstraint*> dynamics_constraints;
  std::vector<drake::solvers::LinearConstraint*> friction_constraints;
  std::vector<drake::solvers::LinearConstraint*> reachability_constraints;
  std::vector<drake::solvers::LinearEqualityConstraint*> init_state_constraint_;
} KoopmanMpcMode;


class KoopmanMPC : public drake::systems::LeafSystem<double> {
 public:
  KoopmanMPC(const drake::multibody::MultibodyPlant<double>& plant,
             drake::systems::Context<double>* plant_context, double dt,
             bool planar, bool used_with_finite_state_machine = true,
             bool use_com = true);


  void AddMode(const KoopmanDynamics& dynamics, koopMpcStance stance, int N);

  void AddContactPoint(std::pair<const drake::multibody::BodyFrame<double>,
                                 Eigen::Vector3d> pt, koopMpcStance stance);

  void AddTrackingObjective(const Eigen::VectorXd& xdes, const Eigen::MatrixXd& Q);
  void AddInputRegularization(const Eigen::MatrixXd& R);
  void AddJointToTrackBaseAngle(const std::string& joint_pos_name,
                                const std::string& joint_vel_name);

  double SetMassFromListOfBodies(std::vector<std::string> bodies);

  void SetMass(double mass) {mass_ = mass;}

  void AddBaseFrame(const std::string &body_name,
      const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(),
      const Eigen::Isometry3d& frame_pose = Eigen::Isometry3d::Identity());

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

  void SetReachabilityLimit(const Eigen::MatrixXd& kl,
      const std::vector<Eigen::VectorXd>& kn);

  void SetMu(double mu) { mu_ = mu; }
  int num_modes() { return nmodes_; }

 private:

  void CalcOptimalMotionPlan(const drake::systems::Context<double>& context,
      lcmt_saved_traj* traj_msg) const;


  double CalcCentroidalMassFromListOfBodies(std::vector<std::string> bodies);

  void MakeStanceFootConstraints();
  void MakeKinematicReachabilityConstraints();
  void MakeDynamicsConstraints();
  void MakeFrictionConeConstraints();
  void MakeStateKnotConstraints();
  void MakeInitialStateConstraints();
  void MakeFlatGroundConstraints();

  Eigen::MatrixXd CalcSwingFootKnotPoints(const Eigen::VectorXd& x,
      const drake::solvers::MathematicalProgramResult& result) const;

  drake::trajectories::PiecewisePolynomial<double> MakePPTrajFromSol(
      drake::solvers::MathematicalProgramResult result) const;

  lcmt_saved_traj MakeLcmTrajFromSol(const drake::solvers::MathematicalProgramResult& result,
                                     double time,
                                     const Eigen::VectorXd& state) const;

  void UpdateInitialStateConstraint(const Eigen::VectorXd& x0,
      int fsm_state, double t_since_last_switch) const;

  void UpdateTrackingObjective(const Eigen::VectorXd& xdes) const;

  Eigen::VectorXd CalcCentroidalStateFromPlant(Eigen::VectorXd x, double t) const;


  // parameters
  bool use_fsm_;
  double dt_;

  // port indices
  int state_port_;
  int fsm_port_;
  int x_des_port_;

  // discrete update indices
  int x_des_idx_;
  int current_fsm_state_idx_;
  int prev_event_time_idx_;

  // discrete update
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  mutable drake::trajectories::PiecewisePolynomial<double> prev_sol_base_traj_;

  std::vector<std::pair<const drake::multibody::BodyFrame<double>,
  Eigen::Vector3d>> contact_points_;

  // Problem variables
  Eigen::MatrixXd Q_;
  std::vector<KoopmanMpcMode> modes_;
  std::vector<int> mode_knot_counts_;
  int total_knots_ = 0;
  std::vector<Eigen::VectorXd> kin_nominal_;
  Eigen::VectorXd kin_lim_;


  // variable counts
  int nx_;  // number of floating base states
  int nu_c_;  // number of centroidal model inputs
  int nxi_; // number of inflated states
  int nz_;  // number of koopman states
  int nmodes_ = 0;

  int nq_;    // number of plant gerenalized positions
  int nv_;    // number of plant generalized velocities
  int nu_p_;  // number of plant actuators (needed?)

  int kLinearDim_;
  int kAngularDim_;

  // Solver
  mutable drake::solvers::MathematicalProgram prog_;
  mutable int x0_idx_[2] = {0, 0};

  // constraints
  std::vector<drake::solvers::LinearEqualityConstraint*> state_knot_constraints_;
  std::vector<drake::solvers::QuadraticCost*> tracking_cost_;
  std::vector<drake::solvers::QuadraticCost*> input_cost_;


  // drake boilerplate
  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_frame_;
  std::string base_;
  Eigen::Isometry3d frame_pose_;
  Eigen::Vector3d com_from_base_origin_;
  int base_angle_pos_idx_;
  int base_angle_vel_idx_;
  mutable drake::systems::Context<double>* plant_context_;

  // constants
  const double kMaxSolveDuration_ = 0.01;
  const bool use_com_;
  const bool planar_;
  const int kNxPlanar = 6;
  const int kNx3d = 13;
  const int kNuPlanar = 6;
  const int kNu3d = 9;
  const int saggital_idx_ = 0;
  const int vertical_idx_ = 2;
  double mu_ = 0;
  double mass_ = 0;
  double planar_inertia_ = 0;
  drake::multibody::RotationalInertia<double> rotational_inertia_;

};
} // dairlib