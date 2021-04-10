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

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "solvers/constraint_factory.h"
#include "systems/controllers/control_utils.h"
#include "systems/controllers/osc/osc_tracking_data.h"
#include "systems/framework/output_vector.h"

namespace dairlib::systems::controllers {

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
  std::vector<drake::solvers::LinearEqualityConstraint*> dynamics_constraints;
  std::vector<drake::solvers::LinearConstraint*> friction_constraints;
  std::vector<drake::solvers::LinearConstraint*> reachability_constraints;
} KoopmanMpcMode;


class KoopmanMPC : public drake::systems::LeafSystem<double> {
 public:
  KoopmanMPC(const drake::multibody::MultibodyPlant<double>& plant,
             const drake::systems::Context<double>* plant_context, double dt,
             bool planar, bool used_with_finite_state_machine = true);

  void AddMode(const KoopmanDynamics& dynamics, koopMpcStance stance, int N);

  void AddTrackingObjective(const Eigen::VectorXd& xdes, const Eigen::MatrixXd& Q);
  void AddInputRegularization(const Eigen::MatrixXd& R);

  void BuildController();


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

 private:

  Eigen::VectorXd SolveQp(const Eigen::VectorXd x,
                          const drake::systems::Context<double>& context);

  void MakeStanceFootConstraints();
  void MakeKinematicReachabilityConstraints();
  void MakeDynamicsConstraints();
  void MakeFrictionConeConstraints();
  void MakeStateKnotConstraints();
  void MakeInitialStateConstraint();

  void UpdateInitialStateConstraint(const Eigen::VectorXd& x0);
  void UpdateTrackingObjective(const Eigen::VectorXd& xdes);

  Eigen::VectorXd CalcCentroidalStateFromPlant();

  // parameters
  bool use_fsm_;
  double dt_;


  // port indices
  int state_port_;
  int fsm_port_;
  int x_des_port_;

  // discrete update
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  std::vector<KoopmanMpcMode> modes_;
  Eigen::VectorXd kin_lim_;
  std::vector<Eigen::VectorXd> kin_nominal_;

  // variable counts
  int nx_;  // number of floating base states
  int nu_;  // number of inputs
  int nxi_; // number of inflated states
  int nz_;  // number of koopman states

  int kLinearDim_;
  int kAngularDim_;

  // Solver
  mutable drake::solvers::MathematicalProgram prog_;

  // constraints
  std::vector<drake::solvers::LinearEqualityConstraint*> state_knot_constraints_;
  std::vector<drake::solvers::QuadraticCost*> tracking_cost_;
  std::vector<drake::solvers::QuadraticCost*> input_cost_;
  drake::solvers::LinearEqualityConstraint* initial_state_constraint_;

  // drake boilerplate
  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::systems::Context<double>* plant_context_;

  // constants
  const int kNxPlanar = 6;
  const int kNx3d = 13;
  const int kNuPlanar = 6;
  const int kNu3d = 9;
  double mu_ = 0;

};
} // dairlib::systems::controllers