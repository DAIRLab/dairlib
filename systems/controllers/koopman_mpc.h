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
#include "systems/controllers/control_utils.h"
#include "systems/controllers/osc/osc_tracking_data.h"
#include "systems/framework/output_vector.h"

namespace dairlib::systems::controllers {

enum koopMpcStance {
  kLeft,
  kRight
};

typedef Eigen::VectorXd (*KoopmanLiftingFunc)(const Eigen::VectorXd& x);

typedef struct KoopmanDynamics {
  KoopmanLiftingFunc x_basis_func;
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd b;
};

typedef struct KoopmanMpcMode {
  koopMpcStance stance;
  KoopmanDynamics dynamics;
  int N;
  std::vector<drake::solvers::VectorXDecisionVariable> xx_;
  std::vector<drake::solvers::VectorXDecisionVariable> uu_;

};


class KoopmanMPC : public drake::systems::LeafSystem<double> {
 public:
  KoopmanMPC(const drake::multibody::MultibodyPlant<double>& plant,
             const drake::systems::Context<double>* plant_context, double dt,
             bool planar, bool used_with_finite_state_machine = true);

  void AddMode(KoopmanMpcMode mode) { modes_.push_back(mode); }

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

 private:

  Eigen::VectorXd SolveQp(const Eigen::VectorXd x,
                          const drake::systems::Context<double>& context);
  void MakeStanceFootConstraints();
  void MakeKinematicReachabilityConstraints();
  void MakeDynamicsConstraints();
  void MakeFrictionConeConstraints();

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

  // system matrices
  std::vector<KoopmanMpcMode> modes_;

  // variable counts
  int nx_;  // number of floating base states
  int nu_;  // number of inputs
  int nxi_; // number of inflated states
  int nz_;  // number of koopman states

  // constraints
  std::vector<drake::solvers::LinearEqualityConstraint*> stance_foot_constraints_;
  std::vector<drake::solvers::LinearConstraint*> reachability_constraints_;
  std::vector<drake::solvers::LinearEqualityConstraint*> dynamics_constraints_;
  std::vector<drake::solvers::LinearConstraint*> friction_constraints_;
  std::vector<drake::solvers::QuadraticCost*> tracking_cost_;


  // drake boilerplate
  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::systems::Context<double>* plant_context_;

  virtual Eigen::VectorXd LiftState(Eigen::VectorXd x) = 0;
  virtual Eigen::VectorXd LiftInput(Eigen::VectorXd u) = 0;

  const int kNxPlanar = 6;
  const int kNx3d = 13;
  const int kNuPlanar = 6;
  const int kNu3d = 9;

};
} // dairlib::systems::controllers