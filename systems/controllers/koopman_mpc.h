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

/// Koopman MPC is an abstract class, requiring specific implementations to
/// define the koopman lifting functions

enum koopMpcStance {
  kLeft,
  kRight
};

typedef struct koopMpcMode {
  koopMpcStance stance;
  int N;
  std::vector<drake::solvers::VectorXDecisionVariable> state_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> input_vars_;

} KoopmanMode;


class KoopmanMPC : public drake::systems::LeafSystem<double> {
 public:
  KoopmanMPC(const drake::multibody::MultibodyPlant<double>& plant,
             const drake::systems::Context<double>* plant_context,
             bool planar, bool used_with_finite_state_machine = true);

  void AddMode(koopMpcMode mode, const Eigen::MatrixXd& A,
               const Eigen::MatrixXd& B, const Eigen::MatrixXd& b);

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
  // parameters
  bool use_fsm_;


  // port indices
  int state_port_;
  int fsm_port_;
  int x_des_port_;

  // discrete update
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  // system matrices
  std::vector<koopMpcMode> modes_;

  // variable counts
  int nx_;  // number of floating base states
  int nu_;  // number of inputs
  int nxi_; // number of inflated states
  int nz_;  // number of koopman states

  // decision variables
  std::vector<drake::solvers::VectorXDecisionVariable> xx_;
  std::vector<drake::solvers::VectorXDecisionVariable> uu_;

  // constraints
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