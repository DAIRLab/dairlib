#include "examples/goldilocks_models/controller/cassie_rom_planner_system.h"

#include <math.h>
#include <string>

using std::cout;
using std::endl;
using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;

using dairlib::systems::OutputVector;

namespace dairlib {
namespace goldilocks_models {

OptimalRomPlanner::OptimalRomPlanner(
    const MultibodyPlant<double>& plant_feedback,
    const std::vector<int>& unordered_fsm_states, double stride_period)
    : unordered_fsm_states_(unordered_fsm_states),
      stride_period_(stride_period) {
  this->set_name("mpc_traj");
  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                        OutputVector<double>(plant_feedback.num_positions(),
                                             plant_feedback.num_velocities(),
                                             plant_feedback.num_actuators()))
                    .get_index();
  fsm_and_lo_time_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(2)).get_index();

  // Provide an instance to allocate the memory first (for the output)
  PiecewisePolynomial<double> pp(VectorXd::Zero(0));
  drake::trajectories::Trajectory<double>& traj_instance = pp;
  this->DeclareAbstractOutputPort("mpc_traj", traj_instance,
                                  &OptimalRomPlanner::SolveMPC);
}

void OptimalRomPlanner::SolveMPC(
    const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd v = robot_output->GetVelocities();

  // Read in finite state machine
  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context,
                                                  fsm_and_lo_time_port_);
  VectorXd fsm_state = fsm_output->get_value();

  // Find fsm_state in unordered_fsm_states_
  auto it = find(unordered_fsm_states_.begin(), unordered_fsm_states_.end(),
                 int(fsm_state(0)));
  int mode_index = std::distance(unordered_fsm_states_.begin(), it);
  if (it == unordered_fsm_states_.end()) {
    cout << "WARNING: fsm state number " << fsm_state(0)
         << " doesn't exist in OptimalRomPlanner\n";
    mode_index = 0;
  }

  // Get time
  double timestamp = robot_output->get_timestamp();
  auto current_time = static_cast<double>(timestamp);

  // Note that you need to map state with springs to state without springs

  // Move the touchdown state to the origin

  // Construct rom traj opt

  // Solve

  // Pack the traj into lcm

  // Assign traj
  auto pp_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  // TODO: implement this
  *pp_traj = PiecewisePolynomial<double>(Vector3d::Zero());
}

}  // namespace goldilocks_models
}  // namespace dairlib
